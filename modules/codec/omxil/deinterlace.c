/*****************************************************************************
 * deinterlace.c: OpenMAX IL deinterlacing filter
 *****************************************************************************
 * Copyright © 2013 VideoLAN
 *
 * Authors: Martin Storsjo <martin@martin.st>
 * Authors: Julian Scheel <julian@jusst.de>
 *
 * This program is free software; you can redistribute it and/or modify it
 * under the terms of the GNU Lesser General Public License as published by
 * the Free Software Foundation; either version 2.1 of the License, or
 * (at your option) any later version.
 *
 * This program is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE. See the
 * GNU Lesser General Public License for more details.
 *
 * You should have received a copy of the GNU Lesser General Public License
 * along with this program; if not, write to the Free Software Foundation,
 * Inc., 51 Franklin Street, Fifth Floor, Boston MA 02110-1301, USA.
 *****************************************************************************/

#ifdef HAVE_CONFIG_H
# include "config.h"
#endif

#include <stdlib.h>
#include <assert.h>

#include <vlc_common.h>
#include <vlc_plugin.h>
#include <vlc_filter.h>

#include "omxil.h"
#include "omxil_core.h"
#include "OMX_Broadcom.h"
#define ALIGN(x, y) (((x) + ((y) - 1)) & ~((y) - 1))

/* Broadcom specific Image filters */
typedef enum OMX_IMAGEFILTERTYPE_BCM {
    OMX_ImageFilterWatercolor = 0x7F000001,
    OMX_ImageFilterPastel,
    OMX_ImageFilterSharpen,
    OMX_ImageFilterFilm,
    OMX_ImageFilterBlur,
    OMX_ImageFilterSaturation,

    OMX_ImageFilterDeInterlaceLineDouble,
    OMX_ImageFilterDeInterlaceAdvanced,

    OMX_ImageFilterColourSwap,
    OMX_ImageFilterWashedOut,
    OMX_ImageFilterColourPoint,
    OMX_ImageFilterPosterise,
    OMX_ImageFilterColourBalance,
    OMX_ImageFilterCartoon,
} OMX_IMAGEFILTERTYPE_BCM;

typedef enum OMX_INDEXTYPE_BCM {
    OMX_IndexConfigCommonImageFilterParameters = 0x7f000018,
} OMX_INDEXTYPE_BCM;

static int Open(vlc_object_t *obj);
static void Close(vlc_object_t *obj);

typedef struct omx_fifo_elem {
    struct omx_fifo_elem* next;
    void* data;
} omx_fifo_elem;

typedef struct omx_fifo {
    omx_fifo_elem* first;
    vlc_mutex_t lock;
    vlc_cond_t wait;
} omx_fifo;

static omx_fifo* omx_fifo_create() {
    omx_fifo *f = malloc(sizeof(omx_fifo));
    f->first = NULL;
    vlc_mutex_init(&f->lock);
    vlc_cond_init(&f->wait);
    return f;
}

static void omx_fifo_destroy(omx_fifo *f) {
    omx_fifo_elem *elem;
    vlc_mutex_lock(&f->lock);
    elem = f->first;
    while (elem) {
        omx_fifo_elem *next = elem->next;
        free(elem);
        elem = next;
    }
    vlc_mutex_unlock(&f->lock);
}

static void* omx_fifo_get(omx_fifo *f) {
    omx_fifo_elem *elem;
    void *data;
    vlc_mutex_lock(&f->lock);
    while (!f->first)
        vlc_cond_wait(&f->wait, &f->lock);

    elem = f->first;
    data = elem->data;
    f->first = elem->next;
    free(elem);
    vlc_mutex_unlock(&f->lock);

    return data;
}

static void* omx_fifo_get_wait(omx_fifo *f, int msecs) {
    void *data = NULL;
    omx_fifo_elem *elem;

    vlc_mutex_lock(&f->lock);

    if(!f->first) {
        mtime_t end = mdate() + msecs;
        vlc_cond_timedwait(&f->wait, &f->lock, end);
    }

    elem = f->first;
    if(elem) {
        data = elem->data;
        f->first = elem->next;
        free(elem);
    }

    vlc_mutex_unlock(&f->lock);
    return data;
}

static void omx_fifo_put(omx_fifo *f, void *data) {
    omx_fifo_elem **last;
    omx_fifo_elem *elem;
    vlc_mutex_lock(&f->lock);
    last = &f->first;
    while (*last)
        last = &(*last)->next;

    elem = malloc(sizeof(omx_fifo_elem));
    elem->next = NULL;
    elem->data = data;
    *last = elem;

    vlc_cond_signal(&f->wait);
    vlc_mutex_unlock(&f->lock);
}

vlc_module_begin()
    set_description(N_("BCM OMXIL deinterlacing filter"))
    set_capability("video filter2", 1)
    set_category(CAT_VIDEO)
    set_subcategory(SUBCAT_VIDEO_VFILTER)
    set_callbacks(Open, Close)
    add_shortcut("deinterlace")
vlc_module_end()

typedef struct filter_sys_t
{
    OMX_HANDLETYPE omx_handle;
    char psz_component[OMX_MAX_STRINGNAME_SIZE];

    OmxPort *ports;
    unsigned int num_ports;
    OmxPort out;
    OmxPort in;
    omx_fifo* timestamps;

    OmxEventQueue event_queue;
} filter_sys_t;

static picture_t *Deinterlace(filter_t *filter, picture_t *src)
{
    picture_t *p_pic = NULL, *p_next_pic = NULL;
    picture_t **pp_pic = &p_pic;
    filter_sys_t *sys = filter->p_sys;
    OMX_BUFFERHEADERTYPE *omx_header;
    OMX_ERRORTYPE omx_error;
    mtime_t *ts, *ts1, *ts2;

    while (true) {
        OMX_FIFO_PEEK(&sys->out.fifo, omx_header);

        if (!omx_header) {
            break;
        }

        if (omx_header->nFilledLen) {
            OMX_FIFO_GET(&sys->out.fifo, omx_header);
            *pp_pic = omx_header->pAppPrivate;
            if (!*pp_pic) {
                msg_Err(filter, "Indirect rendering not yet supported");
            }

            /* Do not use the OMX timestamp but the one on the fifo */
            ts = omx_fifo_get(sys->timestamps);
            (*pp_pic)->date = *ts;
            free(ts);

            omx_header->nFilledLen = 0;
            omx_header->pAppPrivate = 0;
            pp_pic = &(*pp_pic)->p_next;
            OMX_FIFO_PUT(&sys->out.fifo, omx_header);
        }
        else {
            break;
        }
    }
    *pp_pic = NULL;

    while (true) {
        OMX_FIFO_PEEK(&sys->out.fifo, omx_header);

        if (!omx_header) {
            break;
        }

        if(omx_header->pAppPrivate)
            break;

        /* get a new picture */
        if (sys->out.b_direct && !omx_header->pAppPrivate) {
            p_next_pic = filter_NewPicture(filter);
            if (p_next_pic == NULL) {
                break;
            }

            picture_CopyProperties(p_next_pic, src);
            OMX_FIFO_GET(&sys->out.fifo, omx_header);
            omx_header->pAppPrivate = p_next_pic;
            omx_header->pInputPortPrivate = omx_header->pBuffer;
            omx_header->pBuffer = p_next_pic->p[0].p_pixels;
        } else {
            msg_Err(filter, "Indirect rendering not yet supported");
        }

        omx_error = OMX_FillThisBuffer(sys->omx_handle, omx_header);
        if (omx_error != OMX_ErrorNone) {
            msg_Warn(filter, "OMX_FillThisBuffer failed (%x: %s).",
                omx_error, ErrorToString(omx_error));
        }
    }

    /* input */
    OMX_FIFO_GET_TIMEOUT(&sys->in.fifo, omx_header, 5000);
    if (omx_header) {
        if (sys->in.b_direct) {
            omx_header->pOutputPortPrivate = omx_header->pBuffer;
            omx_header->pBuffer = src->p[0].p_pixels;
            omx_header->pAppPrivate = src;
        } else {
            memcpy(omx_header->pBuffer, src->p[0].p_pixels,
                    sys->in.definition.nBufferSize);
            msg_Err(filter, "Indirect rendering not yet supported");
        }

        omx_header->nFlags = OMX_BUFFERFLAG_ENDOFFRAME;
        omx_header->nOffset = 0;
        omx_header->nFilledLen = sys->in.definition.nBufferSize;
        omx_header->nTimeStamp = ToOmxTicks(src->date);

        /* Put two timestamps for this src image onto the fifo */
        ts1 = malloc(sizeof(mtime_t));
        *ts1 = src->date;
        omx_fifo_put(sys->timestamps, ts1);

        ts2 = malloc(sizeof(mtime_t));
        *ts2 = src->date + 20000;
        omx_fifo_put(sys->timestamps, ts2);

        omx_error = OMX_EmptyThisBuffer(sys->omx_handle, omx_header);
        if (omx_error != OMX_ErrorNone) {
            msg_Warn(filter, "OMX_EmptyThisBuffer failed (%x: %s).",
                omx_error, ErrorToString(omx_error));
        }
    } else {
        msg_Err(filter, "Unable to process input image");
        picture_Release(src);
    }

    return p_pic;
}

static OMX_ERRORTYPE OmxEventHandler(OMX_HANDLETYPE omx_handle,
    OMX_PTR app_data, OMX_EVENTTYPE event, OMX_U32 data_1,
    OMX_U32 data_2, OMX_PTR event_data)
{
    VLC_UNUSED(omx_handle);
    filter_t *filter = (filter_t *)app_data;
    filter_sys_t *sys = filter->p_sys;

    PrintOmxEvent((vlc_object_t *) filter, event, data_1, data_2, event_data);
    PostOmxEvent(&sys->event_queue, event, data_1, data_2, event_data);

    return OMX_ErrorNone;
}

static OMX_ERRORTYPE OmxEmptyBufferDone(OMX_HANDLETYPE omx_handle,
    OMX_PTR app_data, OMX_BUFFERHEADERTYPE *omx_header)
{
    VLC_UNUSED(omx_handle);
    filter_t *filter = (filter_t *)app_data;
    filter_sys_t *sys = filter->p_sys;

    if (omx_header->pAppPrivate || omx_header->pOutputPortPrivate) {
        picture_t *pic = omx_header->pAppPrivate;
        omx_header->pBuffer = omx_header->pOutputPortPrivate;
        if (pic) picture_Release(pic);
        omx_header->pAppPrivate = 0;
    }
    OMX_FIFO_PUT(&sys->in.fifo, omx_header);

    return OMX_ErrorNone;
}

static OMX_ERRORTYPE OmxFillBufferDone(OMX_HANDLETYPE omx_handle,
    OMX_PTR app_data, OMX_BUFFERHEADERTYPE *omx_header)
{
    VLC_UNUSED(omx_handle);
    filter_t *filter = (filter_t *)app_data;
    filter_sys_t *sys = filter->p_sys;

    if (omx_header->pInputPortPrivate)
        omx_header->pBuffer = omx_header->pInputPortPrivate;
    OMX_FIFO_PUT(&sys->out.fifo, omx_header);

    return OMX_ErrorNone;
}

static OMX_ERRORTYPE SetDeinterlaceMode(filter_t *filter)
{
    OMX_CONFIG_IMAGEFILTERPARAMSTYPE config;
    filter_sys_t *sys = filter->p_sys;
    OMX_ERRORTYPE omx_error;

    OMX_INIT_STRUCTURE(config);
    config.nPortIndex = sys->out.i_port_index;
    config.nNumParams = 1;
    config.nParams[0] = 3;
    config.eImageFilter = OMX_ImageFilterDeInterlaceAdvanced;
    omx_error = OMX_SetConfig(sys->omx_handle,
            OMX_IndexConfigCommonImageFilterParameters, &config);

    if (omx_error != OMX_ErrorNone)
        msg_Warn(filter, "Could not select deinterlace filter.");

    return omx_error;
}

static OMX_ERRORTYPE SetFormat(filter_t *filter, OmxPort *port)
{
    OMX_PARAM_PORTDEFINITIONTYPE *definition;
    filter_sys_t *sys = filter->p_sys;
    OMX_ERRORTYPE omx_error;

    definition = &port->definition;
    definition->format.image.nFrameWidth =
        filter->fmt_in.video.i_width;
    definition->format.image.nFrameHeight =
        filter->fmt_in.video.i_height;
    definition->format.image.nStride =
        ALIGN(definition->format.image.nFrameWidth, 32);
    definition->format.image.nSliceHeight =
        ALIGN(definition->format.image.nFrameHeight, 16);
    definition->nBufferSize = definition->format.image.nStride *
        definition->format.image.nSliceHeight * 3 / 2;

    omx_error = OMX_SetParameter(sys->omx_handle,
            OMX_IndexParamPortDefinition, definition);
    if (omx_error != OMX_ErrorNone)
        msg_Warn(filter, "Could not configure port format (%x: %s).",
            omx_error, ErrorToString(omx_error));
    OMX_GetParameter(sys->omx_handle, OMX_IndexParamPortDefinition,
            &definition);

    return omx_error;
}

static int Open(vlc_object_t *obj)
{
    char ppsz_components[MAX_COMPONENTS_LIST_SIZE][OMX_MAX_STRINGNAME_SIZE];
    OMX_U8 psz_role[OMX_MAX_STRINGNAME_SIZE];
    filter_t *filter = (filter_t*) obj;
    OMX_PARAM_PORTDEFINITIONTYPE def;
    OMX_PORT_PARAM_TYPE param;
    filter_t *p_dec = filter;
    OMX_ERRORTYPE omx_error;
    filter_sys_t *sys;
    unsigned int i, j;
    int components;

    static OMX_CALLBACKTYPE callbacks =
        { OmxEventHandler, OmxEmptyBufferDone, OmxFillBufferDone };

    if (InitOmxCore(obj) != VLC_SUCCESS)
        return VLC_EGENERIC;

    components = CreateComponentsList(obj, "image_fx", ppsz_components);
    if (components <= 0) {
        DeinitOmxCore();
        msg_Err(filter, "Could not find image_fx component.\n");
        return VLC_EGENERIC;
    }

    sys = calloc(1, sizeof(*sys));
    if (unlikely(sys == NULL))
            return VLC_ENOMEM;
    strcpy(sys->psz_component, ppsz_components[0]);
    filter->p_sys = sys;

    /* Initialize the OMX component */
    omx_error = pf_get_handle(&sys->omx_handle, sys->psz_component, filter,
            &callbacks);
    CHECK_ERROR(omx_error, "OMX_GetHandle(%s) failed (%x: %s)",
            sys->psz_component, omx_error, ErrorToString(omx_error));

    OMX_ComponentRoleEnum(sys->omx_handle, psz_role, 0);
    PrintOmx(obj, sys->omx_handle, OMX_ALL);

    InitOmxEventQueue(&sys->event_queue);

    OMX_INIT_STRUCTURE(param);
    OMX_INIT_STRUCTURE(def);
    omx_error = OMX_GetParameter(sys->omx_handle, OMX_IndexParamImageInit, &param);
    CHECK_ERROR(omx_error, "OMX_GetParameter(OMX_IndexParamImageInit) failed (%x: %s)",
                omx_error, ErrorToString(omx_error));

    sys->num_ports = param.nPorts;
    sys->ports = &sys->out;
    for (i = 0; i < param.nPorts; i++) {
        OmxPort *p_port;

        def.nPortIndex = param.nStartPortNumber + i;
        omx_error = OMX_GetParameter(sys->omx_handle,
                OMX_IndexParamPortDefinition, &def);
        if (omx_error != OMX_ErrorNone) {
            msg_Warn(obj, "Port Defintion could not be retrieved for port %d",
                    (int)def.nPortIndex);
            continue;
        }

        if (def.eDir == OMX_DirInput) {
            def.nBufferCountActual = 5;
            p_port = &sys->in;
        } else {
            def.nBufferCountActual = 2;
            p_port = &sys->out;
        }

        omx_error = OMX_SetParameter(sys->omx_handle,
                OMX_IndexParamPortDefinition, &def);
        if (omx_error != OMX_ErrorNone) {
            msg_Warn(obj, "Port Defintion could not be updated for port %d",
                    (int)def.nPortIndex);
            continue;
        }

        p_port->b_valid = true;
        p_port->i_port_index = def.nPortIndex;
        p_port->definition = def;
        p_port->omx_handle = sys->omx_handle;
    }

    if (!sys->in.b_valid || !sys->out.b_valid) {
        omx_error = OMX_ErrorInvalidComponent;
        CHECK_ERROR(omx_error, "couldn't find an input and/or output port.");
    }

    SetFormat(filter, &sys->in);
    SetFormat(filter, &sys->out);

    /* Initialize timestamp fifo */
    sys->timestamps = omx_fifo_create();

    /* allocate array for port buffers */
    for (i = 0; i < sys->num_ports; i++) {
        OmxPort *port = &sys->ports[i];

        vlc_mutex_init(&port->fifo.lock);
        vlc_cond_init(&port->fifo.wait);
        port->fifo.pp_last = &port->fifo.p_first;
        port->b_flushed = true;
        if (port == &sys->in) {
            port->b_direct = true;
            port->p_fmt = &filter->fmt_in;
            port->fifo.offset = offsetof(OMX_BUFFERHEADERTYPE, pOutputPortPrivate) /
                                sizeof(void*);
        } else {
            port->b_direct = true;
            port->p_fmt = &filter->fmt_out;
            port->fifo.offset = offsetof(OMX_BUFFERHEADERTYPE, pInputPortPrivate) /
                                sizeof(void*);
        }

        port->pp_buffers = malloc(port->definition.nBufferCountActual *
                sizeof(OMX_BUFFERHEADERTYPE*));
        if (unlikely(port->pp_buffers == NULL)) {
            omx_error = OMX_ErrorInsufficientResources;
            CHECK_ERROR(omx_error, "memory allocation failed");
        }
        port->i_buffers = port->definition.nBufferCountActual;

        /* enable port */
        if (!port->definition.bEnabled) {
            omx_error = OMX_SendCommand(sys->omx_handle, OMX_CommandPortEnable,
                    port->i_port_index, NULL);
            CHECK_ERROR(omx_error, "OMX_CommandPortEnable on %i failed (%x)",
                    (int)port->i_port_index, omx_error);

            omx_error = WaitForSpecificOmxEvent(&sys->event_queue,
                    OMX_EventCmdComplete, 0, 0, 0);
            CHECK_ERROR(omx_error, "Wait for PortEnable on %i failed (%x)",
                    (int)port->i_port_index, omx_error);
        }
    }

    /* Put component into idle state */
    omx_error = OMX_SendCommand(sys->omx_handle, OMX_CommandStateSet, OMX_StateIdle, 0);
    CHECK_ERROR(omx_error, "OMX_CommandStateSet Idle failed (%x: %s)", omx_error,
            ErrorToString(omx_error));

    /* allocate buffers/bufferheaders for direct rendering */
    for (i = 0; i < sys->num_ports; i++) {
        OmxPort *port = &sys->ports[i];

        for (j = 0; j < port->i_buffers; j++) {
            if (port->b_direct) {
                omx_error = OMX_UseBuffer(sys->omx_handle, &port->pp_buffers[j],
                        port->i_port_index, 0, port->definition.nBufferSize,
                        (void*)1);
                CHECK_ERROR(omx_error, "OMX_UseBuffer failed (%x: %s)", omx_error,
                        ErrorToString(omx_error));
            } else {
                omx_error = OMX_AllocateBuffer(sys->omx_handle,
                        &port->pp_buffers[j], port->i_port_index, 0,
                        port->definition.nBufferSize);
            }

            if (omx_error != OMX_ErrorNone) {
                msg_Warn(obj, "Buffer allocate failed on buffer %d for port %d (%x: %s)",
                        j, i, omx_error, ErrorToString(omx_error));
                CHECK_ERROR(omx_error, "OMX_FillBuffer failed (%x: %s)", omx_error,
                        ErrorToString(omx_error));
                break;
            }
            OMX_FIFO_PUT(&port->fifo, port->pp_buffers[j]);
        }
    }

    omx_error = WaitForSpecificOmxEvent(&sys->event_queue, OMX_EventCmdComplete,
            0, 0, 0);
    CHECK_ERROR(omx_error, "Wait for Idle failed (%x: %s)", omx_error,
            ErrorToString(omx_error));

    SetDeinterlaceMode(filter);

    omx_error = OMX_SendCommand(sys->omx_handle, OMX_CommandStateSet,
            OMX_StateExecuting, 0);
    CHECK_ERROR(omx_error, "OMX_CommandStateSet Executing failed (%x)", omx_error);

    omx_error = WaitForSpecificOmxEvent(&sys->event_queue,
            OMX_EventCmdComplete, 0, 0, 0);
    CHECK_ERROR(omx_error, "Wait for Executing failed (%x)", omx_error );

    filter->pf_video_filter = Deinterlace;
    filter->fmt_out.video.i_frame_rate = 50;
    filter->fmt_out.video.i_frame_rate_base = 1;

    return VLC_SUCCESS;

error:
    Close(obj);
    return VLC_EGENERIC;
}

static void Close(vlc_object_t *obj)
{
    filter_t *filter = (filter_t*) obj;
    filter_sys_t *sys = filter->p_sys;
    unsigned int i, j;

    if (sys->omx_handle) {
        OMX_STATETYPE state;
        OMX_GetState(sys->omx_handle, &state);

        if (state == OMX_StateExecuting) {
            OMX_SendCommand(sys->omx_handle, OMX_CommandStateSet,
                    OMX_StateIdle, 0);
            while (1) {
                OMX_U32 cmd, state;
                WaitForSpecificOmxEvent(&sys->event_queue, OMX_EventCmdComplete,
                        &cmd, &state, 0);
                if (cmd == OMX_CommandStateSet && state == OMX_StateIdle)
                    break;
            }
        }

        OMX_GetState(sys->omx_handle, &state);
        if (state == OMX_StateIdle) {
            OMX_SendCommand(sys->omx_handle, OMX_CommandStateSet,
                    OMX_StateLoaded, 0);

            for (i = 0; i < sys->num_ports; i++) {
                OmxPort *port = &sys->ports[i];
                for (j = 0; j < sys->ports[i].i_buffers; j++) {
                    OMX_BUFFERHEADERTYPE *buffer;
                    OMX_FIFO_GET(&port->fifo, buffer);
                    OMX_FreeBuffer(sys->omx_handle, port->i_port_index,
                            buffer);
                }
            }

            WaitForSpecificOmxEvent(&sys->event_queue, OMX_EventCmdComplete,
                    0, 0, 0);
        }
        for (i = 0; i < sys->num_ports; i++)
            free(sys->ports[i].pp_buffers);
        pf_free_handle(sys->omx_handle);
        DeinitOmxEventQueue(&sys->event_queue);
    }

    free(sys);
    DeinitOmxCore();
}
