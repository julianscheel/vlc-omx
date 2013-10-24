/*****************************************************************************
 * vout.c: OpenMAX IL video output
 *****************************************************************************
 * Copyright © 2013 VideoLAN
 *
 * Authors: Martin Storsjo <martin@martin.st>
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

#include <vlc_common.h>
#include <vlc_plugin.h>
#include <vlc_vout_display.h>
#include <vlc_picture_pool.h>

#include "omxil.h"
#include "omxil_core.h"
#include "OMX_Broadcom.h"

#define ALIGN(x, y) (((x) + ((y) - 1)) & ~((y) - 1))
#define SCHEDULER_BUFFERED_PICTURES 4

/* Defined in the broadcom version of OMX_Index.h */
#define OMX_IndexConfigDisplayRegion 0x7f000010
#define OMX_IndexConfigLatencyTarget 0x7f0000a5

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

void print_omx_debug_info(vlc_object_t *vlc_obj) {
#define OMX_DEBUG_INFO_LEN 4096
    if (pf_get_debug_information) {
        OMX_STRING debug_info[OMX_DEBUG_INFO_LEN];
        debug_info[0] = 0;
        int len = OMX_DEBUG_INFO_LEN;
        pf_get_debug_information(debug_info, &len);
        msg_Dbg(vlc_obj, "OMX_GetDebugInformation:  \n%s", debug_info);
    }
}

/*****************************************************************************
 * Module descriptor
 *****************************************************************************/
static int  Open (vlc_object_t *);
static void Close(vlc_object_t *);

vlc_module_begin()
    set_category(CAT_VIDEO)
    set_subcategory(SUBCAT_VIDEO_VOUT)
    set_shortname("omxil_vout")
    set_description(N_("OpenMAX IL video output"))
    set_capability("vout display", 0)
    set_callbacks(Open, Close)
vlc_module_end()

/*****************************************************************************
 * Local prototypes
 *****************************************************************************/

static picture_pool_t *Pool  (vout_display_t *, unsigned);
static void           Display(vout_display_t *, picture_t *, subpicture_t *);
static int            Control(vout_display_t *, int, va_list);

#ifdef RPI_OMX
static const vlc_fourcc_t rpi_subpicture_chromas[] = {
    VLC_CODEC_RGBA,
    0
};
#endif

/* */
struct vout_display_sys_t {
    picture_pool_t *pool;

    OMX_HANDLETYPE image_fx_handle;
    OmxEventQueue image_fx_eq;
    OMX_U32 image_fx_port_in;
    OMX_U32 image_fx_port_out;
    bool deinterlace_enabled;

    OMX_HANDLETYPE clock_handle;
    OmxEventQueue clock_eq;
    OMX_U32 clock_port_out;
    bool clock_initialized;

    OMX_HANDLETYPE scheduler_handle;
    OmxEventQueue scheduler_eq;
    OMX_U32 scheduler_port_video_in;
    OMX_U32 scheduler_port_video_out;
    OMX_U32 scheduler_port_clock_in;

    OMX_HANDLETYPE renderer_handle;
    OmxEventQueue renderer_eq;
    OMX_U32 renderer_port_in;

    OmxPort port;
    mtime_t cur_ts;
    mtime_t musecs_per_frame;
};

struct picture_sys_t {
    OMX_BUFFERHEADERTYPE *buf;
    vout_display_sys_t *sys;
};

static int LockSurface(picture_t *);
static void UnlockSurface(picture_t *);

static OMX_ERRORTYPE OmxEventHandler(OMX_HANDLETYPE, OMX_PTR, OMX_EVENTTYPE,
                                     OMX_U32, OMX_U32, OMX_PTR);
static OMX_ERRORTYPE OmxEmptyBufferDone(OMX_HANDLETYPE, OMX_PTR,
                                        OMX_BUFFERHEADERTYPE *);
static OMX_ERRORTYPE OmxFillBufferDone(OMX_HANDLETYPE, OMX_PTR,
                                       OMX_BUFFERHEADERTYPE *);

static OMX_ERRORTYPE OmxEventHandler(OMX_HANDLETYPE omx_handle,
    OMX_PTR app_data, OMX_EVENTTYPE event, OMX_U32 data_1,
    OMX_U32 data_2, OMX_PTR event_data)
{
    VLC_UNUSED(omx_handle);
    vout_display_t *vd = (vout_display_t *)app_data;
    vout_display_sys_t *p_sys = vd->sys;

    if (omx_handle == p_sys->image_fx_handle) {
        PrintOmxEvent((vlc_object_t *) vd, "image_fx", event, data_1, data_2, event_data);
        PostOmxEvent(&p_sys->image_fx_eq, event, data_1, data_2, event_data);
    } else if (omx_handle == p_sys->clock_handle) {
        PrintOmxEvent((vlc_object_t *) vd, "clock", event, data_1, data_2, event_data);
        PostOmxEvent(&p_sys->clock_eq, event, data_1, data_2, event_data);
    } else if (omx_handle == p_sys->scheduler_handle) {
        PrintOmxEvent((vlc_object_t *) vd, "scheduler", event, data_1, data_2, event_data);
        PostOmxEvent(&p_sys->scheduler_eq, event, data_1, data_2, event_data);
    } else if (omx_handle == p_sys->renderer_handle) {
        PrintOmxEvent((vlc_object_t *) vd, "iv_renderer", event, data_1, data_2, event_data);
        PostOmxEvent(&p_sys->renderer_eq, event, data_1, data_2, event_data);
    }

    return OMX_ErrorNone;
}

static OMX_ERRORTYPE OmxEmptyBufferDone(OMX_HANDLETYPE omx_handle,
    OMX_PTR app_data, OMX_BUFFERHEADERTYPE *omx_header)
{
    VLC_UNUSED(omx_handle);
    vout_display_t *vd = (vout_display_t *)app_data;
    vout_display_sys_t *p_sys = vd->sys;

    picture_Release(omx_header->pAppPrivate);
    return OMX_ErrorNone;
}

static OMX_ERRORTYPE OmxFillBufferDone(OMX_HANDLETYPE omx_handle,
    OMX_PTR app_data, OMX_BUFFERHEADERTYPE *omx_header)
{
    VLC_UNUSED(omx_handle);
    VLC_UNUSED(app_data);
    VLC_UNUSED(omx_header);

    return OMX_ErrorNone;
}

static OMX_ERRORTYPE UpdateDisplaySize(vout_display_t *vd, vout_display_cfg_t *cfg)
{
    vout_display_sys_t *p_sys = vd->sys;
    OMX_CONFIG_DISPLAYREGIONTYPE config_display;
    OMX_INIT_STRUCTURE(config_display);

    config_display.nPortIndex = p_sys->renderer_port_in;
    config_display.set = OMX_DISPLAY_SET_PIXEL;
    config_display.pixel_x = cfg->display.width  * vd->fmt.i_height;
    config_display.pixel_y = cfg->display.height * vd->fmt.i_width;
    return OMX_SetConfig(vd->sys->renderer_handle, OMX_IndexConfigDisplayRegion, &config_display);
}

static OMX_ERRORTYPE set_image_format(OMX_HANDLETYPE *handle, OMX_PARAM_PORTDEFINITIONTYPE *def, video_format_t *fmt) {
    OMX_ERRORTYPE omx_error;

    omx_error = OMX_GetParameter(handle, OMX_IndexParamPortDefinition, def);
    if (omx_error != OMX_ErrorNone)
        return omx_error;

    def->format.image.nFrameWidth = fmt->i_width;
    def->format.image.nFrameHeight = fmt->i_height;
    def->format.image.nStride = ALIGN(def->format.image.nFrameWidth, 32);
    def->format.image.nSliceHeight = ALIGN(def->format.image.nFrameHeight, 16);
    def->nBufferSize = def->format.image.nStride * def->format.image.nSliceHeight * 3 / 2;

    omx_error = OMX_SetParameter(handle, OMX_IndexParamPortDefinition, def);
    if (omx_error != OMX_ErrorNone)
        return omx_error;

    return OMX_GetParameter(handle, OMX_IndexParamPortDefinition, def);
}

static OMX_ERRORTYPE set_video_format(OMX_HANDLETYPE *handle, OMX_PARAM_PORTDEFINITIONTYPE *def, video_format_t *fmt) {
    OMX_ERRORTYPE omx_error;

    omx_error = OMX_GetParameter(handle, OMX_IndexParamPortDefinition, def);
    if (omx_error != OMX_ErrorNone)
        return omx_error;

    def->format.video.nFrameWidth = fmt->i_width;
    def->format.video.nFrameHeight = fmt->i_height;
    def->format.video.nStride = ALIGN(def->format.video.nFrameWidth, 32);
    def->format.video.nSliceHeight = ALIGN(def->format.video.nFrameHeight, 16);
    def->nBufferSize = def->format.video.nStride * def->format.video.nSliceHeight * 3 / 2;

    omx_error = OMX_SetParameter(handle, OMX_IndexParamPortDefinition, def);
    if (omx_error != OMX_ErrorNone)
        return omx_error;

    return OMX_GetParameter(handle, OMX_IndexParamPortDefinition, def);
}

static OMX_ERRORTYPE SetDeinterlaceMode(OMX_HANDLETYPE *handle, OMX_U32 port_index, bool enable) {
    OMX_CONFIG_IMAGEFILTERPARAMSTYPE config;

    OMX_INIT_STRUCTURE(config);
    config.nPortIndex = port_index;
    if (enable) {
        config.nNumParams = 1;
        config.nParams[0] = 3;
        config.eImageFilter = OMX_ImageFilterDeInterlaceAdvanced;
    } else {
        config.eImageFilter = OMX_ImageFilterNone;
    }

    return OMX_SetConfig(handle, OMX_IndexConfigCommonImageFilterParameters, &config);
}

static OMX_ERRORTYPE init_omx_component(vlc_object_t *vlc_obj, const char *name, OMX_CALLBACKTYPE callbacks, OMX_HANDLETYPE *handle, char *component_name) {
    char ppsz_components[MAX_COMPONENTS_LIST_SIZE][OMX_MAX_STRINGNAME_SIZE];
    OMX_ERRORTYPE omx_error;

    int components = CreateComponentsList(vlc_obj, name, ppsz_components);
    if (components <= 0)
        return OMX_ErrorComponentNotFound;

    pf_get_handle(handle, ppsz_components[0], vlc_obj, &callbacks);

    if (component_name != NULL)
        strcpy(component_name, ppsz_components[0]);

    return OMX_ErrorNone;
}

static OMX_ERRORTYPE set_nbuffercountactual(OMX_HANDLETYPE *handle, OMX_PARAM_PORTDEFINITIONTYPE *def, int nbuffercountactual) {
    OMX_ERRORTYPE omx_error;
    def->nBufferCountActual = nbuffercountactual;
    omx_error = OMX_SetParameter(handle, OMX_IndexParamPortDefinition, def);
    if (omx_error != OMX_ErrorNone)
       return omx_error;

    return OMX_GetParameter(handle, OMX_IndexParamPortDefinition, def);
}

static OMX_ERRORTYPE disable_all_ports(OmxEventQueue *queue, OMX_HANDLETYPE *handle) {
    OMX_ERRORTYPE omx_error;

    OMX_INDEXTYPE index_types[] = {
        OMX_IndexParamAudioInit,
        OMX_IndexParamImageInit,
        OMX_IndexParamVideoInit,
        OMX_IndexParamOtherInit,
    };

    OMX_PORT_PARAM_TYPE ports;
    OMX_INIT_STRUCTURE(ports);

    for (int i = 0; i < 4; ++i) {
        omx_error = OMX_GetParameter(handle, index_types[i], &ports);
        if (omx_error == OMX_ErrorNone) {
            for (uint32_t j = 0; j < ports.nPorts; ++j) {
                OMX_PARAM_PORTDEFINITIONTYPE port_fmt;
                OMX_INIT_STRUCTURE(port_fmt);
                port_fmt.nPortIndex = ports.nStartPortNumber+j;

                omx_error = OMX_GetParameter(handle, OMX_IndexParamPortDefinition, &port_fmt);
                if (omx_error != OMX_ErrorNone) {
                    if(port_fmt.bEnabled == OMX_FALSE)
                        continue;
                }

                omx_error = OMX_SendCommand(handle, OMX_CommandPortDisable, ports.nStartPortNumber + j, NULL);
                if (omx_error != OMX_ErrorNone)
                    return omx_error;

                omx_error = WaitForSpecificOmxEvent(queue, OMX_EventCmdComplete, 0, 0, 0);
                if (omx_error != OMX_ErrorNone)
                    return omx_error;
            }
        }
    }

    return OMX_ErrorNone;
}

static OMX_ERRORTYPE transition_to_state(OmxEventQueue *queue, OMX_HANDLETYPE *handle, OMX_STATETYPE state_to) {
    OMX_STATETYPE cur_state;
    OMX_ERRORTYPE omx_error;

    omx_error = OMX_GetState(handle, &cur_state);
    if (omx_error != OMX_ErrorNone)
        return omx_error;

    if (cur_state == state_to)
        return OMX_ErrorNone;

    omx_error = OMX_SendCommand(handle, OMX_CommandStateSet, state_to, NULL);
    if (omx_error != OMX_ErrorNone)
        return omx_error;

    return WaitForSpecificOmxEvent(queue, OMX_EventCmdComplete, 0, 0, 0);
}

static void set_misecs_per_frame(vout_display_sys_t *p_sys, video_format_t *fmt) {
    if (fmt && (fmt->i_frame_rate_base > 0) && (fmt->i_frame_rate > 0)) {
        p_sys->musecs_per_frame = (mtime_t)1000000 * fmt->i_frame_rate_base / fmt->i_frame_rate;
    } else {
        p_sys->musecs_per_frame = 0;
    }
}

static int Open(vlc_object_t *p_this)
{
    vout_display_t *vd = (vout_display_t *)p_this;
    vout_display_t *p_dec = vd;
    picture_t** pictures = NULL;
    OMX_PARAM_PORTDEFINITIONTYPE *def;
    OMX_PORT_PARAM_TYPE param;
    OMX_ERRORTYPE omx_error;

    OMX_INIT_STRUCTURE(param);

    OMX_CALLBACKTYPE callbacks =
        { OmxEventHandler, OmxEmptyBufferDone, OmxFillBufferDone };

    if (InitOmxCore(p_this) != VLC_SUCCESS)
        return VLC_EGENERIC;

    /* Allocate structure */
    vout_display_sys_t *p_sys = (struct vout_display_sys_t*) calloc(1, sizeof(*p_sys));
    if (!p_sys) {
        DeinitOmxCore();
        return VLC_ENOMEM;
    }
    vd->sys = p_sys;

    p_sys->deinterlace_enabled = vd->fmt.i_height != 720;
    set_misecs_per_frame(p_sys, &vd->fmt);

    /* Initialize image_fx component */
    InitOmxEventQueue(&p_sys->image_fx_eq);
    omx_error = init_omx_component(p_this, "image_fx", callbacks, &p_sys->image_fx_handle, NULL);
    CHECK_ERROR(omx_error, "init_omx_component(image_fx) failed (%x: %s)",
            omx_error, ErrorToString(omx_error));

    omx_error = OMX_GetParameter(p_sys->image_fx_handle, OMX_IndexParamImageInit, &param);
    CHECK_ERROR(omx_error, "OMX_GetParameter(OMX_IndexParamImageInit) failed (%x: %s)",
            omx_error, ErrorToString(omx_error));

    p_sys->image_fx_port_in = param.nStartPortNumber;
    p_sys->image_fx_port_out = param.nStartPortNumber + 1;

    /* Initialize clock component */
    InitOmxEventQueue(&p_sys->clock_eq);
    omx_error = init_omx_component(p_this, "clock", callbacks, &p_sys->clock_handle, NULL);
    CHECK_ERROR(omx_error, "init_omx_component(clock) failed (%x: %s)",
            omx_error, ErrorToString(omx_error));

    omx_error = OMX_GetParameter(p_sys->clock_handle, OMX_IndexParamOtherInit, &param);
    CHECK_ERROR(omx_error, "OMX_GetParameter(OMX_IndexParamOtherInit) failed (%x: %s)",
            omx_error, ErrorToString(omx_error));

    p_sys->clock_port_out = param.nStartPortNumber;

    /* Initialize scheduler component */
    InitOmxEventQueue(&p_sys->scheduler_eq);
    omx_error = init_omx_component(p_this, "video_scheduler", callbacks, &p_sys->scheduler_handle, NULL);
    CHECK_ERROR(omx_error, "init_omx_component(video_scheduler) failed (%x: %s)",
            omx_error, ErrorToString(omx_error));

    omx_error = OMX_GetParameter(p_sys->scheduler_handle, OMX_IndexParamVideoInit, &param);
    CHECK_ERROR(omx_error, "OMX_GetParameter(OMX_IndexParamVideoInit) failed (%x: %s)",
            omx_error, ErrorToString(omx_error));

    p_sys->scheduler_port_video_in = param.nStartPortNumber;
    p_sys->scheduler_port_video_out = param.nStartPortNumber + 1;

    omx_error = OMX_GetParameter(p_sys->scheduler_handle, OMX_IndexParamOtherInit, &param);
    CHECK_ERROR(omx_error, "OMX_GetParameter(OMX_IndexParamOtherInit) failed (%x: %s)",
            omx_error, ErrorToString(omx_error));

    p_sys->scheduler_port_clock_in = param.nStartPortNumber;

    /* Initialize renderer component */
    InitOmxEventQueue(&p_sys->renderer_eq);
    omx_error = init_omx_component(p_this, "iv_renderer", callbacks, &p_sys->renderer_handle, NULL);
    CHECK_ERROR(omx_error, "init_omx_component(iv_renderer) failed (%x: %s)",
            omx_error, ErrorToString(omx_error));

    omx_error = OMX_GetParameter(p_sys->renderer_handle, OMX_IndexParamVideoInit, &param);
    CHECK_ERROR(omx_error, "OMX_GetParameter(OMX_IndexParamVideoInit) failed (%x: %s)",
            omx_error, ErrorToString(omx_error));

    p_sys->renderer_port_in = param.nStartPortNumber;

    /* Initialize fifo */
    vlc_mutex_init(&p_sys->port.fifo.lock);
    vlc_cond_init(&p_sys->port.fifo.wait);
    p_sys->port.fifo.offset = offsetof(OMX_BUFFERHEADERTYPE, pOutputPortPrivate) / sizeof(void *);
    p_sys->port.fifo.pp_last = &p_sys->port.fifo.p_first;
    p_sys->port.b_direct = true;
    p_sys->port.b_flushed = true;

    /* Configure input port */
    p_sys->port.i_port_index = p_sys->image_fx_port_in;
    p_sys->port.omx_handle = p_sys->image_fx_handle;
    p_sys->port.b_valid = true;

    OMX_INIT_STRUCTURE(p_sys->port.definition);
    p_sys->port.definition.nPortIndex = p_sys->port.i_port_index;

    omx_error = set_image_format(p_sys->port.omx_handle, &p_sys->port.definition, &vd->fmt);
    CHECK_ERROR(omx_error, "Failed to set format for the input port (%x: %s)",
            omx_error, ErrorToString(omx_error));

    omx_error = set_nbuffercountactual(p_sys->port.omx_handle, &p_sys->port.definition, 25);
    CHECK_ERROR(omx_error, "set_nbuffercountactual for input port to %d failed (%x: %s)",
            p_sys->port.definition.nBufferCountActual, omx_error, ErrorToString(omx_error));

    /* Configure output port */
    OMX_PARAM_PORTDEFINITIONTYPE out_port_def;
    OMX_INIT_STRUCTURE(out_port_def);
    out_port_def.nPortIndex = p_sys->image_fx_port_out;
    omx_error = set_image_format(p_sys->image_fx_handle, &out_port_def, &vd->fmt);
    CHECK_ERROR(omx_error, "Failed to set format for the output port (%x: %s)",
            omx_error, ErrorToString(omx_error));


    /* Configure clock */
    omx_error = disable_all_ports(&p_sys->clock_eq, p_sys->clock_handle);
    CHECK_ERROR(omx_error, "disable_all_ports for clock failed (%x: %s)",
            omx_error, ErrorToString(omx_error));
    omx_error = OMX_SendCommand(p_sys->clock_handle, OMX_CommandPortEnable, p_sys->clock_port_out, NULL);
    CHECK_ERROR(omx_error, "OMX_CommandPortEnable %d failed (%x: %s)",
            p_sys->clock_port_out, omx_error, ErrorToString(omx_error));
    omx_error = WaitForSpecificOmxEvent(&p_sys->clock_eq, OMX_EventCmdComplete, 0, 0, 0);
    CHECK_ERROR(omx_error, "Wait for OMX_CommandPortEnable %d failed (%x: %s)",
            p_sys->clock_port_out, omx_error, ErrorToString(omx_error));

    OMX_TIME_CONFIG_CLOCKSTATETYPE clock_state;
    OMX_INIT_STRUCTURE(clock_state);
    omx_error = OMX_GetParameter(p_sys->clock_handle, OMX_IndexConfigTimeClockState, &clock_state);
    CHECK_ERROR(omx_error, "OMX_IndexConfigTimeClockState failed (%x: %s)",
            omx_error, ErrorToString(omx_error));

    clock_state.eState = OMX_TIME_ClockStateWaitingForStartTime;
    clock_state.nWaitMask = OMX_CLOCKPORT0;
    /*clock_state.nOffset = ToOmxTicks(...) */
    omx_error = OMX_SetParameter(p_sys->clock_handle, OMX_IndexConfigTimeClockState, &clock_state);
    CHECK_ERROR(omx_error, "OMX_IndexConfigTimeClockState failed (%x: %s)",
            omx_error, ErrorToString(omx_error));

    OMX_TIME_CONFIG_ACTIVEREFCLOCKTYPE clock_out_type;
    OMX_INIT_STRUCTURE(clock_out_type);
    clock_out_type.eClock = OMX_TIME_RefClockVideo;
    omx_error = OMX_SetConfig(p_sys->clock_handle, OMX_IndexConfigTimeActiveRefClock, &clock_out_type);
    CHECK_ERROR(omx_error, "OMX_IndexConfigTimeActiveRefClock failed (%x: %s)",
            omx_error, ErrorToString(omx_error));

    OMX_CONFIG_LATENCYTARGETTYPE latency;
    OMX_INIT_STRUCTURE(latency);
    latency.nPortIndex = OMX_ALL;
    latency.bEnabled = OMX_TRUE;
    latency.nFilter = 10;
    latency.nTarget = 0;
    latency.nShift = 3;
    latency.nSpeedFactor = -200;
    latency.nInterFactor = 100;
    latency.nAdjCap = 100;
    omx_error = OMX_SetConfig(p_sys->clock_handle, OMX_IndexConfigLatencyTarget, &latency);
    CHECK_ERROR(omx_error, "OMX_IndexConfigLatencyTarget failed (%x: %s)",
            omx_error, ErrorToString(omx_error));


    /* Configure renderer */
    latency.nPortIndex = p_sys->renderer_port_in;
    latency.bEnabled = OMX_TRUE;
    latency.nFilter = 2;
    latency.nTarget = 4000;
    latency.nShift = 3;
    latency.nSpeedFactor = -135;
    latency.nInterFactor = 500;
    latency.nAdjCap = 20;
    omx_error = OMX_SetConfig(p_sys->renderer_handle, OMX_IndexConfigLatencyTarget, &latency);
    CHECK_ERROR(omx_error, "OMX_IndexConfigLatencyTarget failed (%x: %s)",
            omx_error, ErrorToString(omx_error));


    /* Setup tunnels */
    omx_error = pf_setup_tunnel(p_sys->image_fx_handle, p_sys->image_fx_port_out, p_sys->scheduler_handle, p_sys->scheduler_port_video_in);
    CHECK_ERROR(omx_error, "OMX_SetupTunnel failed (%x: %s)",
            omx_error, ErrorToString(omx_error));

    omx_error = pf_setup_tunnel(p_sys->clock_handle, p_sys->clock_port_out, p_sys->scheduler_handle, p_sys->scheduler_port_clock_in);
    CHECK_ERROR(omx_error, "OMX_SetupTunnel failed (%x: %s)",
            omx_error, ErrorToString(omx_error));

    omx_error = pf_setup_tunnel(p_sys->scheduler_handle, p_sys->scheduler_port_video_out, p_sys->renderer_handle, p_sys->renderer_port_in);
    CHECK_ERROR(omx_error, "OMX_SetupTunnel failed (%x: %s)",
            omx_error, ErrorToString(omx_error));

    msg_Dbg(vd, "Tunnel Setup completed");


    /* Transition everything to idle */
    omx_error = OMX_SendCommand(p_sys->image_fx_handle, OMX_CommandStateSet, OMX_StateIdle, NULL);
    CHECK_ERROR(omx_error, "OMX_CommandStateSet OMX_StateIdle failed (%x: %s)",
            omx_error, ErrorToString(omx_error));

    omx_error = OMX_SendCommand(p_sys->scheduler_handle, OMX_CommandStateSet, OMX_StateIdle, NULL);
    CHECK_ERROR(omx_error, "OMX_CommandStateSet OMX_StateIdle failed (%x: %s)",
            omx_error, ErrorToString(omx_error));

    omx_error = OMX_SendCommand(p_sys->renderer_handle, OMX_CommandStateSet, OMX_StateIdle, NULL);
    CHECK_ERROR(omx_error, "OMX_CommandStateSet OMX_StateIdle failed (%x: %s)",
            omx_error, ErrorToString(omx_error));

    omx_error = OMX_SendCommand(p_sys->clock_handle, OMX_CommandStateSet, OMX_StateIdle, NULL);
    CHECK_ERROR(omx_error, "OMX_CommandStateSet OMX_StateIdle failed (%x: %s)",
            omx_error, ErrorToString(omx_error));


    /* Allocate buffers */
    p_sys->port.pp_buffers = malloc(p_sys->port.definition.nBufferCountActual *
                                    sizeof(OMX_BUFFERHEADERTYPE*));
    p_sys->port.i_buffers = p_sys->port.definition.nBufferCountActual;

    unsigned int i;
    for (i = 0; i < p_sys->port.i_buffers; i++) {
        omx_error = OMX_AllocateBuffer(p_sys->port.omx_handle, &p_sys->port.pp_buffers[i],
                                       p_sys->port.i_port_index, 0,
                                       p_sys->port.definition.nBufferSize);
        if (omx_error != OMX_ErrorNone)
            break;
        OMX_FIFO_PUT(&p_sys->port.fifo, p_sys->port.pp_buffers[i]);
    }
    if (omx_error != OMX_ErrorNone) {
        p_sys->port.i_buffers = i;
        for (i = 0; i < p_sys->port.i_buffers; i++)
            OMX_FreeBuffer(p_sys->port.omx_handle, p_sys->port.i_port_index, p_sys->port.pp_buffers[i]);
        msg_Err(vd, "OMX_AllocateBuffer failed (%x: %s)",
                omx_error, ErrorToString(omx_error));
        goto error;
    }

    /* Wait until everything is in idle */
    omx_error = WaitForSpecificOmxEvent(&p_sys->image_fx_eq, OMX_EventCmdComplete, 0, 0, 0);
    CHECK_ERROR(omx_error, "Wait for OMX_EventCmdComplete OMX_StateIdle failed (%x: %s)",
            omx_error, ErrorToString(omx_error));

    omx_error = WaitForSpecificOmxEvent(&p_sys->scheduler_eq, OMX_EventCmdComplete, 0, 0, 0);
    CHECK_ERROR(omx_error, "Wait for OMX_EventCmdComplete OMX_StateIdle failed (%x: %s)",
            omx_error, ErrorToString(omx_error));

    omx_error = WaitForSpecificOmxEvent(&p_sys->renderer_eq, OMX_EventCmdComplete, 0, 0, 0);
    CHECK_ERROR(omx_error, "Wait for OMX_EventCmdComplete OMX_StateIdle failed (%x: %s)",
            omx_error, ErrorToString(omx_error));

    omx_error = WaitForSpecificOmxEvent(&p_sys->clock_eq, OMX_EventCmdComplete, 0, 0, 0);
    CHECK_ERROR(omx_error, "Wait for OMX_EventCmdComplete OMX_StateIdle failed (%x: %s)",
            omx_error, ErrorToString(omx_error));

    omx_error = SetDeinterlaceMode(p_sys->image_fx_handle, p_sys->image_fx_port_out, p_sys->deinterlace_enabled);
    CHECK_ERROR(omx_error, "SetDeinterlaceMode to false failed (%x: %s)",
            omx_error, ErrorToString(omx_error));

    /* Transition everything to executing */
    transition_to_state(&p_sys->image_fx_eq, p_sys->image_fx_handle, OMX_StateExecuting);
    CHECK_ERROR(omx_error, "transition_to_state image_fx to Executing failed (%x: %s)",
            omx_error, ErrorToString(omx_error));

    transition_to_state(&p_sys->clock_eq, p_sys->clock_handle, OMX_StateExecuting);
    CHECK_ERROR(omx_error, "transition_to_state clock to Executing failed (%x: %s)",
            omx_error, ErrorToString(omx_error));

    transition_to_state(&p_sys->scheduler_eq, p_sys->scheduler_handle, OMX_StateExecuting);
    CHECK_ERROR(omx_error, "transition_to_state scheduler to Executing failed (%x: %s)",
            omx_error, ErrorToString(omx_error));

    transition_to_state(&p_sys->renderer_eq, p_sys->renderer_handle, OMX_StateExecuting);
    CHECK_ERROR(omx_error, "transition_to_state renderer to Executing failed (%x: %s)",
            omx_error, ErrorToString(omx_error));

    /* UpdateDisplaySize */
    OMX_CONFIG_DISPLAYREGIONTYPE config_display;
    OMX_INIT_STRUCTURE(config_display);
    config_display.nPortIndex = p_sys->renderer_port_in;

    config_display.set = OMX_DISPLAY_SET_SRC_RECT;
    config_display.src_rect.width = vd->cfg->display.width;
    config_display.src_rect.height = vd->cfg->display.height;
    OMX_SetConfig(p_sys->renderer_handle, OMX_IndexConfigDisplayRegion, &config_display);
    config_display.set = OMX_DISPLAY_SET_FULLSCREEN;
    config_display.fullscreen = OMX_TRUE;
    OMX_SetConfig(p_sys->renderer_handle, OMX_IndexConfigDisplayRegion, &config_display);

    omx_error = UpdateDisplaySize(vd, vd->cfg);
    CHECK_ERROR(omx_error, "Could not update display size (%x: %s)",
            omx_error, ErrorToString(omx_error));

    /* Setup chroma */
    video_format_t fmt = vd->fmt;

    fmt.i_chroma = VLC_CODEC_I420;
    video_format_FixRgb(&fmt);

    /* Setup vout_display */
    vd->fmt     = fmt;
    vd->pool    = Pool;
    vd->display = Display;
    vd->control = Control;
    vd->prepare = NULL;
    vd->manage  = NULL;
#ifdef RPI_OMX
    vd->info.subpicture_chromas = rpi_subpicture_chromas;
#endif

    /* Create the associated picture */
    pictures = calloc(p_sys->port.i_buffers, sizeof(*pictures));
    if (!pictures)
        goto error;
    for (unsigned int i = 0; i < p_sys->port.i_buffers; i++) {
        picture_sys_t *picsys = malloc(sizeof(*picsys));
        if (unlikely(picsys == NULL))
            goto error;
        picsys->sys = p_sys;

        picture_resource_t resource = { .p_sys = picsys };

        picture_t *picture = picture_NewFromResource(&fmt, &resource);
        if (unlikely(picture == NULL))
        {
            free(picsys);
            goto error;
        }
        pictures[i] = picture;
    }

    /* Wrap it into a picture pool */
    picture_pool_configuration_t pool_cfg;
    memset(&pool_cfg, 0, sizeof(pool_cfg));
    pool_cfg.picture_count = p_sys->port.i_buffers;
    pool_cfg.picture       = pictures;
    pool_cfg.lock          = LockSurface;
    pool_cfg.unlock        = UnlockSurface;

    p_sys->pool = picture_pool_NewExtended(&pool_cfg);
    if (!p_sys->pool) {
        for (unsigned int i = 0; i < p_sys->port.i_buffers; i++)
            picture_Release(pictures[i]);
        goto error;
    }

    /* Fix initial state */
    vout_display_SendEventFullscreen(vd, true);

    print_omx_debug_info(p_this);

    free(pictures);
    return VLC_SUCCESS;

error:
    free(pictures);
    Close(p_this);
    return VLC_EGENERIC;
}

static void Close(vlc_object_t *p_this)
{
    vout_display_t *vd = (vout_display_t *)p_this;
    vout_display_sys_t *p_sys = vd->sys;
    vout_display_t *p_dec = vd;
    OMX_ERRORTYPE omx_error;
    OMX_STATETYPE state;
    OMX_TIME_CONFIG_CLOCKSTATETYPE clock_state;

    OMX_INIT_STRUCTURE(clock_state);
    OMX_GetConfig(p_sys->clock_handle, OMX_IndexConfigTimeClockState, &clock_state);
    clock_state.eState = OMX_TIME_ClockStateStopped;
    OMX_SetConfig(p_sys->clock_handle, OMX_IndexConfigTimeClockState, &clock_state);

    pf_setup_tunnel(p_sys->image_fx_handle, p_sys->image_fx_port_out, NULL, 0);
    pf_setup_tunnel(p_sys->scheduler_handle, p_sys->scheduler_port_video_in, NULL, 0);

    pf_setup_tunnel(p_sys->clock_handle, p_sys->clock_port_out, NULL, 0);
    pf_setup_tunnel(p_sys->scheduler_handle, p_sys->scheduler_port_clock_in, NULL, 0);

    pf_setup_tunnel(p_sys->scheduler_handle, p_sys->scheduler_port_video_out, NULL, 0);
    pf_setup_tunnel(p_sys->renderer_handle, p_sys->renderer_port_in, NULL, 0);

    omx_error = OMX_SendCommand(p_sys->image_fx_handle, OMX_CommandStateSet, OMX_StateIdle, NULL);
    CHECK_ERROR(omx_error, "OMX_CommandStateSet OMX_StateIdle failed (%x: %s)",
            omx_error, ErrorToString(omx_error));

    omx_error = OMX_SendCommand(p_sys->scheduler_handle, OMX_CommandStateSet, OMX_StateIdle, NULL);
    CHECK_ERROR(omx_error, "OMX_CommandStateSet OMX_StateIdle failed (%x: %s)",
            omx_error, ErrorToString(omx_error));

    omx_error = OMX_SendCommand(p_sys->renderer_handle, OMX_CommandStateSet, OMX_StateIdle, NULL);
    CHECK_ERROR(omx_error, "OMX_CommandStateSet OMX_StateIdle failed (%x: %s)",
            omx_error, ErrorToString(omx_error));

    omx_error = OMX_SendCommand(p_sys->clock_handle, OMX_CommandStateSet, OMX_StateIdle, NULL);
    CHECK_ERROR(omx_error, "OMX_CommandStateSet OMX_StateIdle failed (%x: %s)",
            omx_error, ErrorToString(omx_error));


    omx_error = WaitForSpecificOmxEvent(&p_sys->image_fx_eq, OMX_EventCmdComplete, 0, 0, 0);
    CHECK_ERROR(omx_error, "Wait for OMX_EventCmdComplete OMX_StateIdle failed (%x: %s)",
            omx_error, ErrorToString(omx_error));

    omx_error = WaitForSpecificOmxEvent(&p_sys->scheduler_eq, OMX_EventCmdComplete, 0, 0, 0);
    CHECK_ERROR(omx_error, "Wait for OMX_EventCmdComplete OMX_StateIdle failed (%x: %s)",
            omx_error, ErrorToString(omx_error));

    omx_error = WaitForSpecificOmxEvent(&p_sys->renderer_eq, OMX_EventCmdComplete, 0, 0, 0);
    CHECK_ERROR(omx_error, "Wait for OMX_EventCmdComplete OMX_StateIdle failed (%x: %s)",
            omx_error, ErrorToString(omx_error));

    omx_error = WaitForSpecificOmxEvent(&p_sys->clock_eq, OMX_EventCmdComplete, 0, 0, 0);
    CHECK_ERROR(omx_error, "Wait for OMX_EventCmdComplete OMX_StateIdle failed (%x: %s)",
            omx_error, ErrorToString(omx_error));


    omx_error = OMX_SendCommand(p_sys->image_fx_handle, OMX_CommandStateSet, OMX_StateLoaded, NULL);
    CHECK_ERROR(omx_error, "OMX_CommandStateSet OMX_StateLoaded failed (%x: %s)",
            omx_error, ErrorToString(omx_error));

    omx_error = OMX_SendCommand(p_sys->scheduler_handle, OMX_CommandStateSet, OMX_StateLoaded, NULL);
    CHECK_ERROR(omx_error, "OMX_CommandStateSet OMX_StateLoaded failed (%x: %s)",
            omx_error, ErrorToString(omx_error));

    omx_error = OMX_SendCommand(p_sys->renderer_handle, OMX_CommandStateSet, OMX_StateLoaded, NULL);
    CHECK_ERROR(omx_error, "OMX_CommandStateSet OMX_StateLoaded failed (%x: %s)",
            omx_error, ErrorToString(omx_error));

    omx_error = OMX_SendCommand(p_sys->clock_handle, OMX_CommandStateSet, OMX_StateLoaded, NULL);
    CHECK_ERROR(omx_error, "OMX_CommandStateSet OMX_StateLoaded failed (%x: %s)",
            omx_error, ErrorToString(omx_error));

    if (p_sys->pool)
        picture_pool_Delete(p_sys->pool);

    for (unsigned int i = 0; i < p_sys->port.i_buffers; i++) {
        OMX_FreeBuffer(p_sys->port.omx_handle, p_sys->port.i_port_index, p_sys->port.pp_buffers[i]);
    }
    free(p_sys->port.pp_buffers);

    omx_error = WaitForSpecificOmxEvent(&p_sys->image_fx_eq, OMX_EventCmdComplete, 0, 0, 0);
    CHECK_ERROR(omx_error, "Wait for OMX_EventCmdComplete OMX_StateIdle failed (%x: %s)",
            omx_error, ErrorToString(omx_error));

    omx_error = WaitForSpecificOmxEvent(&p_sys->scheduler_eq, OMX_EventCmdComplete, 0, 0, 0);
    CHECK_ERROR(omx_error, "Wait for OMX_EventCmdComplete OMX_StateIdle failed (%x: %s)",
            omx_error, ErrorToString(omx_error));

    omx_error = WaitForSpecificOmxEvent(&p_sys->renderer_eq, OMX_EventCmdComplete, 0, 0, 0);
    CHECK_ERROR(omx_error, "Wait for OMX_EventCmdComplete OMX_StateIdle failed (%x: %s)",
            omx_error, ErrorToString(omx_error));

    omx_error = WaitForSpecificOmxEvent(&p_sys->clock_eq, OMX_EventCmdComplete, 0, 0, 0);
    CHECK_ERROR(omx_error, "Wait for OMX_EventCmdComplete OMX_StateIdle failed (%x: %s)",
            omx_error, ErrorToString(omx_error));

    pf_free_handle(p_sys->renderer_handle);
    pf_free_handle(p_sys->clock_handle);
    pf_free_handle(p_sys->scheduler_handle);
    pf_free_handle(p_sys->image_fx_handle);

    DeinitOmxEventQueue(&p_sys->image_fx_eq);
    DeinitOmxEventQueue(&p_sys->scheduler_eq);
    DeinitOmxEventQueue(&p_sys->renderer_eq);
    DeinitOmxEventQueue(&p_sys->clock_eq);

    vlc_mutex_destroy(&p_sys->port.fifo.lock);
    vlc_cond_destroy(&p_sys->port.fifo.wait);

    free(p_sys);
    DeinitOmxCore();
    return;
error:
    msg_Err(vd, "Ouch, Teardown went seriously mad");
}

static picture_pool_t *Pool(vout_display_t *vd, unsigned count)
{
    VLC_UNUSED(count);

    return vd->sys->pool;
}

static int LockSurface(picture_t *picture)
{
    picture_sys_t *picsys = picture->p_sys;
    vout_display_sys_t *p_sys = picsys->sys;
    OMX_BUFFERHEADERTYPE *p_buffer;

    OMX_FIFO_GET_TIMEOUT(&p_sys->port.fifo, p_buffer, 2000);
    if (p_buffer == NULL)
        return VLC_EGENERIC;

    for (int i = 0; i < 3; i++) {
        picture->p[i].p_pixels = p_buffer->pBuffer;
        picture->p[i].i_pitch = p_sys->port.definition.format.video.nStride;
        picture->p[i].i_lines = p_sys->port.definition.format.video.nSliceHeight;
        if (i > 0) {
            picture->p[i].p_pixels = picture->p[i-1].p_pixels + picture->p[i-1].i_pitch*picture->p[i-1].i_lines;
            picture->p[i].i_pitch /= 2;
            picture->p[i].i_lines /= 2;
        }
    }
    p_buffer->nOffset = 0;
    p_buffer->nFlags = 0;
    p_buffer->nTimeStamp = ToOmxTicks(0);
    p_buffer->nFilledLen = 0;
    picsys->buf = p_buffer;

    return VLC_SUCCESS;
}

static void UnlockSurface(picture_t *picture)
{
    picture_sys_t *picsys = picture->p_sys;
    vout_display_sys_t *p_sys = picsys->sys;
    OMX_BUFFERHEADERTYPE *p_buffer = picsys->buf;
    OMX_FIFO_PUT(&p_sys->port.fifo, p_buffer);
}

static void Display(vout_display_t *vd, picture_t *picture, subpicture_t *subpicture)
{
    VLC_UNUSED(vd);
    VLC_UNUSED(subpicture);
    picture_sys_t *picsys = picture->p_sys;
    vout_display_sys_t *p_sys = picsys->sys;
    OMX_BUFFERHEADERTYPE *p_buffer = picsys->buf;
    OMX_TIME_CONFIG_CLOCKSTATETYPE clock_state;
    OMX_TIME_CONFIG_TIMESTAMPTYPE clock_ts;
    OMX_ERRORTYPE omx_error;

    mtime_t now = mdate();
    p_buffer->nTimeStamp = ToOmxTicks(now);

    if (!p_sys->clock_initialized) {
        now -= SCHEDULER_BUFFERED_PICTURES * p_sys->musecs_per_frame;
        OMX_INIT_STRUCTURE(clock_state);
        omx_error = OMX_GetConfig(p_sys->clock_handle, OMX_IndexConfigTimeClockState, &clock_state);
        if (omx_error != OMX_ErrorNone) {
            msg_Err(vd, "OMX_IndexConfigTimeClockState failed (%x: %s)", omx_error,
                    ErrorToString(omx_error));
        }

        OMX_INIT_STRUCTURE(clock_ts);
        clock_ts.nPortIndex = p_sys->clock_port_out;
        if (clock_state.eState == OMX_TIME_ClockStateWaitingForStartTime) {
            clock_ts.nTimestamp = ToOmxTicks(now);
            omx_error = OMX_SetConfig(p_sys->clock_handle, OMX_IndexConfigTimeClientStartTime, &clock_ts);
            if (omx_error != OMX_ErrorNone) {
                msg_Err(vd, "OMX_IndexConfigTimeClientStartTime failed (%x: %s)",
                        omx_error, ErrorToString(omx_error));
            }
            p_sys->clock_initialized = true;
        }

        clock_ts.nTimestamp = ToOmxTicks(now);
        omx_error = OMX_SetConfig(p_sys->clock_handle, OMX_IndexConfigTimeCurrentVideoReference, &clock_ts);
        if (omx_error != OMX_ErrorNone) {
            msg_Err(vd, "OMX_IndexConfigTimeCurrentVideoReference failed (%x: %s)",
                    omx_error, ErrorToString(omx_error));
        }
    }

    if (picture->date != p_sys->cur_ts) {
        p_buffer->nFilledLen = 3*p_sys->port.definition.format.video.nStride*p_sys->port.definition.format.video.nSliceHeight/2;
        p_buffer->nFlags = OMX_BUFFERFLAG_ENDOFFRAME;
        p_buffer->pAppPrivate = picture;
        OMX_EmptyThisBuffer(p_sys->port.omx_handle, p_buffer);
        p_sys->cur_ts = picture->date;
    } else {
        picture_Release(picture);
    }

    if (subpicture)
        subpicture_Delete(subpicture);
}

static int Control(vout_display_t *vd, int query, va_list args)
{
    VLC_UNUSED(args);

    switch (query) {
    case VOUT_DISPLAY_HIDE_MOUSE:
        return VLC_SUCCESS;

    case VOUT_DISPLAY_CHANGE_SOURCE_ASPECT:
        return VLC_SUCCESS;

    case VOUT_DISPLAY_CHANGE_DISPLAY_SIZE: {
        const vout_display_cfg_t *cfg = va_arg(args, const vout_display_cfg_t *);
        return UpdateDisplaySize(vd, cfg) == OMX_ErrorNone ? VLC_SUCCESS : VLC_EGENERIC;
    }

    case VOUT_DISPLAY_CHANGE_FULLSCREEN:
    case VOUT_DISPLAY_CHANGE_WINDOW_STATE:
    case VOUT_DISPLAY_CHANGE_DISPLAY_FILLED:
    case VOUT_DISPLAY_CHANGE_ZOOM:
    case VOUT_DISPLAY_CHANGE_SOURCE_CROP:
    case VOUT_DISPLAY_GET_OPENGL:
        return VLC_EGENERIC;

    default: {
        msg_Err(vd, "Unknown request in omxil vout display");
        return VLC_EGENERIC;
    }
    }
}

