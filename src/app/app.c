/******************************************************************************
 * File Name: app.c
 *
 * Description: This is source code for the PMG1B1 MCU USB-PD Charger Code Example.
 *              This file implements functions for handling of PDStack event
 *              callbacks and power saving.
 *
 * Related Document: See README.md
 *
 *******************************************************************************
 * Copyright 2023, Cypress Semiconductor Corporation (an Infineon company) or
 * an affiliate of Cypress Semiconductor Corporation.  All rights reserved.
 *
 * This software, including source code, documentation and related
 * materials ("Software") is owned by Cypress Semiconductor Corporation
 * or one of its affiliates ("Cypress") and is protected by and subject to
 * worldwide patent protection (United States and foreign),
 * United States copyright laws and international treaty provisions.
 * Therefore, you may use this Software only as provided in the license
 * agreement accompanying the software package from which you
 * obtained this Software ("EULA").
 * If no EULA applies, Cypress hereby grants you a personal, non-exclusive,
 * non-transferable license to copy, modify, and compile the Software
 * source code solely for use in connection with Cypress's
 * integrated circuit products.  Any reproduction, modification, translation,
 * compilation, or representation of this Software except as specified
 * above is prohibited without the express written permission of Cypress.
 *
 * Disclaimer: THIS SOFTWARE IS PROVIDED AS-IS, WITH NO WARRANTY OF ANY KIND,
 * EXPRESS OR IMPLIED, INCLUDING, BUT NOT LIMITED TO, NONINFRINGEMENT, IMPLIED
 * WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE. Cypress
 * reserves the right to make changes to the Software without notice. Cypress
 * does not assume any liability arising out of the application or use of the
 * Software or any product or circuit described in the Software. Cypress does
 * not authorize its products for use in any products where a malfunction or
 * failure of the Cypress product may reasonably be expected to result in
 * significant property damage, injury or death ("High Risk Product"). By
 * including Cypress's product in a High Risk Product, the manufacturer
 * of such system or application assumes all risk of such use and in doing
 * so agrees to indemnify Cypress against all liability.
 *******************************************************************************/

#include "cybsp.h"
#include "cy_pdstack_common.h"
#include "cy_pdstack_dpm.h"
#include "config.h"
#include "psink.h"
#include "swap.h"
#include "vdm.h"
#include "app.h"
#include "cy_pdutils_sw_timer.h"
#include "cy_pdstack_timer_id.h"
#include "cy_gpio.h"

#if BATTERY_CHARGING_ENABLE
#include <battery_charging.h>
#endif /* BATTERY_CHARGING_ENABLE */

/* Variable to hold Application status for each USB-C port. */
app_status_t app_status[NO_OF_TYPEC_PORTS];

/* Global variable to keep track of solution level functions */
app_sln_handler_t *solution_fn_handler;

/* Variable to hold Application status for each USB-C port. */
app_status_t app_status[NO_OF_TYPEC_PORTS];

#if (!CY_PD_CBL_DISC_DISABLE)
static void app_cbl_dsc_timer_cb (cy_timer_id_t id, void *callbackContext);
static void app_cbl_dsc_callback (cy_stc_pdstack_context_t *ptrPdStackContext, cy_en_pdstack_resp_status_t resp,
                                const cy_stc_pdstack_pd_packet_t *pkt_ptr)
{
    /* Keep repeating the DPM command until we succeed. */
    if (resp == CY_PDSTACK_SEQ_ABORTED)
    {
        Cy_PdUtils_SwTimer_Start (ptrPdStackContext->ptrTimerContext, ptrPdStackContext,
                GET_APP_TIMER_ID(ptrPdStackContext, APP_CBL_DISC_TRIGGER_TIMER),
                APP_CBL_DISC_TIMER_PERIOD, app_cbl_dsc_timer_cb);
    }
}


static void app_cbl_dsc_timer_cb (cy_timer_id_t id, void *callbackContext)
{
    cy_stc_pdstack_context_t *pdstack_context = callbackContext;

    if (Cy_PdStack_Dpm_SendPdCommand(pdstack_context, CY_PDSTACK_DPM_CMD_INITIATE_CBL_DISCOVERY, NULL, false, app_cbl_dsc_callback) != CY_PDSTACK_STAT_SUCCESS )
    {
        /* Start timer which will send initiate the DPM command after a delay. */
        app_cbl_dsc_callback(pdstack_context, CY_PDSTACK_SEQ_ABORTED, 0);
    }
}
#endif /* (!CY_PD_CBL_DISC_DISABLE) */

uint8_t app_task(cy_stc_pdstack_context_t *ptrPdStackContext)
{
    fault_handler_task (ptrPdStackContext);

#if BATTERY_CHARGING_ENABLE
        (void)bc_fsm(ptrPdStackContext);
#endif /* BATTERY_CHARGING_ENABLE */
    
    return true;
}

#if CY_PD_REV3_ENABLE

#if CHUNKING_NOT_SUPPORTED

void app_not_supported_sent_cb(cy_stc_pdstack_context_t *ptrPdStackContext, cy_en_pdstack_resp_status_t resp,
                                const cy_stc_pdstack_pd_packet_t *pkt_ptr)
{
    if (resp == CY_PDSTACK_CMD_SENT)
    {
        if (ptrPdStackContext->peFsmState == CY_PDSTACK_PE_FSM_READY)
        {
            Cy_PdStack_Dpm_SetPeEvt(ptrPdStackContext, CY_PD_PE_EVT_TIMEOUT);
        }
    }
}

static void app_send_not_supported_cb (cy_timer_id_t id, void *callbackContext)
{
    cy_stc_pdstack_context_t *ptrPdStackContext = callbackContext;

    Cy_PdStack_Dpm_SendPdCommand(ptrPdStackContext, CY_PDSTACK_DPM_CMD_SEND_NOT_SUPPORTED,
            NULL, true, app_not_supported_sent_cb);
}

#else /* !CHUNKING_NOT_SUPPORTED */

static cy_en_pdstack_ams_type_t gl_saved_ams_type[NO_OF_TYPEC_PORTS];

void extd_msg_cb(cy_stc_pdstack_context_t *ptrPdStackContext, cy_en_pdstack_resp_status_t resp,
                                                    const cy_stc_pdstack_pd_packet_t *pkt_ptr)
{
    (void)pkt_ptr;

    if(resp == CY_PDSTACK_CMD_SENT)
    {
        /* Save the AMS type once the command has been sent. */
        gl_saved_ams_type[ptrPdStackContext->port] = ptrPdStackContext->dpmStat.nonIntrResponse;
    }

    if(resp == CY_PDSTACK_RES_RCVD)
    {
        /* Restore the saved AMS type once response has been received. */
        ptrPdStackContext->dpmStat.nonIntrResponse = gl_saved_ams_type[ptrPdStackContext->port];
    }
}

/* Global variable used as dummy data buffer to send Chunk Request messages. */
static uint32_t gl_extd_dummy_data;

#endif /* CHUNKING_NOT_SUPPORTED */

bool app_extd_msg_handler(cy_stc_pdstack_context_t *ptrPdStackContext, cy_stc_pd_packet_extd_t *pd_pkt_p)
{
#if CHUNKING_NOT_SUPPORTED
    /*
     * If we receive any message with more than one chunk, start ChunkingNotSupportedTimer and send NOT_SUPPORTED
     * message on expiry.
     */
    if (
            (pd_pkt_p->hdr.hdr.chunked == true) &&
            (
             (pd_pkt_p->hdr.hdr.chunkNum != 0) ||
             (pd_pkt_p->hdr.hdr.dataSize > ((pd_pkt_p->hdr.hdr.chunkNum + 1) * MAX_EXTD_MSG_LEGACY_LEN))
            )
       )
    {
        Cy_PdUtils_SwTimer_Start(ptrPdStackContext->timerContext, ptrPdStackContext,
                          GET_APP_TIMER_ID(ptrPdStackContext, APP_CHUNKED_MSG_RESP_TIMER), 45, app_send_not_supported_cb);

        /* Stop the PD_GENERIC_TIMER to prevent premature return to ready state. */
        Cy_PdUtils_SwTimer_Stop(ptrPdStackContext->timerContext, CY_PDSTACK_GET_PD_TIMER_ID(ptrPdStackContext, PD_GENERIC_TIMER));
    }
    else
    {
        /*
         * Don't send any response to response messages. Handling here instead of in the stack so that
         * these messages can be used for PD authentication implementation.
         */
        if ((pd_pkt_p->msg != CY_PDSTACK_EXTD_MSG_SECURITY_RESP) && (pd_pkt_p->msg != CY_PDSTACK_EXTD_MSG_FW_UPDATE_RESP))
        {
            /* Send Not supported message */
            Cy_PdStack_Dpm_SendPdCommand(ptrPdStackContext,
                                         CY_PDSTACK_DPM_CMD_SEND_NOT_SUPPORTED, NULL, true, NULL);
        }
    }

#else /* CHUNKING_NOT_SUPPORTED */

    /* If this is a chunked message which is not complete, send another chunk request. */
    if ((pd_pkt_p->hdr.hdr.chunked == true) && (pd_pkt_p->hdr.hdr.dataSize >
               ((pd_pkt_p->hdr.hdr.chunkNum + 1) * CY_PD_MAX_EXTD_MSG_LEGACY_LEN)))
    {
        cy_stc_pdstack_dpm_pd_cmd_buf_t extd_dpm_buf;

        extd_dpm_buf.cmdSop                = (cy_en_pd_sop_t)pd_pkt_p->sop;
        extd_dpm_buf.extdType              = (cy_en_pdstack_extd_msg_t)pd_pkt_p->msg;
        extd_dpm_buf.extdHdr.val           = 0;
        extd_dpm_buf.extdHdr.extd.chunked  = true;
        extd_dpm_buf.extdHdr.extd.request  = true;
        extd_dpm_buf.extdHdr.extd.chunkNum = pd_pkt_p->hdr.hdr.chunkNum + 1;
        extd_dpm_buf.datPtr                = (uint8_t*)&gl_extd_dummy_data;
        extd_dpm_buf.timeout                = ptrPdStackContext->senderRspTimeout;

        /* Send next chunk request */
        Cy_PdStack_Dpm_SendPdCommand(ptrPdStackContext,CY_PDSTACK_DPM_CMD_SEND_EXTENDED,
                                     &extd_dpm_buf, true, extd_msg_cb);
     }
    else
    {
#if CCG_SLN_EXTN_MSG_HANDLER_ENABLE
        /* If macro is enabled - allow handling the requests from solution space. */
        return false;
#else
        /*
         * Don't send any response to response messages. Handling here instead of in the stack so that
         * these messages can be used for PD authentication implementation.
         */
        if ((pd_pkt_p->msg != CY_PDSTACK_EXTD_MSG_SECURITY_RESP) && (pd_pkt_p->msg != CY_PDSTACK_EXTD_MSG_FW_UPDATE_RESP))
        {
            /* Send Not supported message */
            (void)Cy_PdStack_Dpm_SendPdCommand(ptrPdStackContext,
                                              CY_PDSTACK_DPM_CMD_SEND_NOT_SUPPORTED, NULL, true, NULL);
        }
#endif /* CCG_HANDLE_EXT_MSG_IN_SOL */
    }
#endif /* CHUNKING_NOT_SUPPORTED */

    return true;
}
#endif /* CY_PD_REV3_ENABLE */

static uint8_t gl_app_previous_polarity[NO_OF_TYPEC_PORTS];

void app_event_handler(cy_stc_pdstack_context_t *ptrPdStackContext,
               cy_en_pdstack_app_evt_t evt, const void* dat)
{
    const cy_en_pdstack_app_req_status_t* result;
    bool  skip_soln_cb = false;
    uint8_t port = ptrPdStackContext->port;

    if (port >= NO_OF_TYPEC_PORTS)
    {
        return;
    }

    switch(evt)
    {
        case APP_EVT_TYPEC_STARTED:
            break;

        case APP_EVT_TYPEC_ATTACH:
            /* Clear all fault counters if we have seen a change in polarity from previous connection. */
            if (ptrPdStackContext->dpmConfig.polarity != gl_app_previous_polarity[port])
            {
                fault_handler_clear_counts (port);
            }
            gl_app_previous_polarity[port] = ptrPdStackContext->dpmConfig.polarity ;
#if (PMG1B1_USB_CHARGER)
            /* Used as disconnect level in TypeC only mode. */
            ptrPdStackContext->dpmStat.contract.minVolt = CY_PD_VSAFE_5V;
#endif
            break;

        case APP_EVT_CONNECT:
            app_status[port].cbl_disc_id_finished = false;
            app_status[port].disc_cbl_pending = false;
            break;

        case APP_EVT_HARD_RESET_COMPLETE:
        case APP_EVT_HARD_RESET_SENT:
        case APP_EVT_PE_DISABLED:
        case APP_EVT_HARD_RESET_RCVD:
        case APP_EVT_VBUS_PORT_DISABLE:
        case APP_EVT_DISCONNECT:
        case APP_EVT_TYPE_C_ERROR_RECOVERY:
            app_status[port].cbl_disc_id_finished = false;
            app_status[port].disc_cbl_pending = false;

#if (VBUS_OVP_ENABLE || VBUS_UVP_ENABLE)
            if(evt == APP_EVT_TYPE_C_ERROR_RECOVERY)
            {
                /* Clear port-in-fault flag if all fault counts are within limits. */
                if (!app_port_fault_count_exceeded(ptrPdStackContext))
                {
                    if ((app_get_status(port)->fault_status & APP_PORT_DISABLE_IN_PROGRESS) == 0)
                    {
                        ptrPdStackContext->dpmStat.faultActive = false;
                    }
                }
            }
#endif /* (VBUS_OVP_ENABLE || VBUS_UVP_ENABLE) */
            break;

        case APP_EVT_EMCA_NOT_DETECTED:
        case APP_EVT_EMCA_DETECTED:
            app_status[port].cbl_disc_id_finished = true;
            app_status[port].disc_cbl_pending = false;
            break;

        case APP_EVT_DR_SWAP_COMPLETE:
            result = (const cy_en_pdstack_app_req_status_t*)dat;
            if(*result == CY_PDSTACK_REQ_ACCEPT )
            {
            }
            break;

        case APP_EVT_PD_CONTRACT_NEGOTIATION_COMPLETE:
            /* Set VDM version based on active PD revision. */
#if CY_PD_REV3_ENABLE
            if (ptrPdStackContext->dpmConfig.specRevSopLive >= CY_PD_REV3)
            {
                app_status[port].vdm_version = CY_PD_STD_VDM_VERSION_REV3;
                app_status[port].vdm_minor_version = CY_PDSTACK_STD_VDM_MINOR_VER1;
            }
            else
#endif /* CY_PD_REV3_ENABLE */
            {
                app_status[port].vdm_version = CY_PD_STD_VDM_VERSION_REV2;
                app_status[port].vdm_minor_version = CY_PDSTACK_STD_VDM_MINOR_VER0;
            }
            break;

#if CY_PD_REV3_ENABLE
        case APP_EVT_HANDLE_EXTENDED_MSG:
            {
                if (!(app_extd_msg_handler(ptrPdStackContext, (cy_stc_pd_packet_extd_t *)dat)))
                {
                    skip_soln_cb  = false;
                }
                else
                {
                    skip_soln_cb  = true;
                }
            }
            break;

        case APP_EVT_ALERT_RECEIVED:
            break;
#endif /* CY_PD_REV3_ENABLE */

        case APP_EVT_TYPEC_ATTACH_WAIT:
            break;

        case APP_EVT_TYPEC_ATTACH_WAIT_TO_UNATTACHED:
            break;

        case APP_EVT_DATA_RESET_ACCEPTED:
            app_status[port].cbl_disc_id_finished = false;
            break;

        case APP_EVT_DATA_RESET_CPLT:
            break;

        case APP_EVT_PD_SINK_DEVICE_CONNECTED:
            break;

        case APP_EVT_PKT_RCVD:
#if CHUNKING_NOT_SUPPORTED
            /* Make sure we stop the timer that initiates NOT_SUPPORTED response sending. */
            timer_stop(port, APP_CHUNKED_MSG_RESP_TIMER);
#endif /* CHUNKING_NOT_SUPPORTED */
            break;

        case APP_EVT_CBL_RESET_SENT:
            break;

        default:
            /* Nothing to do. */
            break;
    }

    /* Pass the event notification to the fault handler module. */
    if (fault_event_handler (ptrPdStackContext, evt, dat))
    {
        skip_soln_cb = true;
    }

    if (!skip_soln_cb)
    {
        /* Send notifications to the solution */
       sln_pd_event_handler(ptrPdStackContext, evt, dat) ;
    }
}

/* Callback used to receive fault notification from the HAL and to pass it on to the event handler. */
static bool app_cc_sbu_fault_handler(void *context, bool fault_type)
{
    cy_stc_usbpd_context_t *usbpdContext = (cy_stc_usbpd_context_t *)context;
    cy_stc_pdstack_context_t *pdStackContext = (cy_stc_pdstack_context_t *)usbpdContext->pdStackContext;
    app_event_handler(pdStackContext, (fault_type ? APP_EVT_SBU_OVP : APP_EVT_CC_OVP), NULL);
    return true;
}


app_resp_t* app_get_resp_buf(uint8_t port)
{
    return &app_status[port].appResp;
}

app_status_t* app_get_status(uint8_t port)
{
    return &app_status[port];
}

bool app_timer_start(cy_stc_usbpd_context_t *context, void *callbackContext,
        cy_en_usbpd_timer_id_t id, uint16_t period, cy_timer_cbk_t cbk)
{
    cy_stc_pdutils_sw_timer_t *ptrTimerContext = ((cy_stc_pdstack_context_t *)context->pdStackContext)->ptrTimerContext;
    return Cy_PdUtils_SwTimer_Start(ptrTimerContext, callbackContext, (cy_timer_id_t)id, period, (cy_cb_timer_t)cbk);
}

void app_timer_stop(cy_stc_usbpd_context_t *context, cy_en_usbpd_timer_id_t id)
{
    cy_stc_pdutils_sw_timer_t *ptrTimerContext = ((cy_stc_pdstack_context_t *)context->pdStackContext)->ptrTimerContext;
    Cy_PdUtils_SwTimer_Stop(ptrTimerContext, (cy_timer_id_t)id);
}

bool app_timer_is_running(cy_stc_usbpd_context_t *context, cy_en_usbpd_timer_id_t id)
{
    cy_stc_pdutils_sw_timer_t *ptrTimerContext = ((cy_stc_pdstack_context_t *)context->pdStackContext)->ptrTimerContext;
    return Cy_PdUtils_SwTimer_IsRunning(ptrTimerContext, (cy_timer_id_t)id);
}

uint16_t app_timer_get_multiplier(cy_stc_usbpd_context_t *context)
{
    cy_stc_pdutils_sw_timer_t *ptrTimerContext = ((cy_stc_pdstack_context_t *)context->pdStackContext)->ptrTimerContext;
    return Cy_PdUtils_SwTimer_GetMultiplier(ptrTimerContext);
}

void timer_init(cy_stc_pdstack_context_t *ptrPdStackContext)
{
    ptrPdStackContext->ptrUsbPdContext->timerStopcbk = app_timer_stop;
    ptrPdStackContext->ptrUsbPdContext->timerStartcbk = app_timer_start;
    ptrPdStackContext->ptrUsbPdContext->timerIsRunningcbk = app_timer_is_running;
    ptrPdStackContext->ptrUsbPdContext->timerGetMultipliercbk = app_timer_get_multiplier;

#if VBUS_SLOW_DISCHARGE_EN
    ptrPdStackContext->ptrUsbPdContext->vbusSlowDischargecbk = app_vbus_slow_discharge_cbk;
#endif /*VBUS_SLOW_DISCHARGE_EN */
}
void app_init(cy_stc_pdstack_context_t *ptrPdStackContext)
{
    timer_init(ptrPdStackContext);

    solution_init(ptrPdStackContext);

    /* SAR ADC initialization and enable */
    battery_measure_sar_adc_init(ptrPdStackContext);

    /* Initialize the VDM responses from the configuration table. */
    vdm_data_init(ptrPdStackContext);

#if BATTERY_CHARGING_ENABLE
    (void)bc_init(ptrPdStackContext);
#endif /* BATTERY_CHARGING_ENABLE */

    /* Update custom host config settings to the stack. */
    ptrPdStackContext->dpmStat.typecAccessorySuppDisabled = !(ptrPdStackContext->ptrPortCfg->accessoryEn);
    ptrPdStackContext->dpmStat.typecRpDetachDisabled = !(ptrPdStackContext->ptrPortCfg->rpDetachEn);

    if((ptrPdStackContext->ptrUsbPdContext->usbpdConfig->sbuOvpConfig != NULL) ||
            (ptrPdStackContext->ptrUsbPdContext->usbpdConfig->ccOvpConfig != NULL))
    {
        if((ptrPdStackContext->ptrUsbPdContext->usbpdConfig->sbuOvpConfig->enable != 0) ||
            (ptrPdStackContext->ptrUsbPdContext->usbpdConfig->ccOvpConfig->enable != 0))
        {
            /* Register a callback for notification of CC/SBU faults. */
            ptrPdStackContext->ptrUsbPdContext->ccSbuOvpCbk = app_cc_sbu_fault_handler;
        }
    }
}

#if SYS_DEEPSLEEP_ENABLE
/* Implements PMG1 deep sleep functionality for power saving. */
bool system_sleep(cy_stc_pdstack_context_t *ptrPdStackContext, cy_stc_pdstack_context_t *ptrPdStack1Context)
{
    uint32_t intr_state;
    bool dpm_slept = false;
    bool retval = false;
    bool bc_slept = true;
#if PMG1_PD_DUALPORT_ENABLE
    bool dpm_port1_slept = false;
#endif /* PMG1_PD_DUALPORT_ENABLE */

    /* Do one DPM sleep capability check before locking interrupts out. */
    if (
            (Cy_PdStack_Dpm_IsIdle (ptrPdStackContext, &dpm_slept) != CY_PDSTACK_STAT_SUCCESS) ||
            (!dpm_slept)
       )
    {
        return retval;
    }

    intr_state = Cy_SysLib_EnterCriticalSection();

#if BATTERY_CHARGING_ENABLE
    bc_slept = bc_sleep(ptrPdStackContext);
#endif /* BATTERY_CHARGING_ENABLE */

    if (bc_slept)
    {
        if (
                ((Cy_PdStack_Dpm_PrepareDeepSleep(ptrPdStackContext, &dpm_slept) == CY_PDSTACK_STAT_SUCCESS) &&
                (dpm_slept))
           )
        {
            Cy_PdUtils_SwTimer_EnterSleep(ptrPdStackContext->ptrTimerContext);

            Cy_USBPD_SetReference(ptrPdStackContext->ptrUsbPdContext, true);
#if PMG1_PD_DUALPORT_ENABLE
            Cy_USBPD_SetReference(ptrPdStack1Context->ptrUsbPdContext, true);
#endif /* PMG1_PD_DUALPORT_ENABLE */

            /* Device sleep entry. */
            Cy_SysPm_CpuEnterDeepSleep();

            Cy_USBPD_SetReference(ptrPdStackContext->ptrUsbPdContext, false);
            retval = true;
        }
    }

    Cy_SysLib_ExitCriticalSection(intr_state);

    /* Call dpm_wakeup() if dpm_sleep() had returned true. */
    if (dpm_slept)
    {
        Cy_PdStack_Dpm_Resume(ptrPdStackContext, &dpm_slept);
    }


#if BATTERY_CHARGING_ENABLE
    if (bc_slept)
    {
        (void)bc_wakeup();
    }
#endif /* BATTERY_CHARGING_ENABLE */

    (void)ptrPdStack1Context;
    return retval;
}
#endif /* SYS_DEEPSLEEP_ENABLE */

bool vconn_enable(cy_stc_pdstack_context_t *ptrPdStackContext, uint8_t channel)
{
#if (!PMG1_VCONN_DISABLE)
    /* Reset RX Protocol for cable */
    Cy_PdStack_Dpm_ProtResetRx(ptrPdStackContext, CY_PD_SOP_PRIME);
    Cy_PdStack_Dpm_ProtResetRx(ptrPdStackContext, CY_PD_SOP_DPRIME);

    if (Cy_USBPD_Vconn_Enable(ptrPdStackContext->ptrUsbPdContext, channel) != CY_USBPD_STAT_SUCCESS)
    {
        return false;
    }
    return true;
#else
    return false;
#endif /* (!PMG1_VCONN_DISABLE) */
}

void vconn_disable(cy_stc_pdstack_context_t *ptrPdStackContext, uint8_t channel)
{
#if (!PMG1_VCONN_DISABLE)
    Cy_USBPD_Vconn_Disable(ptrPdStackContext->ptrUsbPdContext, channel);
#endif /* (!PMG1_VCONN_DISABLE) */
}

bool vconn_is_present(cy_stc_pdstack_context_t *ptrPdStackContext)
{
#if (!PMG1_VCONN_DISABLE)
    return Cy_USBPD_Vconn_IsPresent(ptrPdStackContext->ptrUsbPdContext, ptrPdStackContext->dpmConfig.revPol);
#else
    return false;
#endif /* (!PMG1_VCONN_DISABLE) */
}

bool vbus_is_present(cy_stc_pdstack_context_t *ptrPdStackContext, uint16_t volt, int8_t per)
{
    uint8_t level;
    uint8_t retVal;

    /*
     * Re-run calibration every time to ensure that VDDD or the measurement
     * does not break.
     */
    Cy_USBPD_Adc_Calibrate(ptrPdStackContext->ptrUsbPdContext, APP_VBUS_POLL_ADC_ID);
    level =  Cy_USBPD_Adc_GetVbusLevel(ptrPdStackContext->ptrUsbPdContext,
                           APP_VBUS_POLL_ADC_ID,
                           volt, per);

    retVal = Cy_USBPD_Adc_CompSample(ptrPdStackContext->ptrUsbPdContext,
                     APP_VBUS_POLL_ADC_ID,
                     APP_VBUS_POLL_ADC_INPUT, level);

    return retVal;
}


void vbus_discharge_on(cy_stc_pdstack_context_t* context)
{
    Cy_USBPD_Vbus_DischargeOn(context->ptrUsbPdContext);
}

void vbus_discharge_off(cy_stc_pdstack_context_t* context)
{
    Cy_USBPD_Vbus_DischargeOff(context->ptrUsbPdContext);
}

uint16_t vbus_get_value(cy_stc_pdstack_context_t *ptrPdStackContext)
{
    uint16_t retVal;

    /* Measure the actual VBUS voltage. */
    retVal = Cy_USBPD_Adc_MeasureVbus(ptrPdStackContext->ptrUsbPdContext,
                          APP_VBUS_POLL_ADC_ID,
                          APP_VBUS_POLL_ADC_INPUT);

    return retVal;
}

void register_soln_function_handler(cy_stc_pdstack_context_t *ptrPdStackcontext, app_sln_handler_t *handler)
{
    (void)ptrPdStackcontext;

    /* Register the solution level handler functions */
    solution_fn_handler = handler;
}

bool app_is_typec_attached(void)
{
    bool attached = false;
    uint8_t i = 0;

    {
        if (solution_fn_handler->Get_PdStack_Context(i)->dpmConfig.attach == true)
        {
            attached = true;
        }
    }

    return attached;
}
/* End of File */
