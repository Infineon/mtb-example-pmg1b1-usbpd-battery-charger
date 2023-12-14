/******************************************************************************
 * File Name: fault_handlers.c
 *
 * Description: This header file implements fault handling functions.
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
#include "config.h"
#include "psink.h"
#include "pdo.h"
#include "swap.h"
#include "vdm.h"
#include "app.h"
#include "cy_pdutils_sw_timer.h"
#include "timer_id.h"
#include "cy_usbpd_vbus_ctrl.h"

enum
{
    FAULT_TYPE_VBUS_OVP = 0,    /* 0 */
    FAULT_TYPE_VBUS_UVP,        /* 1 */
    FAULT_TYPE_VBUS_OCP,        /* 2 */
    FAULT_TYPE_VBUS_SCP,        /* 3 */
    FAULT_TYPE_CC_OVP,          /* 4 */
    FAULT_TYPE_VCONN_OCP,       /* 5 */
    FAULT_TYPE_SBU_OVP,         /* 6 */
    FAULT_TYPE_OTP,             /* 7 */
    FAULT_TYPE_VBUS_RCP,        /* 8 */
    FAULT_TYPE_COUNT            /* 9 */
};

#if (VBUS_OVP_ENABLE || VBUS_UVP_ENABLE)
/* Variable defined in app.c */
extern app_status_t app_status[];

/* Shorthand for any faults enabled. */
#define FAULT_HANDLER_ENABLE                    (1u)

#endif /* (VBUS_OVP_ENABLE || VBUS_UVP_ENABLE) */

#if FAULT_HANDLER_ENABLE

/*
 * If fault retry count in configuration table is set to this value, then
 * faults are not counted. That is, infinite fault recovery is enabled.
 */
#define FAULT_COUNTER_SKIP_VALUE        (255u)

/* Number of retries defined by user for each fault type. */
static uint8_t gl_app_fault_retry_limit[NO_OF_TYPEC_PORTS][FAULT_TYPE_COUNT] =
{
    {0}
};

/* Number of times each fault condition has been detected during current connection. */
static volatile uint8_t gl_app_fault_count[NO_OF_TYPEC_PORTS][FAULT_TYPE_COUNT] =
{
    {0}
};

/* Check whether any fault count has exceeded limit for the specified PD port. */
bool app_port_fault_count_exceeded(cy_stc_pdstack_context_t * context)
{
    uint32_t i;
    bool     retval = false;
    uint8_t port = context->port;

    if (port < NO_OF_TYPEC_PORTS)
    {
        /*
         * Check whether the count for any fault type has exceeded the limit specified.
         */
        for (i = 0; i < FAULT_TYPE_COUNT; i++)
        {
            if (gl_app_fault_count[port][i] > gl_app_fault_retry_limit[port][i])
            {
                retval = true;
                break;
            }
        }
    }

    return (retval);
}

/* This function stops PD operation and configures Type-C to look for detach of faulty device. */
void app_conf_for_faulty_dev_removal(cy_stc_pdstack_context_t * context)
{
    cy_stc_pd_dpm_config_t *dpm_stat = &(context->dpmConfig);
    uint8_t port = context->port;

    if ((!dpm_stat->attach) || (dpm_stat->curPortRole == CY_PD_PRT_ROLE_SINK))
    {
        /* Set flag to trigger port disable sequence. */
        app_status[port].fault_status |= APP_PORT_SINK_FAULT_ACTIVE;
    }

    /* Stop PE */
    Cy_PdStack_Dpm_PeStop(context);
}

#if (FAULT_HANDLER_ENABLE)

#if ((FAULT_RETRY_DELAY_EN) || (FAULT_INFINITE_RECOVERY_EN))
/* Timer delay callback to recover from faults. */
static void fault_delayed_recovery_timer_cb(cy_timer_id_t id, void *context)
{
    (void)id;

    /*
     * Start DPM for retry.
     */
    Cy_PdStack_Dpm_Start((cy_stc_pdstack_context_t *)context);
}
#endif /* ((FAULT_RETRY_DELAY_EN) || (FAULT_INFINITE_RECOVERY_EN)) */

/* Generic routine that notifies the stack about recovery actions for a fault. */
static void app_handle_fault(cy_stc_pdstack_context_t * context, uint32_t fault_type)
{
    uint8_t port = context->ptrUsbPdContext->port;

    if (fault_type != FAULT_TYPE_VCONN_OCP)
    {
        context->dpmStat.faultActive = true;
    }

    /* Update the fault count. */
    if(gl_app_fault_retry_limit[port][fault_type] == FAULT_COUNTER_SKIP_VALUE)
    {
        /* Do not count faults if infinite fault retry is set. */
    }
    else
    {
        gl_app_fault_count[port][fault_type]++;
    }

    if (gl_app_fault_count[port][fault_type] < (gl_app_fault_retry_limit[port][fault_type] + 1))
    {
#if FAULT_RETRY_DELAY_EN
        /*
         * Fault retry delay timer shall be enabled.
         * Disable the port until configured timeout elapses.
         */
        Cy_PdStack_Dpm_Stop(context);
        Cy_PdUtils_SwTimer_Start(context->timerContext, context, GET_APP_TIMER_ID(ptrPdStackContext, APP_FAULT_RECOVERY_TIMER),
                FAULT_RETRY_DELAY_MS, fault_delayed_recovery_timer_cb);
#else /* !FAULT_RETRY_DELAY_EN */

        context->peStat.hardResetCount = 0;

        /*
         * Try a Hard Reset to recover from fault.
         * If not successful (not in PD contract), try Type-C error recovery.
         */
        if (Cy_PdStack_Dpm_SendPdCommand(context, CY_PDSTACK_DPM_CMD_SEND_HARD_RESET,
                    NULL, false, NULL) != CY_PDSTACK_STAT_SUCCESS)
        {
            Cy_PdStack_Dpm_SendTypecCommand(context, CY_PDSTACK_DPM_CMD_TYPEC_ERR_RECOVERY, NULL);
        }
#endif /* FAULT_RETRY_DELAY_EN */
    }
    else
    {
        app_conf_for_faulty_dev_removal(context);

#if FAULT_INFINITE_RECOVERY_EN
        /*
         * Start a timer to try infinite fault recovery with a timeout.
         */
        Cy_PdUtils_SwTimer_Start (context->timerContext, context, GET_APP_TIMER_ID(ptrPdStackContext, APP_FAULT_RECOVERY_TIMER),
                FAULT_INFINITE_RECOVERY_DELAY_MS, fault_delayed_recovery_timer_cb);
#endif /* FAULT_INFINITE_RECOVERY_EN */
    }
}

#endif /* FAULT_HANDLER_ENABLE */

/* Timer used to re-enable the PD port after a fault. */
static void fault_recovery_timer_cb(cy_timer_id_t id, void *context)
{
    cy_stc_pdstack_context_t *callbackContext = (cy_stc_pdstack_context_t *) context;
    uint16_t period = APP_FAULT_RECOVERY_TIMER_PERIOD;
    uint8_t port = callbackContext->port;

    if (
            (vbus_is_present(callbackContext, CY_PD_VSAFE_0V, 0) == false)
       )
    {
        if ((app_get_status(port)->fault_status & APP_PORT_VBUS_DROP_WAIT_ACTIVE) != 0)
        {
            app_status[port].fault_status &= ~APP_PORT_VBUS_DROP_WAIT_ACTIVE;

            /* VBus has already been removed. Enable the Rd termination to check for physical detach. */
            Cy_USBPD_TypeC_RdEnable (callbackContext->ptrUsbPdContext);
            period = APP_FAULT_RECOVERY_MAX_WAIT;
        }
        else
        {
            /*
             * If VBus is not detected, we can re-enable the PD port.
             */
            app_status[port].fault_status &= ~APP_PORT_DISABLE_IN_PROGRESS;
            callbackContext->dpmStat.faultActive = false;

            Cy_USBPD_TypeC_DisableRd(callbackContext->ptrUsbPdContext, CY_PD_CC_CHANNEL_1);
            Cy_USBPD_TypeC_DisableRd(callbackContext->ptrUsbPdContext, CY_PD_CC_CHANNEL_2);
            Cy_PdStack_Dpm_Start(callbackContext);

            /* Return without restarting the timer. */
            return;
        }
    }

    /* Restart the timer to check VBus and Rp status again. */
    Cy_PdUtils_SwTimer_Start (callbackContext->ptrTimerContext, callbackContext,
            GET_APP_TIMER_ID(callbackContext, APP_FAULT_RECOVERY_TIMER),
            period, fault_recovery_timer_cb);
}

/* Callback used to get notification that PD port disable has been completed. */
static void app_port_disable_cb(cy_stc_pdstack_context_t * context, cy_en_pdstack_dpm_typec_cmd_resp_t resp)
{
    uint16_t period = APP_FAULT_RECOVERY_TIMER_PERIOD;
    uint8_t port = context->port;

    if (
            (vbus_is_present(context, CY_PD_VSAFE_0V, 0) == false)
       )
    {
        /* VBus has already been removed. Enable the Rd termination to check for physical detach. */
        Cy_USBPD_TypeC_RdEnable (context->ptrUsbPdContext);
        period = APP_FAULT_RECOVERY_MAX_WAIT;
    }
    else
    {
        /* VBus has not been removed. Start a task which waits for VBus removal. */
        app_status[port].fault_status |= APP_PORT_VBUS_DROP_WAIT_ACTIVE;
    }

    /* Provide a delay to allow VBus turn-on by port partner and then enable the port. */
    Cy_PdUtils_SwTimer_Start (context->ptrTimerContext, context,
            GET_APP_TIMER_ID(context, APP_FAULT_RECOVERY_TIMER),
            period, fault_recovery_timer_cb);
}

#endif /* FAULT_HANDLER_ENABLE */

/* Clear all fault counters associated with the specified port. */
void fault_handler_clear_counts (uint8_t port)
{
#if FAULT_HANDLER_ENABLE
    if (port < NO_OF_TYPEC_PORTS)
    {
        /* Clear all fault counters on disconnect. */
        memset ((uint8_t *)gl_app_fault_count[port], 0, FAULT_TYPE_COUNT);
    }
#endif /* FAULT_HANDLER_ENABLE */
}

/* Fault-handling specific actions to be performed for various event callbacks. */
bool fault_event_handler(cy_stc_pdstack_context_t * context, cy_en_pdstack_app_evt_t evt, const void *dat)
{
    bool skip_soln_cb = false;
    uint8_t port = context->port;
    (void) port;

#if FAULT_HANDLER_ENABLE
    switch (evt)
    {
        case APP_EVT_TYPE_C_ERROR_RECOVERY:
            if (app_port_fault_count_exceeded(context))
            {
                break;
            }
            /* Fall-through to below case when fault counts are within limits. */

        case APP_EVT_DISCONNECT:
        case APP_EVT_VBUS_PORT_DISABLE:
        case APP_EVT_HARD_RESET_SENT:
            /* Clear the port-in-fault status. */
            if ((app_status[port].fault_status & APP_PORT_DISABLE_IN_PROGRESS) == 0)
            {
                context->dpmStat.faultActive = false;
            }

            if ((evt == APP_EVT_DISCONNECT) || (evt == APP_EVT_VBUS_PORT_DISABLE))
            {
                /* Clear fault counters in cases where an actual disconnect has been detected. */
                fault_handler_clear_counts (port);
            }
            break;

        case APP_EVT_PD_CONTRACT_NEGOTIATION_COMPLETE:
            break;

#if VBUS_OVP_ENABLE
        case APP_EVT_VBUS_OVP_FAULT:
            app_handle_fault(context, FAULT_TYPE_VBUS_OVP);
            break;
#endif /* VBUS_OVP_ENABLE */

#if VBUS_UVP_ENABLE
        case APP_EVT_VBUS_UVP_FAULT:
            app_handle_fault(context, FAULT_TYPE_VBUS_UVP);
            break;
#endif /* VBUS_UVP_ENABLE */

        default:
            break;
    }
#endif /* FAULT_HANDLER_ENABLE */

    return skip_soln_cb;
}

void fault_handler_register_cbks (cy_stc_pdstack_context_t *ptrPdStackContext)
{
    cy_stc_usbpd_context_t *usbpd_ctx = ptrPdStackContext->ptrUsbPdContext;

    usbpd_ctx->vbatGndScpCb = NULL;
    usbpd_ctx->bbIlimCbk = pd_bb_ilim_fault_handler;
    usbpd_ctx->vregInrushCbk = pd_vreg_inrush_det_fault_handler;
    usbpd_ctx->bodCbk = pd_brown_out_fault_handler;
}


bool fault_handler_init_vars (cy_stc_pdstack_context_t * context)
{
#if (VBUS_OVP_ENABLE || VBUS_UVP_ENABLE)
    cy_stc_usbpd_config_t * fault_config = context->ptrUsbPdContext->usbpdConfig;
    uint8_t port = context->port;
#endif /* (VBUS_OVP_ENABLE || VBUS_UVP_ENABLE) */

#if VBUS_OVP_ENABLE
    if (fault_config->vbusOvpConfig == NULL)
    {
        return false;
    }

    gl_app_fault_retry_limit[port][FAULT_TYPE_VBUS_OVP] = fault_config->vbusOvpConfig->retryCount;
#endif /* VBUS_OVP_ENABLE */

#if VBUS_UVP_ENABLE
    if (fault_config->vbusUvpConfig == NULL)
    {
        return false;
    }

    gl_app_fault_retry_limit[port][FAULT_TYPE_VBUS_UVP] = fault_config->vbusUvpConfig->retryCount;
#endif /* VBUS_UVP_ENABLE */

    return true;
}

void fault_handler_task(cy_stc_pdstack_context_t * context)
{
#if FAULT_HANDLER_ENABLE
    uint8_t port = context->port;
    /*
     * If SINK fault handling is pending, queue a port disable command.
     */
    if((app_get_status(context->port)->fault_status & APP_PORT_SINK_FAULT_ACTIVE) != 0)
    {
        if (Cy_PdStack_Dpm_SendTypecCommand (context, CY_PDSTACK_DPM_CMD_PORT_DISABLE,
                    app_port_disable_cb) != CY_PDSTACK_STAT_BUSY)
        {
            app_status[port].fault_status &= ~APP_PORT_SINK_FAULT_ACTIVE;
            app_status[port].fault_status |= APP_PORT_DISABLE_IN_PROGRESS;
        }
    }
#endif /* FAULT_HANDLER_ENABLE */
}

#if VBUS_OVP_ENABLE

#define MAX_OVP_DEBOUNCE_CYCLES         (0x20u)

/* Configure Over-Voltage Protection checks based on parameters in config table. */
void app_ovp_enable(cy_stc_pdstack_context_t * context, uint16_t volt_mV, bool pfet, cy_cb_vbus_fault_t ovp_cb)
{
    uint32_t intr_state;
    cy_stc_fault_vbus_ovp_cfg_t *ovp_config =
        (cy_stc_fault_vbus_ovp_cfg_t *) context->ptrUsbPdContext->usbpdConfig->vbusOvpConfig;

    if (ovp_config->enable)
    {
        intr_state = Cy_SysLib_EnterCriticalSection();

        if (ovp_config->mode != CY_USBPD_VBUS_OVP_MODE_ADC)
        {
            Cy_USBPD_Fault_Vbus_OvpEnable(context->ptrUsbPdContext, volt_mV, ovp_cb, pfet);
        }

        Cy_SysLib_ExitCriticalSection(intr_state);
    }
}

void app_ovp_disable(cy_stc_pdstack_context_t * context, bool pfet)
{
    cy_stc_fault_vbus_ovp_cfg_t * ovp_config = (cy_stc_fault_vbus_ovp_cfg_t *) context->ptrUsbPdContext->usbpdConfig->vbusOvpConfig;

    if (ovp_config->enable)
    {
        /* Disable OVP. */
        if (ovp_config->mode != CY_USBPD_VBUS_OVP_MODE_ADC)
        {
            Cy_USBPD_Fault_Vbus_OvpDisable(context->ptrUsbPdContext, pfet);
        }
    }
}

#endif /* VBUS_OVP_ENABLE */

#if VBUS_UVP_ENABLE

#define MAX_UVP_DEBOUNCE_CYCLES         (0x20u)

/* Configure Under-Voltage Protection checks based on parameters in config table. */
void app_uvp_enable(cy_stc_pdstack_context_t * context, uint16_t volt_mV, bool pfet, cy_cb_vbus_fault_t uvp_cb)
{
    uint32_t intr_state;
    cy_stc_fault_vbus_uvp_cfg_t *uvp_config = (void*) context->ptrUsbPdContext->usbpdConfig->vbusUvpConfig;

    if (uvp_config->enable)
    {
        intr_state = Cy_SysLib_EnterCriticalSection ();

        Cy_USBPD_Fault_Vbus_UvpEnable (context->ptrUsbPdContext, volt_mV, uvp_cb, pfet);

        Cy_SysLib_ExitCriticalSection (intr_state);
    }
}

void app_uvp_disable(cy_stc_pdstack_context_t * context, bool pfet)
{
    cy_stc_fault_vbus_uvp_cfg_t *uvp_config = (void*)context->ptrUsbPdContext->usbpdConfig->vbusUvpConfig;

    /* Disable UVP. */
    if (uvp_config->enable)
    {
        Cy_USBPD_Fault_Vbus_UvpDisable (context->ptrUsbPdContext, pfet);
    }
}

#endif /* VBUS_UVP_ENABLE */

void pd_bb_ilim_fault_handler(void *callbackContext, bool state)
{
#if BB_ILIM_DET_ENABLE
    cy_stc_usbpd_context_t *usbpd_ctx = (cy_stc_usbpd_context_t *)callbackContext;
    cy_stc_pdstack_context_t *context = (cy_stc_pdstack_context_t *)(usbpd_ctx->pdStackContext);
    CY_UNUSED_PARAMETER(state);

    /* Enqueue fault event. */
    app_event_handler(context, APP_EVT_ILIM_FAULT, NULL);
#else /* BB_ILIM_DET_ENABLE */
    CY_UNUSED_PARAMETER(callbackContext);
    CY_UNUSED_PARAMETER(state);
#endif /* BB_ILIM_DET_ENABLE */
}

#if VREG_BROWN_OUT_DET_ENABLE
/* Callback used to check attach still present and do Brownout processing. */
void app_brownout_retry_cb (
        cy_timer_id_t id,            /**< Timer ID for which callback is being generated. */
        void *callbackContext)       /**< Timer module Context. */
{
#if VREG_BROWN_OUT_DET_ENABLE
    cy_stc_pdstack_context_t *context = (cy_stc_pdstack_context_t *)callbackContext;

    if(context->dpmConfig.attach)
    {
        /* Disable the port. */
        if (context->ptrAppCbk != NULL)
        {
#if (!CY_PD_SINK_ONLY)
            if(context->dpmConfig.curPortRole == CY_PD_PRT_ROLE_SOURCE)
            {
                context->ptrAppCbk->psrc_disable(context, NULL);
            }
#endif /* (!CY_PD_SINK_ONLY) */
            if(context->dpmConfig.curPortRole == CY_PD_PRT_ROLE_SINK)
            {
                soln_batt_chgr_hw_disable(context);
                context->ptrAppCbk->psnk_disable(context, NULL);
            }
        }

        /* Enqueue fault event. */
        app_event_handler(context, APP_EVT_VREG_BOD_FAULT, NULL);
    }
#else /* VREG_BROWN_OUT_DET_ENABLE */
    CY_UNUSED_PARAMETER(callbackContext);
#endif /* VREG_BROWN_OUT_DET_ENABLE */
    CY_UNUSED_PARAMETER(id);
}
#endif /* VREG_BROWN_OUT_DET_ENABLE */

/* Function to handle buck-boost iLim fault */
void pd_brown_out_fault_handler(void *callbackContext, bool state)
{
#if VREG_BROWN_OUT_DET_ENABLE
    cy_stc_usbpd_context_t *usbpd_ctx = (cy_stc_usbpd_context_t *)callbackContext;
    cy_stc_pdstack_context_t *context = (cy_stc_pdstack_context_t *)(usbpd_ctx->pdStackContext);

    if(context->dpmConfig.attach)
    {
        /* Detach detection might be delayed, so retry for debouncing if we still in attach */
        Cy_PdUtils_SwTimer_Start (context->ptrTimerContext, context,
            APP_BROWNOUT_CHECK_DETACH_DELAY_ID, APP_BROWNOUT_CHECK_DETACH_DELAY_PERIOD,
            app_brownout_retry_cb);
    }
#if VREG_INRUSH_DET_ENABLE
    /* Disable inrush fault timer */
    Cy_PdUtils_SwTimer_Stop(context->ptrTimerContext, CY_USBPD_APP_HAL_VREG_TIMER);
    /* Disable inrush fault also as both are related to vreg */
    Cy_USBPD_Fault_VregInrushDetDis(context->ptrUsbPdContext);
#endif /* VREG_INRUSH_DET_ENABLE */

    Cy_USBPD_Fault_BrownOutDetDis(context->ptrUsbPdContext);

#endif /* VREG_BROWN_OUT_DET_ENABLE */
    CY_UNUSED_PARAMETER(callbackContext);
    CY_UNUSED_PARAMETER(state);
}

/* Function to handle Vreg inrush detection fault */
void pd_vreg_inrush_det_fault_handler(void *callbackContext, bool state)
{
#if VREG_INRUSH_DET_ENABLE
    cy_stc_usbpd_context_t *usbpd_ctx = (cy_stc_usbpd_context_t *)callbackContext;
    cy_stc_pdstack_context_t *context = (cy_stc_pdstack_context_t *)(usbpd_ctx->pdStackContext);

    if(false == app_is_typec_attached())
    {
        /* Disable inrush fault timer */
        Cy_PdUtils_SwTimer_Stop(context->ptrTimerContext, CY_USBPD_APP_HAL_VREG_TIMER);
    }

    /* Disable interrupt handling until recovery is tried */
    Cy_USBPD_Fault_VregInrushDetDis(usbpd_ctx);

    /* Enqueue fault event. */
    app_event_handler(context, APP_EVT_VREG_INRUSH_FAULT, NULL);
#else /* VREG_INRUSH_DET_ENABLE */
    /* No statement */
#endif /* VREG_INRUSH_DET_ENABLE */

    CY_UNUSED_PARAMETER(callbackContext);
    CY_UNUSED_PARAMETER(state);
}

/* [] End of File */
