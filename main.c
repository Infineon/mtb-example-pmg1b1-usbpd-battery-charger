/******************************************************************************
* File Name:   main.c
*
* Description: This is the source code for the PMG1B1 MCU: USBPD Battery Charger
*              Example for ModusToolbox.
*
* Related Document: See README.md
*
*
*******************************************************************************
* Copyright 2021-2025, Cypress Semiconductor Corporation (an Infineon company) or
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

#include "cy_pdl.h"
#include "cybsp.h"
#include "config.h"
#include "cy_pdutils_sw_timer.h"
#include "cy_usbpd_common.h"
#include "cy_pdstack_common.h"
#include "cy_usbpd_typec.h"
#include "cy_pdstack_dpm.h"
#include "cy_usbpd_vbus_ctrl.h"
#include "cy_usbpd_phy.h"
#include "instrumentation.h"
#include "app.h"
#include "pdo.h"
#include "psink.h"
#include "swap.h"
#include "vdm.h"
#include "cy_usbpd_buck_boost.h"
#include "mtbcfg_ezpd.h"

#if (TEMPERATURE_SENSOR_COUNT != 0u)
#include "thermistor.h"
#endif /* (TEMPERATURE_SENSOR_COUNT != 0u) */

#if BATTERY_CHARGING_ENABLE
#include "battery_charging.h"
#endif /* BATTERY_CHARGING_ENABLE */

#if DEBUG_UART_ENABLE
#include "debug.h"
char temp[80];
#endif

cy_stc_pdutils_sw_timer_t gl_TimerCtx;
cy_stc_usbpd_context_t gl_UsbPdPort0Ctx;

cy_stc_pdstack_context_t gl_PdStackPort0Ctx;

/* Do not change max_current value to avoid 20CSA saturation. */
cy_stc_auto_cfg_settings_t usbpd_port0_auto_config =
{
    .max_current = 500,                /**<  Maximum current that can be supplied in steps of 10 mA units. */
};

/* BB VOUT OCP configuration. */
cy_stc_fault_vbat_ocp_cfg_t bat_ocp_config =
{
    .mode = 3,                       /**< Internal OCP with software debounce. */
    .threshold = 30,                 /**< OCP threshold (in percents over nominal current). */
    .debounce = 20,                  /**< software debounce time (in milliseconds). */
};

cy_stc_pdutils_sw_timer_t *Cy_PdUtils_SwTimer_GetContext(void)
{
    return &gl_TimerCtx;
}

const cy_stc_pdstack_dpm_params_t pdstack_port0_dpm_params =
{
    .dpmSnkWaitCapPeriod = 400,
    .dpmRpAudioAcc = CY_PD_RP_TERM_RP_CUR_DEF,
    .dpmDefCableCap = 300,
    .muxEnableDelayPeriod = 0,
    .typeCSnkWaitCapPeriod = 0,
    .defCur = 90
};

cy_stc_pdstack_context_t *gl_PdStackContexts[NO_OF_TYPEC_PORTS] =
{
    &gl_PdStackPort0Ctx,
};

bool mux_ctrl_init(uint8_t port)
{
    /* No MUXes to be controlled on the PMG1 proto kits. */
    CY_UNUSED_PARAMETER(port);
    return true;
}

#if CCG_SLN_EXTN_MSG_HANDLER_ENABLE
/* Global variable to hold the battery status */
uint32_t gl_bat_stat = CCG_EXT_MSG_INVALID_BAT_REF;
/* Global variable to send the battery capability response */
static uint32_t gl_bat_cap[3] = {
    CCG_PB_VID_PID,
    0x00000000,
    0x00000001
};
#endif /* CCG_SLN_EXTN_MSG_HANDLER_ENABLE */

#if (CY_PDUTILS_TIMER_TYPE == CY_PDUTILS_TIMER_TYPE_WDT)
const cy_stc_sysint_t wdt_interrupt_config =
{
    .intrSrc = (IRQn_Type)srss_interrupt_wdt_IRQn,
    .intrPriority = 0U,
};
#endif /* (CY_PDUTILS_TIMER_TYPE == CY_PDUTILS_TIMER_TYPE_WDT) */

const cy_stc_sysint_t usbpd_port0_intr0_config =
{
    .intrSrc = (IRQn_Type)mtb_usbpd_port0_IRQ,
    .intrPriority = 0U,
};

const cy_stc_sysint_t usbpd_port0_intr1_config =
{
    .intrSrc = (IRQn_Type)mtb_usbpd_port0_DS_IRQ,
    .intrPriority = 0U,
};

cy_stc_pdstack_context_t *get_pdstack_context(uint8_t portIdx)
{
    return (gl_PdStackContexts[portIdx]);
}

/* Solution PD event handler */
void sln_pd_event_handler(cy_stc_pdstack_context_t *ctx, cy_en_pdstack_app_evt_t evt, const void *data)
{
    batt_pd_event_handler(ctx, evt, data);

#if BATTERY_CHARGING_ENABLE
    bc_pd_event_handler (ctx, evt);
#endif /* BATTERY_CHARGING_ENABLE */

    if(evt == APP_EVT_HANDLE_EXTENDED_MSG)
    {
        cy_stc_pd_packet_extd_t * ext_mes = (cy_stc_pd_packet_extd_t * )data;
        if ((ext_mes->msg != CY_PDSTACK_EXTD_MSG_SECURITY_RESP) && (ext_mes->msg != CY_PDSTACK_EXTD_MSG_FW_UPDATE_RESP)
#if CCG_SLN_EXTN_MSG_HANDLER_ENABLE
          &&(ext_mes->msg != CY_PDSTACK_EXTD_MSG_GET_BAT_STATUS) && (ext_mes->msg != CY_PDSTACK_EXTD_MSG_GET_BAT_CAP)
#endif /* CCG_SLN_EXTN_MSG_HANDLER_ENABLE */
           )
        {
            /* Send Not supported message */
            Cy_Pdstack_Dpm_SendPdCommandEc(ctx, CY_PDSTACK_DPM_CMD_SEND_NOT_SUPPORTED, NULL, NULL);
        }

#if CCG_SLN_EXTN_MSG_HANDLER_ENABLE

        /* Handle the response messages here*/
        cy_stc_pdstack_dpm_pd_cmd_buf_t cmd;
        if(ext_mes->msg == CY_PDSTACK_EXTD_MSG_GET_BAT_STATUS)
        {
            gl_bat_stat = CCG_EXT_MSG_INVALID_BAT_REF;
            /* Return the battery status
             * Here we only have a single fixed battery. So we compare with 0.
             * If there is more than one battery, this check has to be updated.
             */
            if(((ext_mes->dat[0].val) & 0xFFFF) == 0)
            {
                if(ctx->dpmConfig.curPortRole == CY_PD_PRT_ROLE_SINK)
                {
                    gl_bat_stat = CCG_EXT_MSG_VALID_BAT_STAT_SNK_IDLE;
                }
                else
                {
                    gl_bat_stat = CCG_EXT_MSG_VALID_BAT_STAT_SRC;
                }
            }

            cmd.cmdSop = CY_PD_SOP;
            cmd.cmdDo[0].val = gl_bat_stat;
            cmd.noOfCmdDo = 1;
            cmd.timeout = CY_PD_3_SENDER_RESPONSE_TIMER_PERIOD;
            Cy_Pdstack_Dpm_SendPdCommandEc(ctx, CY_PDSTACK_DPM_CMD_SEND_BATT_STATUS, &cmd, NULL);
        }

        if(ext_mes->msg == CY_PDSTACK_EXTD_MSG_GET_BAT_CAP)
        {
            /* Initialize with invalid reference in request */
            gl_bat_cap[1] = 0xFFFFFFFFu;
            gl_bat_cap[2] = 0x01;

            if(((ext_mes->dat[0].val) & 0xFFFF) == 0)
            {
                /* Valid reference */
                gl_bat_cap[0] = CCG_PB_VID_PID;
                gl_bat_cap[1] = (CCG_PB_BAT_FUL_CHG_CAP << 16) | CCG_PB_BAT_DES_CAP;
                gl_bat_cap[2] = 0x00;
            }
            else
            {
                gl_bat_cap[0] = 0x0000FFFFu;
            }

            cmd.cmdSop = CY_PD_SOP;
            cmd.extdHdr.val = 0x09;
            cmd.timeout = CY_PD_3_SENDER_RESPONSE_TIMER_PERIOD;
            cmd.extdType = CY_PDSTACK_EXTD_MSG_BAT_CAP;
            cmd.datPtr = (uint8_t *)gl_bat_cap;
            Cy_Pdstack_Dpm_SendPdCommandEc(ctx, CY_PDSTACK_DPM_CMD_SEND_EXTENDED, &cmd, NULL);
        }
#endif /* CCG_SLN_EXTN_MSG_HANDLER_ENABLE */
    }
}

void instrumentation_cb(uint8_t port, inst_evt_t evt)
{
    uint8_t evt_offset = APP_TOTAL_EVENTS;
    evt += evt_offset;
    sln_pd_event_handler(&gl_PdStackPort0Ctx, (cy_en_pdstack_app_evt_t)evt, NULL);
}

#if (CY_PDUTILS_TIMER_TYPE == CY_PDUTILS_TIMER_TYPE_WDT)
static void wdt_interrupt_handler(void)
{
    /* Clear WDT pending interrupt */
    Cy_WDT_ClearInterrupt();

#if (CY_PDUTILS_TIMER_TICKLESS_ENABLE == 0)
    /* Load the timer match register. */
    Cy_WDT_SetMatch((Cy_WDT_GetCount() + gl_TimerCtx.multiplier));
#endif /* (CY_PDUTILS_TIMER_TICKLESS_ENABLE == 0) */

    /* Invoke the timer handler. */
    Cy_PdUtils_SwTimer_InterruptHandler(&(gl_TimerCtx));
}
#endif /* (CY_PDUTILS_TIMER_TYPE == CY_PDUTILS_TIMER_TYPE_WDT) */

static void cy_usbpd0_intr0_handler(void)
{
    Cy_USBPD_Intr0Handler(&gl_UsbPdPort0Ctx);
}

static void cy_usbpd0_intr1_handler(void)
{
#if (CY_PDUTILS_TIMER_TYPE == CY_PDUTILS_TIMER_TYPE_PD_ILO)
    if(0u != (gl_UsbPdPort0Ctx.base->intr1_masked & PDSS_INTR1_MASKED_LF_CNTR_MASKED))
    {
        gl_UsbPdPort0Ctx.base->intr1 = PDSS_INTR1_LF_CNTR_MATCH;

#if (CY_PDUTILS_TIMER_TICKLESS_ENABLE == 0)
        /* Load the timer match register. */
        Cy_LF_SetMatch(&gl_UsbPdPort0Ctx, (Cy_LF_GetCount() + gl_TimerCtx.multiplier));
#endif /* (CY_PDUTILS_TIMER_TICKLESS_ENABLE == 0) */

        Cy_PdUtils_SwTimer_InterruptHandler(&gl_TimerCtx);
    }
#endif /* (CY_PDUTILS_TIMER_TYPE == CY_PDUTILS_TIMER_TYPE_PD_ILO) */
    Cy_USBPD_Intr1Handler(&gl_UsbPdPort0Ctx);
}

cy_stc_pd_dpm_config_t *get_dpm_connect_stat(void)
{
    return &(gl_PdStackPort0Ctx.dpmConfig);
}

/*
 * Application callback functions for the DPM. Since this application
 * uses the functions provided by the stack, loading the stack defaults.
 */
const cy_stc_pdstack_app_cbk_t app_callback =
{
    app_event_handler,
    vconn_enable,
    vconn_disable,
    vconn_is_present,
    vbus_is_present,
    vbus_discharge_on,
    vbus_discharge_off,
    psnk_set_voltage,
    psnk_set_current,
    psnk_enable,
    psnk_disable,
    eval_src_cap,
    eval_dr_swap,
    eval_pr_swap,
    eval_vconn_swap,
    eval_vdm,
   vbus_get_value,
#if CY_PD_USB4_SUPPORT_ENABLE
        NULL,
#endif /* CY_PD_USB4_SUPPORT_ENABLE */
};
#if (TEMPERATURE_SENSOR_COUNT != 0u)
/* Array to map the thermal voltage read by thermistor to corresponding temperature */
static const uint16_t gl_thermistor_volt_map[VOLT_TO_TEMP_MAP_TABLE_COUNT] =
{
    /* This voltage-temperature mapping is based on NTCG164KF104FTDS NTC thermistor
     * If a different thermistor is to be used, the table has to be updated.
     */
    4848, /* Corresponds to 20 degree C */
    4806, /* Corresponds to 25 degree C */
    4756, /* Corresponds to 30 degree C */
    4696, /* Corresponds to 35 degree C */
    4623, /* Corresponds to 40 degree C */
    4538, /* Corresponds to 45 degree C */
    4438, /* Corresponds to 50 degree C */
    4324, /* Corresponds to 55 degree C */
    4194, /* Corresponds to 60 degree C */
    4049, /* Corresponds to 65 degree C */
    3889, /* Corresponds to 70 degree C */
    3715, /* Corresponds to 75 degree C */
    3529, /* Corresponds to 80 degree C */
    3334, /* Corresponds to 85 degree C */
    3131, /* Corresponds to 90 degree C */
    2925, /* Corresponds to 95 degree C */
    2718, /* Corresponds to 100 degree C */
    2513, /* Corresponds to 105 degree C */
    2312, /* Corresponds to 110 degree C */
    2118  /* Corresponds to 115 degree C */
};
#endif /* (TEMPERATURE_SENSOR_COUNT != 0u) */

#if APP_FW_LED_ENABLE
/* LED blink rate in milliseconds */
static uint16_t gl_LedBlinkRate = LED_TIMER_PERIOD_TYPEC_SRC;
#endif

#if APP_FW_LED_ENABLE
void led_timer_cb(
    cy_timer_id_t id,      /**< Timer ID for which callback is being generated. */
    void *callbackContext) /**< Timer module Context. */
{
    cy_stc_pdstack_context_t *stack_ctx = (cy_stc_pdstack_context_t *)callbackContext;

    /* Calculate the desired LED blink rate based on the correct Type-C connection state. */
    if (stack_ctx->dpmConfig.attach)
    {
        if (stack_ctx->dpmConfig.curPortRole == CY_PD_PRT_ROLE_SOURCE)
        {
            /* Turn ON the User LED. */
            Cy_GPIO_Clr(USER_STATUS_LED_PORT, USER_STATUS_LED_PIN);
        }
        else
        {
            /* Toggle the User LED and re-start timer to schedule the next toggle event. */
            Cy_GPIO_Inv(USER_STATUS_LED_PORT, USER_STATUS_LED_PIN);
        }
    }
    else
    {
        /* Turn OFF the User LED. */
        Cy_GPIO_Set(USER_STATUS_LED_PORT, USER_STATUS_LED_PIN);
    }

    Cy_PdUtils_SwTimer_Start (&gl_TimerCtx, callbackContext, id, gl_LedBlinkRate, led_timer_cb);
}
#endif /* APP_FW_LED_ENABLE */

cy_stc_pdstack_app_cbk_t* app_get_callback_ptr(cy_stc_pdstack_context_t * context)
{
    (void)context;
    /* Solution callback pointer is same for all ports */
    return ((cy_stc_pdstack_app_cbk_t *)(&app_callback));
}

/*
 * Solution level callback functions for the application layer
 */
const app_sln_handler_t sln_cbk =
{
    sln_pd_event_handler,
    app_get_callback_ptr,
    get_pdstack_context,
    mux_ctrl_init,
    NULL
};

int main(void)
{
    cy_rslt_t result;
    cy_stc_pdutils_timer_config_t timerConfig;

    /* Initialize the device and board peripherals */
    result = cybsp_init();
    if (result != CY_RSLT_SUCCESS)
    {
        CY_ASSERT(0);
    }

    /*
     * Register the interrupt handler for the watchdog timer. This timer is used to
     * implement the soft timers required by the USB-PD Stack.
     */
#if (CY_PDUTILS_TIMER_TYPE == CY_PDUTILS_TIMER_TYPE_WDT)
    Cy_SysInt_Init(&wdt_interrupt_config, &wdt_interrupt_handler);
    NVIC_EnableIRQ(wdt_interrupt_config.intrSrc);
#endif /* (CY_PDUTILS_TIMER_TYPE == CY_PDUTILS_TIMER_TYPE_WDT) */

    timerConfig.sys_clk_freq = Cy_SysClk_ClkSysGetFrequency();
#if (CY_PDUTILS_TIMER_TYPE == CY_PDUTILS_TIMER_TYPE_PD_ILO)
    timerConfig.hw_timer_ctx = &gl_UsbPdPort0Ctx;
#else
    timerConfig.hw_timer_ctx = NULL;
#endif

    /* Enable global interrupts */
    __enable_irq();

#if DEBUG_UART_ENABLE
    debug_init();

    /* Send a string over serial terminal */
    DEBUG_PRINT("Hello \r\n");
#endif

    /* Initialize the instrumentation related data structures. */
    instrumentation_init();

    /* Register callback function to be executed when instrumentation fault occurs. */
    instrumentation_register_cb((instrumentation_cb_t)instrumentation_cb);

#if (TEMPERATURE_SENSOR_COUNT != 0u)
    register_thermistor_mapping_table(&gl_PdStackPort0Ctx, (const uint16_t *)gl_thermistor_volt_map);
#endif
    /* Register callbacks for solution level functions */
    register_soln_function_handler(&gl_PdStackPort0Ctx, (app_sln_handler_t *)&sln_cbk);

    /* Configure and enable the USBPD interrupts */
    Cy_SysInt_Init(&usbpd_port0_intr0_config, &cy_usbpd0_intr0_handler);
    NVIC_EnableIRQ(usbpd_port0_intr0_config.intrSrc);

    Cy_SysInt_Init(&usbpd_port0_intr1_config, &cy_usbpd0_intr1_handler);
    NVIC_EnableIRQ(usbpd_port0_intr1_config.intrSrc);

    /* Init Auto Config param. Only max_current field is used by PDL to set CSA TRIMS and CC_GAIN.  */
    mtb_usbpd_port0_config.autoConfig = &usbpd_port0_auto_config;
    /* Initialize the USBPD driver */
    Cy_USBPD_Init(&gl_UsbPdPort0Ctx, 0, mtb_usbpd_port0_HW, mtb_usbpd_port0_HW_TRIM,
            (cy_stc_usbpd_config_t *)&mtb_usbpd_port0_config, get_dpm_connect_stat);

    /* Initialize the soft timer module. */
    Cy_PdUtils_SwTimer_Init(&gl_TimerCtx, &timerConfig);

    gl_UsbPdPort0Ctx.vbusCsaRsense = gl_UsbPdPort0Ctx.usbpdConfig->buckBoostConfig->current_sense_res;
    gl_UsbPdPort0Ctx.peak_current_sense_resistor = gl_UsbPdPort0Ctx.usbpdConfig->buckBoostConfig->peak_current_sense_resistor;
    /* Set pointer to Battery OCP context */
#if (BAT_HW_OCP_ENABLE != 0u)
    gl_UsbPdPort0Ctx.usbpdConfig->vbatOcpConfig = &bat_ocp_config;
#endif /* (BAT_HW_OCP_ENABLE != 0u) */
    /* Initialize the Device Policy Manager. */
    Cy_PdStack_Dpm_Init(&gl_PdStackPort0Ctx,
                        &gl_UsbPdPort0Ctx,
                        &mtb_usbpd_port0_pdstack_config,
                        app_get_callback_ptr(&gl_PdStackPort0Ctx),
                        &pdstack_port0_dpm_params,
                        &gl_TimerCtx);

    /* Initialize Buck boost controller */
    Cy_USBPD_BB_Init(&gl_UsbPdPort0Ctx);

    /* Initialize the fault configuration values */
    fault_handler_init_vars(&gl_PdStackPort0Ctx);

    /* Register solution space fault handler callbacks */
    fault_handler_register_cbks(&gl_PdStackPort0Ctx);

    /* Perform application level initialization. */
    app_init(&gl_PdStackPort0Ctx);

    sln_pd_event_handler(&gl_PdStackPort0Ctx, APP_EVT_POWER_CYCLE, NULL);

    /* Start any timers or tasks associated with application instrumentation. */
    instrumentation_start();

    /* Start the device policy manager operation. This will initialize the USB-PD block and enable connect detection. */
    Cy_PdStack_Dpm_Start(&gl_PdStackPort0Ctx);

#if APP_FW_LED_ENABLE
    /* Start a timer that will blink the USER_LED. */
    Cy_PdUtils_SwTimer_Start(&gl_TimerCtx, (void *)&gl_PdStackPort0Ctx, (cy_timer_id_t)LED_TIMER_ID,
                             gl_LedBlinkRate, led_timer_cb);
#endif /* APP_FW_LED_ENABLE */

    /*
     * After the initialization is complete, keep processing the USB-PD device policy manager task in a loop.
     * Since this application does not have any other function, the PMG1 device can be placed in "deep sleep"
     * mode for power saving whenever the PD stack and drivers are idle.
     */
    /* Timer for debugging CV mode. UART prints cells, total, BB out, current */
#if PRINT_CV
    Cy_PdUtils_SwTimer_Start(pdstack_port0_ctx.ptrTimerContext, &pdstack_port0_ctx, MY_ID, MY_PERIOD, &print_cb);
#endif /* PRINT_CV */

    for (;;)
    {
        /* Handle the device policy tasks for each PD port. */
        Cy_PdStack_Dpm_Task(&gl_PdStackPort0Ctx);
        
        /* Perform any application level tasks. */
        app_task(&gl_PdStackPort0Ctx);

        /* Perform tasks associated with instrumentation. */
        instrumentation_task();

#if SYS_DEEPSLEEP_ENABLE
       if(gl_PdStackPort0Ctx.dpmConfig.connect
                || (battery_measure_sar_is_active() == false)
                )
#endif /* SYS_DEEPSLEEP_ENABLE */
       {
           soln_task(&gl_PdStackPort0Ctx);
       }

#if SYS_DEEPSLEEP_ENABLE
        /* If possible, enter sleep mode for power saving. */
       if((gl_PdStackPort0Ctx.dpmConfig.connect == false)
                && (battery_measure_sar_is_active() == false)
                )
       {
           system_sleep(&gl_PdStackPort0Ctx,NULL);
       }
#endif /* SYS_DEEPSLEEP_ENABLE */
#if (CY_PDUTILS_TIMER_TYPE != CY_PDUTILS_TIMER_TYPE_WDT)
        /* Clears the WatchDog to prevent device reset */
        Cy_WDT_ClearWatchdog();
#endif /* (CY_PDUTILS_TIMER_TYPE != CY_PDUTILS_TIMER_TYPE_WDT) */
    }
}

/* [] END OF FILE */
