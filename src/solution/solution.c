/**
 * @file solution.c
 *
 * @brief @{Solution source file solution layer port.@}
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

#include "stdint.h"
#include "stddef.h"
#include "stdbool.h"
#include "stdio.h"
#include "stdlib.h"

#include "config.h"
#include "cy_pdl.h"
#include "cybsp.h"
#include "cy_usbpd_common.h"
#include "cy_pdstack_common.h"
#include "cy_usbpd_typec.h"
#include "cy_pdstack_dpm.h"
#include "cy_pdutils.h"
#include "cy_usbpd_vbus_ctrl.h"
#include "cy_usbpd_idac_ctrl.h"
#include "cy_usbpd_phy.h"
#include "cy_pdutils_sw_timer.h"
#include "cy_usbpd_buck_boost.h"

#if BATTERY_CHARGING_ENABLE
#include "battery_charging.h"
#endif

#include "app.h"

#include "solution_tasks.h"

#if DEBUG_UART_ENABLE
#include "debug.h"
//static char temp[80];
#endif

/* Pause between steps required to turn on buck-boost regulation */
#define VBTR_STEP_WIDTH_US                      (5u)

/* Filter value for Battery UVOV comparators */
#define BAT_UVOV_FILTER                         (0x0Au)

/* 12Bit ADC measurement states */
typedef enum {
    BATT_ADC_IDLE,
    BATT_ADC_START_CONVERSION,
    BATT_ADC_WAIT_RESULT,
    BATT_ADC_DONE
} batt_adc_state_t;

batt_chg_state_t gl_sln_batt_chg_state = BATT_CHG_IDLE;

batt_chg_alt_state_t gl_sln_batt_chg_alt_state = BATT_CHG_ALT_LOOK4BATTERY;

static void soln_batt_chge_cmd(cy_stc_pdstack_context_t *ptrPdStackContext, batt_chge_src_cmd_t src_cmd);

static void sln_ibtr_cb(void * callbackCtx, bool value);

#if (TRICKLE_CHARGE_TIMER_ENABLE || PRE_CHARGE_TIMER_ENABLE || NORMAL_CHARGE_TIMER_ENABLE)
/*
 * @typedef charging_timeout_state_t
 * @brief List of possible charging timeout states.
 */
typedef enum
{
    CHARGING_TIMEOUT_TRICKLE_INIT,              /* Start trickle timeout */
    CHARGING_TIMEOUT_PRECHARGE_INIT,            /* Start pre-charge timeout */
    CHARGING_TIMEOUT_NORMAL_INIT,               /* Start normal CC+CV timeout */
    CHARGING_TIMEOUT_TIMER_STOP,                /* Disable timeout counter */
} charging_timeout_state_t;

void charging_timeout_cmd(cy_stc_pdstack_context_t * ptrPdStackContext, charging_timeout_state_t state);

static uint16_t charging_timeout_counter = 0;

static uint32_t update_current_limit(cy_stc_pdstack_context_t *ptrPdStackContext,uint32_t current_limit)
{
    /* Re-calculate  current limit for the actual Rsense */
    return ((current_limit * CSA_IDEAL_RSENSE) / ptrPdStackContext->ptrUsbPdContext->vbusCsaRsense);
}

void timer_minute_tick_cb(cy_timer_id_t id, void * callbackCtx)
{
    (void)id;
    cy_stc_pdstack_context_t * ptrPdStackContext = (cy_stc_pdstack_context_t *) callbackCtx;
    if(charging_timeout_counter > 1)
    {
        charging_timeout_counter--;
        Cy_PdUtils_SwTimer_Start(ptrPdStackContext->ptrTimerContext, ptrPdStackContext,
                                SOLN_SAMPLE_TIME_1MINUTE_TIMER_ID, SOLN_SAMPLE_TIME_1MINUTE_TIMER_PERIOD,
                                timer_minute_tick_cb);
        DEBUG_PRINT_VAR("\n Timeout ends in %i minutes\n", charging_timeout_counter);
    }
    else
    {
        cy_stc_battery_charging_context_t* ptrBatteryChargingContext = get_battery_charging_context(ptrPdStackContext->port);
        cy_stc_battery_status_t* batt_stat = &(ptrBatteryChargingContext->batteryStatus);

        /* Tried to charge a bad battery. Stop charging. Battery shall be removed. */
        charging_timeout_cmd(ptrPdStackContext, CHARGING_TIMEOUT_TIMER_STOP);
        soln_batt_chgr_hw_disable(ptrPdStackContext);
        gl_sln_batt_chg_state = BATT_CHG_INIT;
        batt_stat->timeout_expired = true;
    }
}

void charging_timeout_cmd(cy_stc_pdstack_context_t * ptrPdStackContext, charging_timeout_state_t state)
{
    static charging_timeout_state_t charging_timeout_state = CHARGING_TIMEOUT_TIMER_STOP;

    if(state != charging_timeout_state)
    {
        bool timer_start = false;
        charging_timeout_state = state;
        switch(state)
        {
#if TRICKLE_CHARGE_TIMER_ENABLE
            case CHARGING_TIMEOUT_TRICKLE_INIT:
                charging_timeout_counter = TRICKLE_CHARGE_TIMER_VALUE;
                timer_start = true;
                DEBUG_PRINT("\n Started trickle timeout \n");
                break;
#endif /* TRICKLE_CHARGE_TIMER_ENABLE */
#if PRE_CHARGE_TIMER_ENABLE
            case CHARGING_TIMEOUT_PRECHARGE_INIT:
                charging_timeout_counter = PRE_CHARGE_TIMER_VALUE;
                timer_start = true;
                DEBUG_PRINT("\n Started pre-charge timeout \n");
                break;
#endif /* PRE_CHARGE_TIMER_ENABLE */
#if NORMAL_CHARGE_TIMER_ENABLE
            case CHARGING_TIMEOUT_NORMAL_INIT:
                charging_timeout_counter = NORMAL_CHARGE_TIMER_VALUE;
                timer_start = true;
                DEBUG_PRINT("\n Started normal charging timeout \n");
                break;
#endif /* NORMAL_CHARGE_TIMER_ENABLE */
            case CHARGING_TIMEOUT_TIMER_STOP:
                Cy_PdUtils_SwTimer_Stop(ptrPdStackContext->ptrTimerContext, SOLN_SAMPLE_TIME_1MINUTE_TIMER_ID);
                DEBUG_PRINT("\n Timeout timer stopped \n");
                break;
            default:
                break;
        }
        if(timer_start == true)
        {
            Cy_PdUtils_SwTimer_Start(ptrPdStackContext->ptrTimerContext, ptrPdStackContext,
                                SOLN_SAMPLE_TIME_1MINUTE_TIMER_ID, SOLN_SAMPLE_TIME_1MINUTE_TIMER_PERIOD,
                                timer_minute_tick_cb);
        }
    }
}
#endif /*(TRICKLE_CHARGE_TIMER_ENABLE || PRE_CHARGE_TIMER_ENABLE || NORMAL_CHARGE_TIMER_ENABLE)*/

#if BAT_HW_OVP_ENABLE
static bool soln_vbat_ovp_cbk(void *callbackContext, bool state)
{
    cy_stc_usbpd_context_t * ptrUsbPdContext = (cy_stc_usbpd_context_t *) callbackContext;
    cy_stc_pdstack_context_t * ptrPdStackContext = (cy_stc_pdstack_context_t *) ptrUsbPdContext->pdStackContext;
    cy_stc_battery_charging_context_t* ptrBatteryChargingContext = get_battery_charging_context(ptrPdStackContext->port);
    cy_stc_battery_status_t* batt_stat = &(ptrBatteryChargingContext->batteryStatus);
    soln_batt_chgr_hw_disable(ptrPdStackContext);

    DEBUG_PRINT("HW_BAT_OVP_DET\n");
    batt_stat->batt_ovp_fault_active = true;
    return true;
}
#endif /* BAT_HW_OVP_ENABLE */

#if BAT_HW_UVP_ENABLE
static bool soln_vbat_uvp_cbk(void *callbackContext, bool state)
{
    cy_stc_usbpd_context_t * ptrUsbPdContext = (cy_stc_usbpd_context_t *) callbackContext;
    cy_stc_pdstack_context_t * ptrPdStackContext = (cy_stc_pdstack_context_t *) ptrUsbPdContext->pdStackContext;
    cy_stc_battery_charging_context_t* ptrBatteryChargingContext = get_battery_charging_context(ptrPdStackContext->port);
    cy_stc_battery_status_t* batt_stat = &(ptrBatteryChargingContext->batteryStatus);
    soln_batt_chgr_hw_disable(ptrPdStackContext);

    DEBUG_PRINT("HW_BAT_UVP_DET\n");
    batt_stat->batt_uvp_fault_active = true;
    return true;
}
#endif /* BAT_HW_UVP_ENABLE */

#if PDL_VOUTBB_RCP_ENABLE
static bool soln_voutbb_rcp_cbk(void *callbackContext, bool state)
{
    cy_stc_usbpd_context_t * ptrUsbPdContext = (cy_stc_usbpd_context_t *) callbackContext;
    cy_stc_pdstack_context_t * ptrPdStackContext = (cy_stc_pdstack_context_t *) ptrUsbPdContext->pdStackContext;
    cy_stc_battery_charging_context_t* ptrBatteryChargingContext = get_battery_charging_context(ptrPdStackContext->port);
    cy_stc_battery_status_t* batt_stat = &(ptrBatteryChargingContext->batteryStatus);

    soln_batt_chgr_hw_disable(ptrPdStackContext);

    batt_stat->batt_rcp_fault_active = true;

    Cy_USBPD_Fault_Voutbb_RcpDisable(ptrUsbPdContext);
    DEBUG_PRINT("HW_VOUTBB_RCP_INTR\n");
    return true;
}
#endif /* PDL_VOUTBB_RCP_ENABLE */

#if BAT_HW_OCP_ENABLE
static bool soln_vbat_ocp_cbk(void *callbackContext, bool state)
{
    cy_stc_usbpd_context_t * ptrUsbPdContext = (cy_stc_usbpd_context_t *) callbackContext;
    cy_stc_pdstack_context_t * ptrPdStackContext = (cy_stc_pdstack_context_t *) ptrUsbPdContext->pdStackContext;
    cy_stc_battery_charging_context_t* ptrBatteryChargingContext = get_battery_charging_context(ptrPdStackContext->port);
    cy_stc_battery_status_t* batt_stat = &(ptrBatteryChargingContext->batteryStatus);

    soln_batt_chgr_hw_disable(ptrPdStackContext);

    batt_stat->batt_ocp_fault_active = true;
    Cy_USBPD_Fault_Vbat_OcpDisable(ptrUsbPdContext, CCG_SRC_FET);
    DEBUG_PRINT("HW_BAT_OCP_INTR\n");
    return true;
}
#endif /* BAT_HW_OCP_ENABLE */

void batt_enable_tmr_cbk(cy_timer_id_t id, void *callbackContext)
{
    cy_stc_pdstack_context_t* ptrPdStackContext = (cy_stc_pdstack_context_t*)callbackContext;
    soln_batt_chge_cmd(ptrPdStackContext, BATT_CHGE_BB_OUT_EN);
}

static void sln_vbtr_cb(void * callbackCtx, bool value)
{
    (void)value;

    /* VBTR transition is completed. */
    if(gl_sln_batt_chg_state == BATT_CHG_BB_SET_VBAT_WAIT)
    {
        gl_sln_batt_chg_state = BATT_CHG_BB_SET_IBAT;
    }
}

/* Non-blocking procedure to set BB Vout in large step when BB output is disconnected from the load. */
void sln_set_volt(cy_stc_usbpd_context_t *context, uint16_t voltmv)
{
    int16_t new_dac, cur_dac;
    uint32_t state;

    Cy_USBPD_Hal_Enable_CV(context);

#if VBUS_CTRL_TRIM_ADJUST_ENABLE
    new_dac = Cy_USBPD_Vbus_GetTrimIdac(context, voltmv);
#else
    new_dac = (volt_mv - CY_PD_VSAFE_5V) / 20u;
#endif
    state  = Cy_SysLib_EnterCriticalSection();

    cur_dac = Cy_USBPD_Hal_Get_Fb_Dac(context);

    /* Configure VBTR operation */
    Cy_USBPD_VBTR_Config(context, cur_dac, new_dac, sln_vbtr_cb);

    /* Start VBTR operation */
    Cy_USBPD_VBTR_Start(context);

    Cy_SysLib_ExitCriticalSection(state);
}

/* Blocking procedure to make small adjustments of BB Vout when BB output is connected to the load. */
void sln_set_volt_nocv(cy_stc_usbpd_context_t *context, uint16_t voltmv)
{
    int16_t dac, new_dac, cur_dac;

    /* CV mode is already enabled here.  */
#if VBUS_CTRL_TRIM_ADJUST_ENABLE
    new_dac = Cy_USBPD_Vbus_GetTrimIdac(context, voltmv);
#else
    new_dac = (volt_mv - CY_PD_VSAFE_5V) / 20u;
#endif

    cur_dac = Cy_USBPD_Hal_Get_Fb_Dac(context);

    if (new_dac > cur_dac)
    {
        for (dac = cur_dac; dac <= new_dac; dac++)
        {
            Cy_USBPD_Hal_Set_Fb_Dac(context, dac);
            Cy_SysLib_DelayUs(VBTR_STEP_WIDTH_US);
        }
    }
    else
    {
        for (dac = cur_dac; dac >= new_dac; dac--)
        {
            Cy_USBPD_Hal_Set_Fb_Dac(context, dac);
            Cy_SysLib_DelayUs(VBTR_STEP_WIDTH_US);
        }
    }
}

#if VREG_BROWN_OUT_DET_ENABLE
void soln_disable_out(cy_stc_pdstack_context_t* ptrPdStackContext)
{
    soln_batt_chge_cmd(ptrPdStackContext, BATT_CHGE_BB_OUT_DIS);
}
#endif /* VREG_BROWN_OUT_DET_ENABLE */

/* Update the Buck boost output voltage and current based on the USB-C Input power and current battery voltage */
void calc_buck_boost_out_pwr_settings (cy_stc_pdstack_context_t* ptrPdStackContext)
{
    cy_stc_battery_charging_context_t* ptrBatteryChargingContext = get_battery_charging_context(ptrPdStackContext->port);
    cy_stc_battery_status_t* batt_stat = &(ptrBatteryChargingContext->batteryStatus);
    uint16_t calc_batt_ip_volt = TOTAL_VBATT_MAX_ALLOWED_VOLT;
    uint32_t calc_output_power;
    uint32_t calc_input_power;

#if BATTERY_CHARGING_ENABLE
    if(bc_get_status(ptrPdStackContext)->connected == true)
    {
        calc_output_power = (uint32_t)((CY_PD_VSAFE_5V * bc_get_status(ptrPdStackContext)->cur_amp)
                                        * INPUT_OUTPUT_EFFICIENCY_REDUCE_PERCENTAGE) / 100 ;
    }
    else
#endif
    {
        calc_output_power = (uint32_t)((ptrPdStackContext->dpmStat.contract.minVolt * ptrPdStackContext->dpmStat.contract.curPwr)
                                        * INPUT_OUTPUT_EFFICIENCY_REDUCE_PERCENTAGE) / 100 ;
    }

    /* Source 5% voltage tolerance */
    calc_output_power = (calc_output_power * VBUS_TOLERANCE_PERCENTAGE) / 100;

    /* Manufacture efficiency */
    calc_output_power = (calc_output_power * MANUFACTURE_EFFICIENCY_PERCENTAGE) / 100;

#if ENABLE_ALL_BATT_MONITORING
    calc_batt_ip_volt = batt_stat->curr_batt_volt + TOTAL_VBATT_HYST_THRESH;
    
    if(calc_batt_ip_volt > TOTAL_VBATT_MAX_ALLOWED_VOLT)
    {
        calc_batt_ip_volt = TOTAL_VBATT_MAX_ALLOWED_VOLT;
    }
#else
    calc_batt_ip_volt = TOTAL_VBATT_MAX_ALLOWED_VOLT;
#endif

    batt_stat->cur_bb_pwr = calc_output_power;
    batt_stat->cur_bb_vout = calc_batt_ip_volt;

    /* Start with min current at first */
    batt_stat->cur_bb_iout = update_current_limit(ptrPdStackContext, MIN_IBAT_CHARGING_CURR);

#if BATTERY_CHARGING_ENABLE
    if(bc_get_status(ptrPdStackContext)->connected == true)
    {
        calc_input_power = (CY_PD_VSAFE_5V * bc_get_status(ptrPdStackContext)->cur_amp) / 100;
    }
    else
#endif
    {
        calc_input_power = (ptrPdStackContext->dpmStat.contract.minVolt * ptrPdStackContext->dpmStat.contract.curPwr) / 100;
    }

    if(calc_input_power >= MIN_USBC_VBUS_TOTAL_POWER)
    {
        DEBUG_PRINT("\n Input Power sufficient");
        batt_stat->is_vbus_pwr_sufficient = true;
    }
    else
    {
        DEBUG_PRINT("\n Input Power NOT sufficient");
        batt_stat->is_vbus_pwr_sufficient = false;
    }
  
    DEBUG_PRINT("\r\n BAT VA UPDATE ");
    DEBUG_PRINT_VAR("\n BB Out Curr:: %d mA\n", (batt_stat->cur_bb_iout * 10));
    DEBUG_PRINT_VAR("\n BB Out Volt:: %d mV\n", batt_stat->cur_bb_vout);
    DEBUG_PRINT_VAR("\n BB Out Pwr:: %ld mW\n", batt_stat->cur_bb_pwr);
}

/* Battery Charger Solution PD event handler */
void batt_pd_event_handler(cy_stc_pdstack_context_t* ptrPdStackContext, cy_en_pdstack_app_evt_t evt, const void *data)
{
    const cy_stc_pdstack_pd_contract_info_t* contract_status;
    bool typec_only = false;

    cy_stc_battery_charging_context_t* ptrBatteryChargingContext = get_battery_charging_context(ptrPdStackContext->port);
    cy_stc_battery_status_t* batt_stat = &(ptrBatteryChargingContext->batteryStatus);

    switch(evt)
    {
        case APP_EVT_TYPEC_STARTED:
            DEBUG_PRINT("\n TypeC Start");
            break;

        case APP_EVT_CONNECT:
#if VREG_INRUSH_DET_ENABLE
            Cy_USBPD_Fault_VregInrushDetEn(ptrPdStackContext->ptrUsbPdContext);
#endif /* VREG_INRUSH_DET_ENABLE */
#if VREG_BROWN_OUT_DET_ENABLE
            if(ptrPdStackContext->dpmConfig.curPortRole == CY_PD_PRT_ROLE_SINK)
            {
                sol_brown_out_control(ptrPdStackContext, true);
            }
#endif /* VREG_BROWN_OUT_DET_ENABLE*/
            DEBUG_PRINT("\n Connect ");
            reset_battery_status(ptrBatteryChargingContext);

            /* Run charger state machine with SOLN_BATT_MONITOR_TASK_TIMER_PERIOD. */
            (void)Cy_PdUtils_SwTimer_StartWocb(ptrPdStackContext->ptrTimerContext, SOLN_BATT_MONITOR_TASK_TIMER_ID,
                    SOLN_BATT_MONITOR_TASK_TIMER_PERIOD);

            gl_sln_batt_chg_state = BATT_CHG_INIT;
            break;

        case APP_EVT_PE_DISABLED:
            typec_only = ((ptrPdStackContext->dpmConfig.contractExist == false) || (evt == APP_EVT_PE_DISABLED));

            if(typec_only == true)
            {
                batt_stat->is_vbus_pwr_sufficient = false;
            }

        /* Intentional fall-through. */
        case APP_EVT_HARD_RESET_SENT:                       /* Intentional fall through */
        case APP_EVT_HARD_RESET_RCVD:                       /* Intentional fall through */
        case APP_EVT_VBUS_PORT_DISABLE:                     /* Intentional fall through */
        case APP_EVT_DISCONNECT:                            /* Intentional fall through */
        case APP_EVT_TYPE_C_ERROR_RECOVERY:                 /* Intentional fall through */
        case APP_EVT_VBUS_UVP_FAULT:
        case APP_EVT_VBUS_OVP_FAULT:
#if BATTERY_CHARGING_ENABLE
        case APP_EVT_BC_DETECTION_COMPLETED:
#endif

#if VREG_INRUSH_DET_ENABLE
            /* Disable Vreg Inrush protection. */
            Cy_USBPD_Fault_VregInrushDetDis(ptrPdStackContext->ptrUsbPdContext);
#endif /* VREG_INRUSH_DET_ENABLE */
#if VREG_BROWN_OUT_DET_ENABLE
            /* Disable Brown-out detector. */
            sol_brown_out_control(ptrPdStackContext, false);
#endif /* VREG_BROWN_OUT_DET_ENABLE*/
            DEBUG_PRINT_VAR(" Charger event is: %#x \n", (evt));
            Cy_PdUtils_SwTimer_Stop (ptrPdStackContext->ptrTimerContext, BATT_TIMER_ID);
            Cy_PdUtils_SwTimer_Stop (ptrPdStackContext->ptrTimerContext, SOLN_BATT_MONITOR_TASK_TIMER_ID);
            /* battery charging is disabled */
            if( (ptrPdStackContext->dpmConfig.curPortRole == CY_PD_PRT_ROLE_SINK)
#if BATTERY_CHARGING_ENABLE
                ||
                ((evt == APP_EVT_BC_DETECTION_COMPLETED) && (bc_get_status(ptrPdStackContext)->connected == false))
#endif
                )
            {
                /* Disable only if charging is active */
                if(gl_sln_batt_chg_state != BATT_CHG_INIT)
                {
                    soln_batt_chgr_hw_disable(ptrPdStackContext);
                }
            }

            gl_sln_batt_chg_state = BATT_CHG_INIT;
            gl_sln_batt_chg_alt_state = BATT_CHG_ALT_LOOK4BATTERY;

            /* Clear pending BAT OVP and OCP flags on TypeC disconnect. */
            if(evt == APP_EVT_DISCONNECT)
            {
               clear_flags_on_usbc_disconnect(ptrBatteryChargingContext);
            }
            break;
            
        case APP_EVT_PD_CONTRACT_NEGOTIATION_COMPLETE: 
            contract_status = (cy_stc_pdstack_pd_contract_info_t*)data;

            if ((contract_status->status == CY_PDSTACK_CONTRACT_NEGOTIATION_SUCCESSFUL) ||
                    (contract_status->status == CY_PDSTACK_CONTRACT_CAP_MISMATCH_DETECTED))
            {
                DEBUG_PRINT_VAR("\n Contract volt:: %i mV\n", ptrPdStackContext->dpmStat.contract.minVolt);
                DEBUG_PRINT_VAR("\n Contract Curr:: %i mA\n", (ptrPdStackContext->dpmStat.contract.curPwr *10));
            }
            break;
        default:
            break;
    }
}

static void soln_batt_chge_cmd(cy_stc_pdstack_context_t *ptrPdStackContext, batt_chge_src_cmd_t src_cmd)
{
    cy_stc_battery_charging_context_t* ptrBatteryChargingContext = get_battery_charging_context(ptrPdStackContext->port);
    cy_stc_battery_status_t* batt_stat = &(ptrBatteryChargingContext->batteryStatus);

    switch(src_cmd)
    {
        case BATT_CHGE_BB_EN:
            if(Cy_USBPD_BB_IsEnabled(ptrPdStackContext->ptrUsbPdContext) == false)
            {
                Cy_USBPD_BB_Enable(ptrPdStackContext->ptrUsbPdContext);
                DEBUG_PRINT( "\n BB En\r\n");
            }
            break;

        case BATT_CHGE_BB_DIS:
            /* Buck Boost is disabled */
#if PDL_VOUTBB_RCP_ENABLE
            Cy_USBPD_Fault_Voutbb_RcpDisable(ptrPdStackContext->ptrUsbPdContext);
#endif /* PDL_VOUTBB_RCP_ENABLE */
            Cy_USBPD_BB_Disable(ptrPdStackContext->ptrUsbPdContext);
            DEBUG_PRINT("\n BB Dis\r\n");
            break;

        case BATT_CHGE_BB_OUT_EN:
#if BAT_HW_UVP_ENABLE
            Cy_USBPD_Fault_Vbat_UvpEnable(ptrPdStackContext->ptrUsbPdContext, PRIMARY_VBATT_UVP_THRESHOLD, BAT_UVOV_FILTER, soln_vbat_uvp_cbk, CCG_SRC_FET);
#endif /* BAT_HW_UVP_ENABLE */
#if BAT_HW_OVP_ENABLE
            Cy_USBPD_Fault_Vbat_OvpEnable(ptrPdStackContext->ptrUsbPdContext, PRIMARY_VBATT_OVP_THRESHOLD, BAT_UVOV_FILTER, soln_vbat_ovp_cbk, CCG_SRC_FET);
#endif /* BAT_HW_OVP_ENABLE */

            Cy_GPIO_Write(B1_VOUT_DC_EN_H_PORT, B1_VOUT_DC_EN_H_PIN, 1u);
            batt_stat->cur_batt_charging_status = true;
            DEBUG_PRINT("\n BAT Switch ON ..\r\n");
            DEBUG_PRINT_VAR("\n >N_CNTis %i \n",batt_stat -> curr_chrg_cycle_num);
            break;

        case BATT_CHGE_BB_OUT_DIS:
            /* Note: CCG_SRC_FET is unused in procedure */
#if BAT_HW_UVP_ENABLE
            Cy_USBPD_Fault_Vbat_UvpDisable(ptrPdStackContext->ptrUsbPdContext, CCG_SRC_FET);
#endif /* BAT_HW_UVP_ENABLE */
#if BAT_HW_OVP_ENABLE
            Cy_USBPD_Fault_Vbat_OvpDisable(ptrPdStackContext->ptrUsbPdContext, CCG_SRC_FET);
#endif /* BAT_HW_OVP_ENABLE */
#if BAT_HW_OCP_ENABLE
            Cy_USBPD_Fault_Vbat_OcpDisable(ptrPdStackContext->ptrUsbPdContext, CCG_SRC_FET);
#endif /* BAT_HW_OCP_ENABLE */

            batt_stat->cur_batt_charging_status = false;
            Cy_GPIO_Write(B1_VOUT_DC_EN_H_PORT, B1_VOUT_DC_EN_H_PIN, 0u);
            DEBUG_PRINT("\n BAT Switch OFF ..\r\n");
            break;

        case BATT_CHGE_BB_IN_EN:
            /* Note: non-functional for SINK_ONLY mode */
            Cy_USBPD_Vbus_GdrvPfetOn(ptrPdStackContext->ptrUsbPdContext, false);
            DEBUG_PRINT("\n InSwON\r\n");
            break;
            
        case BATT_CHGE_BB_IN_DIS:
            Cy_USBPD_Vbus_GdrvPfetOff(ptrPdStackContext->ptrUsbPdContext, false);
            DEBUG_PRINT("\n InSwOFF\r\n");
            break;

        default:
            break;
    }
}

void soln_batt_chgr_hw_disable(cy_stc_pdstack_context_t* ptrPdStackContext)
{
    /* Disable Battery Charging */
    soln_batt_chge_cmd(ptrPdStackContext, BATT_CHGE_BB_DIS);
    soln_batt_chge_cmd(ptrPdStackContext, BATT_CHGE_BB_OUT_DIS);
    soln_batt_chge_cmd(ptrPdStackContext, BATT_CHGE_BB_IN_DIS);
    Cy_PdUtils_SwTimer_Stop (ptrPdStackContext->ptrTimerContext, BATT_TIMER_ID);
}

bool soln_batt_attach_debounced(cy_stc_pdstack_context_t* ptrPdStackContext)
{
    cy_stc_battery_charging_context_t* ptrBatteryChargingContext = get_battery_charging_context(ptrPdStackContext->port);
    cy_stc_battery_status_t* batt_stat = &(ptrBatteryChargingContext->batteryStatus);
    static uint8_t batt_prev_state = NO_BATTERY;

    batt_prev_state = batt_stat->batt_pack_type;

    soln_battery_charger_task(ptrBatteryChargingContext);

    /* If battery just connected, do debounce to measure stable parameters. */
    if((batt_stat->batt_pack_type != NO_BATTERY) && (batt_prev_state == NO_BATTERY))
    {
        if(batt_prev_state == NO_BATTERY)
        {
            DEBUG_PRINT("\nDebounce. No prev Bat .\n");
        }
        Cy_PdUtils_SwTimer_StartWocb(ptrPdStackContext->ptrTimerContext, SOLN_BATT_MONITOR_DEBOUNCE_TIMER_ID,
                SOLN_BATT_MONITOR_DEBOUNCE_TIMER_PERIOD);
        return false;
    }

    if(Cy_PdUtils_SwTimer_IsRunning(ptrPdStackContext->ptrTimerContext, SOLN_BATT_MONITOR_DEBOUNCE_TIMER_ID))
    {
        DEBUG_PRINT("\nDebounce. Timer.\n");
        return false;
    }
    return true;
}

void soln_task(cy_stc_pdstack_context_t* ptrPdStackContext)
{
    cy_stc_battery_charging_context_t* ptrBatteryChargingContext = get_battery_charging_context(ptrPdStackContext->port);
    cy_stc_battery_status_t* batt_stat = &(ptrBatteryChargingContext->batteryStatus);
    static batt_adc_state_t adc_state = BATT_ADC_IDLE;

    switch(adc_state)
    {
    case BATT_ADC_IDLE:
        if(Cy_PdUtils_SwTimer_IsRunning(ptrPdStackContext->ptrTimerContext, SOLN_BATT_MONITOR_TASK_TIMER_ID) == false)
        {
            Cy_PdUtils_SwTimer_StartWocb(ptrPdStackContext->ptrTimerContext, SOLN_BATT_MONITOR_TASK_TIMER_ID, SOLN_BATT_MONITOR_TASK_TIMER_PERIOD);
            adc_state = BATT_ADC_START_CONVERSION;
        }
        break;

    case BATT_ADC_START_CONVERSION:
        /* Check and scan sar adc */
        batt_stat->adc_pending = true;
        battery_measure_sar_adc_scan(ptrPdStackContext);
        adc_state = BATT_ADC_WAIT_RESULT;
        break;

    case BATT_ADC_WAIT_RESULT:
        if(batt_stat->adc_pending == false)
        {
            adc_state = BATT_ADC_DONE;
        }
        break;

    case BATT_ADC_DONE:
        adc_state = BATT_ADC_IDLE;
        break;
    }

    if(adc_state != BATT_ADC_DONE)
    {
        return;
    }

    /* Check battery new attach. Process debouncing and reject results. */
    if(soln_batt_attach_debounced(ptrPdStackContext) == false)
    {
        DEBUG_PRINT("\nDebouncing. Reject this results.\n");
        reset_battery_faults(ptrBatteryChargingContext);
        return;
    }

#if DEBUG_UART_ENABLE
    /* Report charging FSM state and fault flags */
    uint8_t fault_st = 0;
    if(batt_stat->batt_uvp_fault_active == true)
        fault_st |= 1;
    if(batt_stat->batt_ovp_fault_active == true)
        fault_st |= 2;
    if(batt_stat->batt_ocp_fault_active == true)
        fault_st |= 4;
    if(batt_stat->batt_otp_fault_active == true)
        fault_st |= 8;
    if(batt_stat->ntcp0_otp_fault_active == true)
        fault_st |= 0x10;
    if(batt_stat->ntcp1_otp_fault_active == true)
    {
        fault_st |= 0x20;
    }
#if (TRICKLE_CHARGE_TIMER_ENABLE || PRE_CHARGE_TIMER_ENABLE || NORMAL_CHARGE_TIMER_ENABLE)
    if(batt_stat->timeout_expired == true)
    {
        fault_st |= 0x40;
    }
#endif /*(TRICKLE_CHARGE_TIMER_ENABLE || PRE_CHARGE_TIMER_ENABLE || NORMAL_CHARGE_TIMER_ENABLE)*/
    sprintf(temp, "\n CHGR FSM STATE %i ALT %i FLTT %x\n",gl_sln_batt_chg_state,gl_sln_batt_chg_alt_state,fault_st);
    debug_print(temp);
#endif

    /* If battery not present. */
    if(batt_stat->batt_pack_type == NO_BATTERY)
    {
        DEBUG_PRINT("\n No Battery.");
        /* Reset Batt. OVP,OCP,UVP and OTP faults */
        reset_battery_faults(ptrBatteryChargingContext);

        gl_sln_batt_chg_alt_state = BATT_CHG_ALT_LOOK4BATTERY;

        if((gl_sln_batt_chg_state >= BATT_CHG_BB_SET_VBAT) && (gl_sln_batt_chg_state <= BATT_CHG_CHARGE_LOOP))
        {
            soln_batt_chgr_hw_disable(ptrPdStackContext);
        }
        gl_sln_batt_chg_state = BATT_CHG_INIT;

#if (TRICKLE_CHARGE_TIMER_ENABLE || PRE_CHARGE_TIMER_ENABLE || NORMAL_CHARGE_TIMER_ENABLE)
        charging_timeout_cmd(ptrPdStackContext, CHARGING_TIMEOUT_TIMER_STOP);
        batt_stat->timeout_expired = false;
#endif /*(TRICKLE_CHARGE_TIMER_ENABLE || PRE_CHARGE_TIMER_ENABLE || NORMAL_CHARGE_TIMER_ENABLE)*/
    }
    else if((batt_stat->batt_ovp_fault_active == true) ||
            (batt_stat->batt_uvp_fault_active == true) ||
            (batt_stat->batt_otp_fault_active == true) ||
            (batt_stat->ntcp0_otp_fault_active == true) ||
            (batt_stat->ntcp1_otp_fault_active == true) ||
            (batt_stat->batt_rcp_fault_active == true) ||
            (batt_stat->batt_ocp_fault_active == true)
#if (TRICKLE_CHARGE_TIMER_ENABLE || PRE_CHARGE_TIMER_ENABLE || NORMAL_CHARGE_TIMER_ENABLE)
            || (batt_stat->timeout_expired == true)
#endif /*(TRICKLE_CHARGE_TIMER_ENABLE || PRE_CHARGE_TIMER_ENABLE || NORMAL_CHARGE_TIMER_ENABLE)*/
            )
    {
        DEBUG_PRINT("\r\n >>>Battery is at fault ");

        /* If Charging continues then disable it. */
        if((gl_sln_batt_chg_state >= BATT_CHG_BB_SET_VBAT) && (gl_sln_batt_chg_state <= BATT_CHG_CHARGE_LOOP))
        {
            soln_batt_chgr_hw_disable(ptrPdStackContext);
            gl_sln_batt_chg_state = BATT_CHG_INIT;
            return;
        }
        /* SNK role failures processing */
        else if(gl_sln_batt_chg_state == BATT_CHG_TYPEC_ATTACHED_SNK)
        {
            gl_sln_batt_chg_state = BATT_CHG_INIT;
            return;
        }
        else if(gl_sln_batt_chg_state == BATT_CHG_INIT)
        {
            /* Note: infinite UVP retries are allowed */
            if( (batt_stat->batt_ocp_fault_active == true) ||
                (batt_stat->batt_ovp_fault_active == true) ||
                (batt_stat->batt_otp_fault_active == true) ||
                (batt_stat->ntcp0_otp_fault_active == true) ||
                (batt_stat->ntcp1_otp_fault_active == true)
#if (TRICKLE_CHARGE_TIMER_ENABLE || PRE_CHARGE_TIMER_ENABLE || NORMAL_CHARGE_TIMER_ENABLE)
                || (batt_stat->timeout_expired == true)
#endif /*(TRICKLE_CHARGE_TIMER_ENABLE || PRE_CHARGE_TIMER_ENABLE || NORMAL_CHARGE_TIMER_ENABLE)*/
                )
            {
                /* wait here while temp/VbatOV will be back to normal */
                return;
            }
        }
        else
        {
            /* Do nothing */
        }
    }

    switch(gl_sln_batt_chg_state)
    {
    /* Initial FSM state. */
    case BATT_CHG_IDLE:  
        gl_sln_batt_chg_state = BATT_CHG_INIT;
        break;

    /* 
     * Check battery presence and wait here for PD contract.
     * Also loop here after Battery OCP/OVP fault. Wait on battery removal
     */
    case BATT_CHG_INIT:
        /* Enable DPM if battery present and not in OVP/OTP. */
        if (ptrPdStackContext->dpmConfig.dpmEnabled == false)
        {
            Cy_PdStack_Dpm_Start(ptrPdStackContext);
        }
        reset_battery_status(ptrBatteryChargingContext);

        /* check on Type-C attach */
        if(ptrPdStackContext->dpmConfig.attach == true)
        {
            if((ptrPdStackContext->dpmConfig.curPortRole == CY_PD_PRT_ROLE_SINK) &&
                ((ptrPdStackContext->dpmConfig.contractExist == true)
#if BATTERY_CHARGING_ENABLE
                || (bc_get_status(ptrPdStackContext)->connected == true)
#endif /* BATTERY_CHARGING_ENABLE */
                ))
            {
                gl_sln_batt_chg_state = BATT_CHG_TYPEC_ATTACHED_SNK;
#if SIMULATE_ERROR_SNK
                Cy_PdUtils_SwTimer_StartWocb(ptrPdStackContext->ptrTimerContext, DBG_BATT_TIMER_ID, DBG_BATT_TIMER_PERIOD);
#endif /* SIMULATE_ERROR_SNK */
            }
        }
        break;

    case BATT_CHG_TYPEC_ATTACHED_SNK:
        /* Wait on PD contract with sufficient power. */
        batt_stat->cur_batt_charging_status = false;
        batt_stat->cv_mode_entered = false;

        if ((ptrPdStackContext->dpmConfig.contractExist == false)
#if BATTERY_CHARGING_ENABLE
                && (bc_get_status(ptrPdStackContext)->connected == false)
#endif /* BATTERY_CHARGING_ENABLE */
                )
        {
            DEBUG_PRINT("\r\n No PD Contract ");
            gl_sln_batt_chg_alt_state = BATT_CHG_ALT_LOOK4BATTERY;
            break;
        }

        /* Check if Battery is present and PD power is sufficient */
        calc_buck_boost_out_pwr_settings(ptrPdStackContext);

        if(batt_stat->is_vbus_pwr_sufficient == true)
        {
            DEBUG_PRINT("\r\n pwr_sufficient ");
#if BATTERY_SIMULATOR_ENABLE
            gl_sln_batt_chg_alt_state = BATT_CHG_ALT_CHARGE_CC_MODE;
            soln_batt_chge_cmd(ptrPdStackContext, BATT_CHGE_BB_IN_EN);
            soln_batt_chge_cmd(ptrPdStackContext, BATT_CHGE_BB_EN);
            gl_sln_batt_chg_state = BATT_CHG_BB_SET_VBAT;
            break;
#endif /* BATTERY_SIMULATOR_ENABLE */

            /* <10V */
            if(batt_stat->curr_batt_volt < PRIMARY_VBATT_UVP_THRESHOLD)
            {
                gl_sln_batt_chg_alt_state = BATT_CHG_ALT_BAD_BATTERY;
                break;
            }

            /* 15V */
            if(batt_stat->curr_batt_volt < TOTAL_VBATT_DISCHARGED_SNK)
            {
                gl_sln_batt_chg_alt_state = BATT_CHG_ALT_INIT_CHARGE;
            }

            /* set BB out when Vbat <= 20V */
            else if((batt_stat->curr_batt_volt < TOTAL_VBATT_RECHARGE_THRESH)
                    && (batt_stat->is_cell_recharge == true)
                    )
            {
                if(batt_stat->is_cell_full == false)
                {
                    gl_sln_batt_chg_alt_state = BATT_CHG_ALT_CC_MODE;
                }
                else
                {
                    gl_sln_batt_chg_alt_state = BATT_CHG_ALT_CHARGE_FULL;
                    break;
                }
            }
            /* 20.5 -- 21V */
            else if((batt_stat->curr_batt_volt < TOTAL_VBATT_MAX_ALLOWED_VOLT) &&
                    (gl_sln_batt_chg_alt_state != BATT_CHG_ALT_CHARGE_FULL))
            {
                if(batt_stat->is_cell_full == false)
                {
                    gl_sln_batt_chg_alt_state = BATT_CHG_ALT_CC_MODE;
                }
                else
                {
                    gl_sln_batt_chg_alt_state = BATT_CHG_ALT_CHARGE_FULL;
                    break;
                }
            }
            else /* >=21V or Batt was just charged */
            {
                gl_sln_batt_chg_alt_state = BATT_CHG_ALT_CHARGE_FULL;
                break;
            }

#if DEBUG_UART_ENABLE
            sprintf(temp, "\n Contract volt:: %i mV\n", ptrPdStackContext->dpmStat.contract.minVolt);
            debug_print(temp);
            sprintf(temp, "\n Contract Curr:: %i mA\n", (ptrPdStackContext->dpmStat.contract.curPwr *10));
            debug_print(temp);

            debug_print("\r\n CF Mode enabled ");
            sprintf(temp, "\n \n \n BB OutCurr Setting:: %i mA", (batt_stat->cur_bb_iout * 10));
            debug_print(temp);
            sprintf(temp, "\n BB OutVolt Setting:: %i mV", batt_stat->cur_bb_vout);
            debug_print(temp);
#endif /* DEBUG_UART_ENABLE */
            soln_batt_chge_cmd(ptrPdStackContext, BATT_CHGE_BB_IN_EN);
            soln_batt_chge_cmd(ptrPdStackContext, BATT_CHGE_BB_EN);
            gl_sln_batt_chg_state = BATT_CHG_BB_SET_VBAT;
        }
        else
        {
            DEBUG_PRINT("\r\n pwr_not sufficient ");
            /* Contract power is not sufficient */
        }
        break;
        
    case BATT_CHG_BB_SET_VBAT:
        /* Set BB output voltage using VBTR block. */
        DEBUG_PRINT("\r\n CHG_BB_SET_VBAT ");
        if(Cy_USBPD_BB_IsReady(ptrPdStackContext->ptrUsbPdContext) == true)
        {
            DEBUG_PRINT("\r\n BB_IsReady ");
#ifdef BB_RESTART_ON_TIMEOUT_ENABLED
            Cy_PdUtils_SwTimer_Stop (ptrPdStackContext->ptrTimerContext, BATT_TIMER_ID);
#endif

            sln_set_volt(ptrPdStackContext->ptrUsbPdContext,batt_stat->cur_bb_vout);
            gl_sln_batt_chg_state = BATT_CHG_BB_SET_VBAT_WAIT;
        }
        
#ifdef BB_RESTART_ON_TIMEOUT_ENABLED
        else if(Cy_PdUtils_SwTimer_IsRunning(ptrPdStackContext->ptrTimerContext, BATT_TIMER_ID) == false)
        {
            /* Try to do BB_enable every 1 sec */
            soln_batt_chge_cmd(ptrPdStackContext, BATT_CHGE_BB_IN_EN);
            soln_batt_chge_cmd(ptrPdStackContext, BATT_CHGE_BB_EN);
            batt_stat->bb_out_configure = true;
            Cy_PdUtils_SwTimer_StartWocb(ptrPdStackContext->ptrTimerContext, BATT_TIMER_ID, BATT_TIMER_PERIOD);
        }
        else
        {

        }
#endif /* BB_RESTART_ON_TIMEOUT_ENABLED */
        break;

    /* Wait here on VBTR_DONE interrupt. */
    case BATT_CHG_BB_SET_VBAT_WAIT:
        break;
        /* Set BB output current using IBTR block. */

    case BATT_CHG_BB_SET_IBAT:
        ptrPdStackContext->ptrUsbPdContext->ibtrCbk = sln_ibtr_cb;
        Cy_USBPD_CF_Enable(ptrPdStackContext->ptrUsbPdContext, batt_stat->cur_bb_iout);
        gl_sln_batt_chg_state = BATT_CHG_ENA_BAT_FET;
        Cy_PdUtils_SwTimer_StartWocb(ptrPdStackContext->ptrTimerContext, BATT_TIMER_ID, BATT_TIMER_PERIOD);
        break;

    /* Enable Battery FET after BATT_TIMER_PERIOD expiration. */
    case BATT_CHG_ENA_BAT_FET:
        if(Cy_PdUtils_SwTimer_IsRunning(ptrPdStackContext->ptrTimerContext, BATT_TIMER_ID) == false)
        {
            DEBUG_PRINT("\r\n Battery is not at fault and ready to charge ");
            soln_batt_chge_cmd(ptrPdStackContext, BATT_CHGE_BB_OUT_EN);
#if PDL_VOUTBB_RCP_ENABLE
            gl_sln_batt_chg_state = BATT_CHG_ENA_RCP_DELAY;
#else
            gl_sln_batt_chg_state = BATT_CHG_CHARGE_LOOP;
#endif
#if VBUS_SNK_UVP_ENABLE
            /* In SNK mode BB block discharges disconnected VBUS too fast causing
             * false UVP triggering before true TypeC Detach detection.
             * So we need to disable SNK-mode VBUS UVP .
             * Its functionality anyway is duplicated by Brownout detector and Sink Detach detection procedure in TypeC machine
             * UVP will be still enabled in SRC mode.
             */

            if(ptrPdStackContext->dpmConfig.connect)
            {
                app_status_t* app_stat = app_get_status(ptrPdStackContext->port);
                app_uvp_enable(ptrPdStackContext, app_stat->psnk_volt, false, app_psnk_vbus_uvp_cbk);
                DEBUG_PRINT("UVP configured\n");
            }
#endif /* VBUS_UVP_ENABLE */
        }
        break;

#if PDL_VOUTBB_RCP_ENABLE
    /* Delay RCP Enable for 2 cycles of Charger FSM (400mS) to avoid if false triggering. */
    case BATT_CHG_ENA_RCP_DELAY:
        gl_sln_batt_chg_state = BATT_CHG_ENA_RCP_DELAY1;
        break;

    case BATT_CHG_ENA_RCP_DELAY1:
        Cy_USBPD_Fault_Voutbb_RcpEnable(ptrPdStackContext->ptrUsbPdContext, soln_voutbb_rcp_cbk);
        gl_sln_batt_chg_state = BATT_CHG_CHARGE_LOOP;
        break;
#endif /* PDL_VOUTBB_RCP_ENABLE */

    case BATT_CHG_CHARGE_LOOP:
        /* Run dynamic voltage tracking and current adjustment */
        if (batt_stat->cur_batt_charging_status == true)
        {
#if ENABLE_ALL_BATT_MONITORING
            uint16_t calc_batt_ip_volt = batt_stat->curr_batt_volt + TOTAL_VBATT_HYST_THRESH;
            uint16_t calc_batt_ip_curr = batt_stat->cur_bb_pwr / calc_batt_ip_volt;

            /* Limit available current */
            calc_batt_ip_curr = CY_USBPD_GET_MIN(calc_batt_ip_curr, batt_stat->batt_max_curr_rating);
            calc_batt_ip_curr = CY_USBPD_GET_MIN(calc_batt_ip_curr, VBAT_INPUT_CURR_MAX_SETTING);

            switch(gl_sln_batt_chg_alt_state)
            {
            case BATT_CHG_ALT_INIT_CHARGE:
#if PRE_CHARGE_TIMER_ENABLE
                charging_timeout_cmd(ptrPdStackContext, CHARGING_TIMEOUT_PRECHARGE_INIT);
#endif /* PRE_CHARGE_TIMER_ENABLE */
                batt_stat->cur_bb_vout = TOTAL_VBATT_MAX_ALLOWED_VOLT;
                if(batt_stat->curr_batt_volt >= TOTAL_VBATT_DISCHARGED_SNK)
                {
                    gl_sln_batt_chg_alt_state = BATT_CHG_ALT_CC_MODE;
                }
                break;

            case BATT_CHG_ALT_CC_MODE:
#if NORMAL_CHARGE_TIMER_ENABLE
                charging_timeout_cmd(ptrPdStackContext, CHARGING_TIMEOUT_NORMAL_INIT);
#endif /* NORMAL_CHARGE_TIMER_ENABLE */
                if(batt_stat->is_cell_full == true)
                {
                    uint16_t vbus_in_volt = Cy_USBPD_Adc_MeasureVbusIn(ptrPdStackContext->ptrUsbPdContext, CY_USBPD_ADC_ID_0, CY_USBPD_ADC_INPUT_AMUX_B);
                    DEBUG_PRINT_VAR("\n BB FULL measured:: %i mV", vbus_in_volt);
                    calc_batt_ip_volt = vbus_in_volt;
                    batt_stat->cur_bb_vout = calc_batt_ip_volt;
                    batt_stat->cv_mode_entered = true;
                    gl_sln_batt_chg_alt_state = BATT_CHG_ALT_CV_MODE;
                }
                else
                {
                    batt_stat->cur_bb_vout = batt_stat->curr_batt_volt + TOTAL_VBATT_HYST_THRESH;
                }
                break;

            case BATT_CHG_ALT_CV_MODE:
                if(batt_stat->curr_batt_curr < update_current_limit(ptrPdStackContext,STEP_IBAT_CHARGING_CURR))
                {
                    gl_sln_batt_chg_alt_state = BATT_CHG_ALT_CHARGE_FULL;
                }
                break;

            default:
                break;
            }

            switch(gl_sln_batt_chg_alt_state)
            {
            case BATT_CHG_ALT_INIT_CHARGE:
                calc_batt_ip_curr = update_current_limit(ptrPdStackContext,MIN_IBAT_CHARGING_CURR);
                break;

            case BATT_CHG_ALT_CC_MODE:
                /* set I max */
                break;

            case BATT_CHG_ALT_CHARGE_FULL:
                DEBUG_PRINT("\r\n >>>Battery is full BAT FET OFF");
                soln_batt_chgr_hw_disable(ptrPdStackContext);
#if NORMAL_CHARGE_TIMER_ENABLE
                charging_timeout_cmd(ptrPdStackContext, CHARGING_TIMEOUT_TIMER_STOP);
#endif /* NORMAL_CHARGE_TIMER_ENABLE */
                gl_sln_batt_chg_state = BATT_CHG_TYPEC_ATTACHED_SNK;
                batt_stat->curr_chrg_cycle_num ++;
                return;
            
            default:
                break;
            }

            if(gl_sln_batt_chg_alt_state == BATT_CHG_ALT_CC_MODE)
            {
                if(calc_batt_ip_curr > (batt_stat->cur_bb_iout + update_current_limit(ptrPdStackContext,STEP_IBAT_CHARGING_CURR)))
                {
                    uint16_t calc_batt_ip_curr_tmp = batt_stat->cur_bb_iout + update_current_limit(ptrPdStackContext,STEP_IBAT_CHARGING_CURR);
                    /* Check the difference between current value to be set and max. allowed output current */
                    if((calc_batt_ip_curr-calc_batt_ip_curr_tmp) >= update_current_limit(ptrPdStackContext,STEP_IBAT_CHARGING_CURR))
                    {
                        /* If it is not the last current increasing step then do this step */
                        calc_batt_ip_curr=calc_batt_ip_curr_tmp;
                    }
                    else
                    {
                        /* Else set max allowed current and stop current incrementing */
                    }
                }
#ifdef BATT_CURRENT_DECREASE_BY_STEPS
                else if(calc_batt_ip_curr < (batt_stat->cur_bb_iout - update_current_limit(ptrPdStackContext,STEP_IBAT_CHARGING_CURR)))
                {
                    calc_batt_ip_curr = batt_stat->cur_bb_iout - update_current_limit(ptrPdStackContext,STEP_IBAT_CHARGING_CURR);
                }
#endif
            }

#ifdef BATT_VOLTAGE_ADJ_HYSTERESYS_ENA
            /* Do not apply voltage changes less than 100mV */
            if(abs(batt_stat -> cur_bb_vout - calc_batt_ip_volt) >= 100)
#endif
            {
                DEBUG_PRINT("\r\n BAT V UPDATE ");
                sln_set_volt_nocv(ptrPdStackContext->ptrUsbPdContext, batt_stat->cur_bb_vout);
            }

            /* Wait for requested voltage to build up and settle. */
            /* Take in account minimal current adjustment step */
            if(abs(batt_stat->cur_bb_iout - calc_batt_ip_curr) >= update_current_limit(ptrPdStackContext,STEP_IBAT_CHARGING_CURR))
            {
                batt_stat->cur_bb_iout = calc_batt_ip_curr;
#if BAT_HW_OCP_ENABLE
                Cy_USBPD_Fault_Vbat_OcpDisable(ptrPdStackContext->ptrUsbPdContext, false);
#endif /* BAT_HW_OCP_ENABLE */

                /* Call modified CC loop enable procedure to set BAT OCP when IBTR completes. */
                Cy_USBPD_CF_Enable(ptrPdStackContext->ptrUsbPdContext, batt_stat->cur_bb_iout);

                DEBUG_PRINT("\r\n BAT I UPDATE ");
            }
            DEBUG_PRINT_VAR("\n \n \n BB OutCurr Setting:: %i mA", (batt_stat->cur_bb_iout * 10));
            DEBUG_PRINT_VAR("\n BB OutVolt Setting:: %i mV", batt_stat->cur_bb_vout);
#endif /* ENABLE_ALL_BATT_MONITORING  */
        }
        break;

    case BATT_CHG_END_OF_CHARGE:
        /* Loop here when battery is full. */
        break;

    default:
        break;
    }
}

#if VREG_BROWN_OUT_DET_ENABLE
/* Function using to control Brown Out buck-boost comparator and VDDD detector.
 * Shall be enable when VDDD is already 5V and disabled when VDDD=3V.
 */
void sol_brown_out_control(cy_stc_pdstack_context_t * context, bool enable)
{
    PPDSS_REGS_T pd = context->ptrUsbPdContext->base;
    if(enable)
    {
        uint16_t vddd;
        pd->bbctrl_func_ctrl3 &= ~PDSS_BBCTRL_FUNC_CTRL3_BBCTRL_FAULT_DET_DIS_VDDD_BOD;
        pd->bb_40vreg_ctrl |= PDSS_BB_40VREG_CTRL_BB_40VREG_EN_VDDD_DET_COMP;
        Cy_USBPD_Fault_BrownOutDetEn(context->ptrUsbPdContext);
        Cy_SysLib_DelayUs(10);

        Cy_USBPD_Adc_SelectVref(context->ptrUsbPdContext, APP_GPIO_POLL_ADC_ID, CY_USBPD_ADC_VREF_VDDD);
        vddd = Cy_USBPD_Adc_Calibrate(context->ptrUsbPdContext, APP_GPIO_POLL_ADC_ID);
        if(vddd < 3500)
        {
            /* In this place VBUS is already 5V, but VDDD is less 3.5V.
             * This means something is loading VDDD and this event was missed
             * during BrownOut detection.
             */
            pd->intr17_set |= PDSS_INTR17_SET_PDBB_VREG_VDDD_BOD;
        }
    }
    else
    {
        Cy_USBPD_Fault_BrownOutDetDis(context->ptrUsbPdContext);
        pd->bb_40vreg_ctrl &= ~PDSS_BB_40VREG_CTRL_BB_40VREG_EN_VDDD_DET_COMP;
        pd->bbctrl_func_ctrl3 |= PDSS_BBCTRL_FUNC_CTRL3_BBCTRL_FAULT_DET_DIS_VDDD_BOD;
    }
}
#endif /* VREG_BROWN_OUT_DET_ENABLE*/

static void sln_ibtr_cb(void * callbackCtx, bool value)
{
    (void)value;
#if BAT_HW_OCP_ENABLE
    cy_stc_usbpd_context_t *ptrUsbPdContext = (cy_stc_usbpd_context_t *)callbackCtx;
    cy_stc_pdstack_context_t * ptrPdStackContext = (cy_stc_pdstack_context_t *)ptrUsbPdContext->pdStackContext;
    cy_stc_battery_charging_context_t* ptrBatteryChargingContext = get_battery_charging_context(ptrPdStackContext->port);
    cy_stc_battery_status_t* batt_stat = &(ptrBatteryChargingContext->batteryStatus);

    /* Set OCP level to 1A for low output current, because of high current monitor error when I < 1A */
    if(batt_stat->cur_bb_iout <= CY_USBPD_I_1A)
    {
        Cy_USBPD_Fault_Vbat_OcpEnable(ptrUsbPdContext, CY_USBPD_I_1A, soln_vbat_ocp_cbk);
    }
    else
    {
        Cy_USBPD_Fault_Vbat_OcpEnable(ptrUsbPdContext, batt_stat->cur_bb_iout, soln_vbat_ocp_cbk);
    }
#else
    (void)callbackCtx;
#endif /* BAT_HW_OCP_ENABLE */
}
/* [] END OF FILE */
