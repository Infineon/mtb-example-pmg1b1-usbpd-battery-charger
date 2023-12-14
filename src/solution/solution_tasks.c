/**
 * @file solution_tasks.c
 *
 * @brief @{Solution source file solution layer related tasks.@}
 *
 *******************************************************************************
 * Copyright 2021-2023, Cypress Semiconductor Corporation (an Infineon company) or
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
#include "cy_gpio.h"

#include "config.h"
#include "thermistor.h"

#if DEBUG_UART_ENABLE
#include "debug.h"
//char temp[80];
#endif

cy_stc_battery_charging_context_t battery_charging_p0_ctx;

extern batt_chg_state_t gl_sln_batt_chg_state;

cy_stc_battery_charging_context_t * battery_charging_contexts[NO_OF_TYPEC_PORTS] =
{
    &battery_charging_p0_ctx
};

cy_stc_battery_charging_context_t * get_battery_charging_context(uint8_t portIdx)
{
    return (battery_charging_contexts[portIdx]);
}

/* ADC reference voltage for ID Pin and Thermistor measurements */
#define INPUT_ADC_VDDD_REF_VOLT                     (5000u)

/* Configure the SAR interrupt. */
const cy_stc_sysint_t SAR0_IrqConfig =
{
    .intrSrc = (IRQn_Type)SAR0_IRQ,
    .intrPriority = 0UL,
};

static bool sar_is_scanning = false;

extern cy_stc_pdstack_context_t * get_pdstack_context(uint8_t portIdx);

void battery_measure_intr_handler(void)
{
    cy_stc_pdstack_context_t* ptrPdStackContext = get_pdstack_context(0);
    cy_stc_battery_charging_context_t* ptrBatteryContext = get_battery_charging_context(ptrPdStackContext->port);
    cy_stc_battery_measure_t* adcResult = &(ptrBatteryContext->batteryMeasure);

    uint32_t intrMask = Cy_SAR_GetInterruptStatusMasked(SAR0);
    if (0UL != (CY_SAR_INTR_EOS & intrMask))
    {
        /* addition multiply by 1000 in VREF definition for result in mV */
        adcResult->cell1 = (uint32_t)((R_DIV_0) * Cy_SAR_GetResult16(SAR0, 4)) / 4096;
        adcResult->cell2 = (uint32_t)((R_DIV_1) * Cy_SAR_GetResult16(SAR0, 3)) / 4096;
        adcResult->cell3 = (uint32_t)((R_DIV_2) * Cy_SAR_GetResult16(SAR0, 2)) / 4096;
        adcResult->cell4 = (uint32_t)((R_DIV_3) * Cy_SAR_GetResult16(SAR0, 1)) / 4096;
        adcResult->vbat = (uint32_t)((R_DIV_4) * Cy_SAR_GetResult16(SAR0, 0)) / 4096;

#if ENABLE_BATT_TEMP_MONITORING
        adcResult->bat_th = (uint32_t)(2 * SAR0_VREF_MV * Cy_SAR_GetResult16(SAR0, 5)) / 4096;
#endif
    }
    Cy_SAR_ClearInterrupt(SAR0_HW, intrMask);
    Cy_SAR_Disable(SAR0_HW);

    ptrBatteryContext->batteryStatus.adc_pending = false;
    sar_is_scanning = false;
}

void battery_measure_sar_adc_init(cy_stc_pdstack_context_t* ptrPdStackContext)
{
    if (CY_SAR_SUCCESS != Cy_SAR_Init(SAR0_HW, &SAR0_config))
    {
        /* insert error handling here */
    }

    /* Configure the interrupt with vector at SAR0_Isr(). */
    if (CY_SYSINT_SUCCESS != Cy_SysInt_Init(&SAR0_IrqConfig, battery_measure_intr_handler))
    {
        /* insert error handling here */
    }

    /* Enable the interrupt. */
    NVIC_EnableIRQ(SAR0_IrqConfig.intrSrc);

    /* Clear possible interrupt erroneously raised during block enabling */
    Cy_SAR_ClearInterrupt(SAR0_HW, CY_SAR_INTR);
    Cy_SAR_SetInterruptMask(SAR0_HW, CY_SAR_INTR_EOS);
}

#if (ENABLE_NTCP0_TEMP_MONITORING || ENABLE_NTCP1_TEMP_MONITORING)
uint16_t get_adc_8bit(cy_stc_pdstack_context_t* ptrPdStackContext, uint32_t port, uint32_t pin)
{
    uint8_t level;
    uint16_t volt;
    uint8_t state = Cy_SysLib_EnterCriticalSection();

    Cy_GPIO_SetHSIOM(Cy_GPIO_PortToAddr(port), pin, HSIOM_SEL_AMUXA);
    Cy_SysLib_DelayUs(20);

    /* Take ADC sample. */
    level = Cy_USBPD_Adc_Sample(ptrPdStackContext->ptrUsbPdContext, APP_GPIO_POLL_ADC_ID, APP_GPIO_POLL_ADC_INPUT);
    volt = ((uint32_t)(level * INPUT_ADC_VDDD_REF_VOLT) / PD_ADC_NUM_LEVELS);

    Cy_GPIO_SetHSIOM(Cy_GPIO_PortToAddr(port), pin, HSIOM_SEL_GPIO);
    Cy_SysLib_DelayUs(10);
    Cy_SysLib_ExitCriticalSection(state);
    return volt;
}
#endif /* ENABLE_NTCP0_TEMP_MONITORING || ENABLE_NTCP1_TEMP_MONITORING */

void battery_measure_sar_adc_start_cb (
        cy_timer_id_t id,           /**< Timer ID for which callback is being generated. */
        void *callbackContext)       /**< Timer module Context. */
{
    (void)callbackContext;
    (void)id;

    cy_stc_pdstack_context_t* ptrPdStackContext = (cy_stc_pdstack_context_t*) callbackContext;
    cy_stc_battery_charging_context_t* ptrBatteryContext = get_battery_charging_context(ptrPdStackContext->port);
    cy_stc_battery_measure_t* adcResult = &(ptrBatteryContext->batteryMeasure);

    /* NTCs measurement based on 8-bit SAR */
#if ENABLE_NTCP0_TEMP_MONITORING
    adcResult->ntcp0 = get_adc_8bit(ptrPdStackContext, NTCP0_PORT_NUM, NTCP0_PIN);
#endif /* ENABLE_NTCP0_TEMP_MONITORING */

#if ENABLE_NTCP1_TEMP_MONITORING
    adcResult->ntcp1 = get_adc_8bit(ptrPdStackContext, NTCP1_PORT_NUM, NTCP1_PIN);
#endif /* ENABLE_NTCP1_TEMP_MONITORING */
    Cy_SAR_Enable(SAR0_HW);
    sar_is_scanning = true;
    Cy_SAR_StartConvert(SAR0_HW, CY_SAR_START_CONVERT_SINGLE_SHOT);
}

void battery_measure_sar_adc_scan(cy_stc_pdstack_context_t* ptrPdStackContext)
{
    /* enable ADC_EN pin set to High. */
    /* Need some delay.Not nesessary for PMG1_B1_DRP board as it does not have ADC_EN pin. */
    Cy_PdUtils_SwTimer_Start(ptrPdStackContext->ptrTimerContext, ptrPdStackContext,
            TIMER_MEASURE_DELAY_ADC_EN_DELAY, TIMER_MEASURE_DELAY_ADC_EN_DELAY_PERIOD, &battery_measure_sar_adc_start_cb);
}

bool battery_measure_sar_is_active(void)
{
    return sar_is_scanning;
}

void reset_battery_status(cy_stc_battery_charging_context_t *ptrBatteryChargingContext)
{
    cy_stc_battery_status_t* batt_stat = &(ptrBatteryChargingContext->batteryStatus);

    batt_stat->curr_chrg_cycle_num = 1;
    batt_stat->cur_batt_charging_status = false;
    batt_stat->adc_pending = false;
}

void clear_flags_on_usbc_disconnect (cy_stc_battery_charging_context_t *ptrBatteryChargingContext)
{
    cy_stc_battery_status_t* batt_stat = &(ptrBatteryChargingContext->batteryStatus);
    batt_stat->curr_chrg_cycle_num = 1;
    batt_stat->cur_bb_pwr = 0;
    batt_stat->cur_bb_iout = MIN_IBAT_CHARGING_CURR;
    batt_stat->cur_bb_vout = TOTAL_VBATT_MAX_ALLOWED_VOLT;
    batt_stat->is_vbus_pwr_sufficient = false;
    batt_stat->batt_ocp_fault_active = false;
    batt_stat->batt_rcp_fault_active = false;
    batt_stat->batt_ovp_fault_active = false;
    batt_stat->batt_uvp_fault_active = false;
}

void reset_battery_faults(cy_stc_battery_charging_context_t *ptrBatteryChargingContext)
{
    cy_stc_battery_status_t* batt_stat = &(ptrBatteryChargingContext->batteryStatus);

    batt_stat->batt_uvp_fault_active = false;
    batt_stat->batt_ovp_fault_active = false;
    batt_stat->batt_ocp_fault_active = false;
    batt_stat->batt_rcp_fault_active = false;
    batt_stat->batt_otp_fault_active = false;
}

#if PRINT_CV
bool print_flag = false;
void print_cb (
        cy_timer_id_t id,           /**< Timer ID for which callback is being generated. */
        void *callbackContext)       /**< Timer module Context. */
{
    cy_stc_pdstack_context_t* ptrPdStackContext = (cy_stc_pdstack_context_t*) callbackContext;
    print_flag = true;
    Cy_PdUtils_SwTimer_Start(ptrPdStackContext->ptrTimerContext, ptrPdStackContext, MY_ID, MY_PERIOD, &print_cb);
}
#endif /* PRINT_CV */

void run_battery_volt_monitor_task (cy_stc_battery_charging_context_t *ptrBatteryChargingContext)
{
    cy_stc_battery_status_t* batt_stat = &(ptrBatteryChargingContext->batteryStatus);
    cy_stc_battery_measure_t* adcResult = &(ptrBatteryChargingContext->batteryMeasure);
#if (CELL_MONITORING_DISABLE == 0)
    uint8_t cell_num = 0;
    uint16_t curr_cell_volt = 0;
    bool battery_removed = true;
#endif /* (CELL_MONITORING_DISABLE == 0) */
    uint16_t total_batt_volt = 0;
    bool is_cell_chrg_full = false;
    bool is_cell_recharge = true;
#if (CELL_MONITORING_DISABLE == 0)
    bool is_batt_in_ovp = false;
    bool is_batt_in_uvp = false;
#endif /* (CELL_MONITORING_DISABLE == 0) */

    /* Monitor total battery voltage */

    /* for test w/o battery */
#if BATTERY_SIMULATOR_ENABLE
    total_batt_volt = BATTERY_SIMULATOR_VOLTAGE;
#else
    total_batt_volt = adcResult->vbat;
#endif
    /* Update the current battery voltage */
    batt_stat->curr_batt_volt = total_batt_volt;

#if PRINT_CV
uint16_t cell[5];
#endif /* PRINT_CV */

/* Measure individual cell voltages for OVP and UVP only if the Battery Pack ID indicates battery presence */
#if DEBUG_BATT_INFO_ENABLE
    sprintf(temp, "\nTotal measured battery voltage is: %i mV \r\n", total_batt_volt);
    debug_print( temp);
#endif
#if (CELL_MONITORING_DISABLE == 0)
    /* Monitor each cell voltage and if any cell voltage is higher than 4.2V disabled Charging */
    for(cell_num = 0; cell_num < TOTAL_BATTERY_CELL_COUNT; cell_num++ )
    {
        switch(cell_num)
        {
            case 0: curr_cell_volt = adcResult->cell1;
#if PRINT_CV
            cell[0]=curr_cell_volt;
#endif /* PRINT_CV */
                break;
            case 1: curr_cell_volt = adcResult->cell2 - adcResult->cell1;
#if PRINT_CV
            cell[1]=curr_cell_volt;
#endif /* PRINT_CV */
                break;
            case 2: curr_cell_volt = adcResult->cell3 - adcResult->cell2;
#if PRINT_CV
            cell[2]=curr_cell_volt;
#endif /* PRINT_CV */

#if SIMULATE_ERROR_SNK
            if(gl_sln_batt_chg_state == BATT_CHG_CHARGE_LOOP)
            {
                if(Cy_PdUtils_SwTimer_IsRunning(PdStackContext->ptrTimerContext, BATT_TIMER_ID) == false)
                {
                    curr_cell_volt = SIMULATE_CELL_VOLT;
                }
            }
#endif
                break;
            case 3: curr_cell_volt = adcResult->cell4 - adcResult->cell3;
#if PRINT_CV
            cell[3]=curr_cell_volt;
#endif /* PRINT_CV */
                break;
            case 4: curr_cell_volt = adcResult->vbat - adcResult->cell4;
#if PRINT_CV
            cell[4]=curr_cell_volt;
#endif /* PRINT_CV */
                break;
            default:
                break;
        }
#if DEBUG_UART_ENABLE && DEBUG_BATT_INFO_ENABLE && DEBUG_BATT_CELL_INFO_ENABLE
        sprintf(temp, "\n Cell %d voltage is: %i mV",cell_num, curr_cell_volt);
        debug_print( temp);
#endif

        if(curr_cell_volt > BATTERY_REMOVAL_THRESHOLD)
        {
            battery_removed = false;
        }

        /* Deny recharging if at least one cell exceeds threshold */
        if(curr_cell_volt > BATT_PER_CELL_RECHARGE_VOLT)
        {
            is_cell_recharge = false;
        }

        /* check if separate cell is in full charge in CV mode */
        if(curr_cell_volt > BATT_PER_CELL_ALLOWED_MAX_VOLT)
        {
            /* Cell voltage is more than 4.2V , so battery cell is fully charged */
            is_cell_chrg_full = true;
        }

        if(curr_cell_volt >= BATT_PER_CELL_OVP_THRESHOLD)
        {
            /* Cell voltage is more than 4.3V . Battery is in OVP */
            is_batt_in_ovp = true;
        }

        /* SNK case */
        if(curr_cell_volt < BATT_PER_CELL_ALLOWED_MIN_VOLT)
        {
            /* Cell voltage is less 3V , so cell is in UVP */
            is_batt_in_uvp = true;
        }
    }
#endif /* (CELL_MONITORING_DISABLE == 0) */

#if PRINT_CV
    if(print_flag)
    {
        print_flag = false;
        uint16_t vbus_in_volt = Cy_USBPD_Adc_MeasureVbusIn(PdStackContext->ptrUsbPdContext, CY_USBPD_ADC_ID_0, CY_USBPD_ADC_INPUT_AMUX_B);
        sprintf(temp, "%i %i %i %i %i %i %i %i\n",cell[0], cell[1], cell[2], cell[3], cell[4], total_batt_volt, vbus_in_volt, batt_stat->curr_batt_curr *10);
        debug_print(temp);
    }
#endif /* PRINT_CV */

    if(total_batt_volt <= BATTERY_REMOVAL_THRESHOLD)
    {
        batt_stat->batt_pack_type = NO_BATTERY;
        return;
    }
    else
    {
        batt_stat->batt_pack_type = BATTERY_PRESENT;
    }

#if (CELL_MONITORING_DISABLE == 0)
    if(is_batt_in_ovp == true)
    {
        /* Update the OVP flag */
        batt_stat->batt_ovp_fault_active = true;
        DEBUG_PRINT("\n Battery is in OVP");
    }
    else
    {
        /* Here can be OVP flag clear. */
    }

    if(is_batt_in_uvp == true)
    {
        DEBUG_PRINT("\n Battery is in UVP");
        batt_stat->batt_uvp_fault_active = true;
    }
    else
    {
        batt_stat->batt_uvp_fault_active = false;
    }
#endif /* (CELL_MONITORING_DISABLE == 0) */

    if(total_batt_volt >= TOTAL_VBATT_MAX_ALLOWED_VOLT)
    {
        is_cell_chrg_full = true;
    }

    /* check if cell battery is in full charge */
    batt_stat->is_cell_full = is_cell_chrg_full;
    batt_stat->is_cell_recharge = is_cell_recharge;
}

void run_battery_curr_monitor_task (cy_stc_battery_charging_context_t *ptrBatteryChargingContext)
{
    cy_stc_battery_status_t* batt_stat = &(ptrBatteryChargingContext->batteryStatus);
    cy_stc_pdstack_context_t * PdStackContext = ptrBatteryChargingContext->ptrPdStack;
    uint32_t batt_cur_ocp_limit = batt_stat->cur_bb_iout;

    batt_stat->curr_batt_curr = Cy_USBPD_Hal_MeasureCur(PdStackContext->ptrUsbPdContext);
    DEBUG_PRINT_VAR("\n Measured IBAT: %i mA", batt_stat->curr_batt_curr * 10);

    batt_cur_ocp_limit = PRIMARY_IBATT_OCP_CURR_THRESHOLD;

#if DEBUG_BATT_INFO_ENABLE
    DEBUG_PRINT_VAR("\n BatOCP set to: %lu mA", (batt_cur_ocp_limit * 10));
#endif

#if BATT_PRI_OCP_ENABLE
    if(batt_stat->curr_batt_curr > batt_cur_ocp_limit)
    {
        batt_stat->batt_ocp_fault_active = true;
        DEBUG_PRINT("\n Battery OCP is active");
    }
    else
    {
        batt_stat->batt_ocp_fault_active = false;
    }
#endif /* BATT_PRI_OCP_ENABLE */
}

void soln_battery_charger_task(cy_stc_battery_charging_context_t *ptrBatteryChargingContext)
{
    cy_stc_battery_status_t* batt_stat = &(ptrBatteryChargingContext->batteryStatus);
    batt_stat->batt_max_curr_rating = VBAT_INPUT_CURR_MAX_SETTING;
    batt_stat->batt_pack_type = BATTERY_PRESENT;

#if ENABLE_BATT_CURR_MONITORING    
    /* Monitor current and do compensation only in SNK/charging mode */
    if(
        ((gl_sln_batt_chg_state >= BATT_CHG_BB_SET_VBAT) &&
        (gl_sln_batt_chg_state <= BATT_CHG_CHARGE_LOOP))
        &&
        (batt_stat->is_vbus_pwr_sufficient == true) &&
        (batt_stat->batt_ocp_fault_active == false) )
    {
        run_battery_curr_monitor_task(ptrBatteryChargingContext);
    }
#endif /* ENABLE_BATT_CURR_MONITORING */

#if ENABLE_ALL_BATT_MONITORING
    run_battery_volt_monitor_task(ptrBatteryChargingContext);
#endif /* ENABLE_ALL_BATT_MONITORING */

#if ENABLE_BATT_TEMP_MONITORING
    if(batt_stat->batt_pack_type != NO_BATTERY)
    {
        measure_temp_sensor_data(ptrBatteryChargingContext);
    }
#endif /* ENABLE_BATT_TEMP_MONITORING */
#if ENABLE_NTCP0_TEMP_MONITORING
    measure_onboard_ntcp0_sensor_data(ptrBatteryChargingContext);
#endif
#if ENABLE_NTCP1_TEMP_MONITORING
    measure_onboard_ntcp1_sensor_data(ptrBatteryChargingContext);
#endif
}

void solution_init (cy_stc_pdstack_context_t *ptrPdStackContext)
{
    cy_stc_battery_charging_context_t * ptrBatteryContext = get_battery_charging_context(ptrPdStackContext->port);
    ptrBatteryContext->ptrPdStack = ptrPdStackContext;

    reset_battery_status(ptrBatteryContext);
}

/* Thermistor resistance array */
#define TEMP_MAP_MIN                 (-25)
#define TEMP_MAP_MAX                 (70)
#define TEMP_MAP_RESOLUTION_BATT     (5)
#define TEMP_MAP_TABLE_COUNT         (20)
const uint32_t gl_res_temp_map[TEMP_MAP_TABLE_COUNT] =
{
    87717, /* -25C */
    68424, /* -20C */
    53752, /* -15C */
    42558, /* -10C */
    33963, /* -5C  */
    27305, /*  0C  */
    22097, /*  5C  */
    17985, /*  10C */
    14716, /*  15C */
    12101, /*  20C */
    10000, /*  25C */
    8305,  /*  30C */
    6932,  /*  35C */
    5815,  /*  40C */
    4902,  /*  45C */
    4152,  /*  50C */
    3533,  /*  55C */
    3019,  /*  60C */
    2589,  /*  65C */
    2229   /*  70C */
};

/* Sum of resistors according to schematic 1k+6,65k */
#define R240    (7650u)
#define R242    (953u)

int8_t get_bat_temperature(uint16_t battery_voltage, uint16_t therm_volt)
{
    /* calculate thermistor resistance */
    uint32_t therm_res = (uint32_t)(R242 * battery_voltage) / therm_volt - R242 - R240;

    /* calculate temperature */
    if(therm_res > gl_res_temp_map[0])
    {
        return TEMP_MAP_MIN;
    }
    if(therm_res < gl_res_temp_map[TEMP_MAP_TABLE_COUNT - 1])
    {
        return TEMP_MAP_MAX;
    }
    for(uint8_t i = 1; i < TEMP_MAP_TABLE_COUNT; i++)
    {
        if(therm_res >= gl_res_temp_map[i])
        {
            uint32_t resolution = (gl_res_temp_map[i - 1] - gl_res_temp_map[i]);
            uint32_t diff = (therm_res - gl_res_temp_map[i]) * TEMP_MAP_RESOLUTION_BATT;
            return (TEMP_MAP_MIN + TEMP_MAP_RESOLUTION_BATT * i - diff / resolution);
        }
    }
    return TEMP_MAP_MAX;
}

void measure_onboard_ntcp0_sensor_data(cy_stc_battery_charging_context_t *ptrBatteryChargingContext)
{
    cy_stc_battery_status_t* batt_stat = &(ptrBatteryChargingContext->batteryStatus);
    cy_stc_battery_measure_t* adcResult = &(ptrBatteryChargingContext->batteryMeasure);
    static temp_state_t ntcp0_state = STATE_TEMP_NORMAL_CHARGE;

    int8_t ntcp0_temp = ccg_volt_temp_map(ptrBatteryChargingContext->ptrPdStack, adcResult->ntcp0);

#if DEBUG_TEMP_INFO_ENABLE
    sprintf(temp, "\n NTCP0 Temp: %i C", ntcp0_temp);
    debug_print(temp);
#endif

    switch(ntcp0_state)
    {
        case STATE_TEMP_NORMAL_CHARGE:
            if(ntcp0_temp >= PRIMARY_NTCP0_HOT_THRESHOLD)
            {
                ntcp0_state = STATE_TEMP_LIMITED_CHARGE;
            }
            break;
        case STATE_TEMP_LIMITED_CHARGE:
            if(ntcp0_temp >= PRIMARY_NTCP0_OTP_THRESHOLD)
            {
                ntcp0_state = STATE_TEMP_OTP;
            }
            if(ntcp0_temp < (PRIMARY_NTCP0_HOT_THRESHOLD - PRIMARY_NTCP0_HYSTERESIS))
            {
                ntcp0_state = STATE_TEMP_NORMAL_CHARGE;
            }
            break;
        case STATE_TEMP_OTP:
            if(ntcp0_temp < (PRIMARY_NTCP0_OTP_THRESHOLD - PRIMARY_NTCP0_HYSTERESIS))
            {
                ntcp0_state = STATE_TEMP_LIMITED_CHARGE;
            }
            break;
        default:
            break;
    }
    /* set OTP and current limit based on the new state */
    switch(ntcp0_state)
    {
        case STATE_TEMP_LIMITED_CHARGE:
            batt_stat->ntcp0_otp_fault_active = false;
            batt_stat->batt_max_curr_rating = CY_USBPD_GET_MIN(batt_stat->batt_max_curr_rating / 2, CY_USBPD_I_2A);
#if DEBUG_UART_ENABLE
            sprintf(temp, "NTCP0 limits current to %d\n", batt_stat->batt_max_curr_rating * 10);
            debug_print(temp);
#endif
            break;
        case STATE_TEMP_NORMAL_CHARGE:
            batt_stat->ntcp0_otp_fault_active = false;
            break;
        case STATE_TEMP_OTP:
            batt_stat->ntcp0_otp_fault_active = true;
#if DEBUG_UART_ENABLE
            debug_print("NTCP0 OTP\r\n");
#endif
            break;
        default:
            break;
    }
}

void measure_onboard_ntcp1_sensor_data(cy_stc_battery_charging_context_t *ptrBatteryChargingContext)
{
    cy_stc_battery_status_t* batt_stat = &(ptrBatteryChargingContext->batteryStatus);
    cy_stc_battery_measure_t* adcResult = &(ptrBatteryChargingContext->batteryMeasure);
    static temp_state_t ntcp1_state = STATE_TEMP_NORMAL_CHARGE;

    int8_t ntcp1_temp = ccg_volt_temp_map(ptrBatteryChargingContext->ptrPdStack, adcResult->ntcp1);

#if DEBUG_TEMP_INFO_ENABLE
    sprintf(temp, "\n NTCP1 Temp: %i C", ntcp1_temp);
    debug_print(temp);
#endif

    switch(ntcp1_state)
    {
        case STATE_TEMP_NORMAL_CHARGE:
            if(ntcp1_temp >= PRIMARY_NTCP1_HOT_THRESHOLD)
            {
                ntcp1_state = STATE_TEMP_LIMITED_CHARGE;
            }
            break;
        case STATE_TEMP_LIMITED_CHARGE:
            if(ntcp1_temp >= PRIMARY_NTCP1_OTP_THRESHOLD)
            {
                ntcp1_state = STATE_TEMP_OTP;
            }
            if(ntcp1_temp < (PRIMARY_NTCP1_HOT_THRESHOLD - PRIMARY_NTCP1_HYSTERESIS))
            {
                ntcp1_state = STATE_TEMP_NORMAL_CHARGE;
            }
            break;
        case STATE_TEMP_OTP:
            if(ntcp1_temp < (PRIMARY_NTCP1_OTP_THRESHOLD - PRIMARY_NTCP1_HYSTERESIS))
            {
                ntcp1_state = STATE_TEMP_LIMITED_CHARGE;
            }
            break;
        default:
            break;
    }
    /* set OTP and current limit based on the new state */
    switch(ntcp1_state)
    {
        case STATE_TEMP_LIMITED_CHARGE:
            batt_stat->ntcp1_otp_fault_active = false;
            batt_stat->batt_max_curr_rating = CY_USBPD_GET_MIN(batt_stat->batt_max_curr_rating / 2, CY_USBPD_I_2A);
#if DEBUG_UART_ENABLE
            sprintf(temp, "NTCP1 limits current to %d\n", batt_stat->batt_max_curr_rating * 10);
            debug_print(temp);
#endif
            break;
        case STATE_TEMP_NORMAL_CHARGE:
            batt_stat->ntcp1_otp_fault_active = false;
            break;
        case STATE_TEMP_OTP:
            batt_stat->ntcp1_otp_fault_active = true;
#if DEBUG_UART_ENABLE
            debug_print("NTCP1 OTP\r\n");
#endif
            break;
        default:
            break;
    }
}

void measure_temp_sensor_data(cy_stc_battery_charging_context_t *ptrBatteryChargingContext)
{
    cy_stc_battery_status_t* batt_stat = &(ptrBatteryChargingContext->batteryStatus);
    cy_stc_battery_measure_t* adcResult = &(ptrBatteryChargingContext->batteryMeasure);
    static temp_state_t pack_state = STATE_TEMP_NORMAL_CHARGE;

    int8_t pack_temp = get_bat_temperature(adcResult->vbat, adcResult->bat_th);

#if DEBUG_TEMP_INFO_ENABLE
    sprintf(temp, "\n Batt Temp: %i C", pack_temp);
    debug_print(temp);
#endif

    switch(pack_state)
    {
        case STATE_TEMP_DISABLE_CHARGE_COLD:
            if(pack_temp >= PRIMARY_BATT_COLD_THRESHOLD)
            {
                pack_state = STATE_TEMP_LIMITED_CHARGE;
            }
            break;
        case STATE_TEMP_LIMITED_CHARGE:
            if(pack_temp < (PRIMARY_BATT_COLD_THRESHOLD - PRIMARY_BATT_HYSTERESIS))
            {
                pack_state = STATE_TEMP_DISABLE_CHARGE_COLD;
            }
            if(pack_temp >= PRIMARY_BATT_ROOM_THRESHOLD)
            {
                pack_state = STATE_TEMP_NORMAL_CHARGE;
            }
            break;
        case STATE_TEMP_NORMAL_CHARGE:
            if(pack_temp < (PRIMARY_BATT_ROOM_THRESHOLD - PRIMARY_BATT_HYSTERESIS))
            {
                pack_state = STATE_TEMP_LIMITED_CHARGE;
            }
            if(pack_temp >= PRIMARY_BATT_OTP_THRESHOLD)
            {
                pack_state = STATE_TEMP_OTP;
            }
            break;
        case STATE_TEMP_OTP:
            if(pack_temp < (PRIMARY_BATT_OTP_THRESHOLD - PRIMARY_BATT_HYSTERESIS))
            {
                pack_state = STATE_TEMP_NORMAL_CHARGE;
            }
            break;
        default:
            break;
    }

    /* set OTP and current limit based on the new state */
    switch(pack_state)
    {
        case STATE_TEMP_DISABLE_CHARGE_COLD:
            batt_stat->batt_otp_fault_active = true;
#if DEBUG_UART_ENABLE
            debug_print("Battery cold\r\n");
#endif
            break;
        case STATE_TEMP_LIMITED_CHARGE:
            batt_stat->batt_otp_fault_active = false;
            if(batt_stat->batt_max_curr_rating > CY_USBPD_I_1A)
            {
                batt_stat->batt_max_curr_rating = CY_USBPD_I_1A;
            }
#if DEBUG_UART_ENABLE
            debug_print("Pack temp limits current to 1A\r\n");
#endif
            break;
        case STATE_TEMP_NORMAL_CHARGE:
            batt_stat->batt_otp_fault_active = false;
            break;
        case STATE_TEMP_OTP:
            batt_stat->batt_otp_fault_active = true;
#if DEBUG_UART_ENABLE
            debug_print("Battery OTP\r\n");
#endif
            break;
        default:
            break;
    }
}

/* [] END OF FILE */
