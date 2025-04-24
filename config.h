/******************************************************************************
* File Name: config.h
*
* Description: This header file defines the application configuration for the PMG1B1
*              MCU USBPD Charger Example for ModusToolBox.
*
* Related Document: See README.md
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
#ifndef _CONFIG_H_
#define _CONFIG_H_

#include "cybsp.h"
#include "cy_pdutils_sw_timer.h"
#include "timer_id.h"
#include "solution.h"

/* P3.0 used as Debug UART,TX. J7.7 shall be connected to J3.8  */
#define DEBUG_UART_ENABLE                           (0u)

#if DEBUG_UART_ENABLE
#include <stdio.h>
extern char temp[];
#endif

/* Enable/Disable firmware active LED operation */
#define APP_FW_LED_ENABLE                           (1u)

#if DEBUG_UART_ENABLE

#define DEBUG_BATT_INFO_ENABLE                      (1u)
#define DEBUG_BATT_CELL_INFO_ENABLE                 (0u)
#define DEBUG_PWR_INFO_ENABLE                       (0u)
#define DEBUG_TEMP_INFO_ENABLE                      (1u)
#define SIMULATE_ERROR                              (0u)
#define DEBUG_PRINT(string)                         \
{                                                   \
    sprintf(temp,string);                           \
    debug_print(temp);                              \
}
#define DEBUG_PRINT_VAR(string,value)               \
{                                                   \
    sprintf(temp,string,value);                     \
    debug_print(temp);                              \
}

#else
#define DEBUG_BATT_INFO_ENABLE                      (0u)
#define DEBUG_BATT_CELL_INFO_ENABLE                 (0u)
#define DEBUG_PWR_INFO_ENABLE                       (0u)
#define DEBUG_TEMP_INFO_ENABLE                      (0u)
#define DEBUG_PRINT(string)
#define DEBUG_PRINT_VAR(string,value)
#endif

/* Always disable individual cell voltage calculation
 * and protection checks in BATTERY_SIMULATOR mode.
 */
#if BATTERY_SIMULATOR_ENABLE 
#define CELL_MONITORING_DISABLE                     (1u)
#else
/* Disable individual cell voltage calculation
 * and protection checks by default.
 * Set this to 0 if cell voltage monitoring is need 
 * and appropriate cable is connected.
 */
#define CELL_MONITORING_DISABLE                     (1u)
#endif

#if SIMULATE_ERROR
#define SIMULATE_CELL_VOLT                          (5000)
#define SIMULATE_ERROR_SRC                          (0u)
#define SIMULATE_ERROR_SNK                          (1u)
#endif

#define VBUS_SNK_UVP_ENABLE                         (0u)
#define BAT_HW_UVP_ENABLE                           (1u)
#define BAT_HW_OVP_ENABLE                           (1u)
#define BAT_HW_OCP_ENABLE                           (1u)

/* Enable trickle charge */
#define TRICKLE_CHARGE_ENABLE                       (0u)

/* Enable trickle charge timer */
#define TRICKLE_CHARGE_TIMER_ENABLE                 (0u)
/* Trickle charge timeout in minutes. */
#define TRICKLE_CHARGE_TIMER_VALUE                  (60u)

/* Enable pre-charge timer */
#define PRE_CHARGE_TIMER_ENABLE                     (1u)
/* Pre-charge timeout in minutes. */
#define PRE_CHARGE_TIMER_VALUE                      (90u)

/* Enable normal charge timer CC + CV */
#define NORMAL_CHARGE_TIMER_ENABLE                  (1u)
/* Normal charge timeout in minutes. */
#define NORMAL_CHARGE_TIMER_VALUE                   (6 * 60u)

/* This macro enables /disables all the battery monitoring code */
#define ENABLE_ALL_BATT_MONITORING                  (1u)

/* This macro enables /disables the battery current monitoring code */
#define ENABLE_BATT_CURR_MONITORING                 (1u)

/* Enable battery temperature monitoring */
#define ENABLE_BATT_TEMP_MONITORING                 (0u)

/* Enable board temperature monitoring */
#define ENABLE_NTCP0_TEMP_MONITORING                (1u)
#define ENABLE_NTCP1_TEMP_MONITORING                (1u)

/* Enable Battery OCP */
#define BATT_PRI_OCP_ENABLE                         (1u)

/* Efficiency settings of Buck Boost */
#define INPUT_OUTPUT_EFFICIENCY_REDUCE_PERCENTAGE   (93u)
#define VBUS_TOLERANCE_PERCENTAGE                   (95)
#define MANUFACTURE_EFFICIENCY_PERCENTAGE           (100)

#define BATTERY_REMOVAL_THRESHOLD                   (1000u)

/* Min allowed battery cell voltage per cell in mV */
#define BATT_PER_CELL_ALLOWED_MIN_VOLT              (2000u)

/* Completely discharged battery cell voltage per cell in mV/SNK */
#define BATT_PER_CELL_DISCHARGED_VOLT_SNK           (3000u)

/* Completely discharged battery cell voltage per cell in mV/SRC */
#define BATT_PER_CELL_DISCHARGED_VOLT_SRC           (3100u)

/* Battery cell Recharge voltage per cell in mV */
#define BATT_PER_CELL_RECHARGE_VOLT                 (4000u)

/* Maximum allowed battery cell voltage per cell in mV */
#define BATT_PER_CELL_ALLOWED_MAX_VOLT              (4150u)

/* Per Battery cell OVP Threshold in mV*/
#define BATT_PER_CELL_OVP_THRESHOLD                 (4300u)

/* Battery per cell Charging Hysteresis threshold and threshold for OVP detection in mV */
#define BATT_PER_CELL_HYSTERSIS_TSH                 (100u)

/* Total battery cell count */
#define TOTAL_BATTERY_CELL_COUNT                    (5u)

/* Battery Recharging Hysteresis in mV */
#define TOTAL_VBATT_RECHARGE_THRESH                 (BATT_PER_CELL_RECHARGE_VOLT * TOTAL_BATTERY_CELL_COUNT)

/* Battery Charging Hysteresis in mV */
#define TOTAL_VBATT_HYST_THRESH                     (BATT_PER_CELL_HYSTERSIS_TSH * TOTAL_BATTERY_CELL_COUNT)

#if BATTERY_SIMULATOR_ENABLE
#define TOTAL_VBATT_MAX_ALLOWED_VOLT                (20000u)
#else
/* Maximum Total Battery voltage in mV */
#define TOTAL_VBATT_MAX_ALLOWED_VOLT                (BATT_PER_CELL_ALLOWED_MAX_VOLT * TOTAL_BATTERY_CELL_COUNT)
#endif /* BATTERY_SIMULATOR_ENABLE */

/* Maximum Battery voltage for OVP Threshold in mV.
 * Added 200mV to prevent false triggering when switching CC->CV.
 */
#define PRIMARY_VBATT_OVP_THRESHOLD                 ((BATT_PER_CELL_OVP_THRESHOLD * TOTAL_BATTERY_CELL_COUNT) + 200u)

/* Total voltage of completely discharged Battery/SNK role */
#define TOTAL_VBATT_DISCHARGED_SNK                  (BATT_PER_CELL_DISCHARGED_VOLT_SNK * TOTAL_BATTERY_CELL_COUNT)

/* Total Battery voltage below which UVP Is true and charging is not allowed */
#define PRIMARY_VBATT_UVP_THRESHOLD                 (BATT_PER_CELL_ALLOWED_MIN_VOLT * TOTAL_BATTERY_CELL_COUNT)

/* Total allowed MAX current to charge the battery in 10mA.
 * Set this as per Battery datasheet.
 * Max allowed value is 650u (6.5A) if Rsense = 5 mOhm
 * or 320u (3.2A) if Rsense = 10 mOhm
 */
#define VBAT_INPUT_CURR_MAX_SETTING                 (200u)

/* min current to pre-charge the battery in 10mA units (for Rsense = 5 mOhm).
 * Limited by HW.
 * Re-calculated for actually used Rsense by FW, so higher Rsense sets lower
 * (more accurate) pre-carge current value (actual Ibat_min will be 150mA for Rsense = 10 mOhm).
 */
#define MIN_IBAT_CHARGING_CURR                      (30u)

#define STEP_IBAT_CHARGING_CURR                     (30u)

/* Total minimum VBUS Power required for charging the battery in mWatt */
#define MIN_USBC_VBUS_TOTAL_POWER                   (7500u)

/* OCP Threshold in percentage for Battery Charging Current  */
#define PRIMARY_IBATT_OCP_CURR_THRESHOLD            (750u)

#define PRIMARY_BATT_OTP_THRESHOLD                  (50)
#define PRIMARY_BATT_ROOM_THRESHOLD                 (20)
#define PRIMARY_BATT_COLD_THRESHOLD                 (10)
#define PRIMARY_BATT_HYSTERESIS                     (5)

#define PRIMARY_NTCP0_OTP_THRESHOLD                 (85)
#define PRIMARY_NTCP0_HOT_THRESHOLD                 (70)
#define PRIMARY_NTCP0_HYSTERESIS                    (10)

#define PRIMARY_NTCP1_OTP_THRESHOLD                 (100)
#define PRIMARY_NTCP1_HOT_THRESHOLD                 (85)
#define PRIMARY_NTCP1_HYSTERESIS                    (10)

/*******************************************************************************
 * Solution specific features
 ******************************************************************************/

 /* When set Sink requests maximal current provided by Source instead of SNK PDO OperCurrent. */
#define PMG1B1_SINK_PDO_REQ_MAX_ADV_CURR            (1u)

#define TIMER_MEASURE_DELAY_ADC_EN_DELAY            (CY_USBPD_USER_TIMERS_START_ID + 1u)

#define TIMER_MEASURE_DELAY_ADC_EN_DELAY_PERIOD     (10u)

#define BATT_TIMER_ID                               (CY_USBPD_USER_TIMERS_START_ID + 2u)
#define BATT_TIMER_PERIOD                           (900)

#define BATTERY_MONITOR_TASK_ENABLE                 (1u)

#if BATTERY_MONITOR_TASK_ENABLE
#define SOLN_BATT_MONITOR_TASK_TIMER_ID             (CY_USBPD_USER_TIMERS_START_ID + 3u)
#define SOLN_BATT_MONITOR_TASK_TIMER_PERIOD         (200u)
#endif

/* Watchdog reset timer id. */
#define WATCHDOG_TIMER_ID                           (CY_USBPD_USER_TIMERS_START_ID + 5u)

#define SOLN_BATT_MONITOR_DEBOUNCE_TIMER_ID         (CY_USBPD_USER_TIMERS_START_ID + 6u)
#define SOLN_BATT_MONITOR_DEBOUNCE_TIMER_PERIOD     (600u)

/* Retry Timer to check detach condition */
#define APP_BROWNOUT_CHECK_DETACH_DELAY_ID          (CY_USBPD_USER_TIMERS_START_ID + 7u)
#define APP_BROWNOUT_CHECK_DETACH_DELAY_PERIOD      (12u)

#if (TRICKLE_CHARGE_TIMER_ENABLE || PRE_CHARGE_TIMER_ENABLE || NORMAL_CHARGE_TIMER_ENABLE)
/* One minute timer tick */
#define SOLN_SAMPLE_TIME_1MINUTE_TIMER_ID           (CY_USBPD_USER_TIMERS_START_ID + 8u)
#define SOLN_SAMPLE_TIME_1MINUTE_TIMER_PERIOD       (60000u)
#endif /*(TRICKLE_CHARGE_TIMER_ENABLE || PRE_CHARGE_TIMER_ENABLE || NORMAL_CHARGE_TIMER_ENABLE)*/

#if SIMULATE_ERROR
#define DBG_BATT_TIMER_ID                           (CY_USBPD_USER_TIMERS_START_ID + 9u)
#define DBG_BATT_TIMER_PERIOD                       (10000)
#endif

/* Timer for debugging CV mode. UART prints cells, total, BB out, current */
#define PRINT_CV                                    (0u)
#if PRINT_CV
#define MY_ID                                       (CY_USBPD_USER_TIMERS_START_ID + 10u)
#define MY_PERIOD                                   (5000)
#endif /* PRINT_CV */

/* Timer to blink the USER_LED */
#define LED_TIMER_ID                                (CY_PDUTILS_TIMER_USER_START_ID + 11u)

/*******************************************************************************
 * Header files including
 ******************************************************************************/
#ifndef NO_OF_TYPEC_PORTS
#define NO_OF_TYPEC_PORTS                           (CY_IP_MXUSBPD_INSTANCES)
#endif /* NO_OF_TYPEC_PORTS */

/** VBus current defined in 10mA units. */
#define CY_USBPD_I_1A                               (100u)
#define CY_USBPD_I_1_5A                             (150u)
#define CY_USBPD_I_2A                               (200u)
#define CY_USBPD_I_3A                               (300u)
#define CY_USBPD_I_4A                               (400u)
#define CY_USBPD_I_5A                               (500u)

/*******************************************************************************
 * Enable PD spec Rev 3 support
 ******************************************************************************/
#if CY_PD_REV3_ENABLE
    #define CY_PD_FRS_RX_ENABLE                     (0u)
    #define CY_PD_FRS_TX_ENABLE                     (0u)
#endif /* CY_PD_REV3_ENABLE */

/* Use the default Source PDO selection algorithm. */
#define PD_PDO_SEL_ALGO                             (0u)

/*******************************************************************************
 * USB-PD SAR ADC Configurations
 ******************************************************************************/
#define APP_GPIO_POLL_ADC_ID                    (CY_USBPD_ADC_ID_1)
#define APP_GPIO_POLL_ADC_INPUT                 (CY_USBPD_ADC_INPUT_AMUX_A)

#define APP_VBUS_POLL_ADC_ID                    (CY_USBPD_ADC_ID_0)
#define APP_VBUS_POLL_ADC_INPUT                 (CY_USBPD_ADC_INPUT_AMUX_B)

/*******************************************************************************
 * Power Sink (PSINK) controls
 ******************************************************************************/
/* 
 * Allow VBUS_IN discharge below 5V.
 * When VBUS_IN_DISCHARGE_EN macro is enabled, VBUS_IN discharge is enabled for all
 * VBUS downward transitions above 5V, but is disabled for transitions below 5V.
 * Because, for VBUS_IN powered solutions, VBUS_IN should not be accidently
 * brought to the low voltage where system behavior is undefined. 
 * VBUS_IN discharge below 5V may be required for solutions where regulator
 * requires higher discharge strength to set voltage below 5V.
 * It is recommended to enable this feature only for solution which are not
 * VBUS_IN powered.
 */
#define VBUS_IN_DISCH_BELOW_5V_EN                   (0u)

/*******************************************************************************
 * Power Source (PSOURCE) Configuration.
 ******************************************************************************/
/* VBus Monitoring is done using internal resistor divider. */
#define VBUS_MON_INTERNAL                           (1u)

/*******************************************************************************
 * System fault configuration features.
 ******************************************************************************/

/*
 * Enable/Disable delay between fault retries for Type-C/PD faults.
 */
#define FAULT_RETRY_DELAY_EN                        (0u)

#if FAULT_RETRY_DELAY_EN

/*
 * Delay between fault retries in mS.
 */
#define FAULT_RETRY_DELAY_MS                        (500u)

#endif /* FAULT_RETRY_DELAY_EN */

/* 
 * Enable/Disable delayed infinite fault recovery for Type-C/PD faults.
 * Fault recovery shall be tried with a fixed delay after configured 
 * fault retry count is elapsed. 
 */
#define FAULT_INFINITE_RECOVERY_EN                  (0u)

#if FAULT_INFINITE_RECOVERY_EN

/* 
 * Delayed fault recovery period in mS.
 */
#define FAULT_INFINITE_RECOVERY_DELAY_MS            (5000u)

#endif /* FAULT_INFINITE_RECOVERY_EN */

/* Enable watchdog hardware reset for CPU lock-up recovery */
#define WATCHDOG_HARDWARE_RESET_ENABLE              (1u)

/* Disable device reset on error (watchdog expiry or hard fault). */
#define RESET_ON_ERROR_ENABLE                       (1u)

/*
 * Watchdog reset period in ms. This should be set to a value greater than
 * 500 ms to avoid significant increase in power consumption.
 */
#define WATCHDOG_RESET_PERIOD_MS                    (750u)

/* Enable tracking of maximum stack usage. */
#define STACK_USAGE_CHECK_ENABLE                    (0u)

/*
 * The LED toggle period (ms) to be used when Type-C connection hasn't been detected.
 */
#define LED_TIMER_PERIOD_DETACHED                   (1000u)

/*
 * The LED toggle period (ms) to be used when a Type-C power source is connected.
 */
#define LED_TIMER_PERIOD_TYPEC_SRC                  (1000u)

/*
 * The LED toggle period (ms) to be used when a USB-PD power source is connected.
 */
#define LED_TIMER_PERIOD_PD_SRC                     (100u)

/*
 * The LED toggle period (ms) to be used when a BC 1.2 DCP (Downstream Charging Port) source without PD support is connected.
 */
#define LED_TIMER_PERIOD_DCP_SRC                    (3000u)

/*
 * The LED toggle period (ms) to be used when a BC 1.2 CDP (Charging Downstream Port) source without PD support is connected.
 */
#define LED_TIMER_PERIOD_CDP_SRC                    (10000u)

/*******************************************************************************

 * Firmware feature configuration.
 ******************************************************************************/
/*
 * The following macro defines whether we will handle extended
 * message in solution space.
 */
#define CCG_SLN_EXTN_MSG_HANDLER_ENABLE             (1u)

/*******************************************************************************
 * Get Battery status and Get Battery configuration response configuration
 ******************************************************************************/

/* Valid battery_status response when source */
#define CCG_EXT_MSG_VALID_BAT_STAT_SRC              (0xffff0600)

/* Valid battery_status response when sink, battery is Charging. */
#define CCG_EXT_MSG_VALID_BAT_STAT_SNK_CHARGING     (0xffff0200)

/* Valid battery_status response when sink, battery is Idle. */
#define CCG_EXT_MSG_VALID_BAT_STAT_SNK_IDLE         (0xffff0200)

/* Invalid battery_status response */
#define CCG_EXT_MSG_INVALID_BAT_REF                 (0xffff0100)

/* This macro defines the number of batteries */
#define CCG_PB_NO_OF_BATT                           (1u)

/* This macro defines the VID-PID for Power Bank */
#define CCG_PB_VID_PID                              (0xf51104b4)

/* This macro defines the battery design capacity (in 0.1WH)
 * 0x0000 : Battery not present
 * 0xFFFF : Design capacity unknown
 */
#define CCG_PB_BAT_DES_CAP                          (0xFFFF)

/* This macro defines the battery last full charge capacity (in 0.1WH)
 * 0x0000 : Battery not present
 * 0xFFFF : Last full charge capacity unknown
 */
#define CCG_PB_BAT_FUL_CHG_CAP                      (0xFFFF)

/*******************************************************************************
 * Power throttling specific Configuration.
 ******************************************************************************/
/*
 * Set this macro to 1 if the temperature based power throttling is
 * done with thermistor.
 */

#define TEMPERATURE_SENSOR_IS_THERMISTOR (1u)

#if TEMPERATURE_SENSOR_IS_THERMISTOR

/**
@brief Temperature sensor count
*/
#define TEMPERATURE_SENSOR_COUNT (2u)

/*
 * Defines the temperature starting from which mapping is done
 */
#define BASE_MAP_TEMP (20u)

/*
 * Defines the resolution of the map function in deg(C)
 */
#define TEMP_MAP_RESOLUTION (5u)

/*
 * Defines the number of map data available
 */
#define VOLT_TO_TEMP_MAP_TABLE_COUNT (20u)

/*
 * Defines the safe temperature value when the thermistor voltage is below
 * the first entry of the table
 */
#define SAFE_DEFAULT_TEMPERATURE (25u)

/*
 * Macro when set defines the Thermistor type as NTC.
 */
#define THERMISTOR_IS_NTC (1u)

/*
 * Defines the fault value returned by I2C/Thermistor.
 */
#define THROTTLE_SENSOR_FAULT (0xFF)

#endif /* TEMPERATURE_SENSOR_IS_THERMISTOR */



#endif /* _CONFIG_H_ */

/* End of file [] */
