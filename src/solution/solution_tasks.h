/**
 * @file battery_charger_solution.h
 *
 * @brief @{Header file for solution layer.@}
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


#ifndef _SOLUTION_TASKS_H_
#define _SOLUTION_TASKS_H_

#include <stdbool.h>

/* Enumeration for battery Pack ID */
typedef enum
{
    NO_BATTERY = 0,
    BATTERY_PRESENT,
}battery_pack_type;

/* Enumeration for temperature states */
typedef enum
{
    STATE_TEMP_DISABLE_CHARGE_COLD = 0,
    STATE_TEMP_LIMITED_CHARGE,
    STATE_TEMP_NORMAL_CHARGE,
    STATE_TEMP_OTP
}temp_state_t;

/* This structure holds the Battery Status */
typedef struct battery_status
{
    volatile bool bb_enable;                        /* Flag indicated buck boost ON/OFF Status */
    volatile bool bb_out_fet_enable;                /* Flag indicating Buck Boost output fet to battery ON/oFF Status */
    volatile bool cur_batt_charging_status;         /* Flag indicating whether the out fet is on and the battery is currently charging */
    volatile bool is_cell_full;                     /* Flag to indicate whether Charging cell is allowed */
    volatile bool is_cell_recharge;                 /* Flag to indicate whether recharging cell is allowed */
    volatile bool is_vbus_pwr_sufficient;           /* Flag to indicate whether the input VBUS Power is sufficient to charge the battery*/
    uint8_t batt_pack_type;                         /* Stores battery pack ID Type */
    volatile bool batt_ovp_fault_active;            /* Flag to indicate the current ovp fault condition exists. */
    volatile bool batt_ocp_fault_active;            /* Flag to indicate the current ocp fault condition exists. */
    volatile bool batt_rcp_fault_active;            /* Flag to indicate the current rcp fault condition exists. */
    volatile bool batt_uvp_fault_active;            /* Flag to indicate whether the battery is discharged below allowed limit */
    volatile bool batt_otp_fault_active;            /* Flag to indicate the Pack otp fault condition exists. */
    volatile bool ntcp0_otp_fault_active;            /* Flag to indicate the NTCP0 otp fault condition exists. */
    volatile bool ntcp1_otp_fault_active;            /* Flag to indicate the NTCP1 otp fault condition exists. */
    uint8_t curr_chrg_cycle_num;                    /* Number of Full Charge cycles happened in the current Type C Connections */
    bool cv_mode_entered;                           /* Indicates CV mode of charging flowchart */
    uint16_t batt_max_curr_rating;                  /* Maximum allowed battery current rating in 10mA unit */
    volatile uint16_t cur_bb_vout;                  /* Current Buck Boost Output Voltage setting */
    volatile uint16_t cur_bb_iout;                  /* Current Buck Boost Output Current Setting */
    volatile uint32_t cur_bb_pwr;                   /* Current Input power available for Buck boost to operate */
    volatile uint16_t curr_batt_volt;               /* Current battery voltage measured by ADC */
    volatile uint16_t curr_batt_curr;               /* Current Battery Charging current measured using the 5mOhm Rsense in 10mA */
    volatile bool adc_pending;                      /* Flag to indicate the ADC is pending until the new result is ready */
    volatile bool timeout_expired;                  /* Flag to indicate Trickle, Precharge or Normal CC+CV timer timeout expired */
}cy_stc_battery_status_t;

/* Structure contains measured values */
typedef struct
{
    uint16_t cell1;             /* Voltage of 1st cell */
    uint16_t cell2;             /* Voltage of two cells */
    uint16_t cell3;             /* Voltage of three cells */
    uint16_t cell4;             /* Voltage of four cells */
    uint16_t vbat;              /* Voltage of five cells (total battery) */
    uint16_t bat_th;            /* Voltage at battery thermistor pin */
    uint16_t ntcp0;             /* Voltage at NTCP0 sensor */
    uint16_t ntcp1;             /* Voltage at NTCP1 sensor */
    uint16_t packID_mVolt;      /* Voltage at battery ID pin */
}cy_stc_battery_measure_t;

/**
 * @brief This function notifies the application code when SAR ADC is
 * doing measurement to block entering into deepsleep.
 *
 * @return True if SAR ADC is doing measurement
 */
bool battery_measure_sar_is_active(void);

/**
 * @brief This function initializes SAR ADC and configures interrupts.
 *
 * @param ptrPdStackContext PD Stack context.
 *
 * @return None
 */
void battery_measure_sar_adc_init(cy_stc_pdstack_context_t* ptrPdStackContext);

/**
 * @brief This function measures voltages at ID, NTCP0 and NTCP1 sensors
 * using 8-bit SAR and enables 12-bit SAR ADC battery voltages.
 *
 * @param ptrPdStackContext PD Stack context.
 *
 * @return None
 */
void battery_measure_sar_adc_scan(cy_stc_pdstack_context_t* ptrPdStackContext);

/**
 * @brief Structure to Battery Charging Middleware context information.
 */
typedef struct cy_stc_battery_charging_context
{
    /** Battery Status */
    cy_stc_battery_status_t batteryStatus;

    /** Battery Measure */
    cy_stc_battery_measure_t batteryMeasure;

    /** Pointer to PD stack */
    cy_stc_pdstack_context_t *ptrPdStack;
} cy_stc_battery_charging_context_t;

/**
 * @brief This function returns Battery Charging context pointer.
 *
 * @param portIdx Index of port.
 *
 * @return Pointer to Battery Charging context.
 */
cy_stc_battery_charging_context_t * get_battery_charging_context(uint8_t portIdx);

/**
 * @brief This function initializes Battery status.
 *
 * @param ptrPdStackContext PD Stack context.
 *
 * @return None
 */
void battery_charger_init (cy_stc_pdstack_context_t *ptrPdStackContext);

/**
 * @brief This function resets to default Battery charging status.
 *
 * @param ptrBatteryChargingContext Battery Charging context.
 *
 * @return None
 */
void reset_battery_status(cy_stc_battery_charging_context_t *ptrBatteryChargingContext);

/**
 * @brief This function clears Battery Charging Status flags at TYPEC disconnect.
 *
 * @param ptrBatteryChargingContext Battery Charging context.
 *
 * @return None
 */
void clear_flags_on_usbc_disconnect (cy_stc_battery_charging_context_t *ptrBatteryChargingContext);

/**
 * @brief This function calculates Battery Pack ID based on voltage at ID pin.
 *
 * @param ptrBatteryChargingContext Battery Charging context.
 *
 * @return None
 */
void batt_soln_detect_batt_packID (cy_stc_battery_charging_context_t *ptrBatteryChargingContext);

/**
 * @brief This function processes action required for Battery charging/discharging.
 *
 * @param ptrBatteryChargingContext Battery Charging context.
 *
 * @return None
 */
void soln_battery_charger_task(cy_stc_battery_charging_context_t *ptrBatteryChargingContext);

/**
 * @brief This function calculates the Battery Pack temperature and checks OTP.
 *
 * @param ptrBatteryChargingContext Battery Charging context.
 *
 * @return None
 */
void measure_temp_sensor_data (cy_stc_battery_charging_context_t *ptrBatteryChargingContext);

/**
 * @brief This function calculates the temperature near TYPEC connector and checks OTP.
 *
 * @param ptrBatteryChargingContext Battery Charging context.
 *
 * @return None
 */
void measure_onboard_ntcp0_sensor_data(cy_stc_battery_charging_context_t *ptrBatteryChargingContext);

/**
 * @brief This function calculates the temperature near BB inductance and checks OTP.
 *
 * @param ptrBatteryChargingContext Battery Charging context.
 *
 * @return None
 */
void measure_onboard_ntcp1_sensor_data(cy_stc_battery_charging_context_t *ptrBatteryChargingContext);

/**
 * @brief This function clears Battery Charging/Discharging faults.
 *
 * @param ptrBatteryChargingContext Battery Charging context.
 *
 * @return None
 */
void reset_battery_faults(cy_stc_battery_charging_context_t *ptrBatteryChargingContext);

void print_cb (
        cy_timer_id_t id,           /**< Timer ID for which callback is being generated. */
        void *callbackContext);       /**< Timer module Context. */

#endif /* _SOLUTION_TASKS_H_ */

/* [] END OF FILE */

