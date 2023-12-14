/**
 * @file solution.h
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

#ifndef _SOLUTION_H_
#define _SOLUTION_H_
#include "cy_usbpd_common.h"
#include "cy_pdstack_common.h"
#include "cy_usbpd_typec.h"
#include "cy_pdstack_dpm.h"
#include "cy_usbpd_vbus_ctrl.h"
#include "cy_usbpd_phy.h"
#include "cy_pdutils_sw_timer.h"
#include "solution_tasks.h"
#include <stdbool.h>

/* Resistor divider description in Ohm .
 * Required to calculate measured voltages for battery cells.
 * Strongly depends from resistors values in schematic. */
#define Radd        (1000.0)
#define Rdown       (10000.0)
#define Rup_0       (12000.0)
#define Rup_1       (34000.0)
#define Rup_2       (56000.0)
#define Rup_3       (78700.0)
#define Rup_4       (99900.0)
#define Rex         (100000.0)

/* Equations to calculate Battery cell voltages based on measured values at resistor dividers. */
#define R_DIV_0     (2*SAR0_VREF_MV*((Rdown+Rup_0+Radd)/Rdown))
#define R_DIV_1     (2*SAR0_VREF_MV*((Rdown+Rup_1+Radd)/Rdown))
#define R_DIV_2     (2*SAR0_VREF_MV*((Rdown+Rup_2+Radd)/Rdown))
#define R_DIV_3     (2*SAR0_VREF_MV*((Rdown+Rup_3+Radd)/Rdown))
#define R_DIV_4     (2*SAR0_VREF_MV*((Rdown+Rup_4+Radd)/Rdown))

/*******************************************************************************
 * Datatypes
 ******************************************************************************/
/* Battery Charging states */
typedef enum {
    BATT_CHG_IDLE,
    BATT_CHG_INIT,
    BATT_CHG_TYPEC_ATTACHED_SNK,
    BATT_CHG_TYPEC_ATTACHED_SRC,
    BATT_CHG_BB_SET_VBAT,
    BATT_CHG_BB_SET_IBAT,
    BATT_CHG_BB_SET_VBAT_WAIT,
    BATT_CHG_ENA_BAT_FET,
#if PDL_VOUTBB_RCP_ENABLE
    BATT_CHG_ENA_RCP_DELAY,
    BATT_CHG_ENA_RCP_DELAY1,
#endif /* PDL_VOUTBB_RCP_ENABLE */
    BATT_CHG_CHARGE_LOOP,
    BATT_CHG_END_OF_CHARGE
} batt_chg_state_t;

/* Charging algorithm states */
typedef enum {
    BATT_CHG_ALT_LOOK4BATTERY,
    BATT_CHG_ALT_BAD_BATTERY,
    BATT_CHG_ALT_INIT_CHARGE,
    BATT_CHG_ALT_CC_MODE,
    BATT_CHG_ALT_CHARGE_FULL,
    BATT_CHG_ALT_CV_MODE,
} batt_chg_alt_state_t;

/* Commands to control BB, input/output switches */
typedef enum batt_chge_src_cmd 
{
    BATT_CHGE_BB_EN = 0,
    BATT_CHGE_BB_DIS,
    BATT_CHGE_BB_OUT_EN,
    BATT_CHGE_BB_OUT_DIS,
    BATT_CHGE_BB_IN_EN,
    BATT_CHGE_BB_IN_DIS
} batt_chge_src_cmd_t;

/*******************************************************************************
 * Global Function Declaration
 ******************************************************************************/
/**
 * @brief This function initializes solution status structure.
 *
 * @param ptrPdStackContext PD Stack context.
 *
 * @return None
 */
void solution_init (cy_stc_pdstack_context_t *ptrPdStackContext);

/**
 * @brief Battery Charger Solution PD event handler .
 *
 * @param cy_stc_pdstack_context_t PD Stack context.
 *
 * @param cy_en_pdstack_app_evt_t evt PD event
 *
 * @param data Pointer to PD event data
 *
 * @return None
 */
void batt_pd_event_handler(cy_stc_pdstack_context_t* ptrPdStackContext, cy_en_pdstack_app_evt_t evt, const void *data);

/**
 * @brief Battery Solution DRP event handler for charging/discharging. Changes Power role.
 *
 * @param cy_stc_pdstack_context_t PD Stack context.
 *
 * @param cy_en_pdstack_app_evt_t evt PD event
 *
 * @param data Pointer to PD event data
 *
 * @return None
 */
void drp_fsm_event_handler(cy_stc_pdstack_context_t* ptrPdStackContext, cy_en_pdstack_app_evt_t evt, const void *data);

/**
 * @brief This function processes Solution level tasks.
 *
 * @param cy_stc_pdstack_context_t PD Stack context.
 *
 * @return None
 */
void soln_task(cy_stc_pdstack_context_t* ptrPdStackContext);

/**
 * @brief Function to updated LED status based on Faults.
 *
 * @param cy_stc_pdstack_context_t PD Stack context.
 *
 * @return None
 */
void update_led_status(cy_stc_pdstack_context_t* ptrPdStackContext);

/**
 * @brief This function selects 5/9V for Buck in Source mode.
 *
 * @param cy_stc_pdstack_context_t PD Stack context.
 *
 * @param volt_mV Source VBUS Voltage
 *
 * @return None
 */
void sol_batt_src_set_volt(cy_stc_pdstack_context_t * context, uint16_t volt_mV);

/**
 * @brief This function processes enabling SRC Buck.
 *
 * @param cy_stc_pdstack_context_t PD Stack context.
 *
 * @return None
 */
void sol_batt_src_en(cy_stc_pdstack_context_t * context);

/**
 * @brief This function processes disabling SRC Buck.
 *
 * @param cy_stc_pdstack_context_t PD Stack context.
 *
 * @return None
 */
void sol_batt_src_dis(cy_stc_pdstack_context_t * context);

/**
 * @brief This function enables/disables BrownOut Detector for BuckBoost.
 *
 * @param cy_stc_pdstack_context_t PD Stack context.
 *
 * @param enable Enable/Disable BB BOD.
 *
 * @return None
 */
void sol_brown_out_control(cy_stc_pdstack_context_t * context, bool enable);

/**
 * @brief Function disables BuckBoost, input and output FET.
 *
 * @param cy_stc_pdstack_context_t PD Stack context.
 *
 * @return None
 */
void soln_batt_chgr_hw_disable(cy_stc_pdstack_context_t* ptrPdStackContext);

void Cy_USBPD_Vbus_GdrvPfetOn_Sol(cy_stc_usbpd_context_t *context, bool turnOnSeq);

void Cy_USBPD_Vbus_GdrvPfetOff_Sol(cy_stc_usbpd_context_t *context, bool turnOffSeq);


#endif /* _SOLUTION_H_ */

/* [] END OF FILE */

