/******************************************************************************
* File Name: thermistor.h
*
* Description: Thermistor handler header file.
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
/**
* \addtogroup group_ccgxAppCommon Common source files
* \{
*/


#ifndef THERMISTOR_H_
#define THERMISTOR_H_

#include <config.h>
#include <stdbool.h>
#include <stdint.h>
#include <stddef.h>
#include <cy_pdstack_common.h>

/**
   @brief Defines the max temperature value that can be assigned for thermistor
 */
#define THERMISTOR_FAULT_TEMP                           (255u)
    
/**
 * @brief This function maps the thermal voltage read from thermistor to temperature value
 * @param ptrPdStackcontext PD stack context
 * @param therm_volt The thermal voltage that must be mapped to temperature
 * @return The temperature value corresponding to the thermal voltage read
 */
uint8_t ccg_volt_temp_map(cy_stc_pdstack_context_t *ptrPdStackcontext, uint16_t therm_volt);

/**
 * @brief This function reads the temperature of the specified sensor
 * @param ptrPdStackcontext PD stack context
 * @param sensor_id The index of the sensor to be read
 * @return The temperature value corresponding to the thermal voltage read
 */
uint8_t ccg_get_sensor_temperature(cy_stc_pdstack_context_t *ptrPdStackcontext, uint8_t sensor_id);

/**
 * @brief This function maps the thermal voltage read from thermistor to temperature value
 * @param ptrPdStackcontext PD stack context
 * @param volt_temp_table Pointer to the table to be used for thermal voltage to temperature mapping
 * @return None
 */
void register_thermistor_mapping_table(cy_stc_pdstack_context_t *ptrPdStackcontext, const uint16_t *volt_temp_table);

/** \} group_ccgxAppCommon */

#endif /* THERMISTOR_H_ */

