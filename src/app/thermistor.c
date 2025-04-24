/******************************************************************************
* File Name: thermistor.c
*
* Description: Thermistor handler source file.
*
* Related Document: See README.md
*
*******************************************************************************
* Copyright 2022-2025, Cypress Semiconductor Corporation (an Infineon company) or
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

#include "thermistor.h"
#include "cy_usbpd_vbus_ctrl.h"
#include "cy_gpio.h"
#include "cy_pdstack_timer_id.h"

#if (TEMPERATURE_SENSOR_COUNT != 0u)

#if TEMPERATURE_SENSOR_IS_THERMISTOR

/* Pointer to the voltage to temperature mapping table */
static const uint16_t *gl_volt_temp_map;

void register_thermistor_mapping_table(cy_stc_pdstack_context_t *ptrPdStackcontext, const uint16_t *volt_temp_table)
{
    (void)ptrPdStackcontext;
    gl_volt_temp_map = volt_temp_table;
}

/* Function to map voltage to temperature */
uint8_t ccg_volt_temp_map(cy_stc_pdstack_context_t *ptrPdStackcontext, uint16_t therm_volt)
{
    (void)ptrPdStackcontext;
    uint8_t idx, temperature = THROTTLE_SENSOR_FAULT;
    uint8_t start = 0, stop = VOLT_TO_TEMP_MAP_TABLE_COUNT - 1u;
    uint16_t resolution;
    uint16_t diff;
    if(gl_volt_temp_map == NULL)
    {
        return THROTTLE_SENSOR_FAULT;
    }
#if THERMISTOR_IS_NTC
    if(!((therm_volt <= *gl_volt_temp_map) && 
            !(therm_volt < *(gl_volt_temp_map + VOLT_TO_TEMP_MAP_TABLE_COUNT - 1u))))
#else
    if(!((therm_volt >= *gl_volt_temp_map) && 
            !(therm_volt > *(gl_volt_temp_map + VOLT_TO_TEMP_MAP_TABLE_COUNT - 1u))))
#endif /* THERMISTOR_IS_NTC */    
    {
        /* If the code flow comes here, it means that the entry is not present in the table.
         * If the thermistor measured temperature is less than the starting temperature
         * value of the hash table, we will consider the measured tempearture as
         * safe temperature. This will enable the customers to have a smaller hash table
         * of their preferred range.
         * Note: There is no requirement to execute throttling in safe temperature range.
         *
         * However fault condition (0xFF) will be reported if the measured temperature is beyond
         * the highest temperature mapped in the table.
         */
#if THERMISTOR_IS_NTC
        if((therm_volt > (*gl_volt_temp_map)))
#else
        if((therm_volt < (*gl_volt_temp_map)))
#endif /* THERMISTOR_IS_NTC */
        {
            temperature = SAFE_DEFAULT_TEMPERATURE;
        }
    }
    else
    {
#if THERMISTOR_IS_NTC
        if(therm_volt <= *(gl_volt_temp_map + VOLT_TO_TEMP_MAP_TABLE_COUNT / 2u))
#else
        if(therm_volt >= *(gl_volt_temp_map + VOLT_TO_TEMP_MAP_TABLE_COUNT / 2u))
#endif /* THERMISTOR_IS_NTC */
        {
            start = VOLT_TO_TEMP_MAP_TABLE_COUNT / 2u;            
        }
        else
        {
            stop = VOLT_TO_TEMP_MAP_TABLE_COUNT / 2u;
        }
        
        for(idx = start; idx <= stop; idx++)
        {
#if THERMISTOR_IS_NTC
            if(therm_volt < *(gl_volt_temp_map + idx))
#else
            if(therm_volt > *(gl_volt_temp_map + idx))
#endif /* THERMISTOR_IS_NTC */
            {
                /* Do not do anything. Continue searching in the table */
            }
            else
            {
                if (start != idx)
                {
#if THERMISTOR_IS_NTC
                    resolution = (*(gl_volt_temp_map + idx - 1u) - *(gl_volt_temp_map + idx))/TEMP_MAP_RESOLUTION;
                    diff = *(gl_volt_temp_map + idx - 1u) - therm_volt;      
#else
                    resolution = (*(gl_volt_temp_map + idx) - *(gl_volt_temp_map + idx - 1u))/TEMP_MAP_RESOLUTION;
                    diff = therm_volt - *(gl_volt_temp_map + idx - 1u);
#endif /* THERMISTOR_IS_NTC */

                    temperature = BASE_MAP_TEMP + TEMP_MAP_RESOLUTION * (idx - 1u);

                    if (resolution > 0u)
                    {
                        temperature += (uint8_t)(diff / resolution);
                    }
                    else
                    {
                        /* No Action */
                    }
                }
                else
                {
                    temperature = BASE_MAP_TEMP + TEMP_MAP_RESOLUTION * start;
                }
                break;
            } 
        }
    }    
    return temperature;
}
#endif /* TEMPERATURE_SENSOR_IS_THERMISTOR */

#endif /* (TEMPERATURE_SENSOR_COUNT != 0u) */
