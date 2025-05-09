/******************************************************************************
* File Name: srom.h
*
* Description: SROM Code file.
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
#ifndef SROM_H_
#define SROM_H_

#include "config.h"
#include <stdio.h>
#include <string.h>

#define ATTRIBUTES
#define CALL_OUT_FUNCTION(func_name) func_name
#define CALL_IN_FUNCTION(func_name) func_name
#define GET_IN_VARIABLE(var_name) var_name

#define ATTRIBUTES_SCB_I2C      ATTRIBUTES
#define ATTRIBUTES_APP_PDO      ATTRIBUTES
#define ATTRIBUTES_PD_PROT      ATTRIBUTES
#define ATTRIBUTES_SYS_GPIO     ATTRIBUTES
#define ATTRIBUTES_SYS_SYS      ATTRIBUTES
#define ATTRIBUTES_SYS_FLASH    ATTRIBUTES
#define ATTRIBUTES_HPISS_HPI    ATTRIBUTES
#define ROM_STATIC_ATTRIBUTE

#define HPI_GLOBAL_VAR
#define HPI_CONST
#define CRYPTO_STATIC_ATTRIBUTE
#define CRYPTO_ATTRIBUTE
#define CRYPTO_VAR_ATTRIBUTE
#define CRYPTO_RSA_VAR_ATTRIBUTE
#define ROM_CONSTANT

/**
* ROM/Flash function and variable access macro redirection.
* Macro mapping section in each srom_vars_<device>.h file need to be 
* updated when there is any change in ROM code base.
* Macro mapping sections must include all the CALL_MAP functions of
* the code base. Each call mapping function can be mapped to
* either direct function or variable call 
* or through call_in/call_out function or get_in variable as per the ROM
* code definition of the device. 
*/
#define CALL_MAP(str)  (str)

#endif /* SROM_H_ */

/** \} group_ccgxAppCommon */
