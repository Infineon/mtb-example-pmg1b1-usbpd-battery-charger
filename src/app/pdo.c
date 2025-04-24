/******************************************************************************
* File Name: pdo.c
*
* Description: This file implements the PDO evaluation and handling functionality.
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

#include "config.h"
#include "cy_pdstack_common.h"
#include "cy_pdstack_dpm.h"
#include "cy_pdutils.h"
#include "pdo.h"
#include "app.h"

#if (!(CY_PD_SOURCE_ONLY))

/* PDO Variables. */
uint32_t gl_max_min_cur_pwr[NO_OF_TYPEC_PORTS];
uint32_t gl_contract_power[NO_OF_TYPEC_PORTS];
uint32_t gl_contract_voltage[NO_OF_TYPEC_PORTS];
uint32_t gl_op_cur_power[NO_OF_TYPEC_PORTS];

static uint32_t calc_power(uint32_t voltage, uint32_t current)
{
    /*
       Voltage is expressed in 50 mV units.
       Current is expressed in 10 mA units.
       Power should be expressed in 250 mW units.
     */
    return (Cy_PdUtils_DivRoundUp(voltage * current, 500));
}

static uint32_t calc_current(uint32_t power, uint32_t voltage)
{
    /*
       Power is expressed in 250 mW units.
       Voltage is expressed in 50 mV units.
       Current should be expressed in 10 mA units.
     */
    return (Cy_PdUtils_DivRoundUp(power * 500, voltage));
}

/**
 * Checks if SRC pdo is acceptable for SNK pdo.
 * @param pdo_src pointer to current SRC PDO
 * @param pdo_snk pointer to current SNK PDO
 * @return True if current src pdo is acceptable for current snk pdo
 */
static bool is_src_acceptable_snk(cy_stc_pdstack_context_t* context, cy_pd_pd_do_t* pdo_src, uint8_t snk_pdo_idx)
{
    cy_pd_pd_do_t* pdo_snk = (cy_pd_pd_do_t*)&(context->dpmStat.curSnkPdo[snk_pdo_idx]);
    uint8_t port = context->port;

    uint32_t snk_supply_type = pdo_snk->fixed_snk.supplyType;
    uint32_t fix_volt;
    uint32_t maxVolt;
    uint32_t minVolt;
    uint32_t out = false;
    uint32_t max_min_temp, compare_temp;
    uint32_t oper_cur_pwr;

    max_min_temp = context->dpmStat.curSnkMaxMin[snk_pdo_idx] & CY_PD_SNK_MIN_MAX_MASK;

    switch(pdo_src->fixed_src.supplyType)
    {
        case CY_PDSTACK_PDO_FIXED_SUPPLY:  /* Fixed supply PDO */
            fix_volt = pdo_src->fixed_src.voltage;
            maxVolt = Cy_PdUtils_DivRoundUp(fix_volt, 20);
            minVolt = fix_volt - maxVolt;
            maxVolt = fix_volt + maxVolt;

            switch(snk_supply_type)  /* Checking sink PDO type */
            {
                case CY_PDSTACK_PDO_FIXED_SUPPLY:
                    if(fix_volt == pdo_snk->fixed_snk.voltage)
                    {
                        compare_temp = CY_PDUTILS_GET_MAX (max_min_temp, pdo_snk->fixed_snk.opCurrent);
                        if (pdo_src->fixed_src.maxCurrent >= compare_temp)
                        {
#if (!PMG1B1_SINK_PDO_REQ_MAX_ADV_CURR) 
                            gl_op_cur_power[port] = pdo_snk->fixed_snk.opCurrent;
#else /* (POWER_BANK) */
                            gl_op_cur_power[port] = pdo_src->fixed_src.maxCurrent;
#endif /* (POWER_BANK) */
                            out = true;
                        }
                    }
                    break;

                case CY_PDSTACK_PDO_VARIABLE_SUPPLY:
                    if ((minVolt >= pdo_snk->var_snk.minVoltage) && (maxVolt <= pdo_snk->var_snk.maxVoltage))
                    {
                        compare_temp = CY_PDUTILS_GET_MAX (max_min_temp, pdo_snk->var_snk.opCurrent);
                        if (pdo_src->fixed_src.maxCurrent >= compare_temp)
                        {
#if (!PMG1B1_SINK_PDO_REQ_MAX_ADV_CURR)
                            gl_op_cur_power[port] = pdo_snk->var_snk.opCurrent;
#else /* (POWER_BANK) */
                            gl_op_cur_power[port] = pdo_src->fixed_src.maxCurrent;
#endif /* (POWER_BANK) */
                            out = true;
                        }
                    }
                    break;

                case CY_PDSTACK_PDO_BATTERY:
                    if ((minVolt >= pdo_snk->bat_snk.minVoltage) && (maxVolt <= pdo_snk->bat_snk.maxVoltage))
                    {
                        fix_volt = minVolt;

                        /* Calculate the operating current and min/max current values. */
                        oper_cur_pwr = calc_current(pdo_snk->bat_snk.opPower, minVolt);
                        max_min_temp = calc_current(max_min_temp, minVolt);

                        /* Make sure the source can supply the maximum current that may be required. */
                        compare_temp = CY_PDUTILS_GET_MAX(max_min_temp, oper_cur_pwr);
                        if (pdo_src->fixed_src.maxCurrent >= compare_temp)
                        {
                            gl_op_cur_power[port] = oper_cur_pwr;
                            out = true;
                        }
                    }
                    break;

                default:
                    break;
            }

            if (out)
            {
                gl_contract_voltage[port] = fix_volt;
                gl_contract_power[port]   = calc_power (fix_volt, gl_op_cur_power[port]);
                gl_max_min_cur_pwr[port]  = max_min_temp;
            }
            break;

        case CY_PDSTACK_PDO_BATTERY:   /* SRC is a battery */
            maxVolt = pdo_src->bat_src.maxVoltage;
            minVolt = pdo_src->bat_src.minVoltage;
            switch(snk_supply_type)
            {
                case CY_PDSTACK_PDO_FIXED_SUPPLY:
                    /* Battery cannot supply fixed voltage
                     * Battery voltage changes with time
                     * This contract if permitted can be un-reliable */
                    break;

                case CY_PDSTACK_PDO_VARIABLE_SUPPLY:
                    if((minVolt >= pdo_snk->var_snk.minVoltage) && (maxVolt <= pdo_snk->var_snk.maxVoltage))
                    {
                        /* Calculate the expected operating power and maximum power requirement. */
                        oper_cur_pwr = calc_power(maxVolt, pdo_snk->var_snk.opCurrent);
                        max_min_temp = calc_power(maxVolt, max_min_temp);

                        compare_temp = CY_PDUTILS_GET_MAX (oper_cur_pwr, max_min_temp);
                        if (pdo_src->bat_src.maxPower >= compare_temp)
                        {
#if (!PMG1B1_SINK_PDO_REQ_MAX_ADV_CURR)
                            gl_op_cur_power[port] = oper_cur_pwr;
#else /* (POWER_BANK) */
                            gl_op_cur_power[port] = pdo_src->bat_src.maxPower;
#endif /* (POWER_BANK) */
                            out = true;
                        }
                    }
                    break;

                case CY_PDSTACK_PDO_BATTERY:
                    /* Battery connected directly to a battery.
                     * This combination is unreliable */
                    if((minVolt >= pdo_snk->bat_snk.minVoltage) && (maxVolt <= pdo_snk->bat_snk.maxVoltage))
                    {
                        compare_temp = CY_PDUTILS_GET_MAX (max_min_temp, pdo_snk->bat_snk.opPower);
                        if (pdo_src->bat_src.maxPower >= compare_temp)
                        {
                            gl_op_cur_power[port] = pdo_snk->bat_snk.opPower;
                            out = true;
                        }
                    }
                    break;

                default:
                    break;
            }

            if (out)
            {
                gl_contract_voltage[port] = maxVolt;
                gl_max_min_cur_pwr[port]  = max_min_temp;
                gl_contract_power[port]   = gl_op_cur_power[port];
            }
            break;

        case CY_PDSTACK_PDO_VARIABLE_SUPPLY:   /* Variable supply PDO */
            maxVolt = pdo_src->var_src.maxVoltage;
            minVolt = pdo_src->var_src.minVoltage;
            switch (snk_supply_type) /* Checking sink PDO type */
            {
                case CY_PDSTACK_PDO_FIXED_SUPPLY:
                    /* This connection is not feasible
                     * A variable source cannot provide a fixed voltage */
                    break;

                case CY_PDSTACK_PDO_VARIABLE_SUPPLY:
                    if((minVolt >= pdo_snk->var_snk.minVoltage) && (maxVolt <= pdo_snk->var_snk.maxVoltage))
                    {
                        compare_temp = CY_PDUTILS_GET_MAX (pdo_snk->var_snk.opCurrent, max_min_temp);

                        if (pdo_src->var_src.maxCurrent >= compare_temp)
                        {
#if (!PMG1B1_SINK_PDO_REQ_MAX_ADV_CURR)
                            gl_contract_power[port] = calc_power(minVolt, pdo_snk->var_snk.opCurrent);
                            gl_op_cur_power[port]   = pdo_snk->var_snk.opCurrent;
#else /* (POWER_BANK) */
                            gl_contract_power[port] = calc_power(minVolt, pdo_src->var_src.maxCurrent);
                            gl_op_cur_power[port]   = pdo_src->var_src.maxCurrent;
#endif /* (POWER_BANK) */
                            out = true;
                        }
                    }
                    break;

                case CY_PDSTACK_PDO_BATTERY:
                    if((minVolt >= pdo_snk->bat_snk.minVoltage) && (maxVolt <= pdo_snk->bat_snk.maxVoltage))
                    {
                        /* Convert from power to current. */
                        oper_cur_pwr = calc_current(pdo_snk->bat_snk.opPower, minVolt);
                        max_min_temp = calc_current(max_min_temp, minVolt);

                        compare_temp = CY_PDUTILS_GET_MAX (oper_cur_pwr, max_min_temp);
                        if (pdo_src->var_src.maxCurrent >= compare_temp)
                        {
                            gl_contract_power[port] = pdo_snk->bat_snk.opPower;
                            gl_op_cur_power[port]   = oper_cur_pwr;
                            out = true;
                        }
                    }
                    break;

                default:
                    break;
            }

            if (out)
            {
                gl_contract_voltage[port] = maxVolt;
                gl_max_min_cur_pwr[port]  = max_min_temp;
            }
            break;

        default:
            break;
    }

    return out;
}

static cy_pd_pd_do_t form_rdo(cy_stc_pdstack_context_t* context, uint8_t pdo_no, bool capMisMatch, bool giveBack)
{
#if (CY_PD_REV3_ENABLE)
    const cy_stc_pd_dpm_config_t *dpm = &(context->dpmConfig);
#endif /* CY_PD_REV3_ENABLE */

    cy_pd_pd_do_t snkRdo;
    uint8_t port = context->port;

    snkRdo.val = 0u;
    snkRdo.rdo_gen.noUsbSuspend = context->dpmStat.snkUsbSuspEn;
    snkRdo.rdo_gen.usbCommCap = context->dpmStat.snkUsbCommEn;
    snkRdo.rdo_gen.capMismatch = capMisMatch;
    snkRdo.rdo_gen.objPos = pdo_no;
    snkRdo.rdo_gen.giveBackFlag = (capMisMatch) ? false : giveBack;
    snkRdo.rdo_gen.opPowerCur = gl_op_cur_power[port];
    snkRdo.rdo_gen.minMaxPowerCur = gl_max_min_cur_pwr[port];

    if (
            (snkRdo.rdo_gen.giveBackFlag == false) &&
            (snkRdo.rdo_gen.opPowerCur > snkRdo.rdo_gen.minMaxPowerCur)
       )
    {
        snkRdo.rdo_gen.minMaxPowerCur = snkRdo.rdo_gen.opPowerCur;
    }

#if (CY_PD_REV3_ENABLE)
    /* We will always support unchunked extended messages in PD 3.0 mode. */
    if (dpm->specRevSopLive >= CY_PD_REV3)
    {
        snkRdo.rdo_gen.unchunkSup = true;
    }

#endif /* (CY_PD_REV3_ENABLE) */

    return snkRdo;
}

static cy_en_pdstack_pdo_sel_alg_t src_pdo_selection_algorithm = (cy_en_pdstack_pdo_sel_alg_t)PD_PDO_SEL_ALGO;

/*
 * Evaluate the source capabilities listed by the source and pick the appropriate one to request for.
 */
void eval_src_cap(cy_stc_pdstack_context_t* context, const cy_stc_pdstack_pd_packet_t* srcCap,
        cy_pdstack_app_resp_cbk_t app_resp_handler)
{
    uint8_t src_pdo_index, snk_pdo_index;
    cy_pd_pd_do_t* snkPdo = (cy_pd_pd_do_t*)&context->dpmStat.curSnkPdo[0];
    uint8_t port = context->port;
    cy_stc_pdstack_dpm_status_t *dpm = &(context->dpmStat);
    uint16_t src_vsafe5_cur = srcCap->dat[0].fixed_src.maxCurrent; /* Source max current for first PDO */
    cy_pd_pd_do_t snkRdo;
    uint32_t highest_gl_contract_power = 0u;
    bool match = false;
    uint8_t src_pdo_len = srcCap->len;
    uint8_t snk_pdo_len = dpm->curSnkPdocount;

    for(snk_pdo_index = 0u; snk_pdo_index < snk_pdo_len; snk_pdo_index++)
    {
        for(src_pdo_index = 0u; src_pdo_index < src_pdo_len; src_pdo_index++)
        {
            if(is_src_acceptable_snk(context, (cy_pd_pd_do_t*)(&srcCap->dat[src_pdo_index]), snk_pdo_index))
            {
                bool max_cond = false;

                /*
                 * We support 4 different algorithms based on which the most appropriate Source PDO is selected:
                 * CY_PDSTACK_HIGHEST_POWER   : Pick the Fixed Source PDO which delivers maximum amount of power.
                 * CY_PDSTACK_HIGHEST_VOLTAGE : Pick the Fixed Source PDO which delivers power at max. voltage.
                 * CY_PDSTACK_HIGHEST_CURRENT : Pick the Fixed Source PDO which delivers the max. current.
                 * Default (none of the above): Pick the Source PDO which delivers max. amount of power.
                 */
                switch (src_pdo_selection_algorithm)
                {
                    case CY_PDSTACK_HIGHEST_POWER:
                        /* Contract_power is based on SRC PDO */
                        if (srcCap->dat[src_pdo_index].fixed_src.supplyType == CY_PDSTACK_PDO_FIXED_SUPPLY)
                        {
                            uint32_t temp_power = calc_power(srcCap->dat[src_pdo_index].fixed_src.voltage,
                                    srcCap->dat[src_pdo_index].fixed_src.maxCurrent);
                            if (temp_power >= highest_gl_contract_power)
                            {
                                highest_gl_contract_power = temp_power;
                                max_cond = true;
                            }
                        }
                        break;

                    case CY_PDSTACK_HIGHEST_VOLTAGE:
                        /* Only fixed pdo takes part */
                        if ((srcCap->dat[src_pdo_index].fixed_src.supplyType == CY_PDSTACK_PDO_FIXED_SUPPLY)
                                && (gl_contract_voltage[port] >= highest_gl_contract_power))
                        {
                            highest_gl_contract_power = gl_contract_voltage[port];
                            max_cond = true;
                        }
                        break;

                    case CY_PDSTACK_HIGHEST_CURRENT:
                        /* Only fixed pdo takes part */
                        if ((srcCap->dat[src_pdo_index].fixed_src.supplyType == CY_PDSTACK_PDO_FIXED_SUPPLY)
                                && (srcCap->dat[src_pdo_index].fixed_src.maxCurrent >= highest_gl_contract_power))
                        {
                            highest_gl_contract_power = srcCap->dat[src_pdo_index].fixed_src.maxCurrent;
                            max_cond = true;
                        }
                        break;

                    default:
                        /* Contract_power is calculated in is_src_acceptable_snk() */
                        if (gl_contract_power[port] >= highest_gl_contract_power)
                        {
                            highest_gl_contract_power = gl_contract_power[port];
                            max_cond = true;
                        }
                        break;
                }

                if (max_cond)
                {
                    /* Check if sink needs higher capability */
                    if (
                            (snkPdo[0].fixed_snk.highCap) &&
                            (gl_contract_voltage[port] == (CY_PD_VSAFE_5V/CY_PD_VOLT_PER_UNIT))
                       )
                    {
                        /* 5V contract isn't acceptable with highCap = 1 */
                        continue;
                    }

                    snkRdo = form_rdo(context, (src_pdo_index + 1u), false,
                            (context->dpmStat.curSnkMaxMin[snk_pdo_index] & CY_PD_GIVE_BACK_MASK));
                    match = true;
                }
            }
        }
    }

    if(match == false)
    {
        /* Capability mismatch: Ask for vsafe5v PDO with CapMismatch */
        gl_contract_voltage[port] = snkPdo[0].fixed_snk.voltage;
        gl_op_cur_power[port] = snkPdo[0].fixed_snk.opCurrent;
        gl_contract_power[port] = Cy_PdUtils_DivRoundUp(
                gl_contract_voltage[port] * gl_op_cur_power[port], 500u);

        if(src_vsafe5_cur < gl_op_cur_power[port])
        {
            /* SNK operation current can't be bigger than SRC maxCurrent */
            gl_op_cur_power[port] = src_vsafe5_cur;
        }

        gl_max_min_cur_pwr[port] = context->dpmStat.curSnkMaxMin[0];
        snkRdo = form_rdo(context, 1u, true, false);
    }
    /* Config table setting to always request max current instead of operation current */
#if defined(PD_REQUIRED_MAX_POWER)
    {
        uint32_t maxCurrent = srcCap->dat[snkRdo.rdo_gen.objPos - 1].fixed_src.maxCurrent;
        snkRdo.rdo_gen.opPowerCur = maxCurrent;
        snkRdo.rdo_gen.minMaxPowerCur = maxCurrent;
    }
#endif /* defined(PD_REQUIRED_MAX_POWER) */
    (app_get_resp_buf(port))->respDo = snkRdo;
    app_resp_handler(context, app_get_resp_buf(context->port));
}

#endif /* (!(CY_PD_SOURCE_ONLY)) */

#if (!CY_PD_SINK_ONLY)

/*
 * Verify that the Request RDO received from the sink is valid and can be supported.
 */
static cy_en_pdstack_status_t is_rdo_valid (cy_stc_pdstack_context_t* context, cy_pd_pd_do_t rdo)
{
    uint32_t src_pdo_index;
    uint32_t rdo_obj_pos = rdo.rdo_gen.objPos;
    cy_en_pdstack_status_t retval = CY_PDSTACK_STAT_FAILURE;
    cy_stc_pdstack_dpm_status_t *dpm = &(context->dpmStat);

    /* Validate the object position. */
    if ((rdo_obj_pos == 0) || (rdo_obj_pos > dpm->curSrcPdocount))
    {
        return retval;
    }

    src_pdo_index = rdo_obj_pos - 1;

    /* Check if RDO demands a valid power. */
    if ((rdo.rdo_gen.giveBackFlag != false) || (rdo.rdo_gen_gvb.capMismatch == true))
    {
        if (dpm->curSrcPdo[src_pdo_index].src_gen.maxCurPower >= rdo.rdo_gen.opPowerCur)
        {
            retval = CY_PDSTACK_STAT_SUCCESS;
        }
    }
    else
    {
        if (dpm->curSrcPdo[src_pdo_index].src_gen.maxCurPower >= rdo.rdo_gen_gvb.maxPowerCur)
        {
            retval = CY_PDSTACK_STAT_SUCCESS;
        }
    }

    return retval;
}

void eval_rdo(cy_stc_pdstack_context_t* context, cy_pd_pd_do_t rdo, cy_pdstack_app_resp_cbk_t app_resp_handler)
{
#if (CY_PD_REV3_ENABLE)
    /* If EPR is not supported and the MSB of RDO is set in a PD 3.0 contract, trigger hard reset. */
    if ((context->dpmConfig.specRevSopLive >= CY_PD_REV3) && ((rdo.val & 0x80000000u) != 0))
    {
        app_get_resp_buf(context->port)->reqStatus = CY_PDSTACK_REQ_SEND_HARD_RESET;
        app_resp_handler(context, app_get_resp_buf(context->port));
        return;
    }
#endif /* CY_PD_REV3_ENABLE */

    if (is_rdo_valid(context, rdo) == CY_PDSTACK_STAT_SUCCESS)
    {
        app_get_resp_buf(context->port)->reqStatus = CY_PDSTACK_REQ_ACCEPT;
    }
    else
    {
        app_get_resp_buf(context->port)->reqStatus = CY_PDSTACK_REQ_REJECT;
    }

    app_resp_handler(context, app_get_resp_buf(context->port));
}
#endif /* (!CY_PD_SINK_ONLY) */


/* End of File */

