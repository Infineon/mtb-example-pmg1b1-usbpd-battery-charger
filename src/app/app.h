/******************************************************************************
* File Name: app.h
*
* Description: This file defines data structures and function prototypes
*              associated with application level management of the USB-C Port.
*
* Related Document: See README.md
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

#ifndef _APP_H_
#define _APP_H_

/*******************************************************************************
 * Header files including
 ******************************************************************************/
#include <stdint.h>
#include "config.h"
#include "cy_usbpd_vbus_ctrl.h"
#include "cy_usbpd_typec.h"
#include "cy_pdstack_dpm.h"

/*******************************************************************************
 * MACRO Definition
 ******************************************************************************/

#define APP_PSINK_DIS_TIMER_PERIOD                      (250u)
/**< Maximum time allowed for power sink disable operation (in ms). */

#define APP_PSINK_DIS_MONITOR_TIMER_PERIOD              (1u)
/**< Period of VBus voltage checks performed during power sink disable operation. */

#define APP_PSINK_DIS_VBUS_IN_DIS_PERIOD                (20u)
/**< Duration of discharge sequence on the VBUS_IN supply. */

#define APP_FAULT_RECOVERY_TIMER_PERIOD                 (100u)
/**< Period of VBus presence checks after a fault (Over-Voltage) detection while in a sink contract. */

#define APP_FAULT_RECOVERY_MAX_WAIT                     (500u)
/**< Time for which VBus will be monitored to ensure removal of VBus by a faulty power source. */

#define APP_CBL_DISC_TIMER_PERIOD                       (100u)
/**< Delay to be used between cable discovery init commands. */

#define APP_BC_CDP_SM_TIMER_PERIOD                      (30000u)
/**< CDP state machine timeout period. */

#define APP_BC_VDP_DM_SRC_ON_PERIOD                     (50u)
/**< VDP_SRC or VDM_SRC minimum turn on time (the minimum from BC 1.2 spec is 40 ms). */

#define APP_BC_VDMSRC_EN_DIS_PERIOD                     (30u)
/**< VDM_SRC enable/disable maximum period. */
#define APP_BC_VBUS_CYCLE_TIMER_PERIOD                  (200u)
/**< VBUS OFF time to do a VBUS power cycle. */

#define APP_BC_SINK_CONTACT_STABLE_TIMER_PERIOD         (50u)
/**< Sink DCD stable time period. */

#define APP_BC_DCP_DETECT_TIMER_PERIOD                  (1100u)
/**< Tglitch_done time waiting for the portable device to complete detection.
     This is used by QC/AFC devices to proceed with subsequent detection. */

#define APP_BC_APPLE_DETECT_TIMER_PERIOD                (5u)
/**< Debounce time to verify if the attached device is Apple or not. Apple devices
     create a glitch on DP line whereas BC devices continue to drive DP lower.
     This period is used when Apple and BC 1.2 source protocols are supported
     together. */

#define APP_BC_DP_DM_DEBOUNCE_TIMER_PERIOD              (40u)
/**< Debounce time for identifying state change for DP and DM lines. */

#define APP_BC_AFC_DETECT_TIMER_PERIOD                  (100u)
/**< AFC detection time. */

#define APP_BC_GLITCH_BC_DONE_TIMER_PERIOD              (1500)
/**< TGLITCH_BC_DONE timer period. This timer is used in sink mode to detect QC charger. */

#define APP_BC_GLITCH_DM_HIGH_TIMER_PERIOD              (40)
/**< T_GLITCH_DM_HIGH timer period. After DCP opens D+/- short, sink shall wait for this time before requesting VBUS. */

#define APP_BC_V_NEW_REQUEST_TIMER_PERIOD               (200)
/**< T_V_NEW_REQUEST timer period. After entering QC mode, sink must wait this much time before requesting
     next voltage. */

/*****************************************************************************
 * Data Struct Definition
 ****************************************************************************/

/**
 * @typedef app_port_fault_status_mask_t
 * @brief Fault detection and handling related status bits tracked in the fault_status field.
 */
typedef enum {
    APP_PORT_FAULT_NONE                 = 0x00, /**< System functioning without any fault. */
    APP_PORT_VCONN_FAULT_ACTIVE         = 0x01, /**< Status bit that indicates VConn fault is active. */
    APP_PORT_SINK_FAULT_ACTIVE          = 0x02, /**< Status bit that indicates sink fault handling is pending. */
    APP_PORT_SRC_FAULT_ACTIVE           = 0x04, /**< Status bit that indicates source fault handling is pending. */
    APP_PORT_VBUS_DROP_WAIT_ACTIVE      = 0x08, /**< Status bit that indicates wait for VBus drop is pending. */
    APP_PORT_V5V_SUPPLY_LOST            = 0x10, /**< Status bit that indicates that V5V supply (for VConn) has been lost. */
    APP_PORT_DISABLE_IN_PROGRESS        = 0x80  /**< Port disable operation is in progress. */
} app_port_fault_status_mask_t;

/**
 * @typedef app_nb_sys_pwr_state_t
 * @brief List of system power states for Notebook/Desktop designs.
 */
typedef enum
{
    NB_SYS_PWR_STATE_S0 = 0,            /**< Notebook/Desktop is in active (S0) state. */
    NB_SYS_PWR_STATE_S3,                /**< Notebook/Desktop is in Sleep (S3) state. */
    NB_SYS_PWR_STATE_S4,                /**< Notebook/Desktop is in Hibernate (S4) state. */
    NB_SYS_PWR_STATE_S5                 /**< Notebook/Desktop is in Off (S5) state. */
} app_nb_sys_pwr_state_t;

/**
   @brief This structure holds the Discover Identity response information.
 */
typedef struct
{
    uint32_t discId[CY_PD_MAX_NO_OF_DO];        /**< The data objects that make up the Discover Identity Response. */
    uint8_t  respLength;                        /**< Length of the response in number of Data Objects. */
} vdm_info_config_t;

/**
   @brief This structure hold all variables related to application layer functionality.
 */
typedef struct
{
    cy_pdstack_pwr_ready_cbk_t          pwr_ready_cbk;          /**< Registered Power source callback. */
    cy_pdstack_sink_discharge_off_cbk_t snk_dis_cbk;            /**< Registered Power sink callback. */
    app_resp_t                          appResp;               /**< Buffer for APP responses. */
    vdm_resp_t                          vdmResp;               /**< Buffer for VDM responses. */
    uint16_t                            psnk_volt;              /**< Current PSink voltage in mV units. */
    uint16_t                            psnk_cur;               /**< Current PSink current in 10mA units. */
    uint8_t                             vdm_task_en;            /**< Flag to indicate is vdm task manager enabled. */
    uint8_t                             disc_cbl_pending;       /**< Flag to indicate is cable discovery is pending. */
    uint8_t                             cbl_disc_id_finished;   /**< Flag to indicate that cable disc id finished. */
    uint8_t                             vdm_version;            /**< Live VDM version. */
    uint8_t                             vdm_minor_version;      /**< Live VDM minor version. */
    uint8_t                             fault_status;           /**< Fault status bits for this port. */
    bool                                alt_mode_entered;       /**< Alternate modes currently entered. */
    cy_pdstack_vdm_resp_cbk_t           vdm_resp_cbk;           /**< VDM response handler callback. */
} app_status_t;

/**
   @brief This structure hold all solution level functions used as a part of application layer.
 */
typedef struct
{
    void (*sln_pd_event_handler) (
            cy_stc_pdstack_context_t* context,  /**< PD stack context. */ 
            cy_en_pdstack_app_evt_t evt,    /**< Type of event. */
            const void *data             /**< Event related data. */                
            );                          /**< App Solution event handler callback. */
    
    cy_stc_pdstack_app_cbk_t* (*app_get_callback_ptr)(
            cy_stc_pdstack_context_t * context  /**< PD stack context. */ 
            );                          /**< App Solution handler for getting callback structure. */

    cy_stc_pdstack_context_t* (*Get_PdStack_Context)(
            uint8_t portIdx                     /**< PD port index. */ 
            );                          /**< App Solution handler for mapping port index to context. */

    bool (*mux_ctrl_init)(
            uint8_t port                 /**< PD port index. */ 
            );                          /**< App Solution level function to initialize the mux. */
    cy_en_pdstack_status_t (*uvdm_handle_device_reset)(   
            uint32_t reset_sig                  /**< Device Reset Signature */
            );                          /**< App Solution level function to reset device. */       
}app_sln_handler_t;
/*****************************************************************************
 * Global Function Declaration
 *****************************************************************************/

/**
 * @brief Application level init function.
 *
 * This function performs any Application level initialization required
 * for the PMG1 solution. This should be called before calling the
 * dpmInit function.
 *
 * @return None.
 *
 */
void app_init(cy_stc_pdstack_context_t *ptrPdStackContext);

/**
 * @brief Handler for application level asynchronous tasks.
 * @param ptrPdStackContext Pointer to the pdstack context.
 * @return 1 in case of success, 0 in case of task handling error.
 */
uint8_t app_task(cy_stc_pdstack_context_t *ptrPdStackContext);

/**
 * @brief This function return the App callback structure pointer
 * @param ptrPdStackContext Pointer to the pdstack context.
 * @return  Application callback structure pointer
 */
cy_stc_pdstack_app_cbk_t* app_get_callback_ptr(cy_stc_pdstack_context_t * context);

/**
 * @brief Handler for event notifications from the PD stack.
 * @param ptrPdStackContext Pointer to the pdstack context.
 * @param evt Type of event to be handled.
 * @param dat Data associated with the event.
 * @return None
 */
void app_event_handler(cy_stc_pdstack_context_t *ptrPdStackContext, 
               cy_en_pdstack_app_evt_t evt, const void* dat);

/**
 * @brief Get a handle to the application provide PD command response buffer.
 * @param ptrPdStackContext Pointer to the pdstack context.
 * @return Pointer to the response buffer.
 */
app_resp_t* app_get_resp_buf(uint8_t port);

/**
 * @brief Get handle to structure containing information about the system status for a PD port.
 * @param ptrPdStackContext Pointer to the pdstack context.
 * @return Pointer to the system information structure.
 */
app_status_t* app_get_status(uint8_t port);

/**
 * @brief Function to place PMG1 device in power saving mode if possible.
 *
 * This function places the PMG1 device in power saving deep sleep mode
 * if possible. The function checks for each interface (PD, HPI etc.)
 * being idle and then enters sleep mode with the appropriate wake-up
 * triggers. If the device enters sleep mode, the function will only
 * return after the device has woken up.
 *
 * @return true if the device went into sleep, false otherwise.
 */
bool system_sleep(cy_stc_pdstack_context_t *ptrPdStackContext, cy_stc_pdstack_context_t *ptrPdStack1Context);

/*****************************************************************************
  Functions related to power
 *****************************************************************************/

/**
 * @brief This function enables VCONN power
 *
 * @param ptrPdStackContext Pointer to the pdstack context.
 * @param channel Selected CC line.
 *
 * @return True if VConn was turned ON; false if NOT.
 */

bool vconn_enable(cy_stc_pdstack_context_t *ptrPdStackContext, uint8_t channel);

/**
 * @brief This function disables VCONN power
 *
 * @param ptrPdStackContext Pointer to the pdstack context.
 * @param channel Selected CC line.
 *
 * @return None
 */
void vconn_disable(cy_stc_pdstack_context_t *ptrPdStackContext, uint8_t channel);

/**
 * @brief This function checks if power is present on VConn
 *
 * @param ptrPdStackContext Pointer to the pdstack context.
 *
 * @return true if power is present on VConn, else returns false
 */
bool vconn_is_present(cy_stc_pdstack_context_t *ptrPdStackContext);

/**
 * @brief This function checks if power is present on VBus
 *
 * @param ptrPdStackContext Pointer to the pdstack context.
 * @param volt Voltage in mV units.
 * @param per  Threshold margin.
 *
 * @return true if power is present on VBus, else returns false
 */
bool vbus_is_present(cy_stc_pdstack_context_t *ptrPdStackContext, uint16_t volt, int8_t per);

/**
 * @brief This function return current VBUS voltage in mV
 *
 * @param ptrPdStackContext Pointer to the pdstack context.
 *
 * @return VBUS voltage in mV
 */
uint16_t vbus_get_value(cy_stc_pdstack_context_t *ptrPdStackContext);

/**
 * @brief This function turns on dischange FET
 *
 * @param ptrPdStackContext Pointer to the pdstack context.
 *
 * @return None
 */
void vbus_discharge_on(cy_stc_pdstack_context_t* context);

/**
 * @brief This function turns off dischange FET
 *
 * @param ptrPdStackContext Pointer to the pdstack context.
 *
 * @return None
 */
void vbus_discharge_off(cy_stc_pdstack_context_t* context);

/**
 * @brief Enable and configure the Over-Voltage protection circuitry.
 * @param ptrPdStackContext Pointer to the pdstack context.
 * @param volt_mV Expected VBus voltage.
 * @param pfet Whether PFET is used for the power supply control.
 * @param ovp_cb Callback function to be triggered when there is an OV event.
 * @return None
 */
void app_ovp_enable(cy_stc_pdstack_context_t *ptrPdStackContext, uint16_t volt_mV, bool pfet, cy_cb_vbus_fault_t ovp_cb);

/**
 * @brief Disable the Over-Voltage protection circuitry.
 * @param ptrPdStackContext Pointer to the pdstack context.
 * @param pfet Whether PFET is used for the power supply control.
 * @return None
 */
void app_ovp_disable(cy_stc_pdstack_context_t *ptrPdStackContext, bool pfet);

/**
 * @brief Enable and configure the Under-Voltage protection circuitry.
 * @param ptrPdStackContext Pointer to the pdstack context.
 * @param volt_mV Expected VBus voltage.
 * @param pfet Whether PFET is used for the power supply control.
 * @param uvp_cb Callback function to be triggered when there is an UV event.
 * @return None
 */
void app_uvp_enable(cy_stc_pdstack_context_t *ptrPdStackContext, uint16_t volt_mV, bool pfet, cy_cb_vbus_fault_t uvp_cb);

/**
 * @brief Disable the Under-Voltage protection circuitry.
 * @param ptrPdStackContext Pointer to the pdstack context.
 * @param pfet Whether PFET is used for the power supply control.
 * @return None
 */
void app_uvp_disable(cy_stc_pdstack_context_t *ptrPdStackContext, bool pfet);

/*****************************************************************************
  Functions to be provided at the solution level.
 *****************************************************************************/

/**
 * @brief Solution handler for PD events reported from the stack.
 *
 * The function provides all PD events to the solution. For a solution
 * supporting HPI, the solution function should re-direct the calls to
 * hpi_pd_event_handler.
 *
 * @param ptrPdStackContext Pointer to the pdstack context.
 * @param evt Event that is being notified.
 * @param data Data associated with the event. This is an opaque pointer that
 * needs to be de-referenced based on event type.
 *
 * @return None
 */
void sln_pd_event_handler(cy_stc_pdstack_context_t *ptrPdStackContext, 
               cy_en_pdstack_app_evt_t evt, const void *data);


/**
 * @brief Initialize fault-handling related variables from the configuration table.
 * @param ptrPdStackContext Pointer to the pdstack context.
 * @return false if the configuration table is invalid, true otherwise.
 */
bool fault_handler_init_vars (cy_stc_pdstack_context_t *ptrPdStackContext);

/**
 * @brief Register solution space function handlers.
 *
 * @param ptrPdStackContext PD stack context.
 *
 * @return None.
 */
void fault_handler_register_cbks (cy_stc_pdstack_context_t *ptrPdStackContext);

/**
 * @brief Clear the fault occurrence counts after a Type-C detach is detected.
 * @param ptrPdStackContext Pointer to the pdstack context.
 * @return None.
 */
void fault_handler_clear_counts (uint8_t port);

/**
 * @brief Perform any fault handler related tasks.
 * @param ptrPdStackContext Pointer to the pdstack context.
 * @return None
 */
void fault_handler_task (cy_stc_pdstack_context_t *ptrPdStackContext);

/**
 * @brief Handle any application events associated with fault handling logic.
 * @param ptrPdStackContext Pointer to the pdstack context.
 * @param evt Event code.
 * @param dat Data associated with the event code.
 * @return true if the event does not need to be passed up to the solution layer.
 */
bool fault_event_handler(cy_stc_pdstack_context_t *ptrPdStackContext, cy_en_pdstack_app_evt_t evt, const void *dat);

/**
 * @brief Function to initialize timer callbacks
 *
 * @param ptrPdStackContext PD stack context.
 */
void timer_init(cy_stc_pdstack_context_t *ptrPdStackContext);

/**
 * @brief Prepare the PMG1 to wait for physical detach of faulty port partner.
 *
 * @param ptrPdStackContext Pointer to the pdstack context.
 *
 * @return None
 */
void app_conf_for_faulty_dev_removal(cy_stc_pdstack_context_t * ptrPdStackcontext);

/**
 * @brief Check whether any fault count has exceeded limit for the specified PD port. 
 *
 * @param ptrPdStackContext Pointer to the pdstack context.
 *
 * @return true if fault count has exceeded limit, false otherwise.
 */    
bool app_port_fault_count_exceeded(cy_stc_pdstack_context_t * ptrPdStackcontext);

/**
 * @brief Function registers solution level functions
 *
 * @param ptrPdStackcontext PD stack context.
 * @param handler pointer to solution handler. 
 *
 * @return None
 */
void register_soln_function_handler(cy_stc_pdstack_context_t *ptrPdStackcontext, app_sln_handler_t *handler);

/**
 * @brief Status of Type-C attach on all Type-C ports.
 *
 * @return True if any port is attached. False if all ports are unattached.
 */
bool app_is_typec_attached(void);

/** Callback function from interrupt handler to handle buck-boost inductor current limit fault.
 *
 * \param callbackContext USBPD Context pointer.
 *
 * \param state Current state of fault. True indicates fault is active.
 */
void pd_bb_ilim_fault_handler(void *callbackContext, bool state);

/** Interrupt handler for VDDD regulator brown out fault.
 *
 * \param callbackContext USBPD Context pointer.
 *
 * \param state Current state of fault. True indicates fault is active.
 */
void pd_brown_out_fault_handler(void *callbackContext, bool state);

/** Interrupt handler for VDDD regulator inrush current fault.
 *
 * \param callbackContext USBPD Context pointer.
 *
 * \param state Current state of fault. True indicates fault is active.
 */
void pd_vreg_inrush_det_fault_handler(void *callbackContext, bool state);
#endif /* _APP_H_ */

/* End of File */
