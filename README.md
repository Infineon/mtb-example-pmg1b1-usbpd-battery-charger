# EZ-PD&trade; PMG1B1 MCU: USBPD battery charger

bThis code example demonstrates 5-cell Li-ion battery charging with USB Type-C attach detection and USB Power Delivery contract negotiation using a single chip EZ-PD™ PMG1-B1 MCU device. The EZ-PD&trade; PMG1-B1 MCU device can negotiate upto 100W (20V @5A) Power Delivery contract with a USB-C source. The EZ-PD&trade PMG1-B1's integrated buck boost peripheral converts the negotiated VBUS power to charge the 5-cell Li-ion battery.

[View this README on GitHub.](https://github.com/Infineon/mtb-example-pmg1b1-usbpd-battery-charger)

[Provide feedback on this code example.](https://cypress.co1.qualtrics.com/jfe/form/SV_1NTns53sK2yiljn?Q_EED=eyJVbmlxdWUgRG9jIElkIjoiQ0UyMzMzNTQiLCJTcGVjIE51bWJlciI6IjAwMi0zMzM1NCIsIkRvYyBUaXRsZSI6IkVaLVBEJnRyYWRlOyBQTUcxQjEgTUNVOiBVU0JQRCBiYXR0ZXJ5IGNoYXJnZXIiLCJyaWQiOiJrb25kIiwiRG9jIHZlcnNpb24iOiIxLjAuMCIsIkRvYyBMYW5ndWFnZSI6IkVuZ2xpc2giLCJEb2MgRGl2aXNpb24iOiJNQ0QiLCJEb2MgQlUiOiJXSVJFRCIsIkRvYyBGYW1pbHkiOiJUWVBFLUMifQ==)

## Requirements

- [ModusToolbox&trade;](https://www.infineon.com/modustoolbox) v3.0 or later (tested with v3.0)
- Board support package (BSP) minimum required version: 3.0.0
- Programming language: C
- Associated parts: [EZ-PD&trade; PMG1-B1 MCU](https://www.infineon.com/PMG1)

## Supported toolchains (make variable 'TOOLCHAIN')

- GNU Arm&reg; Embedded Compiler v10.3.1 (`GCC_ARM`) - Default value of `TOOLCHAIN`
- Arm&reg; Compiler v6.16 (`ARM`)
- IAR C/C++ Compiler v9.30.1 (`IAR`)

## Supported kits (make variable 'TARGET')

- [EZ-PD&trade; PMG1-B1 DRP Prototyping Kit](https://www.infineon.com/EVAL_PMG1_B1_DRP) (`EVAL_PMG1_B1_DRP`) - Default value of `TARGET`

## Hardware setup

1. Connect the board to your PC using the USB cable through the KitProg3 USB connector. This cable is used for programming the EZ-PD&trade; PMG1 device. Additionally, it is used during debugging.

2. Ensure the SW4, SW5, SW6, SW7 and SW8 are in default 2-3 positions.

3. Connect the positive and negative terminals of 5-cell battery to J9; follow polarity as shown on the EVAL_PMG1_B1_DRP kit board.

4. Connect the USBPD port to the USB-C power adapter using the USB Type-C cable. This cable is used for the USB power delivery source and it provides power to the user LED.

See the kit user guide for more details on configuring the board.


## Software setup
This code example does not need any additional software or tools however, [EZ-PD&trade; Protocol Analyzer Utility](https://www.infineon.com/cms/en/product/evaluation-boards/cy4500) can be used to analyze and debug the USB PD communication on the Configuration Channel (CC) line.

## Using the code example

### Create the project

The ModusToolbox&trade; tools package provides the Project Creator as both a GUI tool and a command line tool.

<details><summary><b>Use Project Creator GUI</b></summary>

1. Open the Project Creator GUI tool.

   There are several ways to do this, including launching it from the dashboard or from inside the Eclipse IDE. For more details, see the [Project Creator user guide](https://www.infineon.com/ModusToolboxProjectCreator) (locally available at *{ModusToolbox&trade; install directory}/tools_{version}/project-creator/docs/project-creator.pdf*).

2. On the **Choose Board Support Package (BSP)** page, select a kit supported by this code example. See [Supported kits](#supported-kits-make-variable-target).

   > **Note:** To use this code example for a kit not listed here, you may need to update the source files. If the kit does not have the required resources, the application may not work.

3. On the **Select Application** page:

   a. Select the **Applications(s) Root Path** and the **Target IDE**.

   > **Note:** Depending on how you open the Project Creator tool, these fields may be pre-selected for you.

   b.	Select this code example from the list by enabling its check box.

   > **Note:** You can narrow the list of displayed examples by typing in the filter box.

   c. (Optional) Change the suggested **New Application Name** and **New BSP Name**.

   d. Click **Create** to complete the application creation process.

</details>

<details><summary><b>Use Project Creator CLI</b></summary>

The 'project-creator-cli' tool can be used to create applications from a CLI terminal or from within batch files or shell scripts. This tool is available in the *{ModusToolbox&trade; install directory}/tools_{version}/project-creator/* directory.

Use a CLI terminal to invoke the 'project-creator-cli' tool. On Windows, use the command-line 'modus-shell' program provided in the ModusToolbox&trade; installation instead of a standard Windows command-line application. This shell provides access to all ModusToolbox&trade; tools. You can access it by typing "modus-shell" in the search box in the Windows menu. In Linux and macOS, you can use any terminal application.
The following example clones the "[PMG1B1 USBPD Sink](https://github.com/Infineon/mtb-example-pmg1b1-usbpd-sink)" application with the desired name "MyUsbPdSink" configured for the *EVAL_PMG1_B1_DRP* BSP into the specified working directory, *C:/mtb_projects*:

   ```
   project-creator-cli --board-id EVAL_PMG1_B1_DRP --app-id mtb-example-pmg1b1-usbpd-battery-charger --user-app-name MyUsbPdDrp --target-dir "C:/mtb_projects"
   ```

The 'project-creator-cli' tool has the following arguments:

Argument | Description | Required/optional
---------|-------------|-----------
`--board-id` | Defined in the <id> field of the [BSP](https://github.com/Infineon?q=bsp-manifest&type=&language=&sort=) manifest | Required
`--app-id`   | Defined in the <id> field of the [CE](https://github.com/Infineon?q=ce-manifest&type=&language=&sort=) manifest | Required
`--target-dir`| Specify the directory in which the application is to be created if you prefer not to use the default current working directory | Optional
`--user-app-name`| Specify the name of the application if you prefer to have a name other than the example's default name | Optional

>**Note:** The project-creator-cli tool uses the `git clone` and `make getlibs` commands to fetch the repository and import the required libraries. For details, see the "Project creator tools" section of the [ModusToolbox&trade; software user guide](https://www.infineon.com/ModusToolboxUserGuide) (locally available at *{ModusToolbox&trade; software install directory}/docs_{version}/mtb_user_guide.pdf*).

To work with a different supported kit later, use the [Library Manager](https://www.infineon.com/ModusToolboxLibraryManager) to choose the BSP for the supported kit. You can invoke the Library Manager GUI tool from the terminal using the `make library-manager` command or use the Library Manager CLI tool "library-manager-cli" to change the BSP.

The "library-manager-cli" tool has the following arguments:

Argument | Description | Required/optional
---------|-------------|-----------
`--add-bsp-name` | Name of the BSP that should be added to the application | Required
`--set-active-bsp` | Name of the BSP that should be as active BSP for the application | Required
`--add-bsp-version`| Specify the version of the BSP that should be added to the application if you do not wish to use the latest from manifest | Optional
`--add-bsp-location`| Specify the location of the BSP (local/shared) if you prefer to add the BSP in a shared path | Optional

<br />

Following example adds the EVAL_PMG1_B1_DRP BSP to the already created application and makes it the active BSP for the app:

   ```
   library-manager-cli --project "C:/mtb_projects/MyUsbPdDrp" --add-bsp-name EVAL_PMG1_B1_DRP --add-bsp-version "latest-v3.X" --add-bsp-location "local"

   library-manager-cli --project "C:/mtb_projects/MyUsbPdDrp" --set-active-bsp APP_EVAL_PMG1_B1_DRP
   ```

</details>

### Open the project

After the project has been created, you can open it in your preferred development environment.


<details><summary><b>Eclipse IDE</b></summary>

If you opened the Project Creator tool from the included Eclipse IDE, the project will open in Eclipse automatically.

For more details, see the [Eclipse IDE for ModusToolbox&trade; user guide](https://www.infineon.com/MTBEclipseIDEUserGuide) (locally available at *{ModusToolbox&trade; install directory}/docs_{version}/mt_ide_user_guide.pdf*).

</details>

<details><summary><b>Command line</b></summary>

If you prefer to use the CLI, open the appropriate terminal, and navigate to the project directory. On Windows, use the command-line 'modus-shell' program; on Linux and macOS, you can use any terminal application. From there, you can run various `make` commands.

For more details, see the [ModusToolbox&trade; tools package user guide](https://www.infineon.com/ModusToolboxUserGuide) (locally available at *{ModusToolbox&trade; install directory}/docs_{version}/mtb_user_guide.pdf*).

</details>


## Operation

1. Ensure that the steps listed in the [Hardware setup](#hardware-setup) section are complete.

2. Program the board using one of the following:

   <details><summary><b>Using Eclipse IDE for ModusToolbox&trade; software</b></summary>

      1. Select the application project in the Project Explorer.

      2. In the **Quick Panel**, scroll down, and click **\<Application Name> Program (KitProg3_MiniProg4)**.
   </details>

   <details><summary><b>Using CLI</b></summary>

     From the terminal, execute the `make program` command to build and program the application using the default toolchain to the default target. The default toolchain and target are specified in the application's Makefile but you can override those values manually:
      ```
      make program TOOLCHAIN=<toolchain>
      ```

      Example:
      ```
      make program TOOLCHAIN=GCC_ARM
      ```
   </details>

1. Connect a USB-C adapter source and EZ-PD&trade; Protocol Analyzer on the USB PD port of the kit to confirm the PD sink and battery charging functionality. The PD contract can be observed on the Protocol Analyzer connected in between. 
2. A multimeter is used to measure the battery voltage at the J9 output load terminals.

3. LED3 (STATUS LED) blinking indicates charging the battery, LED4 (POWER LED) indicates the board is powered.

## Debugging

You can debug the example to step through the code. In the IDE, use the **\<Application Name> Debug (KitProg3_MiniProg4)** configuration in the **Quick Panel**. Ensure that the board is connected to your PC using the USB cable connected to the programming/debugging port of the kit. See the "Debug mode" section in the kit user guide.

For more details, see the "Program and debug" section in the [Eclipse IDE for ModusToolbox&trade; software user guide](https://www.infineon.com/MTBEclipseIDEUserGuide).

## Design and implementation

EZ-PD&trade; PMG1 MCU devices support a USBPD block that integrates Type-C terminations, comparators, and the Power Delivery transceiver required to detect the attachment of a partner device and negotiate power contracts with it. In addition, the EZ-PD™ PMG1-B1 MCU's buck-boost subsystem has a built-in constant current loop(CC) and a constant voltage loop(CV), which transition automatically from CC to CV mode and vice-versa based on the connected battery pack status. In CC mode, the EZ-PD™ PMG1-B1 modulates the feedback to keep the charging current constant; in CV mode, the battery pack voltage is constant.

On reset, the application initializes the USBPD block and buck boost block with the following settings:

   - The receiver clock input of the block is connected to a 12-MHz PERI-derived clock

   - The transmitter clock input of the block is connected to a 600-kHz PERI-derived clock

   - The SAR ADC clock input of the block is connected to a 1-MHz PERI-derived clock

   - The SAR ADC in the USBPD block is configured to measure the VBUS_TYPE-C voltage through an internal divider

   - The buck boost clock input of the block is connected to a 24-MHz PERI-derived clock
   
   - The VBUS voltage and the current transition clocks are set to 24-KHz PERI-derived clock

   - The buck boost PWM Operating mode is set to FCCM and the PWM frequency is set to 400 KHz

   - The buck boost output voltage is initialized to 20 V

This application uses the PDStack Middleware Library in a sink role as shown in **Figure 1**

**Figure 1. Firmware flowchart**

<img src = "images/usbpd_drp_flow.png" width = "400"/>
<br>

The PDStack Middleware Library configures the USBPD block on the EZ-PD&trade; PMG1 MCU device to detect Type-C connection state changes and USBPD messages, and notify the stack through callback functions. The callback function registers the pending tasks, which are then handled by PDStack through the `Cy_PdStack_Dpm_Task` function. This function is expected to be called at appropriate times from the main processing loop of the application.

**Figure 2. PDStack task flowchart**

<img src = "images/pdstacktask.png" width = "600"/>

<br>

The PDStack Middleware Library implements the state machines defined in the [USB Type-C Cable and Connector](https://www.usb.org/document-library/usb-type-cr-cable-and-connector-specification-revision-20) and the [USB Power Delivery](https://www.usb.org/document-library/usb-power-delivery) specifications. PDStack consists of the following main modules:

- **Type-C Manager:** Responsible for detecting a Type-C connection and identifying the type of connection. It uses the configurable Rp/Rd terminations provided by the USBPD block and the internal line state comparators. The Type-C manager implements the state machines defined in the **USB Type-C Cable and Connector specification** and provides the following functionality:

    - **Manage CC terminations**: Applies Rp/Rd terminations according to the port role

    - **Attach detection**: Perform the required debounce and determine the type of device attached

    - **Detach detection**: Monitor the CC line and VBus for detecting a device detach

- **Protocol Layer:** Forms the messages used to communicate between a pair of ports/cable plugs. It is responsible for forming capabilities messages, requests, responses, and acknowledgements. It receives inputs from the Policy Engine indicating which messages to send and relays the responses back to the policy engine.

- **Policy Engine:** Provides a mechanism to monitor and control the USB Power Delivery system within a particular consumer, provider, or cable plug. It implements the state machines defined in the **USB Power Delivery** specification and contains implementations of all PD atomic message sequences (AMS). It interfaces with the protocol layer for PD message transmission/reception for controlling the reception of message types according conditions such as the current state of the port. It also interfaces with the Type-C Manager for error conditions like Type-C error recovery.

- **Device Policy Manager (DPM):** Provides an interface to the application layer to initialize, monitor, and configure the PDStack middleware operation. The DPM provides the following functionality:

    - Initialize the Policy Engine and Type-C Manager

    - Start the Type-C state machine followed by the Policy Engine state machine

    - Stop and disable the Type-C port

    - Allow entry/exit from Deep Sleep to achieve low-power based on the port status

    - Provide APIs for the application to send PD/Type-C commands

    - Provide event callbacks to the application for application-specific handling

The PDStack library uses a set of callbacks registered by the application to perform board-specific tasks such as turning the consumer or provider power path ON/OFF and identifying the optimal source power profile to be used for charging. In this example, these functions are implemented using the appropriate APIs provided as part of the Peripheral Driver Library (PDL).

The stack also provides notification of various connection and PD policy state changes so that the rest of the system can be configured as required.

The MCU's application layer run the Battery Charging Task which monitors the system for battery connection/disconnection, USB-C adapter connection/disconnection, battery voltage, battery charging current and board temperatures  etc periodically. The application layer also implements a smart battery charging algorithm that enables maximum utilization of the available input USB-C power.

An overvoltage (OV) comparator in the USBPD block is used to detect cases where the power source is supplying incorrect voltage levels, and automatically shutdown the power switches to protect the rest of the components on the board.

### Compile-time configurations

The EZ-PD&trade; PMG1B1 MCU USBPD charger application functionality can be customized through a set of compile-time parameters that can be turned ON/OFF through *config.h* and *Makefile* header files.

| Macro name          | Description                           | Allowed values |
| :------------------ | :------------------------------------ | :------------- |
| `CY_PD_SINK_ONLY`     | Specifies that the application supports only the USB PD sink (consumer) role | Set to 1u |
| `NO_OF_TYPEC_PORTS`   | Specifies the number of USB-C ports supported | Set to 1u |
| `CY_PD_REV3_ENABLE`   | Enable USB PD Revision 3.1 support | 1u or 0u |
| `CY_PD_CBL_DISC_DISABLE` | Disable cable discovery | 1u |
| `VBUS_OVP_ENABLE` | Enable VBus overvoltage fault detection | 1u or 0u|
| `VBUS_UVP_ENABLE` | Enable VBus undervoltage fault detection | 0u |
| `VBUS_RCP_ENABLE` | Enable VBus reverse current fault detection| 1u or 0u|
| `VBUS_SCP_ENABLE` | Enable VBus short-circuit fault detection | 1u or 0u|
| `VCONN_OCP_ENABLE` | Enable VConn overcurrent fault detection | 1u or 0u|
| `VBUS_OCP_ENABLE` | Enable VBUS overcurrent fault detection | 1u or 0u|
| `PD_PDO_SEL_ALGO`     | Specifies the algorithm to be used while selecting the best source capability to power the board | 0u – Pick the source PDO delivering the maximum power <br> 1u – Pick the fixed source PDO delivering the maximum power <br>2u – Pick the fixed source PDO delivering the maximum current<br>3u – Pick the fixed source PDO delivering the maximum voltage |
| `SYS_DEEPSLEEP_ENABLE` | Enables device entry into Deep Sleep mode for power saving when the CPU is idle | 1u or 0u |
| `DEBUG_UART_ENABLE` | Enables UART-based debug output | 1u or 0u |
| `TOTAL_BATTERY_CELL_COUNT` | Battery cell count | 5u |
| `VBAT_INPUT_CURR_MAX_SETTING` | Total allowed BB MAX output current in 10mA (Rsense = 5 mΩ) | 200u |
| `CELL_MONITORING_DISABLE` | Disable invidual battery cell monitoring by ADC | 1u or 0u |
| `ENABLE_BATT_CURR_MONITORING` | Enable the battery current monitoring | 1u or 0u |
<br>

### PDStack Library selection

The USB Type-C Connection Manager, USB Power Delivery (USBPD) protocol layer, and USBPD device Policy Engine state machine implementations are provided in the form of pre-compiled libraries as part of the PDStack Middleware Library.

Multiple variants of the PDStack library with different feature sets are provided; you can choose the appropriate version based on the features required by the target application.

In this application, the *PMG1_PD3_SNK_LITE* library with support for USB Type-C DRP EPR operation and USB PD revision 3.1 messaging is chosen by default.

### Buck boost configuration

The properties of the buck boost subsytem can be configured using the Device Configurator Utility.

By default, these parameters are set to the appropriate values for a battery charging application. To view or change the configuration, click the **Device Configurator 4.10** under **Tools** in the Quick Panel to launch the configurator.

The buck boost configuration parameters are part of the **USB-C Power Delivery 0** resource. By default, the USB-C Power Delivery resource is enabled as shown in **Figure 3**.

**Figure 3. Buck boost configuration using Device Configurator**

<img src = "images/buck_boost_info.png" width = "1000"/>

<br>

### USBPD port configuration

The properties of the USB-C port including the port role and the default response to various USBPD messages can be configured using the EZ-PD&trade; Configurator Utility.

These parameters have been set to the appropriate values for a Power Delivery DRP application by default. To view or change the configuration, click the **EZ-PD&trade; Configurator 1.21** under **Tools** in the Quick Panel to launch the configurator.

**Figure 4. USB Type-C port configuration using EZ-PD&trade; Configurator**

<img src = "images/ezpd_port_info.png" width = "1000"/>

<br>

The properties of the USB-C port are configured using the *Port Information* section. Because this application supports sink role, the **Port Role** must be set as **Sink** and **DRP Toggle** must be disabled. Other parameters such as **Manufacturer Vendor ID** and **Manufacturer Product ID** can be set to desired values.

**Figure 5. Sink capability configuration using EZ-PD&trade; Configurator**

<img src = "images/ezpd_sink_pdo.png" width = "1000"/>

<br>

The power capabilities supported by the application in the sink role are specified using the **Sink PDO** section. See the **USB Power Delivery** specification for details on how to encode the various sink capabilities. A maximum of seven PDOs can be added using the configurator.

**Figure 6. Extended sink capability configuration using EZ-PD&trade; Configurator**

<img src = "images/ezpd_skedb_info.png" width = "1000"/>

<br>

The **SKEDB** section is used to input the extended sink capabilities response that is sent by the application when queried by the power source. See the Power Delivery specification for more details on the extended sink capabilities format.

Once the parameters have been updated as desired, save the configuration and build the application.


### Resources and settings

**Table 1. Application resources**

| Resource  | Alias/object   | Purpose                               |
| :-------  | :------------  | :------------------------------------ |
| USBPD | PD_PORT0 | USBPD block used for PD communication |
| SCB0 | UART | UART interface to output debug information |
| Programmable Analog (PASS) 0 | PASS 0 12-bit SAR ADC 0   | 12-bit SAR ADC used to monitor battery voltage and board temperatures |


<br>

### List of application files and their usage

| File                         | Purpose                               |
| :--------------------------- | :------------------------------------ |
| *src/app/app.c & .h*                | Defines data structures, function prototypes, and implements functions to handle application-level USB Type-C and PD events |
| *src/app/fault_handlers.c*          | Implements functions to handle faults related to USB Type-C and PD |
| *src/app/pdo.c & .h*                | Defines function prototypes and implements functions to evaluate source capabilities (power data object) |
| *src/app/psink.c & .h*              | Defines function prototypes and implements functions for power consumer path control |
| *src/app/psource.c & .h*              | Defines function prototypes and implements functions for power provider path control |
| *src/app/swap.c & .h*               | Defines function prototypes and implements functions to evaluate the USB PD role swap requests |
| *src/app/vdm.c & .h*                | Defines data structures, function prototypes and implements functions to handle vendor defined messages (VDM) |
| *src/app/thermistor.c & .h*                | Defines data structures, function prototypes and implements functions to handle thermistor |
| *src/solution/debug.c & .h* | Defines data structures, function prototypes and implements functions for UART-based debugging functionality |
| *src/solution/solution.c & .h* | Defines data structures, function prototypes and implements functions solution layer |
| *src/solution/solution_tasks.c & .h* | Defines data structures, function prototypes and implements functions related to solution layer sub-tasks |
| *src/system/instrumentation.c & .h* | Defines data structures, function prototypes and implements functions to monitor CPU resource usage |

<br>

## Related resources

Resources | Links
-----------|------------------
Application notes |[AN232553](https://www.infineon.com/an232553) – Getting started with EZ-PD&trade; PMG1 MCU on ModusToolbox&trade; software <br> [AN232565](https://www.infineon.com/an232565) – EZ-PD&trade; PMG1 MCU hardware design guidelines and checklist
Code examples | [Using ModusToolbox&trade; software](https://github.com/Infineon?q=mtb-example-pmg1%20NOT%20Deprecated) on GitHub
Device documentation | [EZ-PD&trade; PMG1 MCU datasheets](https://www.infineon.com/PMG1DS)
Development kits | Select your kits from the [Evaluation board finder](https://www.infineon.com/cms/en/design-support/finder-selection-tools/product-finder/evaluation-board)
Libraries on GitHub | [mtb-pdl-cat2](https://github.com/Infineon/mtb-pdl-cat2) – Peripheral Driver Library (PDL) and docs
Middleware on GitHub  | [pdstack](https://github.com/Infineon/pdstack) – PDStack Middleware Library and docs <br> [pdutils](https://github.com/Infineon/pdutils) – PDUtils Middleware Library and docs
Tools |  [ModusToolbox&trade;](https://www.infineon.com/modustoolbox) – ModusToolbox&trade; software is a collection of easy-to-use libraries and tools enabling rapid development with Infineon MCUs for applications ranging from wireless and cloud-connected systems, edge AI/ML, embedded sense and control, to wired USB connectivity using PSoC&trade; Industrial/IoT MCUs, AIROC&trade; Wi-Fi and Bluetooth&reg; connectivity devices, XMC&trade; Industrial MCUs, and EZ-USB&trade;/EZ-PD&trade; wired connectivity controllers. ModusToolbox&trade; incorporates a comprehensive set of BSPs, HAL, libraries, configuration tools, and provides support for industry-standard IDEs to fast-track your embedded application development.

## Other resources


Infineon provides a wealth of data at [www.infineon.com](https://www.infineon.com) to help you select the right device, and quickly and effectively integrate it into your design.


## Document history

 Document title: *CE233354* – *EZ-PD&trade; PMG1B1 MCU: USBPD battery charger*

 Version | Description of change
 ------- | ---------------------
 1.0.0   | New code example

<br />

-------------------------------------------------------------------------------

© Cypress Semiconductor Corporation, 2021-2023. This document is the property of Cypress Semiconductor Corporation, an Infineon Technologies company, and its affiliates (“Cypress”).  This document, including any software or firmware included or referenced in this document (“Software”), is owned by Cypress under the intellectual property laws and treaties of the United States and other countries worldwide.  Cypress reserves all rights under such laws and treaties and does not, except as specifically stated in this paragraph, grant any license under its patents, copyrights, trademarks, or other intellectual property rights.  If the Software is not accompanied by a license agreement and you do not otherwise have a written agreement with Cypress governing the use of the Software, then Cypress hereby grants you a personal, non-exclusive, nontransferable license (without the right to sublicense) (1) under its copyright rights in the Software (a) for Software provided in source code form, to modify and reproduce the Software solely for use with Cypress hardware products, only internally within your organization, and (b) to distribute the Software in binary code form externally to end users (either directly or indirectly through resellers and distributors), solely for use on Cypress hardware product units, and (2) under those claims of Cypress’s patents that are infringed by the Software (as provided by Cypress, unmodified) to make, use, distribute, and import the Software solely for use with Cypress hardware products.  Any other use, reproduction, modification, translation, or compilation of the Software is prohibited.
<br />
TO THE EXTENT PERMITTED BY APPLICABLE LAW, CYPRESS MAKES NO WARRANTY OF ANY KIND, EXPRESS OR IMPLIED, WITH REGARD TO THIS DOCUMENT OR ANY SOFTWARE OR ACCOMPANYING HARDWARE, INCLUDING, BUT NOT LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE. No computing device can be absolutely secure. Therefore, despite security measures implemented in Cypress hardware or software products, Cypress shall have no liability arising out of any security breach, such as unauthorized access to or use of a Cypress product. CYPRESS DOES NOT REPRESENT, WARRANT, OR GUARANTEE THAT CYPRESS PRODUCTS, OR SYSTEMS CREATED USING CYPRESS PRODUCTS, WILL BE FREE FROM CORRUPTION, ATTACK, VIRUSES, INTERFERENCE, HACKING, DATA LOSS OR THEFT, OR OTHER SECURITY INTRUSION (collectively, “Security Breach”). Cypress disclaims any liability relating to any Security Breach, and you shall and hereby do release Cypress from any claim, damage, or other liability arising from any Security Breach. In addition, the products described in these materials may contain design defects or errors known as errata which may cause the product to deviate from published specifications. To the extent permitted by applicable law, Cypress reserves the right to make changes to this document without further notice. Cypress does not assume any liability arising out of the application or use of any product or circuit described in this document. Any information provided in this document, including any sample design information or programming code, is provided only for reference purposes. It is the responsibility of the user of this document to properly design, program, and test the functionality and safety of any application made of this information and any resulting product. “High-Risk Device” means any device or system whose failure could cause personal injury, death, or property damage. Examples of High-Risk Devices are weapons, nuclear installations, surgical implants, and other medical devices. “Critical Component” means any component of a High-Risk Device whose failure to perform can be reasonably expected to cause, directly or indirectly, the failure of the High-Risk Device, or to affect its safety or effectiveness. Cypress is not liable, in whole or in part, and you shall and hereby do release Cypress from any claim, damage, or other liability arising from any use of a Cypress product as a Critical Component in a High-Risk Device. You shall indemnify and hold Cypress, including its affiliates, and its directors, officers, employees, agents, distributors, and assigns harmless from and against all claims, costs, damages, and expenses, arising out of any claim, including claims for product liability, personal injury or death, or property damage arising from any use of a Cypress product as a Critical Component in a High-Risk Device. Cypress products are not intended or authorized for use as a Critical Component in any High-Risk Device except to the limited extent that (i) Cypress’s published data sheet for the product explicitly states Cypress has qualified the product for use in a specific High-Risk Device, or (ii) Cypress has given you advance written authorization to use the product as a Critical Component in the specific High-Risk Device and you have signed a separate indemnification agreement.
<br>
Cypress, the Cypress logo, and combinations thereof, ModusToolbox, PSoC, CAPSENSE, EZ-USB, F-RAM, and TRAVEO are trademarks or registered trademarks of Cypress or a subsidiary of Cypress in the United States or in other countries. For a more complete list of Cypress trademarks, visit www.infineon.com. Other names and brands may be claimed as property of their respective owners.
