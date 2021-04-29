/* ----------------------------------------------------------------------------
 * Copyright (c) 2018 Semiconductor Components Industries, LLC (d/b/a
 * ON Semiconductor), All Rights Reserved
 *
 * Copyright (C) RivieraWaves 2009-2018
 *
 * This module is derived in part from example code provided by RivieraWaves
 * and as such the underlying code is the property of RivieraWaves [a member
 * of the CEVA, Inc. group of companies], together with additional code which
 * is the property of ON Semiconductor. The code (in whole or any part) may not
 * be redistributed in any form without prior written permission from
 * ON Semiconductor.
 *
 * The terms of use and warranty for this code are covered by contractual
 * agreements between ON Semiconductor and the licensee.
 *
 * This is Reusable Code.
 *
 * ----------------------------------------------------------------------------
 * app.c
 * - Main application file
 * ------------------------------------------------------------------------- */

#include <app.h>
#include <output_driver.h>
#include <app_audio.h>

// Second Data Block
#define DATA_BASE_ADDR              (0x3F000)
#define DATA_ALIGNMENT              (16) //.\_build\opus4_wm.axf: Error: L6438E: __AT section medel_ble.o(.ARM.__AT_0x0002F061) address 0x0002f061 must be at least 4 byte aligned.

#define CH_A_LIST_LEFT_ADDR         (DATA_BASE_ADDR)
#define CH_A_LIST_LEFT_BLOCK_SIZE   (1)

#define CH_A_LIST_RIGHT_ADDR        (CH_A_LIST_LEFT_ADDR  + CH_A_LIST_LEFT_BLOCK_SIZE  * DATA_ALIGNMENT)
#define CH_A_LIST_RIGHT_BLOCK_SIZE  (1)

#define CH_B_LIST_ADDR              (CH_A_LIST_RIGHT_ADDR + CH_A_LIST_RIGHT_BLOCK_SIZE * DATA_ALIGNMENT)
#define CH_B_LIST_BLOCK_SIZE        (3) // 3 * 16 for 16 + 16 + 1 Channels

#define AES_KEY_ADDR                (CH_B_LIST_ADDR       + CH_B_LIST_BLOCK_SIZE       * DATA_ALIGNMENT)
#define AES_KEY_BLOCK_SIZE          (16)

#define DCDC_ENABLE_FLAG_ADDR       (AES_KEY_ADDR  + (AES_KEY_BLOCK_SIZE * sizeof(uint8_t)))
#define DCDC_ENABLE_FLAG_BLOCK_SIZE (1)

#define BLE_CONN_BWS_ADDR           (DCDC_ENABLE_FLAG_ADDR  + (DCDC_ENABLE_FLAG_BLOCK_SIZE * sizeof(uint32_t)))
#define BLE_CONN_BWS_BLOCK_SIZE     (1)

#define DELAYED_EEPROM_WRITE_TIMER_ADDR        (BLE_CONN_BWS_ADDR + (BLE_CONN_BWS_BLOCK_SIZE  * sizeof(uint32_t)))
#define DELAYED_EEPROM_WRITE_TIMER_BLOCK_SIZE  (1)

// Last Data Block
#define GAP_DEVICE_LEN_ADDR       (0x3F070)
#define GAP_DEVICE_LEN_BLOCK_SIZE (1)

#define GAP_DEVICE_NAME_LEN_MAX   32 // Max number of octets that our GAP Characterisitic "Device Name" can be in length (BLECore4.2 says 248).
#define GAP_DEVICE_NAME_ADDR      (GAP_DEVICE_LEN_ADDR + GAP_DEVICE_LEN_BLOCK_SIZE * sizeof(uint32_t))
#define GAP_DEVICE_NAME_BLOCK_SIZE (GAP_DEVICE_NAME_LEN_MAX) // RFU reserve a fixed number of bytes

extern const int16_t sin_signal[];
extern  struct ReadOnlyProperties_t ashaReadOnlyProperties;
struct gapm_set_dev_config_cmd devConfigCmd;
extern const struct DISS_DeviceInfo_t deviceInfo;
volatile int16_t asrc_sync_buf[2500-451] = {0};

#define DEFAULT_DEVICE_NAME {'R','O','N','D','O','\0'};

const uint8_t D_AES_KEY[16] __attribute__((section(".param_data_table")))={0};
const uint8_t D_DELAYED_EEPROM_WRITE_TIME __attribute__((section(".param_data_table")))=2;
const uint8_t D_GAP_DEVICE_NAME_LEN __attribute__((section(".param_data_table")))=1;
const uint8_t D_GAP_DEVICE_NAME[22] __attribute__((section(".param_data_table")))=DEFAULT_DEVICE_NAME;


uint8_t asha_side = ASHA_CAPABILITIES_SIDE_LEFT;


struct gapm_start_connection_cmd startConnectionCmd =
{
    .op = {
        .code = GAPM_CONNECTION_AUTO,
        .addr_src =GAPM_STATIC_ADDR,
        .state = 0
    },

    .scan_interval = 10,
    .scan_window = 10,
    .con_intv_min = GAPM_DEFAULT_CON_INTV_MIN,
    .con_intv_max = GAPM_DEFAULT_CON_INTV_MAX,
    .con_latency = GAPM_DEFAULT_CON_LATENCY,
    .superv_to = GAPM_DEFAULT_SUPERV_TO,
    .ce_len_min = 0x6,
    .ce_len_max = 0x6,
    .nb_peers = 1,
    .peers =  {{APP_BD_ADDRESS,APP_BD_ADDRESS_TYPE},}
};

//
//const uint8_t D_AES_KEY[16];
//const uint8_t D_DELAYED_EEPROM_WRITE_TIME;
//const uint16_t D_GAP_DEVICE_NAME_LEN;
//const uint8_t D_GAP_DEVICE_NAME[22];



/* ----------------------------------------------------------------------------
 * Function      : void BASS_Setup(void)
 * ----------------------------------------------------------------------------
 * Description   : Configure the Battery Service Server
 * Inputs        : None
 * Outputs       : None
 * Assumptions   : None
 * ------------------------------------------------------------------------- */
static void BASS_Setup(void)
{
    BASS_Initialize(APP_BAS_NB, APP_BASS_ReadBatteryLevel);
    BASS_NotifyOnBattLevelChange(TIMER_SETTING_S(1));     /* Periodically monitor the battery level. Only notify changes */
    APP_BASS_SetBatMonAlarm(BATMON_SUPPLY_THRESHOLD_CFG); /* BATMON alarm configuration */
}

/* ----------------------------------------------------------------------------
 * Function      : void DISS_Setup(void)
 * ----------------------------------------------------------------------------
 * Description   : Configure the Device Information Service Server
 * Inputs        : None
 * Outputs       : None
 * Assumptions   : None
 * ------------------------------------------------------------------------- */
/**
 * @brief Configure the Device Information Service Server
 */
static void DISS_Setup(void)
{
    DISS_Initialize(APP_DIS_FEATURES, (const struct DISS_DeviceInfo_t*) &deviceInfo);
}

int main(void)
{

    /* Configure hardware and initialize BLE stack */
    ble_init();

    TRACE_INIT();

    Sys_DIO_Config(GPIO_READ_SIDE,DIO_MODE_GPIO_IN_0);


    Sys_Delay_ProgramROM(SystemCoreClock / 10);

    PRINTF("\r\n==============================================\n");
    if (DIO_DATA->ALIAS[GPIO_READ_SIDE] == 1)
    {
    	PRINTF("\r\n===== Initializing ble_android_asha RIGHT ====\n");
    	asha_side = ASHA_CAPABILITIES_SIDE_RIGHT;
    	devConfigCmd.addr.addr[5] = 0xD6;
    	devConfigCmd.role = GAP_ROLE_ALL;
    }
    else
    {
    	PRINTF("\r\n===== Initializing ble_android_asha LEFT =====\n");
    }
    PRINTF("\r\n==============================================\r\n");

    /* Run the following command when erasing flash/bond_list is desirable */
    //BondList_RemoveAll();

    /* Configure application-specific advertising data and scan response  data*/
    APP_SetAdvScanData();

    /* Configure Battery Service Server */
    BASS_Setup();

    /* Configure Device Information Service Server */
    DISS_Setup();

    /* Initialize Android Audio Streaming Hearing Aid service */
    ashaReadOnlyProperties.deviceCapabilities = asha_side;
    ASHA_Initialize(&ashaReadOnlyProperties, APP_ASHA_CallbackHandler);

    /* Add application message handlers */
    MsgHandler_Add(TASK_ID_GAPM, APP_GAPM_GATTM_Handler);
    MsgHandler_Add(GATTM_ADD_SVC_RSP, APP_GAPM_GATTM_Handler);
    MsgHandler_Add(TASK_ID_GAPC, ble_gapc_main_handler);
    MsgHandler_Add(APP_LED_TIMEOUT, APP_LED_Timeout_Handler);
    MsgHandler_Add(APP_BATT_LEVEL_LOW, APP_BASS_BattLevelLow_Handler);

    Sys_DIO_Config(0,DIO_MODE_GPIO_OUT_0);

    NVIC_EnableIRQ(DIO0_IRQn);


    /* Reset the GAP manager. Trigger GAPM_CMP_EVT / GAPM_RESET when finished. See APP_GAPM_GATTM_Handler */
   GAPM_ResetCmd();

    while (1)
    {
        Kernel_Schedule();    /* Dispatch all events in Kernel queue */
        Sys_Watchdog_Refresh();
        SYS_WAIT_FOR_EVENT;    /* Wait for an event before re-executing the scheduler */

    }
}

