/* ----------------------------------------------------------------------------
 * Copyright (c) 2015-2017 Semiconductor Components Industries, LLC (d/b/a
 * ON Semiconductor), All Rights Reserved
 *
 * This code is the property of ON Semiconductor and may not be redistributed
 * in any form without prior written permission from ON Semiconductor.
 * The terms of use and warranty for this code are covered by contractual
 * agreements between ON Semiconductor and the licensee.
 *
 * This is Reusable Code.
 *
 * ----------------------------------------------------------------------------
 * rsl10_map_nvr.h
 * - Memory map for the non-volatile record sectors
 * ----------------------------------------------------------------------------
 * $Revision: 1.17 $
 * $Date: 2017/11/03 20:50:39 $
 * ------------------------------------------------------------------------- */

#ifndef RSL10_MAP_NVR_H
#define RSL10_MAP_NVR_H

#include <rsl10.h>

/* ----------------------------------------------------------------------------
 * System Information and Lock Sector
 * ------------------------------------------------------------------------- */
#define SYS_INFO_BASE                   FLASH_NVR1_BASE

#define SYS_INFO_START_ADDR             (SYS_INFO_BASE + 0x00)
#define SYS_INFO_START_MEM_CFG          (SYS_INFO_BASE + 0x04)

/* ----------------------------------------------------------------------------
 * Bonding Information Sector
 * ------------------------------------------------------------------------- */
#define SIZEOF_BONDLIST                 28

typedef struct
{
   uint8_t  STATE;
   uint8_t  RESERVED0[3];
   uint8_t  LTK[16];
   uint16_t EDIV;
   uint8_t  RESERVED1[2];
   uint8_t  ADDR[6];
   uint8_t  ADDR_TYPE;
   uint8_t  RESERVED2;
   uint8_t  CSRK[16];
   uint8_t  IRK[16];
   uint8_t  RAND[8];
} BondInfo_Type;

#define BOND_INFO_BASE                  FLASH_NVR2_BASE
#define BOND_INFO                       ((BondInfo_Type *) BOND_INFO_BASE)[SIZEOF_BONDLIST*sizeof(BondInfo_Type)]
#define BOND_INFO_STATE_EMPTY           0xFF
#define BOND_INFO_STATE_INVALID         0x00

/* ----------------------------------------------------------------------------
 * Device Information and Initialization Sector
 * ------------------------------------------------------------------------- */
#define DEVICE_INFO_BASE                FLASH_NVR3_BASE
#define DEVICE_INFO_BLUETOOTH_ADDR      (DEVICE_INFO_BASE + 0x00)
#define DEVICE_INFO_BLUETOOTH_IRK       (DEVICE_INFO_BASE + 0x10)
#define DEVICE_INFO_BLUETOOTH_CSRK      (DEVICE_INFO_BASE + 0x20)

/* Debug port memory access (lock) configuration */
#define LOCK_INFO_BASE                  (FLASH_NVR3_BASE + 0x40)
#define LOCK_INFO_SETTING               (LOCK_INFO_BASE + 0x00)
#define LOCK_INFO_KEY                   (LOCK_INFO_BASE + 0x04)

/* Manufacturing initialization routine */
#define MANU_INFO_INIT                  (FLASH_NVR3_BASE + 0x80)
#define MANU_INFO_LENGTH                (MANU_INFO_INIT + 0x00)
#define MANU_INFO_FUNCTION_VERSION      (MANU_INFO_INIT + 0x02)
#define MANU_INFO_MAX_LENGTH            (DEVICE_INFO_ECDH_BASE - MANU_INFO_INIT - 2)

/* ECDH Private/Public Key Pairs */
#define DEVICE_INFO_ECDH_BASE           (FLASH_NVR3_TOP - 0x5F)
#define DEVICE_INFO_ECDH_PRIVATE        (DEVICE_INFO_ECDH_BASE + 0x00)
#define DEVICE_INFO_ECDH_PUBLIC_X       (DEVICE_INFO_ECDH_BASE + 0x20)
#define DEVICE_INFO_ECDH_PUBLIC_Y       (DEVICE_INFO_ECDH_BASE + 0x40)

/* ----------------------------------------------------------------------------
 * Manufacturing Information Sector
 * ------------------------------------------------------------------------- */
/* Calibration information */
#define MANU_INFO_BASE                  FLASH_NVR4_0_BASE

#define MANU_CAL_INFO_TARGET_POS        16
#define MANU_INFO_BANDGAP               (MANU_INFO_BASE + 0x00)
#define MANU_INFO_VDDRF                 (MANU_INFO_BASE + 0x10)
#define MANU_INFO_VDDPA                 (MANU_INFO_BASE + 0x20)
#define MANU_INFO_VDDC                  (MANU_INFO_BASE + 0x30)
#define MANU_INFO_VDDC_STANDBY          (MANU_INFO_BASE + 0x40)
#define MANU_INFO_VDDM                  (MANU_INFO_BASE + 0x50)
#define MANU_INFO_VDDM_STANDBY          (MANU_INFO_BASE + 0x60)
#define MANU_INFO_DCDC                  (MANU_INFO_BASE + 0x70)
#define MANU_INFO_OSC_32K               (MANU_INFO_BASE + 0x80)
#define MANU_INFO_OSC_RC                (MANU_INFO_BASE + 0x90)
#define MANU_INFO_OSC_RC_MULT           (MANU_INFO_BASE + 0xB0)
#define MANU_INFO_VERSION               (MANU_INFO_BASE + 0xF8)
#define MANU_INFO_CRC                   (MANU_INFO_BASE + 0xFC)

/* Bluetooth Address - Device Information Backup */
#define MANU_INFO_BLUETOOTH_ADDR        (FLASH_NVR4_1_BASE + 0x20)

/* Manufacturing and test information */
#define MANU_INFO_DATA_BASE             FLASH_NVR4_2_BASE
#define MANU_INFO_DATA                  (MANU_INFO_DATA_BASE + 0x00)
#define MANU_INFO_DATA_CRC              (MANU_INFO_DATA_BASE + 0x1C)
#define MANU_INFO_TEST_DATA1            (MANU_INFO_DATA_BASE + 0x20)
#define MANU_INFO_TEST_DATA1_CRC        (MANU_INFO_DATA_BASE + 0x3C)
#define MANU_INFO_TEST_DATA2            (MANU_INFO_DATA_BASE + 0x40)
#define MANU_INFO_TEST_DATA2_CRC        (MANU_INFO_DATA_BASE + 0x5C)
#define MANU_INFO_TEST_DATA3            (MANU_INFO_DATA_BASE + 0x60)
#define MANU_INFO_TEST_DATA3_CRC        (MANU_INFO_DATA_BASE + 0x7C)
#define MANU_INFO_TEST_DATA4            (MANU_INFO_DATA_BASE + 0x80)
#define MANU_INFO_TEST_DATA4_CRC        (MANU_INFO_DATA_BASE + 0x9C)

/* Flash configuration information */
#define MANU_FLASH_CFG_BASE             FLASH_NVR4_3_BASE
#define MANU_FLASH_RR0                  (MANU_FLASH_CFG_BASE + 0xC0)
#define MANU_FLASH_RR1                  (MANU_FLASH_CFG_BASE + 0xC8)
#define MANU_FLASH_RR2                  (MANU_FLASH_CFG_BASE + 0xD0)
#define MANU_FLASH_RR3                  (MANU_FLASH_CFG_BASE + 0xD8)
#define MANU_FLASH_CBD0                 (MANU_FLASH_CFG_BASE + 0xE0)
#define MANU_FLASH_CBD1                 (MANU_FLASH_CFG_BASE + 0xE4)
#define MANU_FLASH_CBD2                 (MANU_FLASH_CFG_BASE + 0xE8)
#define MANU_FLASH_CBD3                 (MANU_FLASH_CFG_BASE + 0xEC)
#define MANU_FLASH_CBD4                 (MANU_FLASH_CFG_BASE + 0xF0)
#define MANU_FLASH_CBD5                 (MANU_FLASH_CFG_BASE + 0xF4)
#define MANU_FLASH_CBD6                 (MANU_FLASH_CFG_BASE + 0xF8)
#define MANU_FLASH_CBD7                 (MANU_FLASH_CFG_BASE + 0xFC)

#endif /* RSL10_MAP_NVR_H */
