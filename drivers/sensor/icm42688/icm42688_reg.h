/* SPDX-License-Identifier: Apache-2.0 */
#pragma once
#include <stdint.h>

/* ================= Common ================= */
#define ICM42688_REG_WHO_AM_I        0x75u
#define ICM42688_WHOAMI_VALUE        0x47u

#define ICM42688_REG_REG_BANK_SEL    0x76u

/* ================= Bank 0 ================= */
#define ICM42688_REG_DEVICE_CONFIG      0x11u
#define ICM42688_REG_INT_CONFIG         0x14u
#define ICM42688_REG_FIFO_CONFIG        0x16u

#define ICM42688_REG_TEMP_DATA1         0x1Du
#define ICM42688_REG_TEMP_DATA0         0x1Eu

#define ICM42688_REG_ACCEL_X1           0x1Fu
#define ICM42688_REG_GYRO_X1            0x25u

#define ICM42688_REG_INT_STATUS         0x2Du
#define ICM42688_REG_FIFO_COUNTH        0x2Eu
#define ICM42688_REG_FIFO_COUNTL        0x2Fu
#define ICM42688_REG_FIFO_DATA          0x30u
#define ICM42688_REG_FIFO_WATERMARK_L   0x6Eu  /* FIFO watermark level low byte */
#define ICM42688_REG_FIFO_WATERMARK_H   0x6Fu  /* FIFO watermark level high byte */

/* APEX outputs (Bank0) */
#define ICM42688_REG_APEX_DATA0         0x31u /* STEP_CNT[7:0] */
#define ICM42688_REG_APEX_DATA1         0x32u /* STEP_CNT[15:8] */
#define ICM42688_REG_APEX_DATA2         0x33u /* STEP_CADENCE */
#define ICM42688_REG_APEX_DATA3         0x34u /* [1:0] ACTIVITY_CLASS, [3] DMP_IDLE */
#define ICM42688_REG_APEX_DATA4         0x35u /* TAP_NUM, TAP_AXIS, TAP_DIR */
#define ICM42688_REG_APEX_DATA5         0x36u /* DOUBLE_TAP_TIMING */
#define ICM42688_REG_INT_STATUS2        0x37u
#define ICM42688_REG_INT_STATUS3        0x38u

#define ICM42688_REG_SIGNAL_PATH_RESET  0x4Bu
#define ICM42688_REG_INTF_CONFIG0       0x4Cu
#define ICM42688_REG_INTF_CONFIG1       0x4Du
#define ICM42688_REG_PWR_MGMT0          0x4Eu
#define ICM42688_REG_GYRO_CONFIG0       0x4Fu
#define ICM42688_REG_ACCEL_CONFIG0      0x50u

/* INTF_CONFIG1 (Bank0, 0x4D) - RTC and Clock Selection */
#define ICM42688_INTF_CONFIG1_RTC_MODE      (1u << 2)  /* 0=No RTC clock, 1=RTC clock required */
#define ICM42688_INTF_CONFIG1_CLKSEL_SHIFT  0
#define ICM42688_INTF_CONFIG1_CLKSEL_MASK   (3u << 0)  /* bits[1:0]: Clock selection */
#define ICM42688_CLKSEL_INT_RC              0u  /* Always internal RC oscillator */
#define ICM42688_CLKSEL_AUTO_PLL_RC         1u  /* Auto: PLL if available, else RC (default) */
#define ICM42688_CLKSEL_DISABLE_ALL         3u  /* Disable all clocks */

/* GYRO_CONFIG1 (Bank0, 0x51) - UI Filter config + Temperature DLPF + DEC2_M2 */
#define ICM42688_REG_GYRO_CONFIG1       0x51u
#define ICM42688_TEMP_FILT_BW_SHIFT     5
#define ICM42688_TEMP_FILT_BW_MASK      (7u << ICM42688_TEMP_FILT_BW_SHIFT)
#define ICM42688_GYRO_UI_FILT_ORD_SHIFT 2
#define ICM42688_GYRO_UI_FILT_ORD_MASK  (3u << ICM42688_GYRO_UI_FILT_ORD_SHIFT)
#define ICM42688_GYRO_DEC2_M2_ORD_SHIFT 0
#define ICM42688_GYRO_DEC2_M2_ORD_MASK  (3u << ICM42688_GYRO_DEC2_M2_ORD_SHIFT)

/* GYRO_ACCEL_CONFIG0 (Bank0, 0x52) - UI Filter bandwidth */
#define ICM42688_REG_GYRO_ACCEL_CONFIG0 0x52u
#define ICM42688_ACCEL_UI_FILT_BW_SHIFT 4
#define ICM42688_ACCEL_UI_FILT_BW_MASK  (0x0Fu << ICM42688_ACCEL_UI_FILT_BW_SHIFT)
#define ICM42688_GYRO_UI_FILT_BW_SHIFT  0
#define ICM42688_GYRO_UI_FILT_BW_MASK   (0x0Fu << ICM42688_GYRO_UI_FILT_BW_SHIFT)

/* ACCEL_CONFIG1 (Bank0, 0x53) - UI Filter order + DEC2_M2 */
#define ICM42688_REG_ACCEL_CONFIG1      0x53u
#define ICM42688_ACCEL_DEC2_M2_ORD_SHIFT 5
#define ICM42688_ACCEL_DEC2_M2_ORD_MASK (3u << ICM42688_ACCEL_DEC2_M2_ORD_SHIFT)
#define ICM42688_ACCEL_UI_FILT_ORD_SHIFT 3
#define ICM42688_ACCEL_UI_FILT_ORD_MASK (3u << ICM42688_ACCEL_UI_FILT_ORD_SHIFT)

/*
 * UI Filter Bandwidth values (GYRO_ACCEL_CONFIG0):
 * For ODR >= 400Hz, BW is ODR/BW_SEL
 * For ODR < 400Hz, values select specific cutoffs
 */
#define ICM42688_UI_FILT_BW_ODR_DIV_2   0   /* ODR/2 (Nyquist) */
#define ICM42688_UI_FILT_BW_ODR_DIV_4   1   /* ODR/4 */
#define ICM42688_UI_FILT_BW_ODR_DIV_5   2   /* ODR/5 */
#define ICM42688_UI_FILT_BW_ODR_DIV_8   3   /* ODR/8 */
#define ICM42688_UI_FILT_BW_ODR_DIV_10  4   /* ODR/10 */
#define ICM42688_UI_FILT_BW_ODR_DIV_16  5   /* ODR/16 */
#define ICM42688_UI_FILT_BW_ODR_DIV_20  6   /* ODR/20 */
#define ICM42688_UI_FILT_BW_ODR_DIV_40  7   /* ODR/40 */
#define ICM42688_UI_FILT_BW_LPF_LL_BW   14  /* Low Latency option */
#define ICM42688_UI_FILT_BW_LPF_LL_X10  15  /* Low Latency x10 */

/* UI Filter Order values */
#define ICM42688_UI_FILT_ORD_1ST        0   /* 1st order filter */
#define ICM42688_UI_FILT_ORD_2ND        1   /* 2nd order filter */
#define ICM42688_UI_FILT_ORD_3RD        2   /* 3rd order filter */

#define ICM42688_REG_FIFO_CONFIG1       0x5Fu
#define ICM42688_FIFO_HIRES_EN          (1u << 4)  /* Enable 20-bit FIFO packets with 16-bit temp data */
#define ICM42688_REG_FIFO_CONFIG2       0x60u
#define ICM42688_REG_FIFO_CONFIG3       0x61u
#define ICM42688_REG_FSYNC_CONFIG       0x62u

/* FSYNC configuration registers */
#define ICM42688_REG_SYNC_CONFIG          0x55u

/* FSYNC_CONFIG (0x62) bit definitions */
#define ICM42688_FSYNC_UI_SEL_SHIFT       0
#define ICM42688_FSYNC_UI_SEL_MASK        (3u << ICM42688_FSYNC_UI_SEL_SHIFT)
#define ICM42688_FSYNC_UI_SEL_DISABLE     0u  /* FSYNC disabled */
#define ICM42688_FSYNC_UI_SEL_ACCEL       1u  /* FSYNC triggers accel sample */
#define ICM42688_FSYNC_UI_SEL_GYRO        2u  /* FSYNC triggers gyro sample */
#define ICM42688_FSYNC_UI_SEL_BOTH        3u  /* FSYNC triggers both */

#define ICM42688_FSYNC_POLARITY           (1u << 2)  /* 0=rising edge, 1=falling edge */
#define ICM42688_FSYNC_DISABLED           (1u << 4)  /* 1=FSYNC disabled */

/* SYNC_CONFIG (0x55) bit definitions */
#define ICM42688_SYNC_UI_SEL_SHIFT        0
#define ICM42688_SYNC_UI_SEL_MASK         (3u << ICM42688_SYNC_UI_SEL_SHIFT)
#define ICM42688_SYNC_UI_SEL_DISABLE      0u
#define ICM42688_SYNC_UI_SEL_ACCEL        1u
#define ICM42688_SYNC_UI_SEL_GYRO         2u
#define ICM42688_SYNC_UI_SEL_BOTH         3u

#define ICM42688_SYNC_POLARITY            (1u << 2)
#define ICM42688_SYNC_DISABLED            (1u << 3)

#define ICM42688_REG_INT_CONFIG0        0x63u
#define ICM42688_REG_INT_CONFIG1        0x64u
#define ICM42688_REG_INT_SOURCE0        0x65u
#define ICM42688_REG_INT_SOURCE1        0x66u
#define ICM42688_REG_INT_SOURCE3        0x68u
#define ICM42688_REG_INT_SOURCE4        0x69u

/* TMST_CONFIG (Bank0, 0x54) - Timestamp configuration */
#define ICM42688_REG_TMST_CONFIG        0x54u
#define ICM42688_TMST_TO_REGS_EN        (1u << 4)  /* Enable timestamp to registers */
#define ICM42688_TMST_RES               (1u << 3)  /* 0=1us, 1=16us resolution */
#define ICM42688_TMST_DELTA_EN          (1u << 2)  /* Delta timestamp mode */
#define ICM42688_TMST_FSYNC_EN          (1u << 1)  /* FSYNC timestamp enable */
#define ICM42688_TMST_EN                (1u << 0)  /* Timestamp counter enable */

/* Bank1: Sensor Configuration and Filters */
#define ICM42688_REG_SENSOR_CONFIG0     0x03u  /* Bank1 - Axis disable */

/* Bank1: Notch Filter and AAF Configuration */
#define ICM42688_REG_GYRO_CONFIG_STATIC2  0x0Bu  /* Bank1 - GYRO_AAF_DIS, GYRO_NF_DIS */
#define ICM42688_REG_GYRO_CONFIG_STATIC3  0x0Cu  /* Bank1 - GYRO_AAF_DELT */
#define ICM42688_REG_GYRO_CONFIG_STATIC4  0x0Du  /* Bank1 - GYRO_AAF_DELTSQR[7:0] */
#define ICM42688_REG_GYRO_CONFIG_STATIC5  0x0Eu  /* Bank1 - GYRO_AAF_BITSHIFT, DELTSQR[11:8] */
#define ICM42688_REG_GYRO_CONFIG_STATIC6  0x0Fu  /* Bank1 - GYRO_X_NF_COSWZ */
#define ICM42688_REG_GYRO_CONFIG_STATIC7  0x10u  /* Bank1 - GYRO_Y_NF_COSWZ */
#define ICM42688_REG_GYRO_CONFIG_STATIC8  0x11u  /* Bank1 - GYRO_Z_NF_COSWZ */
#define ICM42688_REG_GYRO_CONFIG_STATIC9  0x12u  /* Bank1 - GYRO_NF_COSWZ_SEL, NF_BW_SEL */

/* Timestamp value registers (Bank1) */
#define ICM42688_REG_TMSTVAL0           0x62u  /* Bank1 - TMST[7:0] */
#define ICM42688_REG_TMSTVAL1           0x63u  /* Bank1 - TMST[15:8] */
#define ICM42688_REG_TMSTVAL2           0x64u  /* Bank1 - TMST[19:16] */

/* APEX_CONFIG0 in Bank0 (0x56) */
#define ICM42688_REG_APEX_CONFIG0       0x56u
#define ICM42688_APEX_DMP_POWER_SAVE    (1u << 7)
#define ICM42688_APEX_TAP_ENABLE        (1u << 6)
#define ICM42688_APEX_PED_ENABLE        (1u << 5)
#define ICM42688_APEX_TILT_ENABLE       (1u << 4)
#define ICM42688_APEX_R2W_ENABLE        (1u << 3)
#define ICM42688_APEX_DMP_ODR_MASK      (3u << 0) /* bits[1:0] */
#define ICM42688_APEX_DMP_ODR_25HZ      0u
#define ICM42688_APEX_DMP_ODR_50HZ      2u

/* ================= Banks ================= */
#define ICM42688_BANK0 0
#define ICM42688_BANK0 0
#define ICM42688_BANK1 1
#define ICM42688_BANK2 2
#define ICM42688_BANK4 4

/* ================= Bank1: SENSOR_CONFIG0 fields ================= */
#define ICM42688_ZG_DISABLE             (1u << 5)
#define ICM42688_YG_DISABLE             (1u << 4)
#define ICM42688_XG_DISABLE             (1u << 3)
#define ICM42688_ZA_DISABLE             (1u << 2)
#define ICM42688_YA_DISABLE             (1u << 1)
#define ICM42688_XA_DISABLE             (1u << 0)

/* ================= Bank1: GYRO_CONFIG_STATIC2 fields ================= */
#define ICM42688_GYRO_AAF_DIS           (1u << 1)
#define ICM42688_GYRO_NF_DIS            (1u << 0)

/* ================= Bank1: GYRO_CONFIG_STATIC3 fields (Gyro AAF) ================= */
#define ICM42688_GYRO_AAF_DELT_SHIFT    0
#define ICM42688_GYRO_AAF_DELT_MASK     0x3Fu  /* bits [5:0] */

/* ================= Bank1: GYRO_CONFIG_STATIC5 fields (Gyro AAF) ================= */
#define ICM42688_GYRO_AAF_BITSHIFT_SHIFT  4
#define ICM42688_GYRO_AAF_BITSHIFT_MASK   (0x0Fu << 4)  /* bits [7:4] */
#define ICM42688_GYRO_AAF_DELTSQR_H_MASK  0x0Fu  /* bits [3:0] for DELTSQR[11:8] */

/* ================= Bank1: GYRO_CONFIG_STATIC9 fields (Notch Filter) ================= */
#define ICM42688_GYRO_NF_COSWZ_SEL_SHIFT  6
#define ICM42688_GYRO_NF_COSWZ_SEL_MASK   (3u << 6)
#define ICM42688_GYRO_NF_BW_SEL_SHIFT     3
#define ICM42688_GYRO_NF_BW_SEL_MASK      (7u << 3)

/* ================= Bank2: ACCEL_CONFIG_STATIC2 fields (Accel AAF) ================= */
#define ICM42688_ACCEL_AAF_DELT_SHIFT   1
#define ICM42688_ACCEL_AAF_DELT_MASK    (0x3Fu << 1)  /* bits [6:1] */
#define ICM42688_ACCEL_AAF_DIS          (1u << 0)

/* ================= Bank2: ACCEL_CONFIG_STATIC4 fields (Accel AAF) ================= */
#define ICM42688_ACCEL_AAF_BITSHIFT_SHIFT  4
#define ICM42688_ACCEL_AAF_BITSHIFT_MASK   (0x0Fu << 4)  /* bits [7:4] */
#define ICM42688_ACCEL_AAF_DELTSQR_H_MASK  0x0Fu  /* bits [3:0] for DELTSQR[11:8] */
#define ICM42688_BANK4 4

/* ================= SIGNAL_PATH_RESET bits ================= */
#define ICM42688_SPR_DMP_INIT_EN        (1u << 6)
#define ICM42688_SPR_DMP_MEM_RESET_EN   (1u << 5)
#define ICM42688_SPR_ABORT_AND_RESET    (1u << 3)
#define ICM42688_SPR_TMST_STROBE        (1u << 2)
#define ICM42688_SPR_FIFO_FLUSH         (1u << 1)
#define ICM42688_SPR_TEMP_SRDST         (1u << 2)  /* Temperature signal path reset */
#define ICM42688_SPR_GYRO_SRDST         (1u << 1)  /* Gyro signal path reset */
#define ICM42688_SPR_ACCEL_SRDST        (1u << 0)  /* Accel signal path reset */

/* ================= INTF_CONFIG0 bits ================= */
#define ICM42688_INTF_FIFO_HOLD_LAST_DATA_EN (1u << 7)
#define ICM42688_INTF_FIFO_COUNT_REC         (1u << 6)
#define ICM42688_INTF_FIFO_COUNT_ENDIAN      (1u << 5)
#define ICM42688_INTF_SENSOR_DATA_ENDIAN     (1u << 4)

/* ================= PWR_MGMT0 fields ================= */
#define ICM42688_PWR_TEMP_DIS          (1u << 5)
#define ICM42688_PWR_IDLE              (1u << 4)
#define ICM42688_PWR_GYRO_MODE_SHIFT   2
#define ICM42688_PWR_ACCEL_MODE_SHIFT  0

/* PWR_MGMT0 modes */
#define ICM42688_GYRO_MODE_OFF      0x0
#define ICM42688_GYRO_MODE_STBY     0x1
#define ICM42688_GYRO_MODE_LN       0x3

#define ICM42688_ACCEL_MODE_OFF     0x0
#define ICM42688_ACCEL_MODE_LP      0x2
#define ICM42688_ACCEL_MODE_LN      0x3

/* ================= INT_STATUS bits ================= */
/* Ты уже использовал DATA_RDY_INT как bit3 — оставляем так. */
#define ICM42688_INT_STATUS_DATA_RDY_INT    (1u << 3)
/* Добавляем FIFO interrupts (по таблице INT_STATUS: FIFO_THS_INT, FIFO_FULL_INT) */
#define ICM42688_INT_STATUS_FIFO_THS_INT    (1u << 2)
#define ICM42688_INT_STATUS_FIFO_FULL_INT   (1u << 1)

/* INT_STATUS3 bits (Bank0, 0x38) */
#define ICM42688_INT_STATUS3_STEP_DET_INT      (1u << 5)
#define ICM42688_INT_STATUS3_STEP_CNT_OVF_INT  (1u << 4)
#define ICM42688_INT_STATUS3_TILT_DET_INT      (1u << 3)
#define ICM42688_INT_STATUS3_WAKE_INT          (1u << 2)
#define ICM42688_INT_STATUS3_SLEEP_INT         (1u << 1)
#define ICM42688_INT_STATUS3_TAP_DET_INT       (1u << 0)

/* INT_STATUS2 bits (Bank0, 0x37) */
#define ICM42688_INT_STATUS2_SMD_INT           (1u << 3)
#define ICM42688_INT_STATUS2_WOM_Z_INT         (1u << 2)
#define ICM42688_INT_STATUS2_WOM_Y_INT         (1u << 1)
#define ICM42688_INT_STATUS2_WOM_X_INT         (1u << 0)

/* INT_CONFIG0 fields */
#define ICM42688_INT_CONFIG0_UI_DRDY_INT_CLEAR_SHIFT 4
#define ICM42688_INT_CONFIG0_UI_DRDY_INT_CLEAR_MASK  (3u << ICM42688_INT_CONFIG0_UI_DRDY_INT_CLEAR_SHIFT)
#define ICM42688_INT_CONFIG0_FIFO_THS_INT_CLEAR_SHIFT 2
#define ICM42688_INT_CONFIG0_FIFO_THS_INT_CLEAR_MASK  (3u << ICM42688_INT_CONFIG0_FIFO_THS_INT_CLEAR_SHIFT)
#define ICM42688_INT_CONFIG0_FIFO_FULL_INT_CLEAR_SHIFT 0
#define ICM42688_INT_CONFIG0_FIFO_FULL_INT_CLEAR_MASK  (3u << ICM42688_INT_CONFIG0_FIFO_FULL_INT_CLEAR_SHIFT)

/* UI_DRDY_INT_CLEAR encoding */
#define ICM42688_UI_DRDY_CLEAR_ON_STATUS_READ          0u
#define ICM42688_UI_DRDY_CLEAR_ON_SENSOR_REG_READ      2u
#define ICM42688_UI_DRDY_CLEAR_ON_BOTH                 3u

/* FIFO int clear encoding */
#define ICM42688_FIFO_INT_CLEAR_ON_STATUS_READ         0u

/* INT_CONFIG1 bits */
#define ICM42688_INT_CONFIG1_INT_ASYNC_RESET       (1u << 4)

/* INT_SOURCE0 (Bank0, 0x65) -> INT1 routing */
#define ICM42688_INT_SRC_UI_DRDY_INT1_EN  (1u << 3)
#define ICM42688_INT_SRC_FIFO_THS_INT1_EN (1u << 2)
#define ICM42688_INT_SRC_FIFO_FULL_INT1_EN (1u << 1)

/* INT_SOURCE3 (Bank0, 0x68) -> INT2 routing */
#define ICM42688_INT_SRC_UI_DRDY_INT2_EN  (1u << 3)
#define ICM42688_INT_SRC_FIFO_THS_INT2_EN (1u << 2)
#define ICM42688_INT_SRC_FIFO_FULL_INT2_EN (1u << 1)

/* ================= APEX tap output decode ================= */
/* APEX_DATA4 (0x35): [4:3] TAP_NUM, [2:1] TAP_AXIS, [0] TAP_DIR */
#define ICM42688_APEX_DATA4_TAP_NUM_MASK   (3u << 3)
#define ICM42688_APEX_DATA4_TAP_NUM_SHIFT  3
#define ICM42688_APEX_DATA4_TAP_AXIS_MASK  (3u << 1)
#define ICM42688_APEX_DATA4_TAP_AXIS_SHIFT 1
#define ICM42688_APEX_DATA4_TAP_DIR_MASK   (1u << 0)

/* ================= Bank4 registers (APEX config + routing) ================= */
#define ICM42688_REG_APEX_CONFIG1       0x40u /* Bank4 */
#define ICM42688_REG_APEX_CONFIG2       0x41u /* Bank4 */
#define ICM42688_REG_APEX_CONFIG3       0x42u /* Bank4 */
#define ICM42688_REG_APEX_CONFIG4       0x43u /* Bank4 */
#define ICM42688_REG_APEX_CONFIG5       0x44u /* Bank4 */
#define ICM42688_REG_APEX_CONFIG6       0x45u /* Bank4 */
#define ICM42688_REG_APEX_CONFIG7       0x46u /* Bank4 */
#define ICM42688_REG_APEX_CONFIG8       0x47u /* Bank4 */
#define ICM42688_REG_APEX_CONFIG9       0x48u /* Bank4 */

/* INT_SOURCE6/7 in Bank4 */
#define ICM42688_REG_INT_SOURCE6        0x4Du /* Bank4 */
#define ICM42688_REG_INT_SOURCE7        0x4Eu /* Bank4 */

/* INT_SOURCE6/7 bits (Bank4) */
#define ICM42688_INT_SRC_STEP_DET_EN      (1u << 5)
#define ICM42688_INT_SRC_STEP_OVF_EN      (1u << 4)
#define ICM42688_INT_SRC_TILT_DET_EN      (1u << 3)
#define ICM42688_INT_SRC_WAKE_DET_EN      (1u << 2)
#define ICM42688_INT_SRC_SLEEP_DET_EN     (1u << 1)
#define ICM42688_INT_SRC_TAP_DET_EN       (1u << 0)

/* APEX_CONFIG4 fields */
#define ICM42688_APEX_CFG4_TILT_WAIT_TIME_MASK   (3u << 6)
#define ICM42688_APEX_CFG4_TILT_WAIT_TIME_SHIFT  6
#define ICM42688_APEX_CFG4_SLEEP_TIMEOUT_MASK    (7u << 3)
#define ICM42688_APEX_CFG4_SLEEP_TIMEOUT_SHIFT   3
#define ICM42688_APEX_CFG4_TAP_DOUBLE_EN         (1u << 2)
#define ICM42688_APEX_CFG4_TAP_SINGLE_EN         (1u << 1)

/* APEX_CONFIG5 mounting matrix */
#define ICM42688_APEX_CFG5_MOUNTING_MATRIX_MASK  (7u << 0)

/* APEX_CONFIG6 sleep gesture delay */
#define ICM42688_APEX_CFG6_SLEEP_GESTURE_DELAY_MASK (7u << 0)

/* APEX_CONFIG9 sensitivity mode */
#define ICM42688_APEX_CFG9_SENSITIVITY_MODE_MASK (1u << 0)

/* ================= Wake-on-Motion (WOM) ================= */
/* SMD_CONFIG (Bank0, 0x57) */
#define ICM42688_REG_SMD_CONFIG         0x57u
#define ICM42688_SMD_WOM_INT_MODE       (1u << 3)  /* 0=OR, 1=AND */
#define ICM42688_SMD_WOM_MODE           (1u << 2)  /* 0=initial sample, 1=previous sample */
#define ICM42688_SMD_SMD_MODE_MASK      (3u << 0)
#define ICM42688_SMD_SMD_MODE_DISABLED  0u
#define ICM42688_SMD_SMD_MODE_SHORT     2u         /* WOM + short window */
#define ICM42688_SMD_SMD_MODE_LONG      3u         /* WOM + long window */

/* WOM thresholds in Bank4 */
#define ICM42688_REG_ACCEL_WOM_X_THR    0x4Au  /* Bank4 */
#define ICM42688_REG_ACCEL_WOM_Y_THR    0x4Bu  /* Bank4 */
#define ICM42688_REG_ACCEL_WOM_Z_THR    0x4Cu  /* Bank4 */

/* INT_SOURCE1 (Bank0, 0x66) -> INT1 routing */
#define ICM42688_INT_SRC1_WOM_X_INT1_EN   (1u << 0)
#define ICM42688_INT_SRC1_WOM_Y_INT1_EN   (1u << 1)
#define ICM42688_INT_SRC1_WOM_Z_INT1_EN   (1u << 2)
#define ICM42688_INT_SRC1_SMD_INT1_EN     (1u << 3)

/* INT_SOURCE4 (Bank0, 0x69) -> INT2 routing */
#define ICM42688_INT_SRC4_WOM_X_INT2_EN   (1u << 0)
#define ICM42688_INT_SRC4_WOM_Y_INT2_EN   (1u << 1)
#define ICM42688_INT_SRC4_WOM_Z_INT2_EN   (1u << 2)
#define ICM42688_INT_SRC4_SMD_INT2_EN     (1u << 3)
/* ================= Self-Test ================= */
/* SELF_TEST_CONFIG (Bank0, 0x70) */
#define ICM42688_REG_SELF_TEST_CONFIG   0x70u

#define ICM42688_ACCEL_ST_POWER   (1u << 6)  /* Set to 1 for accel self-test, then back to 0 */

#define ICM42688_EN_AZ_ST         (1u << 5)
#define ICM42688_EN_AY_ST         (1u << 4)
#define ICM42688_EN_AX_ST         (1u << 3)

#define ICM42688_EN_GZ_ST         (1u << 2)
#define ICM42688_EN_GY_ST         (1u << 1)
#define ICM42688_EN_GX_ST         (1u << 0)


/* Self-test data registers (Bank1) */
#define ICM42688_REG_XG_ST_DATA         0x5Fu  /* Bank1 - Gyro X self-test data */
#define ICM42688_REG_YG_ST_DATA         0x60u  /* Bank1 - Gyro Y self-test data */
#define ICM42688_REG_ZG_ST_DATA         0x61u  /* Bank1 - Gyro Z self-test data */

/* Bank2: Accelerometer AAF Configuration */
#define ICM42688_REG_ACCEL_CONFIG_STATIC2 0x03u  /* Bank2 - ACCEL_AAF_DELT, AAF_DIS */
#define ICM42688_REG_ACCEL_CONFIG_STATIC3 0x04u  /* Bank2 - ACCEL_AAF_DELTSQR[7:0] */
#define ICM42688_REG_ACCEL_CONFIG_STATIC4 0x05u  /* Bank2 - ACCEL_AAF_BITSHIFT, DELTSQR[11:8] */

/* Self-test data registers (Bank2) */
#define ICM42688_REG_XA_ST_DATA         0x3Bu  /* Bank2 - Accel X self-test data */
#define ICM42688_REG_YA_ST_DATA         0x3Cu  /* Bank2 - Accel Y self-test data */
#define ICM42688_REG_ZA_ST_DATA         0x3Du  /* Bank2 - Accel Z self-test data */

/* ================= Offset Calibration ================= */
/* User Offset Registers (Bank4, 0x77-0x7F) - 12-bit signed offsets, packed nibbles */
#define ICM42688_REG_OFFSET_USER0   0x77u  /* GYRO_X_OFFUSER[7:0] */
#define ICM42688_REG_OFFSET_USER1   0x78u  /* [7:4]=GYRO_Y_OFFUSER[11:8], [3:0]=GYRO_X_OFFUSER[11:8] */
#define ICM42688_REG_OFFSET_USER2   0x79u  /* GYRO_Y_OFFUSER[7:0] */
#define ICM42688_REG_OFFSET_USER3   0x7Au  /* GYRO_Z_OFFUSER[7:0] */
#define ICM42688_REG_OFFSET_USER4   0x7Bu  /* [7:4]=ACCEL_X_OFFUSER[11:8], [3:0]=GYRO_Z_OFFUSER[11:8] */
#define ICM42688_REG_OFFSET_USER5   0x7Cu  /* ACCEL_X_OFFUSER[7:0] */
#define ICM42688_REG_OFFSET_USER6   0x7Du  /* ACCEL_Y_OFFUSER[7:0] */
#define ICM42688_REG_OFFSET_USER7   0x7Eu  /* [7:4]=ACCEL_Z_OFFUSER[11:8], [3:0]=ACCEL_Y_OFFUSER[11:8] */
#define ICM42688_REG_OFFSET_USER8   0x7Fu  /* ACCEL_Z_OFFUSER[7:0] */

/* Offset resolution per datasheet:
 * Gyro:  1/32 dps per LSB (full range ±64 dps)
 * Accel: 0.5 mg per LSB (full range ±1 g)
 */
#define ICM42688_GYRO_OFFSET_LSB_DPS    (1.0f / 32.0f)  /* dps per LSB */
#define ICM42688_ACCEL_OFFSET_LSB_MG    0.5f            /* mg per LSB */
