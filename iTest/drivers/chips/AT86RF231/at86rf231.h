/*
 *	File: at86rf231.h
 *	Description:
 *		Peach-specific definition of the "at86rf231"
 *	Created on: 2013-7-30
 *	Author: 
 *		Dong Yang 		<dyang@bjtu.edu.cn>
 *		Hongchao Wang 	<hcwang@bjtu.edu.cn>
 *		Hao Zhang 		<zh1047@gmail.com>
 */

#ifndef AT86RF231_H_
#define AT86RF231_H_

//=========================== include =========================================

//=========================== define ==========================================

/**
\brief Possible values for the status of the radio.

After you get an interrupt from the radio, read the status register
(<tt>RG_IRQ_STATUS</tt>) to know what type it is, amoung the following.
*/
enum radio_irqstatus_enum {
	AT_IRQ_BAT_LOW           = 0x80,   //< Supply voltage below the programmed threshold.
	AT_IRQ_TRX_UR            = 0x40,   ///< Frame buffer access violation.
	AT_IRQ_AMI               = 0x20,   ///< Address matching.
	AT_IRQ_CCA_ED_DONE       = 0x10,   ///< End of a CCA or ED measurement.
	AT_IRQ_TRX_END           = 0x08,   ///< Completion of a frame transmission/reception.
	AT_IRQ_RX_START          = 0x04,   ///< Start of a PSDU reception.
	AT_IRQ_PLL_UNLOCK        = 0x02,   ///< PLL unlock.
	AT_IRQ_PLL_LOCK          = 0x01,   ///< PLL lock.
};

#define HAVE_REGISTER_MAP   (1)

// Offset for register TRX_STATUS
#define RG_TRX_STATUS             (0x01)

/* Access parameters for sub-register CCA_DONE in register @ref RG_TRX_STATUS */
#define SR_CCA_DONE            0x01, 0x80, 7
/* Access parameters for sub-register CCA_STATUS in register @ref RG_TRX_STATUS */
#define SR_CCA_STATUS          0x01, 0x40, 6
#define SR_RESERVED_01_3       0x01, 0x20, 5
/* Access parameters for sub-register TRX_STATUS in register @ref RG_TRX_STATUS */
#define SR_TRX_STATUS          0x01, 0x1F, 0

/* Constant P_ON for sub-register @ref RG_SR_TRX_STATUS */
#define ST_P_ON                             (0)
/* Constant BUSY_RX for sub-register @ref RG_SR_TRX_STATUS */
#define ST_BUSY_RX                          (1)
/* Constant BUSY_TX for sub-register @ref RG_SR_TRX_STATUS */
#define ST_BUSY_TX                          (2)
/* Constant RX_ON for sub-register @ref RG_SR_TRX_STATUS */
#define ST_RX_ON                            (6)
/* Constant TRX_OFF for sub-register @ref RG_SR_TRX_STATUS */
#define ST_TRX_OFF                          (8)
/* Constant PLL_ON for sub-register @ref RG_SR_TRX_STATUS */
#define ST_PLL_ON                           (9)
/* Constant SLEEP for sub-register @ref RG_SR_TRX_STATUS */
#define ST_SLEEP                            (15)
/* Constant BUSY_RX_AACK for sub-register @ref RG_SR_TRX_STATUS */
#define ST_BUSY_RX_AACK                     (17)
/* Constant BUSY_TX_ARET for sub-register @ref RG_SR_TRX_STATUS */
#define ST_BUSY_TX_ARET                     (18)
/* Constant RX_AACK_ON for sub-register @ref RG_SR_TRX_STATUS */
#define ST_RX_AACK_ON                       (22)
/* Constant TX_ARET_ON for sub-register @ref RG_SR_TRX_STATUS */
#define ST_TX_ARET_ON                       (25)
/* Constant RX_ON_NOCLK for sub-register @ref RG_SR_TRX_STATUS */
#define ST_RX_ON_NOCLK                      (28)
/* Constant RX_AACK_ON_NOCLK for sub-register @ref RG_SR_TRX_STATUS */
#define ST_RX_AACK_ON_NOCLK                 (29)
/* Constant BUSY_RX_AACK_NOCLK for sub-register @ref RG_SR_TRX_STATUS */
#define ST_BUSY_RX_AACK_NOCLK               (30)
/* Constant STATE_TRANSITION_IN_PROGRESS for sub-register @ref RG_SR_TRX_STATUS */
#define ST_STATE_TRANSITION_IN_PROGRESS     (31)

// Offset for register TRX_STATE
#define RG_TRX_STATE              (0x02)

/* Access parameters for sub-register TRAC_STATUS in register @ref RG_TRX_STATE */
#define SR_TRAC_STATUS            0x02, 0xE0, 5
/* Access parameters for sub-register TRX_CMD in register @ref RG_TRX_STATE */
#define SR_TRX_CMD                0x02, 0x1F, 0

/* Constant CMD_NOP for sub-register @ref SR_TRX_CMD */
#define CMD_NOP                   (0)
/* Constant CMD_TX_START for sub-register @ref SR_TRX_CMD */
#define CMD_TX_START              (2)
/* Constant CMD_FORCE_TRX_OFF for sub-register @ref SR_TRX_CMD */
#define CMD_FORCE_TRX_OFF         (3)
/* Constant CMD_FORCE_PLL_ON for sub-register @ref SR_TRX_CMD */
#define CMD_FORCE_PLL_ON          (4)
/* Constant CMD_RX_ON for sub-register @ref SR_TRX_CMD */
#define CMD_RX_ON                 (6)
/* Constant CMD_TRX_OFF for sub-register @ref SR_TRX_CMD */
#define CMD_TRX_OFF               (8)
/* Constant CMD_PLL_ON for sub-register @ref SR_TRX_CMD */
#define CMD_PLL_ON                (9)
/* Constant CMD_RX_AACK_ON for sub-register @ref SR_TRX_CMD */
#define CMD_RX_AACK_ON            (22)
/* Constant CMD_TX_ARET_ON for sub-register @ref SR_TRX_CMD */
#define CMD_TX_ARET_ON            (25)

// Offset for register TRX_CTRL_0
#define RG_TRX_CTRL_0             (0x03)

/* Access parameters for sub-register PAD_IO in register @ref RG_TRX_CTRL_0 */
#define SR_PAD_IO                 0x03, 0xC0, 6
/* Access parameters for sub-register PAD_IO_CLKM in register @ref RG_TRX_CTRL_0 */
#define SR_PAD_IO_CLKM            0x03, 0x30, 4
/* Access parameters for sub-register CLKM_SHA_SEL in register @ref RG_TRX_CTRL_0 */
#define SR_CLKM_SHA_SEL           0x03, 0x08, 3
/* Access parameters for sub-register CLKM_CTRL in register @ref RG_TRX_CTRL_0 */
#define SR_CLKM_CTRL              0x03, 0x07, 0

/* Constant CLKM_2mA for sub-register @ref SR_PAD_IO_CLKM */
#define CLKM_2mA                  (0)
/* Constant CLKM_4mA for sub-register @ref SR_PAD_IO_CLKM */
#define CLKM_4mA                  (1)
/* Constant CLKM_6mA for sub-register @ref SR_PAD_IO_CLKM */
#define CLKM_6mA                  (2)
/* Constant CLKM_8mA for sub-register @ref SR_PAD_IO_CLKM */
#define CLKM_8mA                  (3)

/* Constant CLKM_no_clock for sub-register @ref SR_CLKM_CTRL */
#define CLKM_no_clock             (0)
/* Constant CLKM_1MHz for sub-register @ref SR_CLKM_CTRL */
#define CLKM_1MHz                 (1)
/* Constant CLKM_2MHz for sub-register @ref SR_CLKM_CTRL */
#define CLKM_2MHz                 (2)
/* Constant CLKM_4MHz for sub-register @ref SR_CLKM_CTRL */
#define CLKM_4MHz                 (3)
/* Constant CLKM_8MHz for sub-register @ref SR_CLKM_CTRL */
#define CLKM_8MHz                 (4)
/* Constant CLKM_16MHz for sub-register @ref SR_CLKM_CTRL */
#define CLKM_16MHz                (5)
/* Constant CLKM_250KHz for sub-register @ref SR_CLKM_CTRL */
#define CLKM_250KHz               (6)
/* Constant CLKM_62P5KHz for sub-register @ref SR_CLKM_CTRL */
#define CLKM_62P5KHz              (7)

// Offset for register TRX_CTRL_1
#define RG_TRX_CTRL_1             (0x04)

/* Access parameters for sub-register PA_EXT_EN in register @ref RG_TRX_CTRL_1 */
#define SR_PA_EXT_EN              0x04, 0x80, 7
/* Access parameters for sub-register IRQ_2_EXT_EN in register @ref RG_TRX_CTRL_1 */
#define SR_IRQ_2_EXT_EN           0x04, 0x40, 6
/* Access parameters for sub-register TX_AUTO_CRC_ON in register @ref RG_TRX_CTRL_1 */
#define SR_TX_AUTO_CRC_ON         0x04, 0x20, 5
/* Access parameters for sub-register RX_BL_CTRL in register @ref RG_TRX_CTRL_1 */
#define SR_RX_BL_CTRL             0x04, 0x10, 4
/* Access parameters for sub-register SPI_CMD_MODE in register @ref RG_TRX_CTRL_1 */
#define SR_SPI_CMD_MODE           0x04, 0x0C, 2
/* Access parameters for sub-register IRQ_MASK_MODE in register @ref RG_TRX_CTRL_1 */
#define SR_IRQ_MASK_MODE          0x04, 0x02, 1
/* Access parameters for sub-register IRQ_POLARITY in register @ref RG_TRX_CTRL_1 */
#define SR_IRQ_POLARITY           0x04, 0x01, 0

// Offset for register PHY_TX_PWR
#define RG_PHY_TX_PWR             (0x05)

/* Access parameters for sub-register PA_BUF_LT in register @ref RG_PHY_TX_PWR */
#define SR_PA_BUF_LT              0x05, 0xC0, 6
/* Access parameters for sub-register PA_LT in register @ref RG_PHY_TX_PWR */
#define SR_PA_LT                  0x05, 0x30, 4
/* Access parameters for sub-register TX_PWR in register @ref RG_PHY_TX_PWR */
#define SR_TX_PWR                 0x05, 0x0F, 0

// Offset for register PHY_RSSI
#define RG_PHY_RSSI               (0x06)

/* Access parameters for sub-register RX_CRC_VALID in register @ref RG_PHY_RSSI */
#define SR_RX_CRC_VALID           0x06, 0x80, 7
/* Access parameters for sub-register RND_VALUE in register @ref RG_PHY_RSSI */
#define SR_RND_VALUE              0x06, 0x60, 5
/* Access parameters for sub-register RSSI in register @ref RG_PHY_RSSI */
#define SR_RSSI                   0x06, 0x1F, 0

// Offset for register PHY_ED_LEVEL
#define RG_PHY_ED_LEVEL           (0x07)

/* Access parameters for sub-register ED_LEVEL in register @ref RG_PHY_ED_LEVEL */
#define SR_ED_LEVEL               0x07, 0xFF, 0

// Offset for register PHY_CC_CCA
#define RG_PHY_CC_CCA             (0x08)

/* Access parameters for sub-register CCA_REQUEST in register @ref RG_PHY_CC_CCA */
#define SR_CCA_REQUEST            0x08, 0x80, 7
/* Access parameters for sub-register CCA_MODE in register @ref RG_PHY_CC_CCA */
#define SR_CCA_MODE               0x08, 0x60, 5
/* Access parameters for sub-register CHANNEL in register @ref RG_PHY_CC_CCA */
#define SR_CHANNEL                0x08, 0x1F, 0

// Offset for register CCA_THRES
#define RG_CCA_THRES              (0x09)

#define SR_reserved_09_1          0x09, 0xF0, 4
/* Access parameters for sub-register CCA_ED_THRES in register @ref RG_CCA_THRES */
#define SR_CCA_ED_THRES           0x09, 0x0F, 0

// Offset for register RX_CTRL
#define RG_RX_CTRL                (0x0A)

#define SR_reserved_0a_1          0x0A, 0xF0, 4
/* Access parameters for sub-register PDT_THRES in register @ref RG_RX_CTRL */
#define SR_PDT_THRES              0x0A, 0x0F, 0

// Offset for register SFD_VALUE
#define RG_SFD_VALUE              (0x0B)

/* Access parameters for sub-register SFD_VALUE in register @ref RG_SFD_VALUE */
#define SR_SFD_VALUE              0x0B, 0xFF, 0

// Offset for register TRX_CTRL_2
#define RG_TRX_CTRL_2             (0x0C)

/* Access parameters for sub-register RX_SAFE_MODE in register @ref RG_TRX_CTRL_2 */
#define SR_RX_SAFE_MODE           0x0C, 0x80, 7
#define SR_reserved_0c_2          0x0C, 0x7C, 2
/* Access parameters for sub-register OQPSK_DATA_RATE in register @ref RG_TRX_CTRL_2 */
#define SR_OQPSK_DATA_RATE        0x0C, 0x03, 0

// Offset for register ANT_DIV
#define RG_ANT_DIV                (0x0D)

/* Access parameters for sub-register ANT_SEL in register @ref RG_ANT_DIV */
#define SR_ANT_SEL                0x0D, 0x80, 7
#define SR_reserved_0d_2          0x0D, 0x70, 4
/* Access parameters for sub-register ANT_DIV_EN in register @ref RG_ANT_DIV */
#define SR_ANT_DIV_EN             0x0D, 0x08, 3
/* Access parameters for sub-register ANT_EXT_SW_EN in register @ref RG_ANT_DIV */
#define SR_ANT_EXT_SW_EN          0x0D, 0x04, 2
/* Access parameters for sub-register ANT_CTRL in register @ref RG_ANT_DIV */
#define SR_ANT_CTRL               0x0D, 0x03, 0

// Offset for register IRQ_MASK
#define RG_IRQ_MASK               (0x0E)

/* Access parameters for sub-register IRQ_MASK in register @ref RG_IRQ_MASK */
#define SR_IRQ_MASK               0x0E, 0xFF, 0
/* Access parameters for sub-register MASK_BAT_LOW in register @ref RG_IRQ_MASK */
#define SR_IRQ_MASK_BAT_LOW       0x0E, 0x80, 7
/* Access parameters for sub-register MASK_TRX_UR in register @ref RG_IRQ_MASK */
#define SR_IRQ_MASK_TRX_UR        0x0E, 0x40, 6
/* Access parameters for sub-register MASK_AMI in register @ref RG_IRQ_MASK */
#define SR_IRQ_MASK_AMI           0x0E, 0x20, 5
/* Access parameters for sub-register MASK_CCA_ED_DONE in register @ref RG_IRQ_MASK */
#define SR_IRQ_MASK_CCA_ED_DONE   0x0E, 0x10, 4
/* Access parameters for sub-register MASK_TRX_END in register @ref RG_IRQ_MASK */
#define SR_IRQ_MASK_TRX_END       0x0E, 0x08, 3
/* Access parameters for sub-register MASK_RX_START in register @ref RG_IRQ_MASK */
#define SR_IRQ_MASK_RX_START      0x0E, 0x04, 2
/* Access parameters for sub-register MASK_PLL_UNLOCK in register @ref RG_IRQ_MASK */
#define SR_IRQ_MASK_PLL_UNLOCK    0x0E, 0x02, 1
/* Access parameters for sub-register MASK_PLL_LOCK in register @ref RG_IRQ_MASK */
#define SR_IRQ_MASK_PLL_LOCK      0x0E, 0x01, 0

// Offset for register IRQ_STATUS
#define RG_IRQ_STATUS             (0x0F)

/* Access parameters for sub-register BAT_LOW in register @ref RG_IRQ_STATUS */
#define SR_IRQ_7_BAT_LOW          0x0F, 0x80, 7
/* Access parameters for sub-register TRX_UR in register @ref RG_IRQ_STATUS */
#define SR_IRQ_6_TRX_UR           0x0F, 0x40, 6
/* Access parameters for sub-register AMI in register @ref RG_IRQ_STATUS */
#define SR_IRQ_5_AMI              0x0F, 0x20, 5
/* Access parameters for sub-register CCA_ED_DONE in register @ref RG_IRQ_STATUS */
#define SR_IRQ_4_CCA_ED_DONE      0x0F, 0x10, 4
/* Access parameters for sub-register TRX_END in register @ref RG_IRQ_STATUS */
#define SR_IRQ_3_TRX_END          0x0F, 0x08, 3
/* Access parameters for sub-register RX_START in register @ref RG_IRQ_STATUS */
#define SR_IRQ_2_RX_START         0x0F, 0x04, 2
/* Access parameters for sub-register PLL_UNLOCK in register @ref RG_IRQ_STATUS */
#define SR_IRQ_1_PLL_UNLOCK       0x0F, 0x02, 1
/* Access parameters for sub-register PLL_LOCK in register @ref RG_IRQ_STATUS */
#define SR_IRQ_0_PLL_LOCK         0x0F, 0x01, 0

// Offset for register VREG_CTRL
#define RG_VREG_CTRL              (0x10)

/* Access parameters for sub-register AVREG_EXT in register @ref RG_VREG_CTRL */
#define SR_AVREG_EXT              0x10, 0x80, 7
/* Access parameters for sub-register AVDD_OK in register @ref RG_VREG_CTRL */
#define SR_AVDD_OK                0x10, 0x40, 6
#define SR_reserved_10_3          0x10, 0x30, 4
/* Access parameters for sub-register DVREG_EXT in register @ref RG_VREG_CTRL */
#define SR_DVREG_EXT              0x10, 0x08, 3
/* Access parameters for sub-register DVDD_OK in register @ref RG_VREG_CTRL */
#define SR_DVDD_OK                0x10, 0x04, 2
#define SR_reserved_10_6          0x10, 0x03, 0

// Offset for register BATMON
#define RG_BATMON                 (0x11)

#define SR_reserved_11_1          0x11, 0xC0, 6
/* Access parameters for sub-register BATMON_OK in register @ref RG_BATMON */
#define SR_BATMON_OK              0x11, 0x20, 5
/* Access parameters for sub-register BATMON_HR in register @ref RG_BATMON */
#define SR_BATMON_HR              0x11, 0x10, 4
/* Access parameters for sub-register BATMON_VTH in register @ref RG_BATMON */
#define SR_BATMON_VTH             0x11, 0x0F, 0

// Offset for register XOSC_CTRL
#define RG_XOSC_CTRL              (0x12)

/* Access parameters for sub-register XTAL_MODE in register @ref RG_XOSC_CTRL */
#define SR_XTAL_MODE              0x12, 0xF0, 4
/* Access parameters for sub-register XTAL_TRIM in register @ref RG_XOSC_CTRL */
#define SR_XTAL_TRIM              0x12, 0x0F, 0

// Offset for register RX_SYN
#define RG_RX_SYN                 (0x15)

/* Access parameters for sub-register RX_PDT_DIS in register @ref RG_RX_SYN */
#define SR_RX_PDT_DIS             0x15, 0x80, 7
#define SR_reserved_15_2          0x15, 0x70, 4
/* Access parameters for sub-register RX_PDT_LEVEL in register @ref RG_RX_SYN */
#define SR_RX_PDT_LEVEL           0x15, 0x0F, 0

// Offset for register XAH_CTRL_1
#define RG_XAH_CTRL_1             (0x17)

#define SR_reserved_17_1          0x17, 0xC0, 6
/* Access parameters for sub-register AACK_FLTR_RES_FT in register @ref RG_XAH_CTRL_1 */
#define SR_AACK_FLTR_RES_FT       0x17, 0x20, 5
/* Access parameters for sub-register AACK_UPLD_RES_FT in register @ref RG_XAH_CTRL_1 */
#define SR_AACK_UPLD_RES_FT       0x17, 0x10, 4
#define SR_reserved_17_4          0x17, 0x08, 3
/* Access parameters for sub-register AACK_ACK_TIME in register @ref RG_XAH_CTRL_1 */
#define SR_AACK_ACK_TIME          0x17, 0x04, 2
/* Access parameters for sub-register AACK_PROM_MODE in register @ref RG_XAH_CTRL_1 */
#define SR_AACK_PROM_MODE         0x17, 0x02, 1
#define SR_reserved_17_7          0x17, 0x01, 0

// Offset for register FTN_CTRL
#define RG_FTN_CTRL               (0x18)

/* Access parameters for sub-register FTN_CTRL in register @ref RG_FTN_CTRL */
#define SR_FTN_CTRL               0x18, 0x80, 7
#define SR_reserved_18_2          0x18, 0x7F, 0

// Offset for register PLL_CF
#define RG_PLL_CF                 (0x1A)

/* Access parameters for sub-register PLL_CF_START in register @ref RG_PLL_CF */
#define SR_PLL_CF_START           0x1A, 0x80, 7
#define SR_reserved_1a_2          0x1A, 0x7F, 0

// Offset for register PPL_DCU
#define RG_PPL_DCU                (0x1B)

/* Access parameters for sub-register PLL_DCU_START in register @ref RG_PLL_DCU */
#define SR_PLL_DCU_START          0x1B, 0x80, 7
#define SR_reserved_1b_2          0x1B, 0x7F, 0

// Offset for register PART_NUM
#define RG_PART_NUM               (0x1C)

/* Access parameters for sub-register PART_NUM in register @ref RG_PART_NUM */
#define SR_PART_NUM               0x1C, 0xFF, 0

// Offset for register VERSION_NUM
#define RG_VERSION_NUM            (0x1D)

/* Access parameters for sub-register VERSION_NUM in register @ref RG_VERSION_NUM */
#define SR_VERSION_NUM            0x1D, 0xFF, 0

// Offset for register MAN_ID_0
#define RG_MAN_ID_0               (0x1E)

/* Access parameters for sub-register MAN_ID_0 in register @ref RG_MAN_ID_0 */
#define SR_MAN_ID_0               0x1E, 0xFF, 0

// Offset for register MAN_ID_1
#define RG_MAN_ID_1               (0x1F)

/* Access parameters for sub-register MAN_ID_1 in register @ref RG_MAN_ID_1 */
#define SR_MAN_ID_1               0x1F, 0xFF, 0

// Offset for register SHORT_ADDR_0
#define RG_SHORT_ADDR_0           (0x20)

/* Access parameters for sub-register SHORT_ADDR_0 in register @ref RG_SHORT_ADDR_0 */
#define SR_SHORT_ADDR_0           0x20, 0xFF, 0

// Offset for register SHORT_ADDR_1
#define RG_SHORT_ADDR_1           (0x21)

/* Access parameters for sub-register SHORT_ADDR_1 in register @ref RG_SHORT_ADDR_1 */
#define SR_SHORT_ADDR_1           0x21, 0xFF, 0

// Offset for register PAN_ID_0
#define RG_PAN_ID_0               (0x22)

/* Access parameters for sub-register PAN_ID_0 in register @ref RG_PAN_ID_0 */
#define SR_PAN_ID_0               0x22, 0xFF, 0

// Offset for register PAN_ID_1
#define RG_PAN_ID_1               (0x23)

/* Access parameters for sub-register PAN_ID_1 in register @ref RG_PAN_ID_1 */
#define SR_PAN_ID_1               0x23, 0xFF, 0

// Offset for register IEEE_ADDR_0
#define RG_IEEE_ADDR_0            (0x24)

/* Access parameters for sub-register IEEE_ADDR_0 in register @ref RG_IEEE_ADDR_0 */
#define SR_IEEE_ADDR_0            0x24, 0xFF, 0

// Offset for register IEEE_ADDR_1
#define RG_IEEE_ADDR_1            (0x25)

/* Access parameters for sub-register IEEE_ADDR_1 in register @ref RG_IEEE_ADDR_1 */
#define SR_IEEE_ADDR_1            0x25, 0xFF, 0

// Offset for register IEEE_ADDR_2
#define RG_IEEE_ADDR_2            (0x26)

/* Access parameters for sub-register IEEE_ADDR_2 in register @ref RG_IEEE_ADDR_2 */
#define SR_IEEE_ADDR_2            0x26, 0xFF, 0

// Offset for register IEEE_ADDR_3
#define RG_IEEE_ADDR_3            (0x27)

/* Access parameters for sub-register IEEE_ADDR_3 in register @ref RG_IEEE_ADDR_3 */
#define SR_IEEE_ADDR_3            0x27, 0xFF, 0

// Offset for register IEEE_ADDR_4
#define RG_IEEE_ADDR_4            (0x28)

/* Access parameters for sub-register IEEE_ADDR_4 in register @ref RG_IEEE_ADDR_4 */
#define SR_IEEE_ADDR_4            0x28, 0xFF, 0

// Offset for register IEEE_ADDR_5
#define RG_IEEE_ADDR_5            (0x29)

/* Access parameters for sub-register IEEE_ADDR_5 in register @ref RG_IEEE_ADDR_5 */
#define SR_IEEE_ADDR_5            0x29, 0xFF, 0

// Offset for register IEEE_ADDR_6
#define RG_IEEE_ADDR_6            (0x2A)

/* Access parameters for sub-register IEEE_ADDR_6 in register @ref RG_IEEE_ADDR_6 */
#define SR_IEEE_ADDR_6            0x2A, 0xFF, 0

// Offset for register IEEE_ADDR_7
#define RG_IEEE_ADDR_7            (0x2B)

/* Access parameters for sub-register IEEE_ADDR_7 in register @ref RG_IEEE_ADDR_7 */
#define SR_IEEE_ADDR_7            0x2B, 0xFF, 0

// Offset for register XAH_CTRL_0
#define RG_XAH_CTRL_0             (0x2C)

/* Access parameters for sub-register MAX_FRAME_RETRIES in register @ref RG_XAH_CTRL_0 */
#define SR_MAX_FRAME_RETRIES      0x2C, 0xF0, 4
/* Access parameters for sub-register MAX_CSMA_RETRIES in register @ref RG_XAH_CTRL_0 */
#define SR_MAX_CSMA_RETRIES       0x2C, 0x0E, 1
/* Access parameters for sub-register SLOTTED_OPERATION in register @ref RG_XAH_CTRL_0 */
#define SR_SLOTTED_OPERATION      0x2C, 0x01, 0

// Offset for register CSMA_SEED_0
#define RG_CSMA_SEED_0            (0x2D)

/* Access parameters for sub-register CSMA_SEED_0 in register @ref RG_CSMA_SEED_0 */
#define SR_CSMA_SEED_0            0x2D, 0xFF, 0

// Offset for register CSMA_SEED_1
#define RG_CSMA_SEED_1            (0x2E)

/* Access parameters for sub-register AACK_FVN_MODE in register @ref RG_CSMA_SEED_1 */
#define SR_AACK_FVN_MODE          0x2E, 0xC0, 6
/* Access parameters for sub-register AACK_SET_PD in register @ref RG_CSMA_SEED_1 */
#define SR_AACK_SET_PD            0x2E, 0x20, 5
/* Access parameters for sub-register AACK_DIS_ACK in register @ref RG_CSMA_SEED_1 */
#define SR_AACK_DIS_ACK           0x2E, 0x10, 4
/* Access parameters for sub-register AACK_I_AM_COORD in register @ref RG_CSMA_SEED_1 */
#define SR_AACK_I_AM_COORD        0x2E, 0x08, 3
/* Access parameters for sub-register CSMA_SEED_1 in register @ref RG_CSMA_SEED_1 */
#define SR_CSMA_SEED_1            0x2E, 0x07, 0

// Offset for register CSMA_BE
#define RG_CSMA_BE                (0x2F)

/* Access parameters for sub-register MAX_BE in register @ref RG_CSMA_BE */
#define SR_MAX_BE                 0x2F, 0xF0, 4
/* Access parameters for sub-register MIN_BE in register @ref RG_CSMA_BE */
#define SR_MIN_BE                 0x2F, 0x0F, 0

//=========================== typedef =========================================

//=========================== variables =======================================

//=========================== prototypes ======================================

#endif /* AT86RF231_H_ */
