/* */
/* according to << Hi3516A/Hi3516D HD IP Camera Soc USER MANUANL, 03, 2015-06-19>> */
/* */

#ifndef NFC_H
#define NFC_H

#include <linux/version.h>
#include <linux/kernel.h>
#include <linux/sched.h>
#include <linux/init.h>
#include <linux/slab.h>
#include <linux/module.h>
#include <linux/mtd/mtd.h>
#include <linux/mtd/nand.h>
#include <linux/delay.h>
#include <linux/dma-mapping.h>
#include <asm/io.h>
#include <asm/setup.h>
#include <asm/errno.h>
#include <linux/platform_device.h>
#include <linux/mtd/partitions.h>
#include <mach/clkdev.h>
#include <linux/clk.h>
#include <linux/clkdev.h>
#include <mach/clkdev.h>

#ifdef __cplusplus
#if __cplusplus
extern "C" {
#endif
#endif /* __cplusplus */

#define NFC_PERIPHERY_REGBASE		IO_ADDRESS(0x20030000)

#define PERI_CRG52                  (0x00D0)
#define PERI_CRG52_CLK_EN			(1U << 1)
#define PERI_CRG52_CLK_SEL_198M		(1U << 2)

#define NFC_REG_BASE_ADDRESS            (0x10000000)
#define NFC_BUFFER_BASE_ADDRESS         (0x50000000)


#define NFC_MAX_CHIP                 (1)

#define NFC_REG_SIZE                 (0x100)
#define NFC_BUFFER_SIZE              (8192 + 1024)

#define NFC_CHIP_DELAY                           (25)

#define NFC_ADDR_CYCLE_MASK                      0x4
#define NFC_DMA_ADDR_OFFSET                      4096

/*----------- register NFC_CON ------------*/

#define NFC_CON                                 0x00

#define NFC_CON_OP_MODE_SHIFT       (0)
#define NFC_CON_OP_MODE_MASK        (0x01)
#define OP_MODE_BOOT        (0x00)
#define OP_MODE_NORMAL      (0x01)

#define NFC_CON_PAGE_SIZE_SHIFT     (1)
#define NFC_CON_PAGESIZE_MASK       (0x07)
#define PAGESIZE_2K     0x01
#define PAGESIZE_4K     0x02
#define PAGESIZE_8K     0x03
#define PAGESIZE_16K    0x04

#define NFC_CON_BUS_WIDTH_SHIFT     (4)
#define NFC_CON_BUS_WIDTH_MASK      (0x01)
#define BUS_WIDTH_8BIT      0
#define BUS_WIDTH_RESERVED  1

#define NFC_CON_CS_CTRL_SHIFT       (7)
#define NFC_CON_CS_CTRL_MASK        (0x01)
#define CS_CTRL_BUSY_0      0
#define CS_CTRL_BUSY_1      1

#define NFC_CON_RB_SEL_SHIFT       (8)
#define NFC_CON_RB_SEL_MASK        (0x01)
#define RB_SEL_SHARED       0
#define RB_SEL_EXCLUDED     1

#define NFC_CON_ECCTYPE_SHIFT       (9)
#define NFC_CON_ECCTYPE_MASK        (0x0f)
#define ECC_TYPE_NONE   0x00
#define ECC_TYPE_8BIT   0x02
#define ECC_TYPE_24BIT  0x04
#define ECC_TYPE_40BIT  0x05
#define ECC_TYPE_64BIT  0x06

#define NFC_CON_RANDOMIZER_EN_SHIFT (14)
#define NFC_CON_RANDOMIZER_EN_MASK  (0x01)
#define RANDOMIZER_EN_CLOSE 0
#define RANDOMIZER_EN_OPEN  1

/* ?: reserved in manual, why here? */

#define NFC_CON_NF_MODE_SHIFT       15
#define NFC_CON_NF_MODE_MASK        (3 << NFC_CON_NF_MODE_SHIFT)

#define NFC_CON_NF_MODE_TOGGLE      (1 << NFC_CON_NF_MODE_SHIFT)
#define NFC_CON_NF_MODE_ONFI_23     (2 << NFC_CON_NF_MODE_SHIFT)
#define NFC_CON_NF_MODE_ONFI_30     (3 << NFC_CON_NF_MODE_SHIFT)


/*----------- register NFC_PWIDTH------------*/

#define NFC_PWIDTH                              0x04

#define NFC_PWIDTH_W_LCNT_SHIFT         (0)
#define NFC_PWIDTH_W_LCNT_MASK          (0x0f)
#define DEFAULT_W_LCNT          (0x05)
#define MIN_W_LCNT              (0x01)
#define MAX_W_LCNT              (0x0f)

#define NFC_PWIDTH_R_LCNT_SHIFT         (4)
#define NFC_PWIDTH_R_LCNT_MASK          (0x0f)
#define DEFAULT_R_LCNT          (7)
#define MIN_R_LCNT              (0x01)
#define MAX_R_LCNT              (0x0f)

#define NFC_PWIDTH_RW_HCNT_SHIFT        (8)
#define NFC_PWIDTH_RW_HCNT_MASK         (0x0f)
#define DEFAULT_RW_HCNT          (3)
#define MIN_RW_HCNT              (0x01)
#define MAX_RW_HCNT              (0x0f)

/*----------- register NFC_OPIDLE ------------*/

#define NFC_OPIDLE                              0x08

#define NFC_OPIDLE_FB_WAIT_SHIFT      (20)
#define NFC_OPIDLE_FB_WAIT_MASK       (0x0f)
#define DEFAULT_FRB_WAIT          (3)
#define MIN_FRB_WAIT              (0x00)
#define MAX_FRB_WAIT              (0x0f)

#define NFC_OPIDLE_CMD1_WAIT_SHIFT    (16)
#define NFC_OPIDLE_CMD1_WAIT_MASK     (0x0f)
#define DEFAULT_CMD1_WAIT          (3)
#define MIN_CMD1_WAIT              (0x00)
#define MAX_CMD1_WAIT              (0x0f)

#define NFC_OPIDLE_ADDR_WAIT_SHIFT    (12)
#define NFC_OPIDLE_ADDR_WAIT_MASK     (0x0f)
#define DEFAULT_ADDR_WAIT          (3)
#define MIN_ADDR_WAIT              (0x00)
#define MAX_ADDR_WAIT              (0x0f)

#define NFC_OPIDLE_CMD2_WAIT_SHIFT    (4)
#define NFC_OPIDLE_CMD2_WAIT_MASK     (0x0f)
#define DEFAULT_CMD2_WAIT          (3)
#define MIN_CMD2_WAIT              (0x00)
#define MAX_CMD2_WAIT              (0x0f)

#define NFC_OPIDLE_WAIT_READY_WAIT_SHIFT    (0)
#define NFC_OPIDLE_WAIT_READY_WAIT_MASK     (0x0f)
#define DEFAULT_WAIT_READY_WAIT    (3)
#define MIN_WAIT_READY_WAIT        (0x00)
#define MAX_WAIT_READY_WAIT        (0x0f)

/*----------- register NFC_CMD ------------*/

#define NFC_CMD                                 0x0C

#define NFC_CMD_CMD1_SHIFT      (0)
#define NFC_CMD_CMD1_MASK       (0xff)

#define NFC_CMD_CMD2_SHIFT      (8)
#define NFC_CMD_CMD2_MASK       (0xff)

#define NFC_CMD_READ_STATUS_CMD_SHIFT      (16)
#define NFC_CMD_READ_STATUS_CMD_MASK       (0xff)

/*----------- register NFC_ADDRL ------------*/

#define NFC_ADDRL                               0x10

#define DEFAULT_ADDRL                (0)

/*----------- register NFC_ADDRH ------------*/

#define NFC_ADDRH                               0x14
#define NFC_ADDRH_ADDR_H_SHIFT      (0)
#define NFC_ADDRH_ADDR_H_MASK       (0xff)

#define DEFAULT_ADDRH                (0)

/*----------- register NFC_DATA_NUM ------------*/

#define NFC_DATA_NUM                            0x18

#define DEFAULT_NFC_DATA_NUM        (2048)
#define MIN_NFC_DATA_NUM            (0)
#define MAX_NFC_DATA_NUM            (9344)


/*----------- register NFC_OP ------------*/

#define NFC_OP                                  0x1C

#define NFC_OP_ADDR_CYCLE_SHIFT     (9)
#define NFC_OP_ADDR_CYCLE_MASK      (0x07)

#define NFC_OP_NF_CS_SHIFT          (7)
#define NFC_OP_NF_CS_MASK           (0x03)


#define READ_STATUS_EN       (1U << 0)
#define READ_DATA_EN         (1U << 1)
#define WAIT_READY_EN        (1U << 2)
#define CMD2_EN              (1U << 3)
#define WRITE_DATA_EN        (1U << 4)
#define ADDR_EN              (1U << 5)
#define CMD1_EN              (1U << 6)
#define READID_EN            (1U << 12)
#define RW_REG_EN            (1U << 13)

/*----------- register NFC_STATUS ------------*/

#define NFC_STATUS                               0x20

#define NFC_INTS                                 0x28
#define NFC_INTS_UE                 (1U << 6)

#define NFC_INTCLR                               0x2C
#define NFC_INTCLR_UE               (1U << 6)
#define NFC_INTCLR_CE               (1U << 5)

#define NFC_DMA_CTRL                             0x60
#define NFC_DMA_CTRL_DMA_START      (1U << 0)
#define NFC_DMA_CTRL_WE             (1U << 1)
#define NFC_DMA_CTRL_BURST4_EN      (1U << 4)
#define NFC_DMA_CTRL_BURST8_EN      (1U << 5)
#define NFC_DMA_CTRL_BURST16_EN     (1U << 6)
#define NFC_DMA_CTRL_ADDR_NUM_SHIFT (7)
#define NFC_DMA_CTRL_ADDR_NUM_MASK  (1)
#define NFC_DMA_CTRL_CS_SHIFT       (8)
#define NFC_DMA_CTRL_CS_MASK        (0x03)

#define NFC_DMA_ADDR_DATA                        0x64
#define NFC_DMA_ADDR_OOB                         0x68
#define NFC_DMA_ADDR_DATA1                       0xB4
#define NFC_DMA_ADDR_DATA2                       0xB8
#define NFC_DMA_ADDR_DATA3                       0xBC
#define NFC_DMA_ADDR_DATA4                       0xEC
#define NFC_DMA_ADDR_DATA5                       0xF0
#define NFC_DMA_ADDR_DATA6                       0xF4
#define NFC_DMA_ADDR_DATA7                       0xF8

#define NFC_DMA_LEN                              0x6C
#define NFC_DMA_LEN_OOB_SHIFT       (16)
#define NFC_DMA_LEN_OOB_MASK        (0x1FFF)

#define NFC_DMA_PARA                             0x70
#define NFC_DMA_PARA_DATA_RW_EN     (1U << 0)
#define NFC_DMA_PARA_OOB_RW_EN      (1U << 1)
#define NFC_DMA_PARA_DATA_EDC_EN    (1U << 2)
#define NFC_DMA_PARA_OOB_EDC_EN     (1U << 3)
#define NFC_DMA_PARA_EXT_LEN_SHIFT  (6)
#define NFC_DMA_PARA_EXT_LEN_MASK   (0x03)

#define NFC_VERSION                              0x74
#define NFC_LOG_READ_ADDR                        0x7C
#define NFC_LOG_READ_LEN                         0x80

#define NFC_ECC_REG0                             0xA0
#define NFC_ECC_REG1                             0xA4
#define NFC_ECC_REG2                             0xA8
#define NFC_ECC_REG3                             0xAC

#define NFC_RANDOMIZER                           0xC0
#define NFC_RANDOMIZER_PAD           0x02
#define NFC_RANDOMIZER_ENABLE        0x01
/* read nand id or nand status, return from nand data length */
#define NFC_NANDINFO_LEN             0x10

#define NFC_BOOT_CFG                             0xC4
#define NFC_BOOT_CFG_RANDOMIZER_PAD         0x01
#define NFC_BOOT_CFG_SAVE_PIN_MODE_SHIFT    13
#define NFC_BOOT_CFG_SAVE_PIN_MODE   \
	(1U << NFC_BOOT_CFG_SAVE_PIN_MODE_SHIFT)
#define NFC_BOOT_CFG_SYC_NAND_PAD_SHIFT     12
#define NFC_BOOT_CFG_SYC_NAND_PAD    \
	(1U << NFC_BOOT_CFG_SYC_NAND_PAD_SHIFT)

#define NFC_SYNC_TIMING                          0xD0

/* TOGGLE: sync nand timing config */
#define NFC_SYNC_TOGGLE_PRE_RDQS     (0xF << 16)
#define NFC_SYNC_TOGGLE_POST_RDQS    (0xF << 12)
#define NFC_SYNC_TOGGLE_PRE_WDQS     (0xF << 8)
#define NFC_SYNC_TOGGLE_POST_WDQS    (0xF << 4)
#define NFC_SYNC_TOGGLE_RW_PSTH      (0xF << 0)

/* DMA address align with 32 bytes. */
#define NFC_DMA_ALIGN                            64

#define NFC_READ_1CMD_0ADD_NODATA \
	(NFC_OP_CMD1_EN \
	| ((chipselect & NFC_OP_NF_CS_MASK) \
		<< NFC_OP_NF_CS_SHIFT))

#define NFC_READ_1CMD_1ADD_DATA    \
	(NFC_OP_CMD1_EN \
	| NFC_OP_ADDR_EN \
	| NFC_OP_READ_DATA_EN \
	| ((chipselect & NFC_OP_NF_CS_MASK) \
		<< NFC_OP_NF_CS_SHIFT) \
	| (1 << NFC_OP_ADDR_CYCLE_SHIFT))

#define NFC_READ_1CMD_1ADD_DATA_WAIT_READY    \
	(NFC_OP_CMD1_EN \
	| NFC_OP_ADDR_EN \
	| NFC_OP_READ_DATA_EN \
	| NFC_OP_WAIT_READY_EN \
	| ((chipselect & NFC_OP_NF_CS_MASK) \
		<< NFC_OP_NF_CS_SHIFT) \
	| (1 << NFC_OP_ADDR_CYCLE_SHIFT))

#define NFC_READ_1CMD_1ADD_DATA_SYNC \
	(NFC_OP_CMD1_EN \
	| NFC_OP_ADDR_EN \
	| NFC_OP_READ_DATA_EN \
	| NFC_OP_WAIT_READY_EN \
	| NFC_OP_RW_REG_EN \
	| ((chipselect & NFC_OP_NF_CS_MASK) \
		<< NFC_OP_NF_CS_SHIFT) \
	| (1 << NFC_OP_ADDR_CYCLE_SHIFT))

#define NFC_READ_2CMD_5ADD    \
	(NFC_OP_CMD1_EN \
	| NFC_OP_CMD2_EN \
	| NFC_OP_ADDR_EN \
	| NFC_OP_READ_DATA_EN \
	| NFC_OP_WAIT_READY_EN \
	| ((chipselect & NFC_OP_NF_CS_MASK) \
		<< NFC_OP_NF_CS_SHIFT) \
	| (5 << NFC_OP_ADDR_CYCLE_SHIFT))

#define NFC_WRITE_0CMD_1ADD_DATA \
	(NFC_OP_ADDR_EN \
	| NFC_OP_WRITE_DATA_EN \
	| ((chipselect & NFC_OP_NF_CS_MASK) \
		<< NFC_OP_NF_CS_SHIFT) \
	| (1 << NFC_OP_ADDR_CYCLE_SHIFT))

#define NFC_WRITE_0CMD_1ADD_DATA_WAIT_READY \
	(NFC_OP_ADDR_EN \
	| NFC_OP_WRITE_DATA_EN \
	| NFC_OP_WAIT_READY_EN \
	| ((chipselect & NFC_OP_NF_CS_MASK) \
		<< NFC_OP_NF_CS_SHIFT) \
	| (1 << NFC_OP_ADDR_CYCLE_SHIFT))

#define NFC_WRITE_0CMD_1ADD_DATA_SYNC \
	(NFC_OP_ADDR_EN \
	 | NFC_OP_WRITE_DATA_EN \
	 | NFC_OP_RW_REG_EN \
	 | ((chipselect & NFC_OP_NF_CS_MASK) \
		 << NFC_OP_NF_CS_SHIFT) \
	 | (1 << NFC_OP_ADDR_CYCLE_SHIFT))

#define NFC_WRITE_0CMD_1ADD_DATA_SYNC_WAIT_READY \
	(NFC_OP_ADDR_EN \
	 | NFC_OP_WRITE_DATA_EN \
	 | NFC_OP_RW_REG_EN \
	 | NFC_OP_WAIT_READY_EN \
	 | ((chipselect & NFC_OP_NF_CS_MASK) \
		 << NFC_OP_NF_CS_SHIFT) \
	 | (1 << NFC_OP_ADDR_CYCLE_SHIFT))

#define NFC_WRITE_1CMD_1ADD_DATA  \
	(NFC_OP_CMD1_EN \
	| NFC_OP_ADDR_EN \
	| NFC_OP_WRITE_DATA_EN \
	| ((chipselect & NFC_OP_NF_CS_MASK) \
		<< NFC_OP_NF_CS_SHIFT) \
	| (1 << NFC_OP_ADDR_CYCLE_SHIFT))

#define NFC_WRITE_1CMD_1ADD_DATA_WAIT_READY  \
	(NFC_OP_CMD1_EN \
	| NFC_OP_ADDR_EN \
	| NFC_OP_WRITE_DATA_EN \
	| NFC_OP_WAIT_READY_EN \
	| ((chipselect & NFC_OP_NF_CS_MASK) \
		<< NFC_OP_NF_CS_SHIFT) \
	| (1 << NFC_OP_ADDR_CYCLE_SHIFT))

#define NFC_WRITE_1CMD_1ADD_DATA_SYNC  \
	(NFC_OP_CMD1_EN \
	 | NFC_OP_ADDR_EN \
	 | NFC_OP_WRITE_DATA_EN \
	 | NFC_OP_RW_REG_EN \
	 | ((chipselect & NFC_OP_NF_CS_MASK) \
		 << NFC_OP_NF_CS_SHIFT) \
	 | (1 << NFC_OP_ADDR_CYCLE_SHIFT))

#define NFC_WRITE_1CMD_1ADD_DATA_SYNC_WAIT_READY  \
	(NFC_OP_CMD1_EN \
	 | NFC_OP_ADDR_EN \
	 | NFC_OP_WRITE_DATA_EN \
	 | NFC_OP_WAIT_READY_EN \
	 | NFC_OP_RW_REG_EN \
	 | ((chipselect & NFC_OP_NF_CS_MASK) \
		 << NFC_OP_NF_CS_SHIFT) \
	 | (1 << NFC_OP_ADDR_CYCLE_SHIFT))

#define NFC_WRITE_1CMD_2ADD_DATA  \
	(NFC_OP_CMD1_EN \
	| NFC_OP_ADDR_EN \
	| NFC_OP_WRITE_DATA_EN \
	| ((chipselect & NFC_OP_NF_CS_MASK) \
		<< NFC_OP_NF_CS_SHIFT) \
	| (2 << NFC_OP_ADDR_CYCLE_SHIFT))

#define NFC_WRITE_1CMD_2ADD_DATA_SYNC  \
	(NFC_OP_CMD1_EN \
	 | NFC_OP_ADDR_EN \
	 | NFC_OP_WRITE_DATA_EN \
	 | NFC_OP_RW_REG_EN \
	 | ((chipselect & NFC_OP_NF_CS_MASK) \
		 << NFC_OP_NF_CS_SHIFT) \
	 | (2 << NFC_OP_ADDR_CYCLE_SHIFT))

#define NFC_WRITE_2CMD_0ADD_NODATA \
	(NFC_OP_CMD1_EN \
	| NFC_OP_CMD2_EN \
	| ((chipselect & NFC_OP_NF_CS_MASK) \
		<< NFC_OP_NF_CS_SHIFT))

#define NFC_WRITE_2CMD_0ADD_NODATA_SYNC \
	(NFC_OP_CMD1_EN \
	| NFC_OP_CMD2_EN \
	| NFC_OP_RW_REG_EN \
	| ((chipselect & NFC_OP_NF_CS_MASK) \
		<< NFC_OP_NF_CS_SHIFT))

#define NFC_WRITE_1CMD_0ADD_NODATA \
	(NFC_OP_CMD1_EN \
	| ((chipselect & NFC_OP_NF_CS_MASK) \
		<< NFC_OP_NF_CS_SHIFT))

#define NFC_WRITE_1CMD_0ADD_NODATA_WAIT_READY \
	(NFC_OP_CMD1_EN \
	| NFC_OP_WAIT_READY_EN \
	| ((chipselect & NFC_OP_NF_CS_MASK) \
		<< NFC_OP_NF_CS_SHIFT))

#define WAIT_CONTROLLER_FINISH() \
do { \
	unsigned int timeout = 0x800000; \
	while ((nfc_read(NFC_STATUS) & 0x1) == 0x0 && timeout) \
		timeout--; \
	if (!timeout) \
		PR_ERR("Wait NAND controller finish timeout.\n"); \
} while (0)

#define nfc_read(reg) \
	readl((char *)reg_base + (reg))

#define nfc_write(value, reg) \
	writel((value), (char *)reg_base + (reg))

#define NFC_CMD_SEQ(cmd0, cmd1)        \
	(((cmd0) & 0xFF) | ((cmd1) & 0xFF) << 8)

#define GET_PAGE_INDEX \
    ((addr_l >> 16) | (addr_h << 16))

#define DUMP_DATA(_p, _n) do { \
	int ix; \
	unsigned char *rr = (unsigned char *)(_p); \
	for (ix = 0; ix < _n; ix++) { \
		printk(KERN_INFO "%02X ", rr[ix]); \
		if (!((ix + 1) % 16)) \
			printk(KERN_INFO "\n"); \
	} \
} while (0)

#define DBG_OUT(fmt, args...) do {\
	printk(KERN_WARN "%s(%d): " fmt, __FILE__, __LINE__, ##args); \
} while (0)

#if 1
#  define DBG_MSG(_fmt, arg...)
#else
#  define DBG_MSG(_fmt, arg...) \
	printk(KERN_INFO "%s(%d): " _fmt, __FILE__, __LINE__, ##arg);
#endif

#define PR_BUG(fmt, args...) do {\
	printk("%s(%d): bug " fmt, __FILE__, __LINE__, ##args); \
	asm("b ."); \
} while (0)

#define PR_ERR(fmt, args...) do {\
	printk(KERN_ERR "%s(%d): " fmt, __FILE__, __LINE__, ##args); \
} while (0)

#define PR_MSG(_fmt, arg...) \
	printk(_fmt, ##arg)

uint8_t nfc_read_byte(void);
u16 nfc_read_word(void);
void nfc_write_buf(const uint8_t *buf, int len);
void nfc_read_buf(uint8_t *buf, int len);
int nfc_nand_init(struct nand_chip *chip);


#define NFC_IOC_CMD_READID 0x11
#define NAND_IOC_CMD_RESET 0x12
#define NFC_IOC_CMD_STATUS 0x13

#ifdef __cplusplus
#if __cplusplus
}
#endif
#endif /* __cplusplus */

#endif	/* NFC_H */

