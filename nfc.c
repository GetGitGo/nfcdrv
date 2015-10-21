/* @file nfc.c
 * @brief kernel driver for transmiting user data package by nand flash controller.  
 *
 *
 */

#include "nfc.h" 

#define NFC_DRIVER_VERSION	 "1.0"

MODULE_AUTHOR("www.vhd.com.cn");
MODULE_DESCRIPTION("nfc Driver");
MODULE_LICENSE("GPL");
MODULE_VERSION(NFC_DRIVER_VERSION);

static volatile void *reg_base;
static volatile void *buf_base;

static unsigned int offset;
static unsigned int command;

static int ecctype;


/* Current system has already gone to sync mode */
#define NFC_IS_SYNC(_host) (nfc_con & NFC_CON_NF_MODE_MASK)
static unsigned long nfc_con;

static unsigned int addr_cycle;
static unsigned int addr_value[2];

static unsigned int dma_buffer;

/* This is maybe an un-aligment address, only for malloc or free */
static char *buforg;
static char *buffer;

static int  version;

static uint pagesize = DEFAULT_PAGESIZE;
module_param(w_lcnt, uint, S_IRUGO);
MODULE_PARM_DESC(uint,
		 "nfc register NFC_CON:page_size 2K:1 3K:2 8K:3 16K:4");

static uint w_lcnt = DEFAULT_NFC_W_LCNT;
module_param(w_lcnt, uint, S_IRUGO);
MODULE_PARM_DESC(uint,
		 "nfc register NFC_PWIDTH:w_lcnt");

static uint r_lcnt = DEFAULT_NFC_R_LCNT;
module_param(r_lcnt, uint, S_IRUGO);
MODULE_PARM_DESC(uint,
		 "nfc register NFC_PWIDTH:r_lcnt");

static uint rw_hcnt = DEFAULT_NFC_RW_HCNT;
module_param(rw_hcnt, uint, S_IRUGO);
MODULE_PARM_DESC(uint,
		 "nfc register NFC_PWIDTH:rw_hcnt");

static uint chipselect = 0;
module_param(chipselect, uint, S_IRUGO);
MODULE_PARM_DESC(uint,
		 "nfc chip select:0,1");

static struct nfc_host *host = NULL; 

/*
* nfc_contoller_enable - Enable or disable nand flash controller
* @param enable : 1 - enable, 0 - disable
*/
void nfc_controller_enable(int enable)
{
	unsigned int reg_val = readl(NFC_PERIPHERY_REGBASE + PERI_CRG52);

	if (enable)
		reg_val |= (PERI_CRG52_CLK_EN | PERI_CRG52_CLK_SEL_198M);
	else
		reg_val &= ~PERI_CRG52_CLK_EN;

	writel(reg_val, (NFC_PERIPHERY_REGBASE + PERI_CRG52));
}

/*
* nfc_dma_transfer - read or write data from or to device via DMA
*
* @param todev: read from (0) or write to (1)
*
*/
static void nfc_dma_transfer(int todev)
{
	unsigned long reg_val;
	unsigned int dma_addr = (unsigned int)dma_buffer;

	nfc_write( dma_addr, NFC_DMA_ADDR_DATA);

	dma_addr += NFC_DMA_ADDR_OFFSET;
	nfc_write( dma_addr, NFC_DMA_ADDR_DATA1);

	dma_addr += NFC_DMA_ADDR_OFFSET;
	nfc_write( dma_addr, NFC_DMA_ADDR_DATA2);

	dma_addr += NFC_DMA_ADDR_OFFSET;
	nfc_write( dma_addr, NFC_DMA_ADDR_DATA3);

	nfc_write( NFC_DMA_PARA_DATA_RW_EN, NFC_DMA_PARA);

	reg_val = (NFC_DMA_CTRL_DMA_START
		| NFC_DMA_CTRL_BURST4_EN
		| NFC_DMA_CTRL_BURST8_EN
		| NFC_DMA_CTRL_BURST16_EN
		| ((addr_cycle == 4 ? 1 : 0)
		   << NFC_DMA_CTRL_ADDR_NUM_SHIFT)
		| ((chipselect & NFC_DMA_CTRL_CS_MASK)
		   << NFC_DMA_CTRL_CS_SHIFT));

	if (todev)
		reg_val |= NFC_DMA_CTRL_WE;

	nfc_write( reg_val, NFC_DMA_CTRL);

	do {
		unsigned int timeout = 0xF0000000;
		while ((nfc_read( NFC_DMA_CTRL))
			& NFC_DMA_CTRL_DMA_START && timeout) {
			_cond_resched();
			timeout--;
		}
		if (!timeout)
			PR_BUG("Wait DMA finish timeout.\n");
	} while (0);
}

/*
* nfc_send_cmd_readid - 
* @param 
* @return 
*/
static int nfc_send_cmd_readid(struct nfc_host *host)
{
	unsigned int regval;

	nfc_write( NFC_NANDINFO_LEN, NFC_DATA_NUM);
	nfc_write( NAND_CMD_READID, NFC_CMD);
	nfc_write( 0, NFC_ADDRL);

	/* no need to config NFC_OP_WAIT_READY_EN, here not config. */
	regval = NFC_OP_CMD1_EN
		 | NFC_OP_ADDR_EN
		 | NFC_OP_READ_DATA_EN
		 | ((chipselect & NFC_OP_NF_CS_MASK)
		    << NFC_OP_NF_CS_SHIFT)
		 | (1 << NFC_OP_ADDR_CYCLE_SHIFT);

	nfc_write( regval, NFC_OP);

	addr_cycle = 0x0;

	WAIT_CONTROLLER_FINISH();

	return 0;
}

/*
* nfc_send_cmd_status - 
* @param 
* @return 
*/
static int nfc_send_cmd_status(struct nfc_host *host)
{
	unsigned int regval;

	nfc_write( NFC_NANDINFO_LEN, NFC_DATA_NUM);
	nfc_write( NAND_CMD_STATUS, NFC_CMD);

	/* no need config NFC_OP_WAIT_READY_EN, here not config */
	regval = NFC_OP_CMD1_EN
		 | NFC_OP_READ_DATA_EN
		 | ((chipselect & NFC_OP_NF_CS_MASK)
		    << NFC_OP_NF_CS_SHIFT);

	nfc_write( regval, NFC_OP);

	WAIT_CONTROLLER_FINISH();

	return 0;
}

/*
* nfc_send_cmd_reset - 
* @param 
* @return 
*/
static int nfc_send_cmd_reset( void )
{
	unsigned int regval;

	nfc_write( NAND_CMD_RESET, NFC_CMD);

	/* need to config NFC_OP_WAIT_READY_EN */
	regval = NFC_OP_CMD1_EN
		 | (((chipselect & NFC_OP_NF_CS_MASK)
		    << NFC_OP_NF_CS_SHIFT)
		 | NFC_OP_WAIT_READY_EN);

	nfc_write( regval, NFC_OP);

	WAIT_CONTROLLER_FINISH();

	return 0;
}

/*
* nfc_read_byte - 
* @param 
* @return 
*/
uint8_t nfc_read_byte(void)
{
	if (command == NAND_CMD_STATUS)
		return readb(chip->buf_base);

	offset++;

	if (command == NAND_CMD_READID)
		return readb(chip->buf_base + offset - 1);

	return readb(buffer + column + offset - 1);
}

/*
* nfc_read_word - 
* @param 
* @return 
*/
u16 nfc_read_word( void )
{
	offset += 2;
	return readw(buffer + column + offset - 2);
}

/*
* nfc_write_buf - 
* @param 
* @return 
*/
void nfc_write_buf(const uint8_t *buf, int len)
{
	memcpy(buffer + column + offset, buf, len);
	offset += len;
}

/*
* nfc_read_buf - 
* @param 
* @return 
*/
void nfc_read_buf(uint8_t *buf, int len)
{
	memcpy(buf, buffer + column + offset, len);
	offset += len;
}

/*
* nfc_dev_init - 
* @param 
* @return 
*/
int nfc_dev_init( void )
{
	int size;
	int result = 0;
	unsigned int regval;

	host = kcalloc(1,sizeof(struct nfc_host), GFP_KERNEL);
	if (!host) {
		PR_BUG("failed to allocate device structure.\n");
		return -ENOMEM;
	}

	version = nfc_read( NFC_VERSION);

	addr_value[0] = 0;
	addr_value[1] = 0;
	addr_cycle    = 4;

	nfc_controller_enable(1);

	reg_base = ioremap(CONFIG_NFC_REG_BASE_ADDRESS, NFC_REG_SIZE);
	if (!reg_base) {
		PR_BUG("ioremap failed\n");
		free(host);
		return -1;
	}

	buf_base = ioremap(CONFIG_NFC_BUFFER_BASE_ADDRESS, NFC_BUFFER_SIZE);
	if (!buf_base) {
		PR_BUG("ioremap failed\n");
		return -1;
	}

	buffer = dma_alloc_coherent(dev,
		(NAND_MAX_PAGESIZE + NAND_MAX_OOBSIZE),
		&dma_buffer, GFP_KERNEL);
	if (!buffer) {
		PR_BUG("Can't malloc memory for NAND driver.");
		return -EIO;
	}

	nfc_con = NFC_CON_OP_MODE_NORMAL 
        | NFC_CON_READY_BUSY_SEL;
		& (~(NFC_CON_ECCTYPE_MASK
		     << NFC_CON_ECCTYPE_SHIFT))
		& (~NFC_CON_RANDOMIZER_EN);

	nfc_write(
		(SET_NFC_PWIDTH(CONFIG_NFC_W_LCNT,
				     CONFIG_NFC_R_LCNT,
				     CONFIG_NFC_RW_HCNT)),
		NFC_PWIDTH);

	regval = nfc_read( NFC_BOOT_CFG);
	/* check if chip is sync mode. */
	if (regval & NFC_BOOT_CFG_SYC_NAND_PAD) {
		/*
		 * NAND default is sync mode, and read id, reset in sync mode.
		 */
		nfc_con |= NFC_CON_NF_MODE_TOGGLE;

		/* set synchronous clock and timing. */
		nfc_controller_enable( ENABLE);
	}

	memset((char *)chip->buf_base, 0xff, NFC_BUFFER_BASE_ADDRESS_LEN);

fail:
	if (buffer) {
		dma_free_coherent(dev,
			(NAND_MAX_PAGESIZE + NAND_MAX_OOBSIZE),
			buffer,
			dma_buffer);
		buffer = NULL;
	}
	iounmap(buf_base);
	iounmap(reg_base);
	free(host);

	return result;
}

#ifdef CONFIG_PROC_FS
static int nfc_proc_open(struct inode *inode, struct file *file);

static const struct file_operations nfc_proc_fops = {
	.owner		= THIS_MODULE,
	.open		= rtc_proc_open,
	.read		= seq_read,
	.llseek		= seq_lseek,
	.release	= single_release,
};


/*
* nfc_proc_open - driver's proc file operation "open" 
*
* @param inode:
* @param file:
*
*/
static int nfc_proc_show(struct seq_file *seq, void *v)
{
	seq_printf(seq, "rtc_timet:\n");
	seq_puts(seq, "alarm: ");
}

/*
* nfc_proc_open - driver's proc file operation "open" 
*
* @param inode:
* @param file:
*
*/
static int nfc_proc_open(struct inode *inode, struct file *file)
{
	return single_open(file, nfc_proc_show, NULL);
}
#endif

/*
* nfc_open - driver's file operation "open" 
*
* @param inode:
* @param file:
*
*/
static int nfc_open(struct inode* inode, struct file* file)
{
    return 0 ;
}

/*
* nfc_close - driver's file operation "close" 
*
* @param inode:
* @param file:
*
*/
static int  nfc_close(struct inode* inode, struct file* file)
{
    return 0;
}

/*
* nfc_write - driver's file operation "write" 
*
* @param file:
* @param buffer:
* @param count:
* @param ppos:
# @return 
*/
static ssize_t nfc_write(struct file *file, const char __user *buffer,
			    size_t count, loff_t *ppos)
{
	size_t ret;

	nfc_write( addr_value[0] & 0xffff0000, NFC_ADDRL);
	nfc_write( addr_value[1], NFC_ADDRH);
	nfc_write( 
        ((NAND_CMD_STATUS << 16) | (NAND_CMD_PAGEPROG << 8) | NAND_CMD_SEQIN),
		NFC_CMD);

	nfc_dma_transfer( 1);

	return (ssize_t)count;
}

/*
* nfc_ioctl - driver's file operation "ioctl" 
*
* @param file:
* @param cmd:
* @param arg:
*
* @return 
*/
static long nfc_ioctl(struct file* file, unsigned int cmd, unsigned long arg)
{

    switch (cmd) {
        case NFC_IOC_CMD_READID:
            memset((unsigned char *)(chip->buf_base), 0, 0x10);
            command = NAND_CMD_READID;
            nfc_send_cmd_readid();
            break;

        case NFC_IOC_CMD_STATUS:
            command = NAND_CMD_STATUS;
            nfc_send_cmd_status();
            break;

        case NAND_IOC_CMD_RESET:
            command = NAND_CMD_RESET;
            nfc_send_cmd_reset();
            break;

        default:
            break;
    }
}

static struct file_operations nfc_fops =
{
    .owner      = THIS_MODULE,
    .open       = nfc_fo_open,
    .release    = nfc_close,
    .write      = nfc_write,
    .unlocked_ioctl = nfc_ioctl,
};

static struct miscdevice nfc_dev =
{
    .minor   = MISC_DYNAMIC_MINOR,
    .name    = "nfc",
    .fops    = &nfc_fops,
};


/*
* nfc_init - module init and exit 
* @param 
* @return 
*/
static int __init nfc_init(void)
{
#ifdef CONFIG_PROC_FS
	struct proc_dir_entry *ent;
#endif
    int     ret;

	if (chipselect > NFC_MAX_CHIP){
		PR_BUG("invalid chipselect: %u, set 0\n", chipselect);
	    chipselect = 0;
    }

    if ( pagesize != NFC_PAGESIZE_2K || pagesize != NFC_PAGESIZE_4K ||
         pagesize != NFC_PAGESIZE_8K || pagesize != NFC_PAGESIZE_16K ){
		PR_BUG("invalid pagesize: %u, set 2K:0\n", pagesize);
	    pagesize = 0;
    }

	if (w_lcnt > DEFAULT_NFC_W_LCNT ){
		PR_BUG("invalid w_lcnt: %u, set 0\n", w_lcnt);
	    w_lcnt = 0;
    }

    nfc_dev_init();

    ret = misc_register(&nfc_dev);
    if (ret != 0) {
        printk("register i2c device failed with %#x!\n", ret);
        return -1;
    }

#ifdef CONFIG_PROC_FS
/*
 *	Info exported via "/proc/driver/nfc".
 */
	ent = proc_create("driver/nfc", 0, NULL, &nfc_proc_fops);
	if (!ent) printk(KERN_WARNING "rtc: Failed to register with procfs.\n");
#endif

    return 0;
}

/*
* nfc_exit - 
* @param 
* @return 
*/
static void __exit nfc_exit(void)
{
    int i;

    misc_deregister(&nfc_dev);
    nfc_controller_enable(0);
}


module_init(nfc_init);
module_exit(nfc_exit);

