/* @file nfc.c
 * @brief kernel driver for transmiting user data package by nand flash controller.  
 *
 *
 */

#include "nfc.h" 

#define NFC_DRIVER_VERSION	 "1.0"

static volatile int * reg_base;
static char * buf_base;

static unsigned int command;

static char *buffer;
static unsigned int dma_buffer;


/* configure variables of register NFC_CON */

static uint n_pagesize = DEFAULT_PAGESIZE;
module_param(n_pagesize, uint, S_IRUGO);
MODULE_PARM_DESC(n_pagesize, "nfc register NFC_CON:page_size 1:2K 2:3K 3:8K 4:16K");

/* configure variables of register NFC_PWIDTH */

static uint rw_hcnt = DEFAULT_RW_HCNT;
module_param(rw_hcnt, uint, S_IRUGO);
MODULE_PARM_DESC(rw_hcnt, "nfc register NFC_PWIDTH:rw_hcnt 0x1~0xf: 2~16 timing circles");

static uint r_lcnt = DEFAULT_R_LCNT;
module_param(r_lcnt, uint, S_IRUGO);
MODULE_PARM_DESC(r_lcnt, "nfc register NFC_PWIDTH:r_lcnt 0x2~0xf: 3~16 timing circles");

static uint w_lcnt = DEFAULT_W_LCNT;
module_param(w_lcnt, uint, S_IRUGO);
MODULE_PARM_DESC(w_lcnt, "nfc register NFC_PWIDTH:w_lcnt 0x1~0xf: 2~16 timing circles");


/* configure variables of register NFC_OPIDLE */

static uint frb_wait = DEFAULT_FRB_WAIT;
module_param(frb_wait, uint, S_IRUGO);
MODULE_PARM_DESC(frb_wait, "nfc register NFC_OPIDLE:frb_wait 0x0~0xf: frb_wait*8 timing circles");

static uint cmd1_wait = DEFAULT_CMD1_WAIT;
module_param(cmd1_wait, uint, S_IRUGO);
MODULE_PARM_DESC(cmd1_wait, "nfc register NFC_OPIDLE:cmd1_wait 0x0~0xf: 1~16 timing circles");

static uint addr_wait = DEFAULT_ADDR_WAIT;
module_param(addr_wait, uint, S_IRUGO);
MODULE_PARM_DESC(addr_wait, "nfc register NFC_OPIDLE:addr_wait 0x0~0xf: addr_wait*8 timing circles");

static uint cmd2_wait = DEFAULT_CMD2_WAIT;
module_param(cmd2_wait, uint, S_IRUGO);
MODULE_PARM_DESC(cmd2_wait, "nfc register NFC_OPIDLE:cmd2_wait 0x0~0xf: 1~16 timing circles");

static uint wait_ready_wait = DEFAULT_WAIT_READY_WAIT;
module_param(wait_ready_wait, uint, S_IRUGO);
MODULE_PARM_DESC(wait_ready_wait, "nfc register NFC_OPIDLE:wait_ready_wait 0x0~0xf: wait_ready_wait*8 timing circles");

/* configure variables of register NFC_ADDRL */

static uint addr_l = DEFAULT_ADDRL;
module_param(addr_l, uint, S_IRUGO);
MODULE_PARM_DESC(addr_l, "nfc register NFC_ADDRL:addr_l 31:0");

/* configure variables of register NFC_ADDRH */

static uint addr_h = DEFAULT_ADDRH;
module_param(addr_h, uint, S_IRUGO);
MODULE_PARM_DESC(addr_h,"nfc register NFC_ADDRH:addr_h 7:0");

/* configure variables of register NFC_OP */

static uint chipselect = 0;
module_param(chipselect, uint, S_IRUGO);
MODULE_PARM_DESC(uint, "nfc register NFC_OP:chipselect");

static uint addr_cycle = 4;
module_param(addr_cycle, uint, S_IRUGO);
MODULE_PARM_DESC(addr_cycle, "nfc register NFC_OP:addr_cycle 0:cs0 1:cs1");

/*
* nfc_contoller_enable - Enable or disable nand flash controller
* @param enable : 1 - enable, 0 - disable
*/
static void nfc_controller_enable(int enable)
{
	unsigned int reg_val;
    int i;

    
    /* check pin mux */

    for ( i = 0; i <= ( NFC_MUXCTRL_END - NFC_MUXCTRL_BEGIN); i += 4){
	    reg_val = readl(NFC_MUXCTRL_REGBASE + NFC_MUXCTRL_BEGIN + i );
        if ( reg_val != 0 ){
	        writel(0, NFC_MUXCTRL_REGBASE + NFC_MUXCTRL_BEGIN + i );
        }
    }

	reg_val = readl(NFC_PERIPHERY_REGBASE + PERI_CRG52);

	if (enable){
		reg_val |= (PERI_CRG52_CLK_EN | PERI_CRG52_CLK_SEL_198M);

		//reg_val |= PERI_CRG52_CLK_EN; 
		//reg_val &= ~PERI_CRG52_CLK_SEL_198M; // 24M clock
    }
	else {
		reg_val &= ~PERI_CRG52_CLK_EN;
    }

	writel(reg_val, (NFC_PERIPHERY_REGBASE + PERI_CRG52));
}

/*
* nfc_dma_tx- read or write data from or to device via DMA
*
* @param todev: read from (0) or write to (1)
*
*/
static void nfc_dma_tx( void )
{
	unsigned int reg_val;
	unsigned int dma_addr = (unsigned int)dma_buffer;

	nfc_write( dma_addr, NFC_BADDR_D0 );

	nfc_write( NFC_OP_PARA_DATA_RW_EN, NFC_OP_PARA);

	reg_val = DMA_START 
		    | DMA_WR_EN
            | BURST4_EN 
            | BURST8_EN 
            | BURST16_EN
        //    | WR_CMD_DISABLE // without cmd sequence
		    | ((addr_cycle == 4 ? 1 : 0) << NFC_DMA_CTRL_ADDR_NUM_SHIFT)
		    | (chipselect << NFC_DMA_CTRL_CS_SHIFT);


	nfc_write( reg_val, NFC_DMA_CTRL);

	do {
		unsigned int timeout = 0xF0000000;
		while ((nfc_read( NFC_DMA_CTRL))
			& DMA_START && timeout) {
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
static int nfc_send_cmd_readid( void )
{
	unsigned int regval;

    DBG_OUT("entry\n");

    writel(0, buf_base);

	nfc_write( NFC_NANDINFO_LEN, NFC_DATA_NUM);
	nfc_write( NAND_CMD_READID, NFC_CMD);
	nfc_write( 0, NFC_ADDRL);

	/* no need to config NFC_OP_WAIT_READY_EN, here not config. */
	regval = CMD1_EN
		 | ADDR_EN
		 | READ_DATA_EN
		 | (chipselect << NFC_OP_NF_CS_SHIFT)
		 | (1 << NFC_OP_ADDR_CYCLE_SHIFT);

	nfc_write( regval, NFC_OP);

	WAIT_CONTROLLER_FINISH();

    return readl(buf_base);
}

/*
* nfc_send_cmd_status - 
* @param 
* @return 
*/
static int nfc_send_cmd_status( void )
{
	unsigned int regval;

    DBG_OUT("entry\n");

    writel(0, buf_base);

	nfc_write( NFC_NANDINFO_LEN, NFC_DATA_NUM);
	nfc_write( NAND_CMD_STATUS, NFC_CMD);

	/* no need config NFC_OP_WAIT_READY_EN, here not config */
	regval = CMD1_EN
		    | READ_DATA_EN
		    | (chipselect << NFC_OP_NF_CS_SHIFT);

	nfc_write( regval, NFC_OP);

	WAIT_CONTROLLER_FINISH();

    return readl(buf_base);
}

/*
* nfc_send_cmd_reset - 
* @param 
* @return 
*/
static int nfc_send_cmd_reset( void )
{
	unsigned int regval;

    DBG_OUT("entry\n");
    
	nfc_write( NAND_CMD_RESET, NFC_CMD);

	/* need to config NFC_OP_WAIT_READY_EN */
	regval = CMD1_EN
		 | ((chipselect << NFC_OP_NF_CS_SHIFT)
		 | WAIT_READY_EN);

	nfc_write( regval, NFC_OP);

	WAIT_CONTROLLER_FINISH();

	return 0;
}

/*
* nfc_dev_init - 
* @param 
* @return 
*/
static int nfc_init( void )
{
    int regv;

    DBG_OUT("entry\n");

	nfc_controller_enable(1);

    reg_base = ioremap(NFC_REG_BASE_ADDRESS, NFC_REG_SIZE);
	if (!reg_base) {
		PR_BUG("ioremap failed\n");
		return -1;
	}

	buf_base = ioremap(NFC_BUFFER_BASE_ADDRESS, NFC_BUFFER_SIZE);
	if (!buf_base) {
		PR_BUG("ioremap failed\n");
		return -1;
	}

	buffer = dma_alloc_coherent(NULL, n_pagesize,
		&dma_buffer, GFP_KERNEL);
	if (!buffer) {
		PR_BUG("Can't malloc memory for NAND driver.");
		return -EIO;
	}

    DBG_OUT("reg_base:0x%p buf_base:0x%p buffer:0x%p dma_buffer:0x%x",
        reg_base, buf_base, buffer, dma_buffer);

    regv = nfc_read(NFC_CON);
    DBG_OUT("nfc_con:0x%08x\n",regv); 

	nfc_write(( OP_MODE_NORMAL << NFC_CON_OP_MODE_SHIFT ) 
            | ( n_pagesize << NFC_CON_PAGE_SIZE_SHIFT )
            | ( BUS_WIDTH_8BIT << NFC_CON_BUS_WIDTH_SHIFT )       
            | ( CS_CTRL_BUSY_0 << NFC_CON_CS_CTRL_SHIFT ) 
            | ( RB_SEL_EXCLUDED << NFC_CON_RB_SEL_SHIFT )
            | ( ECC_TYPE_NONE << NFC_CON_ECCTYPE_SHIFT ) 
            | ( RANDOMIZER_EN_0 << NFC_CON_RANDOMIZER_EN_SHIFT),
        NFC_CON);

	nfc_write(( w_lcnt << NFC_PWIDTH_W_LCNT_SHIFT )
            | ( r_lcnt << NFC_PWIDTH_R_LCNT_SHIFT ) 
            | ( rw_hcnt << NFC_PWIDTH_RW_HCNT_SHIFT ), 
		NFC_PWIDTH);

	nfc_write(( wait_ready_wait << NFC_OPIDLE_WAIT_READY_WAIT_SHIFT )
	        | ( cmd2_wait << NFC_OPIDLE_CMD2_WAIT_SHIFT )
	        | ( addr_wait << NFC_OPIDLE_ADDR_WAIT_SHIFT )
	        | ( cmd1_wait << NFC_OPIDLE_CMD1_WAIT_SHIFT )
	        | ( frb_wait << NFC_OPIDLE_FRB_WAIT_SHIFT ),
		NFC_OPIDLE);

    nfc_write( addr_l, NFC_ADDRL );

    nfc_write( addr_h, NFC_ADDRL );

    nfc_write( 0, NFC_DATA_NUM ); 

    /* for debug, diable all interrupts */
    nfc_write( 0, NFC_INTEN );

    /* No support LOCK */
    nfc_write( 0, NFC_LOCK );

#if 0
 /* what is this? */
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
#endif

	memset((char *)buf_base, 0xff, NFC_BUFFER_SIZE);

	return 0;
}

#ifdef CONFIG_PROC_FS
static int nfcdrv_proc_open(struct inode *inode, struct file *file);

static const struct file_operations nfcdrv_proc_fops = {
	.owner		= THIS_MODULE,
	.open		= nfcdrv_proc_open,
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
static int nfcdrv_proc_show(struct seq_file *seq, void *v)
{
	seq_printf(seq, "NFC REGISTER: name :addr:value (HEX)\n");

#define SEQ_OUT_4REG(reg1, reg2, reg3, reg4) \
    { \
	    seq_printf(seq, "%-17s:%02x:%08x  ", #reg1, reg1, nfc_read(reg1)); \
	    seq_printf(seq, "%-17s:%02x:%08x  ", #reg2, reg2, nfc_read(reg2)); \
	    seq_printf(seq, "%-17s:%02x:%08x  ", #reg3, reg3, nfc_read(reg3)); \
	    seq_printf(seq, "%-17s:%02x:%08x  ", #reg4, reg4, nfc_read(reg4)); \
	    seq_printf(seq, "\n"); \
    }

#define SEQ_OUT_3REG(reg1, reg2, reg3) \
    { \
	    seq_printf(seq, "%-17s:%02x:%08x  ", #reg1, reg1, nfc_read(reg1)); \
	    seq_printf(seq, "%-17s:%02x:%08x  ", #reg2, reg2, nfc_read(reg2)); \
	    seq_printf(seq, "%-17s:%02x:%08x  ", #reg3, reg3, nfc_read(reg3)); \
	    seq_printf(seq, "\n"); \
    }

    SEQ_OUT_4REG( NFC_CON,          NFC_PWIDTH,         NFC_OPIDLE,         NFC_CMD);
    SEQ_OUT_4REG( NFC_ADDRL,        NFC_ADDRH,          NFC_DATA_NUM,       NFC_OP);
    SEQ_OUT_4REG( NFC_STATUS,       NFC_INTEN,          NFC_INTS,           NFC_INTCLR);
    SEQ_OUT_4REG( NFC_LOCK,         NFC_LOCK_SA0,       NFC_LOCK_SA1,       NFC_LOCK_SA2);
    SEQ_OUT_4REG( NFC_LOCK_SA3,     NFC_LOCK_EA0,       NFC_LOCK_EA1,       NFC_LOCK_EA2);
    SEQ_OUT_4REG( NFC_LOCK_EA3,     NFC_EXPCMD,         NFC_EXBCMD,         NFC_ECC_TEST);
    SEQ_OUT_4REG( NFC_DMA_CTRL,     NFC_BADDR_D0,       NFC_BADDR_OOB,      NFC_DMA_LEN);
    SEQ_OUT_4REG( NFC_OP_PARA,      NFC_VERSION,        NFC_SEGMENT_ID,     NFC_FIFO_EMPTY);
    SEQ_OUT_4REG( NFC_BOOT_SET,     NFC_LP_CTRL,        NFC_ERR_NUM0_BUF0,  NFC_ERR_NUM1_BUF0);
    SEQ_OUT_4REG( NFC_ERR_NUM0_BUF1,NFC_ERR_NUM1_BUF1,  NFC_RB_MODE,        NFC_BADDR_D1);
    SEQ_OUT_3REG( NFC_BADDR_D2,     NFC_BADDR_D3,       NFC_MEM_CTRL);

#define SEQ_OUT_4V(v1, v2, v3, v4) \
    { \
	    seq_printf(seq, "%-17s:%08x  ", #v1, v1); \
	    seq_printf(seq, "%-17s:%08x  ", #v2, v2); \
	    seq_printf(seq, "%-17s:%08x  ", #v3, v3); \
	    seq_printf(seq, "%-17s:%08x  ", #v4, v4); \
	    seq_printf(seq, "\n"); \
    }

#define SEQ_OUT_1V( v1 ) \
    { \
	    seq_printf(seq, "%-17s:%08x  ", #v1, v1); \
	    seq_printf(seq, "\n"); \
    }


    seq_printf(seq, "\nCONFIGURE ITEMS: name :value (HEX)\n");

    SEQ_OUT_4V(n_pagesize,w_lcnt,r_lcnt,rw_hcnt);
    SEQ_OUT_4V(frb_wait, cmd1_wait, addr_wait, cmd2_wait); 
    SEQ_OUT_4V(wait_ready_wait, addr_l, addr_h, addr_cycle); 
    SEQ_OUT_1V(chipselect);

    return 0;
}

/*
* nfc_proc_open - driver's proc file operation "open" 
*
* @param inode:
* @param file:
*
*/
static int nfcdrv_proc_open(struct inode *inode, struct file *file)
{
	return single_open(file, nfcdrv_proc_show, NULL);
}
#endif 

/*
* nfc_open - driver's file operation "open" 
*
* @param inode:
* @param file:
*
*/
static int nfcdrv_open(struct inode* inode, struct file* file)
{
    DBG_OUT("entry\n");

    return 0 ;
}

/*
* nfc_close - driver's file operation "close" 
*
* @param inode:
* @param file:
*
*/
static int  nfcdrv_close(struct inode* inode, struct file* file)
{
    DBG_OUT("entry\n");

    return 0;
}

/*
* nfc_pagesize_i2v - map index to value 
*
* @param index:
*
*/
static int nfc_pagesize_i2v( int index )
{
    switch( index ){
        case PAGESIZE_2K: 
            return 2*1024; 
            break;
        case PAGESIZE_4K:
            return 4*1024; 
            break;
        case PAGESIZE_8K: 
            return 8*1024; 
            break;
        case PAGESIZE_16K:
            return 16*1024; 
            break;
        default:
            return 2*1024; 
            break;
    }

    return -1;
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
static int nfcdrv_write( NFC_IOC_WRITE_S * arg )
{
    int i;

    char * dst = buffer;
    char * src = (char * )arg->data;
    size_t size = arg->size + 4;
    int v_psize = nfc_pagesize_i2v( n_pagesize );


    DBG_OUT("entry\n");

    if( src == NULL ){
        DBG_OUT("usr_buffer NULL!\n");
        return -1;
    }

    if( arg->size == 0 ){
        DBG_OUT("count 0!\n");
        return -1;
    }

    if( size > MAX_FRAME_SIZE ){
        DBG_OUT("count exceed MAX_FRAME_SIZE(%d)!\n", MAX_FRAME_SIZE);
        return -1;
    }

    addr_l = arg->addr_l & 0xffff0000;
    addr_h = arg->addr_h;

    DBG_OUT("data:%p, size:0x%x, addr_l:0x%x, addr_h:0x%x\n",
        src, size, addr_l, addr_h);

	nfc_write( NAND_CMD_FRAMESTART, NFC_CMD);

	nfc_write( addr_h, NFC_ADDRH );

    DBG_OUT("src:0x%p size:0x%x\n", src, size);

    if ( size % v_psize == 0 ){

       nfc_write( addr_l, NFC_ADDRL );
        writel( arg->size, dst );
        memcpy( dst + 4, src, v_psize - 4); 

        nfc_dma_tx();

        DBG_OUT("ADDRL:0x%x src:0 size:0x%x\n",  addr_l, v_psize - 4);

        src += v_psize - 4;
        size -= v_psize;

        for ( i = 0; i < size/v_psize; i++ ){
            nfc_write( addr_l + v_psize * ( i + 1 ), NFC_ADDRL);
            memcpy( dst, src + v_psize * i, v_psize); 
            nfc_dma_tx();

            DBG_OUT("ADDRL:0x%x src:0x%x size:0x%x\n",  addr_l + v_psize * ( i + 1 ), 
                v_psize * i, v_psize);
        }
    }
    else{

        if ( size < v_psize ){
            nfc_write( addr_l, NFC_ADDRL );
            writel( arg->size, dst );
            memcpy( dst+4, src, arg->size); 
            nfc_dma_tx();
            DBG_OUT("ADDRL:0x%x src:0 size:0x%x\n",  addr_l, arg->size);
        }
        else{
            nfc_write( addr_l, NFC_ADDRL );
            writel( arg->size, dst );
            memcpy( dst+4, src, v_psize - 4); 
            nfc_dma_tx();
            DBG_OUT("ADDRL:0x%x src:0 size:0x%x\n",  addr_l, v_psize - 4);

            src += v_psize - 4;
            size -= v_psize;

            for ( i = 0; i < size/v_psize; i++ ){
                nfc_write( addr_l + v_psize * ( i + 1 ), NFC_ADDRL);
                memcpy( dst, src + v_psize * i, v_psize); 
                nfc_dma_tx();

                DBG_OUT("ADDRL:0x%x src:0x%x size:0x%x\n",  addr_l + v_psize * ( i + 1 ), 
                        v_psize * i, v_psize);
            }

            nfc_write( addr_l + v_psize * ( i + 1 ), NFC_ADDRL);
            memcpy( dst, src + v_psize * i, size%v_psize ); 
            nfc_dma_tx();

            DBG_OUT("ADDRL:0x%x src:0x%x size:0x%x\n",  addr_l + v_psize * ( i + 1 ), 
                    v_psize * i, size % v_psize );
        }
    }

	return (ssize_t)arg->size;
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
static long nfcdrv_ioctl(struct file* file, unsigned int cmd, unsigned long arg)
{
    int ret = 0;

    DBG_OUT("cmd:%x\n", cmd);

    switch (cmd) {
        case NFC_IOC_CMD_READID:
            memset(buf_base, 0, 0x10);
            command = NAND_CMD_READID;
            *((__user uint *)arg) = nfc_send_cmd_readid();
            break;

        case NFC_IOC_CMD_STATUS:
            command = NAND_CMD_STATUS;
            *((__user uint *)arg) = nfc_send_cmd_status();
            break;

        case NFC_IOC_CMD_RESET:
            command = NAND_CMD_RESET;
            ret = nfc_send_cmd_reset();
            break;

        case NFC_IOC_WRITE:
            ret = nfcdrv_write( (NFC_IOC_WRITE_S *)arg );
            break;

        default:
            break;
    }

    return ret;
}

static struct file_operations nfcdrv_fops =
{
    .owner      = THIS_MODULE,
    .open       = nfcdrv_open,
    .release    = nfcdrv_close,
    .unlocked_ioctl = nfcdrv_ioctl,
};

static struct miscdevice nfcdrv_dev =
{
    .minor   = MISC_DYNAMIC_MINOR,
    .name    = "nfc",
    .fops    = &nfcdrv_fops,
};

/*
* validate_configs - validate all configure items 
* @param 
* @return 
*/
static int __init validate_configs ( void )
{
    DBG_OUT("entry\n");

    /* register NFC_CON */

    if ( n_pagesize != PAGESIZE_2K && n_pagesize != PAGESIZE_4K && 
         n_pagesize != PAGESIZE_8K && n_pagesize != PAGESIZE_16K ){
		PR_BUG("invalid n_pagesize: 0x%x, set to PAGESIZE_2K:0x%x\n", 
            n_pagesize, PAGESIZE_2K);
	    n_pagesize = PAGESIZE_2K;
    }

    /* register NFC_PWIDTH */

	if (w_lcnt < MIN_W_LCNT || w_lcnt > MAX_W_LCNT ){
		PR_BUG("invalid w_lcnt: 0x%x, set to 0x%x\n", w_lcnt, DEFAULT_W_LCNT);
	    w_lcnt = DEFAULT_W_LCNT;
    }

	if (r_lcnt < MIN_R_LCNT || r_lcnt > MAX_R_LCNT ){
		PR_BUG("invalid r_lcnt: 0x%x, set to 0x%x\n", r_lcnt, DEFAULT_R_LCNT);
	    r_lcnt = DEFAULT_R_LCNT;
    }

	if (rw_hcnt < MIN_RW_HCNT || r_lcnt > MAX_RW_HCNT ){
		PR_BUG("invalid rw_hcnt: 0x%x, set to 0x%x\n", rw_hcnt, DEFAULT_RW_HCNT);
	    rw_hcnt = DEFAULT_RW_HCNT;
    }

    DBG_OUT("n_pagesize:0x%02x w_lcnt:0x%02x r_lcnt:0x%02x rw_hcnt:0x%02x\n",
        n_pagesize,w_lcnt,r_lcnt,rw_hcnt);

    /* register NFC_OPIDLE */

	if (frb_wait < MIN_FRB_WAIT || frb_wait > MAX_FRB_WAIT ){
		PR_BUG("invalid frb_wait: 0x%x, set to 0x%x\n", frb_wait, DEFAULT_FRB_WAIT);
	    frb_wait = DEFAULT_FRB_WAIT;
    }

	if (cmd1_wait < MIN_CMD1_WAIT || cmd1_wait > MAX_CMD1_WAIT ){
		PR_BUG("invalid cmd1_wait: 0x%x, set to 0x%x\n", cmd1_wait, DEFAULT_CMD1_WAIT);
	    cmd1_wait = DEFAULT_CMD1_WAIT;
    }

	if (addr_wait < MIN_ADDR_WAIT || addr_wait > MAX_ADDR_WAIT ){
		PR_BUG("invalid addr_wait: 0x%x, set to 0x%x\n", addr_wait, DEFAULT_ADDR_WAIT);
	    addr_wait = DEFAULT_ADDR_WAIT;
    }

	if (cmd2_wait < MIN_CMD2_WAIT || cmd2_wait > MAX_CMD2_WAIT ){
		PR_BUG("invalid cmd2_wait: 0x%x, set to 0x%x\n", cmd2_wait, DEFAULT_CMD2_WAIT);
	    cmd2_wait = DEFAULT_CMD2_WAIT;
    }

	if (wait_ready_wait < MIN_WAIT_READY_WAIT 
        || wait_ready_wait > MAX_WAIT_READY_WAIT ){
		PR_BUG("invalid wait_ready_wait: 0x%x, set to 0x%x\n", wait_ready_wait, 
            DEFAULT_WAIT_READY_WAIT);
	    wait_ready_wait = DEFAULT_WAIT_READY_WAIT;
    }

    DBG_OUT("frb_wait:0x%02x cmd1_wait:0x%02x addr_wait:0x%02x cmd2_wait:0x%02x wait_ready_wait:0x%02x\n",
        frb_wait, cmd1_wait, addr_wait, cmd2_wait, wait_ready_wait);

    /* register NFC_ADDRL */
	if ((addr_l & 0xffff) > 0 ){
		PR_BUG("invalid addr_hy: 0x%x, set to 0x%x\n", addr_l, DEFAULT_ADDRL);
        addr_l = DEFAULT_ADDRL;
    } 

    /* register NFC_ADDRH */

	if ((addr_h & ~NFC_ADDRH_ADDR_H_MASK) > 0 ){
		PR_BUG("invalid addr_hy: 0x%x, set to 0x%x\n", cmd2_wait, DEFAULT_ADDRH);
        addr_h = DEFAULT_ADDRH;
    } 

    /* register NFC_OP */

	if ( addr_cycle != 4 && addr_cycle != 5 ){
		PR_BUG("invalid addr_cycle: 0x%x, set to 4\n", addr_cycle);
	    addr_cycle = 4;
    }

	if (chipselect > NFC_MAX_CHIP){
		PR_BUG("invalid chipselect: 0x%x, set to 0\n", chipselect);
	    chipselect = 0;
    }

    DBG_OUT("addr_l:0x%x addr_h:0x%x addr_cycle:0x%x chipselect:0x%x\n", 
        addr_l, addr_h, addr_cycle, chipselect );

    return 0;
}


/*
* nfc_init - module init and exit 
* @param 
* @return 
*/
static int __init nfcdrv_init(void)
{
#ifdef CONFIG_PROC_FS
	struct proc_dir_entry *ent;
#endif
    int     ret;

    DBG_OUT("entry\n");

    validate_configs ();

    nfc_init();

#ifdef CONFIG_PROC_FS
/*
 *	Info exported via "/proc/driver/nfc".
 */
	ent = proc_create("driver/nfc", 0, NULL, &nfcdrv_proc_fops);
	if (!ent) {
        DBG_OUT(KERN_WARNING "rtc: Failed to register with procfs.\n");
    }
#endif

    ret = misc_register(&nfcdrv_dev);
    if (ret != 0) {
        DBG_OUT("register i2c device failed with %#x!\n", ret);
        return -1;
    }

    return 0;
}

/*
* nfc_exit - 
* @param 
* @return 
*/
static void __exit nfcdrv_exit(void)
{
    DBG_OUT("entry\n");

#ifdef CONFIG_PROC_FS
	remove_proc_entry("driver/nfc", NULL);
#endif

    misc_deregister(&nfcdrv_dev);
    nfc_controller_enable(0);
}

module_init(nfcdrv_init);
module_exit(nfcdrv_exit);

MODULE_DESCRIPTION("nfc Driver");
MODULE_LICENSE("GPL");
MODULE_AUTHOR("www.vhd.com.cn");
