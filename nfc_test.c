#include <stdio.h>
#include <ctype.h>
#include <sys/ioctl.h>
#include <sys/types.h>
#include <sys/time.h>
#include <sys/stat.h>
#include <fcntl.h>
#include <unistd.h>


#include "msgout.h"
#include "nfc_usr.h"

#define BUFFER_SIZE 1024*1024
char buffer[BUFFER_SIZE];

NFC_IOC_WRITE_S ioc_write_arg; 

int main(int argc , char* argv[])
{
    int fd = -1;
    int ret = 0;
    int req;


    fd = open("/dev/nfc", O_RDWR);
    if (fd < 0) {
        perror("Open nfc fail:");
        return -1;
    }

    ioc_write_arg.data = buffer;
    ioc_write_arg.size = 2048 - 0x14; 

    /* About addr_l and addr_h: */
    /* According to the manual, the low 16bits of addr_l are for 
       inside page address,so, pay attention when giving values 
       of both variables */
    ioc_write_arg.addr_l = 0x10000; 
    ioc_write_arg.addr_h = 0; 

    req = NFC_IOC_WRITE;
    ret = ioctl( fd, req, &ioc_write_arg); 
    dbgout("ioctl write size:0x%x ret:0x%x\n", 
        ioc_write_arg.size, ret);

#if 0 
    req = NFC_IOC_CMD_RESET;
    ioctl( fd, req, NULL ); 
    dbgout("ioctl req:0x%x\n", req );
#endif

#if 0 
    req = NFC_IOC_CMD_READID;
    ioctl( fd, req, &ret ); 
    dbgout("ioctl req:0x%x ret:0x%x\n", req, ret);
#endif

#if 0 
    req = NFC_IOC_CMD_STATUS;
    ioctl( fd, req, &ret ); 
    dbgout("ioctl req:0x%x ret:0x%x\n", req, ret);
#endif

    close(fd);

    return 0;
}
