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


int main(int argc , char* argv[])
{
    int fd = -1;
    int ret = 0;
    int size;
    int req;


    fd = open("/dev/nfc", 0);
    if (fd < 0) {
        perror("Open nfc fail:");
        return -1;
    }

#if 0
    size = 1024;
    ret = write( fd, buffer, size);
    dbgout("write size:0x%x ret:0x%x\n", size, ret);
    size = 1024 + 0x33;
    ret = write( fd, buffer, size);
    dbgout("write size:0x%x ret:0x%x\n", size, ret);

    size = 2048;
    ret = write( fd, buffer, size);
    dbgout("write size:0x%x ret:0x%x\n", size, ret);
    size = 2048 + 0x55;
    ret = write( fd, buffer, size);
    dbgout("write size:0x%x ret:0x%x\n", size, ret);

    size = 4096;
    ret = write( fd, buffer, size);
    dbgout("write size:0x%x ret:0x%x\n", size, ret);
    size = 4096 + 0x77;
    ret = write( fd, buffer, size);
    dbgout("write size:0x%x ret:0x%x\n", size, ret);
 
    size = 8192;
    ret = write( fd, buffer, size);
    dbgout("write size:0x%x ret:0x%x\n", size, ret);
    size = 8192 + 0x99;
    ret = write( fd, buffer, size);
    dbgout("write size:0x%x ret:0x%x\n", size, ret);

    size = 1024*128;
    ret = write( fd, buffer, size);
    dbgout("write size:0x%x ret:0x%x\n", size, ret);
    size = 1024*128 + 0x111;
    ret = write( fd, buffer, size);
    dbgout("write size:0x%x ret:0x%x\n", size, ret);

    size = 1024*512;
    ret = write( fd, buffer, size);
    dbgout("write size:0x%x ret:0x%x\n", size, ret);
    size = 1024*512 + 0x333;
    ret = write( fd, buffer, size);
    dbgout("write size:0x%x ret:0x%x\n", size, ret);

    size = 1024*1024;
    ret = write( fd, buffer, size);
    dbgout("write size:0x%x ret:0x%x\n", size, ret);
    size = 1024*1024 - 0x555;
    ret = write( fd, buffer, size);
    dbgout("write size:0x%x ret:0x%x\n", size, ret);
#endif

    req = NFC_IOC_CMD_RESET;
    ioctl( fd, req, NULL ); 
    dbgout("ioctl req:0x%x\n", req );

    req = NFC_IOC_CMD_READID;
    ioctl( fd, req, &ret ); 
    dbgout("ioctl req:0x%x ret:0x%x\n", req, ret);

    req = NFC_IOC_CMD_STATUS;
    ioctl( fd, req, &ret ); 
    dbgout("ioctl req:0x%x ret:0x%x\n", req, ret);

    close(fd);

    return 0;
}
