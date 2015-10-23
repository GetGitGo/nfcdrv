#include <stdio.h>
#include <ctype.h>
#include <sys/ioctl.h>
#include <sys/types.h>
#include <sys/stat.h>
#include <fcntl.h>
#include <unistd.h>


#define BUFFER_SIZE 2048
char buffer[BUFFER_SIZE];


int main(int argc , char* argv[])
{
    int fd = -1;
    int ret = 0;


    fd = open("/dev/nfc", 0);
    if (fd < 0) {
        perror("Open nfc fail:");
        return -1;
    }

    ret = write( fd, buffer, BUFFER_SIZE );
    printf("write %d bytes, return %d", BUFFER_SIZE, ret );
 
    close(fd);

    return 0;
}
