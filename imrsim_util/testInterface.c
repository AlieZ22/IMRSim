#include <stdio.h>
#include <stdlib.h>
#include <sys/types.h>
#include <fcntl.h>
#include <unistd.h>
#include <string.h>
#include <sys/ioctl.h>
#include <linux/types.h>

#define   u8       __u8
#define   u32      __u32
#define   u64      __u64
#define   sector_t __u64

#include "imrsim_types.h"
#include "imrsim_ioctl.h"

int main(int argc, char *argv[])
{
    int fd;
    char ops;
    int len;
    char *buffer;

    fd = open(argv[1], O_RDWR|O_APPEND);
    if(fd == -1){
        perror("open file failed.\n");
    }
    ops = argv[2][0];
    len = atoi(argv[3]);
    if(ops == 'r'){
        buffer = (char*)malloc(len*sizeof(char));
        if(read(fd, buffer, len) != -1){
            printf("READ: %s\n",buffer);
        }
        free(buffer);
    }else if(ops == 'w'){
        buffer = argv[4];
        if(write(fd, buffer,len) != -1){
            printf("WRITE: success!\n");
        }
    }else{
        printf("ops field is not correct.\n");
    }

    close(fd);
    return 0;
}