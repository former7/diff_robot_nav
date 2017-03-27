/**
  ******************************************************************************
  * @file       serial.c
  * @author     Giant Song
  * @version    V1.0
  * @date       2016-09-01
  * @brief      串口初始化、发送和接受
  * @attention  无
  ******************************************************************************
  */

#include <stdio.h>
#include <stdlib.h>
#include <unistd.h>
#include <sys/types.h>
#include <sys/stat.h>
#include <fcntl.h>   //文件控制定义
#include <termios.h> //终端控制定义
#include <errno.h>


#include "serial.h"

int serial_fd = 0;   //USB转串口的设备号

//打开串口并初始化设置
int init_serial(void){
    serial_fd = open(DEVICE, O_RDWR | O_NOCTTY | O_NDELAY);//可读写、非阻塞
    printf("try to open fd:%d\n",serial_fd);
    if (serial_fd < 0){
        perror("open serial failed ");
        return -1;
    }
    struct termios options; //串口主要设置结构体termios <termios.h>
    /**
     *tcgetattr函数用于获取与终端相关的参数。
     *参数fd为终端的文件描述符，返回的结果保存在termios结构体中
     */
    tcgetattr(serial_fd, &options);
    /**2. 修改所获得的参数*/
    options.c_cflag |= (CLOCAL | CREAD);//设置控制模式状态，本地连接，接收使能
    options.c_cflag &= ~CSIZE;          //字符长度，设置数据位之前一定要屏掉这个位
    options.c_cflag &= ~CRTSCTS;        //无硬件流控
    options.c_cflag |= CS8;             //8位数据长度
    options.c_cflag &= ~CSTOPB;         //1位停止位
    options.c_iflag |= IGNPAR;          //无奇偶检验位
    options.c_oflag = 0;                //输出模式
    options.c_lflag = 0;                //不激活终端模式
    cfsetospeed(&options, B115200);     //设置输出波特率
    cfsetispeed(&options, B115200);     //设置输入波特率
    /**3. 设置新属性，TCSANOW：所有改变立即生效*/
    tcflush(serial_fd, TCIFLUSH);       //溢出数据可以接收，但不读
    tcsetattr(serial_fd, TCSANOW, &options);
    return 0;
}

/**
 *@brief   串口发送数据
 *@fd      串口描述符
 *@data    待发送数据
 *@datalen 数据长度
 */
int uart_send(char *data, int datalen){
    int len = 0;
    len = write(serial_fd, data, datalen);//实际写入的长度
    if(len == datalen){
        return len;
    }
    else{
        tcflush(serial_fd, TCOFLUSH);    //TCOFLUSH刷新写入的数据但不传送
        return -1;
    }
}

/**
 *@brief 串口接收数据
 */
int uart_recv(char *data, int datalen){
    int len=0, ret = 0;
    fd_set fs_read;
    struct timeval tv_timeout;

    FD_ZERO(&fs_read);    //将fs_read集合清零，使之不包含任何fd
    FD_SET(serial_fd,&fs_read);  //将fd加入集合fs_read
    tv_timeout.tv_sec  = (2);
    tv_timeout.tv_usec = 0;

    /*做好准备的文件描述符的个数，超时为0，错误为 -1*/
    ret = select(serial_fd+1, &fs_read, NULL, NULL, &tv_timeout);//测试指定的fd可读？可写？有异常条件待处理？
//    printf("ready文件个数 = %d\n", ret);
    //如果返回0，代表在描述符状态改变前已超过timeout时间,错误返回-1

    if (FD_ISSET(serial_fd, &fs_read)) {
        len = read(serial_fd, data, datalen);
        return len;
    }
    else {
        perror("the file can not read");
        return -1;
    }

    return 0;
}

/************************End********************************/
