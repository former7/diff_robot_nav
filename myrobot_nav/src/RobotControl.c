/**
  ******************************************************************************
  * @file       RobotControl.c
  * @author     Giant Song
  * @version    V1.0
  * @date       2016-09-01
  * @brief      机器人行走控制
  * @attention  无
  ******************************************************************************
  */

#include "serial.h"
#include "RobotControl.h"
#include <pthread.h>
#include <stdio.h>
#include <string.h>
/*为了引进sleep函数*/
#include <unistd.h>

#define PacketHead "Head"  //下位机发送协议头
#define PacketTail "Tail"  //下位机发送协议尾

struct SendMessage SendCmd;    //发送命令协议结构体
struct RecvMessage RecvCmd;    //接受命令协议结构体
char RecvBuff[512];            //最多存储下位机发送来的40条机器人状态，根据下位机情况更改
RobotInfo robotInfo;

/**
 *@brief     机器人以速度Speed直向行走，可正可负
 *@attention 速度精确到小数点后一位
 */
void MoveStraight(float Speed)
{
    char tempChar = 0;
    tempChar = (int) (Speed * 10);
    SendCmd.RValue = tempChar;
    SendCmd.LValue = tempChar;
    uart_send((char *)(&SendCmd),sizeof(SendCmd));
}

/**
 *@brief     机器人以速度LSpeed左转弯，为正
 *@attention 速度精确到小数点后一位
 */
void TurnLeft(float LSpeed)
{
    char tempChar = 0;
    tempChar = (int) (LSpeed * 10);
    SendCmd.RValue = tempChar;
    SendCmd.LValue = -tempChar;
    uart_send((char *)(&SendCmd),sizeof(SendCmd));
}

/**
 *@brief     机器人以速度RSpeed右转弯，为正
 *@attention 速度精确到小数点后一位
 */
void TurnRight(float RSpeed)
{
    char tempChar = 0;
    tempChar = (int) (RSpeed * 10);
    SendCmd.RValue = -tempChar;
    SendCmd.LValue = tempChar;
    uart_send((char *)(&SendCmd),sizeof(SendCmd));
}

/**
 *@brief     机器人以速度RSpeed右转弯，为正
 *@attention 速度精确到小数点后一位
 */
void MoveRobot(float RSpeed, float LSpeed)
{
    /*if(RSpeed == 0 && LSpeed == 0){
      SendCmd.prio = (unsigned char)(0x01);
    }
    else{
      SendCmd.prio = (unsigned char)(0x02);
    }*/
    char RtempChar = 0,LtempChar = 0;
    RtempChar = (int) (RSpeed * 100);
    LtempChar = (int) (LSpeed * 100);
    SendCmd.RValue = RtempChar;
    SendCmd.LValue = LtempChar;
    uart_send((char *)(&SendCmd),sizeof(SendCmd));
    usleep(10000);
}

void strcopy( char* pRecent )
{
    RecvCmd.battery      = pRecent[0];
    RecvCmd.RSpeed       = pRecent[1];
    RecvCmd.LSpeed       = pRecent[2];
    RecvCmd.RHighMileage = pRecent[3];
    RecvCmd.RLowMileage  = pRecent[4];
    RecvCmd.LHighMileage = pRecent[5];
    RecvCmd.LLowMileage  = pRecent[6];
}
/**
 *@brief     接受线程函数
 *@attention 将最新数据存储到RecvCmd中
 */
void ReceiveMessage(void)
{
    int length = 0,len = 0;
    char * pRecentData;
    int HeadLen = sizeof(PacketHead) - 1;
    int TailLen = sizeof(PacketTail) - 1;
    int WholeLen = sizeof(RecvCmd) + HeadLen + TailLen;
    while(1)
    {
        len = uart_recv( (char *)RecvBuff,sizeof(RecvBuff) ) ;
        length = len / WholeLen; //length:每次接受的数据包个数;len:每次接受的字节数
//        printf("len = %d,length = %d\n",len,length);
        if(length >= 1 )
        {
            pRecentData =(char *) &RecvBuff[  sizeof(RecvBuff) * (length -1)];
            if( (strncmp(pRecentData, PacketHead, HeadLen) == 0)
                    && (strncmp( (char*)(pRecentData + WholeLen - TailLen), PacketTail,TailLen)) == 0 )
            {
                strcopy((char*)(pRecentData+HeadLen));
                memset(RecvBuff, '0', sizeof(RecvBuff));
//          printf("recv:%x,%x,%x,%x,%x,%x,%x\n",RecvCmd.battery,RecvCmd.RSpeed,RecvCmd.LSpeed,RecvCmd.RHighMileage,RecvCmd.RLowMileage,RecvCmd.LHighMileage,RecvCmd.LLowMileage);
            }
        }
//    sleep(1);
    }
}

/**
 *@brief     返回机器人的状态信息结构体
 *@attention 机器人状态可能是及时数据，取决于下位机
 */
RobotInfo GetRobotInfo(void)
{

    int length = 0,len = 0;
    char * pRecentData;
    int HeadLen = sizeof(PacketHead) - 1;
    int TailLen = sizeof(PacketTail) - 1;
    int WholeLen = sizeof(RecvCmd) + HeadLen + TailLen;
    len = uart_recv( (char *)RecvBuff,sizeof(RecvBuff) ) ;
    length = len / WholeLen;
    if(length >= 1 )
    {

        pRecentData =(char *) &RecvBuff[  sizeof(RecvBuff) * (length -1)];
        if( (strncmp(pRecentData, PacketHead, HeadLen) == 0)
                && (strncmp( (char*)(pRecentData + WholeLen - TailLen), PacketTail,TailLen)) == 0 )
        {
            strcopy((char*)(pRecentData+HeadLen));
            memset(RecvBuff, '0', sizeof(RecvBuff));
            //   printf("recv:%x,%x,%x,%x,%x,%x,%x\n",RecvCmd.battery,RecvCmd.RSpeed,RecvCmd.LSpeed,RecvCmd.RHighMileage,RecvCmd.RLowMileage,RecvCmd.LHighMileage,RecvCmd.LLowMileage);
        }
    }
    robotInfo.battery  = RecvCmd.battery;
    robotInfo.LSpeed   = RecvCmd.LSpeed / 10.0;
    robotInfo.RSpeed   = RecvCmd.RSpeed / 10.0;
    robotInfo.Rmileage = (signed char)RecvCmd.RHighMileage + RecvCmd.RLowMileage / 100.0;
    robotInfo.Lmileage = (signed char)RecvCmd.LHighMileage + RecvCmd.LLowMileage / 100.0;
    return robotInfo;
}

/**
 *@brief     初始化串口与发送数据协议的结构体
 *@attention
 */
void RobotControlInit(void)
{
    // pthread_t receive_id;                 //接受线程id
    init_serial();
    SendCmd.head1  = (unsigned char)(0x48);
    SendCmd.head2  = (unsigned char)(0x65);
    SendCmd.head3  = (unsigned char)(0x61);
    SendCmd.head4  = (unsigned char)(0x64);
    SendCmd.type   = (unsigned char)(0x53);
    SendCmd.id     = (unsigned char)(0x00);
    SendCmd.prio   = (unsigned char)(0x01);
    SendCmd.extra  = (char)(0x00);
    SendCmd.RValue = (char)(0x00);
    SendCmd.LValue = (char)(0x00);
    SendCmd.tail1  = (unsigned char)(0x54);
    SendCmd.tail2  = (unsigned char)(0x61);
    SendCmd.tail3  = (unsigned char)(0x69);
    SendCmd.tail4  = (unsigned char)(0x6c);
    // pthread_create(&receive_id,NULL,(void*)ReceiveMessage,NULL);
}
/*
int main(){
    RobotControlInit();
    RobotInfo robot;
    while(1){
       robot = GetRobotInfo();
       MoveStraight(0.2);
       MoveRobot(0.2,-0.2);
       printf("%f,%f\n",robot.Rmileage,robot.Lmileage);
       sleep(1);
    }
    return 0 ;

}
*/
/************************End********************************/

