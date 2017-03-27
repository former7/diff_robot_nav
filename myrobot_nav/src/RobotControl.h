#ifndef ROBOTCONTROL_H_INCLUDED
#define ROBOTCONTROL_H_INCLUDED

/**发送命令数据协议结构体（下位机与上位机通信标准）*/
struct SendMessage{
    unsigned char head1;  //48:H
    unsigned char head2;  //65:e
    unsigned char head3;  //61:a
    unsigned char head4;  //64:d
    unsigned char type;   //s or g(set or get)
    unsigned char id;     //命令类型
    unsigned char prio;   //命令优先级
    char extra;           //保留数据
    char RValue;          //传输的主要控制数据：右电机
    char LValue;          //传输的主要控制数据：左电机
    unsigned char tail1;  //54：T
    unsigned char tail2;  //61：a
    unsigned char tail3;  //69:i
    unsigned char tail4;  //6c:l
};
/**接受数据协议结构体（下位机与上位机通信标准）,没有包括头和尾*/
struct RecvMessage{
    char battery;         //电池电量
    char RSpeed;          //右电机速度，除以10为实际速度
    char LSpeed;          //左电机速度，除以10为实际速度
    char RHighMileage;
    char RLowMileage;
    char LHighMileage;
    char LLowMileage;
};

/**机器人状态信息：速度、电量、里程*/
 typedef struct RobotStatus{
    unsigned char battery;
    float RSpeed;
    float LSpeed;
    float Rmileage;
    float Lmileage;
}RobotInfo;

extern void RobotControlInit(void);
extern void MoveStraight(float Speed);
extern void MoveRobot(float RSpeed, float LSpeed);
extern void TurnLeft(float LSpeed);
extern void TurnRight(float RSpeed);
extern RobotInfo GetRobotInfo(void);
#endif // ROBOTCONTROL_H_INCLUDED

/************************End********************************/

