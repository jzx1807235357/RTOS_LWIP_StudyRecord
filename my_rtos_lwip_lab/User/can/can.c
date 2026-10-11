/*显性 Dominant 隐形Recessive
*Dominant 0 电压 CANH=3.5V CANL=1.5V  
*Recessive 1 CANH=2.5V CANL=2.5V
*wired_and  仲裁时ID更小（更多显性位） 空闲总线为隐形

CAN规则：只有总线空闲时才能开发
空闲，IFS之后一直是隐形可以打出SOF，但已有人在发则停止
同一条CAN总线 所有节点波特率必须一致 否则收发对不齐
经典高速CAN波特率：
1Mbps
500k 250k 125k 
*标准数据帧
[SOF][11bit ID][RTR][IDE][r0][DLC][0~8字节数据][CRC][ACK][EOF]
*SOF Start of Frame 帧起始 1bit 固定为显性
*[RTR]Remote Transmission Request 远程发送请求 1bit 显性0 数据帧（0~8字节数据） 隐形1 远程帧（其他发）
*[IDE]Identifier Extension 1bit  显性0 标准帧 11bit ID 隐形1 扩展帧 29位ID
*[DLC]Data Length Code 数据长度码 4bit 0~8byte 实际的payload长度
*[0~8byte]payload
*[CRC]循环冗余校验15bit CRC序列+CRC界定符1bit隐形，从SOF到DATA算出每一个校验值，挂载数据后面，接收方再用同样的方法算一遍
STM32 CAN MAC自动算 自动检 
*[ACK] 应答Field 2bit (ACK Slot 1bit （应答槽）+1bit ACk Delimiter) 
自己发 ACK Slot隐形1留坑，CRC校验通过的节点打显性0表示收到，如果没有ACK显性 则说明发松失败会重发
ACK Delimiter 隐形1 
*[EOF]End of Frame  7bit 全部为隐性，CAN有填充位 如果数据区连续5个相同位会差一个相反位，EOF属于不填充区域
所以连续7个隐形表示结束

连续帧
IFS 帧间隔 InterFrame Space 至少3个隐性位 之后才是总线空闲


Arbitration 
ID越小，优先级越高，无中央裁判，节点自己比ID RTR=0显性数据帧比RTR=1远程帧优先级高

Error Frame 错误帧
Bit Error 位错误：发出去和监听到不一致
Stuff Error 填充错误：该填充的地方规则不对，连续6个相同等位
CRC Error 校验对不上
Form Error 格式错误 固定是隐形的位却是显性 比如ACK Delimiter EOF
ACK Error 应答错误 发送方在ACK槽口未看到任何设备打显性应答
*/
#include "stm32f4xx.h"
#include "stm32f4xx_can.h"

void Can1_Init(void)
{
    //结构体
    GPIO_InitTypeDef GPIO_InitStructure; 
    CAN_InitTypeDef CAN_InitStructure;
    CAN_FilterInitTypeDef CAN_FilterInitStructure;

    //时钟
    RCC_AHB1PeriphClockCmd(RCC_AHB1Periph_GPIOB,ENABLE);
    RCC_APB1PeriphClockCmd(RCC_APB1Periph_CAN1,ENABLE);

    //Gpio Config
    GPIO_InitStructure.GPIO_Mode=GPIO_Mode_AF;//引脚复用
    GPIO_InitStructure.GPIO_OType=GPIO_OType_PP;
    GPIO_InitStructure.GPIO_PuPd=GPIO_PuPd_UP;
    GPIO_InitStructure.GPIO_Speed=GPIO_Speed_50MHz;
    GPIO_InitStructure.GPIO_Pin=GPIO_Pin_8|GPIO_Pin_9;
    GPIO_Init(GPIOB,&GPIO_InitStructure);

    GPIO_PinAFConfig(GPIOB,GPIO_PinSource8,GPIO_AF_CAN1);//PB8 CAN1_RX
    GPIO_PinAFConfig(GPIOB,GPIO_PinSource9,GPIO_AF_CAN1);//PB9 CAN1_Tx

    CAN_InitStructure.CAN_TTCM=DISABLE;//时间触发关，Time Triggerd Communication Mode：开则按软件时刻表关有数据就发
    CAN_InitStructure.CAN_ABOM=DISABLE;//Automatic BUS-OFF Management错误太多会Bus-OFF ENABLE会自己尝试恢复否则需要自行处理 
    CAN_InitStructure.CAN_AWUM=DISABLE;//Automatic WakeUp Mode,CAN可以进入睡眠模式 使能如果总线出现活动就自己从睡眠中醒来，否则需要软件主动唤醒
    CAN_InitStructure.CAN_NART=DISABLE;//NART=No Automatic Retransmission，如无ACK或总线错误，使能则失败只重发一次 失能则一直重发
    CAN_InitStructure.CAN_RFLM=DISABLE;//Receive FIFO Loced Mode DISABLE新的盖掉旧的（要最新数据）ENABLE锁住，不收新的（保住还没读的旧帧）
    
    //STM32 CAN有3个MailBox，每一个MailBox都是一个完整的CAN报文，软件调用发送则硬件找一个void MailBox Place 
    CAN_InitStructure.CAN_TXFP=DISABLE;//Transmit FIFO Priority;关=三个MailBox ID小优先发，否则谁先提交谁先发
    CAN_InitStructure.CAN_Mode=CAN_Mode_LoopBack;//CAN_Mode_Normal需要ACK CAN_Mode_LoopBack：发出去内部收不需要ACK CAN_Mode_Silent 只听不发
    CAN_InitStructure.CAN_SJW=CAN_SJW_1tq;//Synchronization Jump Width Time ，tq=Time Quantum
    CAN_InitStructure.CAN_BS1=CAN_BS1_11tq;//Bit Segment 1=11tq
    CAN_InitStructure.CAN_BS2=CAN_BS2_3tq;//Bit Segment 2=3tq
    CAN_InitStructure.CAN_Prescaler=6;//45M/(6*(1+11+3))=500Kbps
    CAN_Init(CAN1,&CAN_InitStructure);//

    //CAN Filter 总线上帧很多 关心的进FIFO 不关心的丢掉
    CAN_FilterInitStructure.CAN_FilterNumber=0;//过滤器编号
    CAN_FilterInitStructure.CAN_FilterMode=CAN_FilterMode_IdMask;//标识符+掩码模式
    CAN_FilterInitStructure.CAN_FilterScale=CAN_FilterScale_32bit;//一个过滤器32bit
    CAN_FilterInitStructure.CAN_FilterIdHigh=0x0000;//要匹配的ID高16
    CAN_FilterInitStructure.CAN_FilterIdLow=0x0000;//要匹配的ID低16
    CAN_FilterInitStructure.CAN_FilterMaskIdHigh=0x0000;//掩码高16
    CAN_FilterInitStructure.CAN_FilterMaskIdLow=0x0000;//掩码低16bit
    CAN_FilterInitStructure.CAN_FilterFIFOAssignment=CAN_Filter_FIFO0;//通过的进FIFO0
    CAN_FilterInitStructure.CAN_FilterActivation=ENABLE;//打开这个过滤器
    CAN_FilterInit(&CAN_FilterInitStructure);
}