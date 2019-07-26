/**
  ******************************************************************************
  * File Name          : CAN.c
  * Description        : This file provides code for the configuration
  *                      of the CAN instances.
  ******************************************************************************
  ** This notice applies to any and all portions of this file
  * that are not between comment pairs USER CODE BEGIN and
  * USER CODE END. Other portions of this file, whether 
  * inserted by the user or by software development tools
  * are owned by their respective copyright owners.
  *
  * COPYRIGHT(c) 2019 STMicroelectronics
  *
  * Redistribution and use in source and binary forms, with or without modification,
  * are permitted provided that the following conditions are met:
  *   1. Redistributions of source code must retain the above copyright notice,
  *      this list of conditions and the following disclaimer.
  *   2. Redistributions in binary form must reproduce the above copyright notice,
  *      this list of conditions and the following disclaimer in the documentation
  *      and/or other materials provided with the distribution.
  *   3. Neither the name of STMicroelectronics nor the names of its contributors
  *      may be used to endorse or promote products derived from this software
  *      without specific prior written permission.
  *
  * THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS"
  * AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE
  * IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE ARE
  * DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT HOLDER OR CONTRIBUTORS BE LIABLE
  * FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL
  * DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR
  * SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER
  * CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY,
  * OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE
  * OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
  *
  ******************************************************************************
  */

/* Includes ------------------------------------------------------------------*/
#include "can.h"

#include "gpio.h"

/* USER CODE BEGIN 0 */
#include "stdlib.h"
#include <math.h>
#include "can.h"
#include "tim.h"


float Vy=10,Vx=10,angularVell=10;//设置一下基础速度


typedef struct
{
    float v1;
    float v2;
    float v3;
}ActThreeVell;

/* USER CODE END 0 */

CAN_HandleTypeDef hcan;

/* CAN init function */
void MX_CAN_Init(void)
{

  hcan.Instance = CAN1;
  hcan.Init.Prescaler = 3;
  hcan.Init.Mode = CAN_MODE_NORMAL;
  hcan.Init.SyncJumpWidth = CAN_SJW_1TQ;
  hcan.Init.TimeSeg1 = CAN_BS1_9TQ;
  hcan.Init.TimeSeg2 = CAN_BS2_2TQ;
  hcan.Init.TimeTriggeredMode = DISABLE;
  hcan.Init.AutoBusOff = ENABLE;
  hcan.Init.AutoWakeUp = DISABLE;
  hcan.Init.AutoRetransmission = DISABLE;
  hcan.Init.ReceiveFifoLocked = DISABLE;
  hcan.Init.TransmitFifoPriority = ENABLE;
  if (HAL_CAN_Init(&hcan) != HAL_OK)
  {
    _Error_Handler(__FILE__, __LINE__);
  }

}

void HAL_CAN_MspInit(CAN_HandleTypeDef* canHandle)
{

  GPIO_InitTypeDef GPIO_InitStruct;
  if(canHandle->Instance==CAN1)
  {
  /* USER CODE BEGIN CAN1_MspInit 0 */

  /* USER CODE END CAN1_MspInit 0 */
    /* CAN1 clock enable */
    __HAL_RCC_CAN1_CLK_ENABLE();
  
    /**CAN GPIO Configuration    
    PA11     ------> CAN_RX
    PA12     ------> CAN_TX 
    */
    GPIO_InitStruct.Pin = GPIO_PIN_11;
    GPIO_InitStruct.Mode = GPIO_MODE_INPUT;
    GPIO_InitStruct.Pull = GPIO_NOPULL;
    HAL_GPIO_Init(GPIOA, &GPIO_InitStruct);

    GPIO_InitStruct.Pin = GPIO_PIN_12;
    GPIO_InitStruct.Mode = GPIO_MODE_AF_PP;
    GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_HIGH;
    HAL_GPIO_Init(GPIOA, &GPIO_InitStruct);

    /* CAN1 interrupt Init */
    HAL_NVIC_SetPriority(USB_LP_CAN1_RX0_IRQn, 1, 0);
    HAL_NVIC_EnableIRQ(USB_LP_CAN1_RX0_IRQn);
  /* USER CODE BEGIN CAN1_MspInit 1 */

  /* USER CODE END CAN1_MspInit 1 */
  }
}

void HAL_CAN_MspDeInit(CAN_HandleTypeDef* canHandle)
{

  if(canHandle->Instance==CAN1)
  {
  /* USER CODE BEGIN CAN1_MspDeInit 0 */

  /* USER CODE END CAN1_MspDeInit 0 */
    /* Peripheral clock disable */
    __HAL_RCC_CAN1_CLK_DISABLE();
  
    /**CAN GPIO Configuration    
    PA11     ------> CAN_RX
    PA12     ------> CAN_TX 
    */
    HAL_GPIO_DeInit(GPIOA, GPIO_PIN_11|GPIO_PIN_12);

    /* CAN1 interrupt Deinit */
    HAL_NVIC_DisableIRQ(USB_LP_CAN1_RX0_IRQn);
  /* USER CODE BEGIN CAN1_MspDeInit 1 */

  /* USER CODE END CAN1_MspDeInit 1 */
  }
} 

/* USER CODE BEGIN 1 */



/* 标准CAN_ID与扩展CAN_ID混合过滤配置代码 */
/* 只需将需要接收的ID写入对应接收数组即可 */
/* 注：应考虑冗余位 */

void CAN_Config(void)
{
  CAN_FilterTypeDef  sFilterConfig;

  uint32_t StdIdArray[3]={1,3,56};//标准帧接收ID
  uint32_t ExtIdArray[3]={1,3,56};//拓展帧接收ID
  uint32_t mask,num,tmp,i,standard_mask,extend_mask,mix_mask;

  /* Configure the CAN Filter */
  sFilterConfig.FilterBank = 0;
  sFilterConfig.FilterMode = CAN_FILTERMODE_IDMASK;
  sFilterConfig.FilterScale = CAN_FILTERSCALE_32BIT;
  //sFilterConfig.FilterIdHigh = 0x0000;
  sFilterConfig.FilterIdHigh = ((ExtIdArray[0]<<3) >>16)&0xffff;;
  //sFilterConfig.FilterIdLow = 0x0000;
  sFilterConfig.FilterIdLow = ((ExtIdArray[0]<<3)&0xffff);
  
  standard_mask =0x7ff;		//计算屏蔽码
  num =sizeof(StdIdArray)/sizeof(StdIdArray[0]);
  for(i =0; i<num; i++){	//计算标准帧屏蔽码
    tmp =StdIdArray[i] ^ (~StdIdArray[0]);
    standard_mask &=tmp;
  }
  
  extend_mask =0x1fffffff;
  num =sizeof(ExtIdArray)/sizeof(ExtIdArray[0]);
  for(i =0; i<num; i++){	//计算拓展帧屏蔽码
    tmp =ExtIdArray[i] ^ (~ExtIdArray[0]);
    extend_mask &=tmp;
  }
  //计算混合屏蔽码及最终屏蔽码,并对齐寄存器
  mix_mask =(StdIdArray[0]<<18)^ (~ExtIdArray[0]);
  mask =(standard_mask<<18)& extend_mask &mix_mask;
  mask <<=3;
  
  //设置掩码
  sFilterConfig.FilterMaskIdHigh = (mask>>16)&0xffff;
  sFilterConfig.FilterMaskIdLow = (mask&0xffff);
  sFilterConfig.FilterFIFOAssignment = CAN_RX_FIFO0;
  sFilterConfig.FilterActivation = ENABLE;
  sFilterConfig.SlaveStartFilterBank = 14;
  
  if (HAL_CAN_ConfigFilter(&hcan, &sFilterConfig) != HAL_OK)
  {
    /* Filter configuration Error */
    Error_Handler();
  }

  /* Start the CAN peripheral */
  if (HAL_CAN_Start(&hcan) != HAL_OK)
  {
    /* Start Error */
    Error_Handler();
  }

  /* Activate CAN RX notification */
  HAL_CAN_ActivateNotification(&hcan, CAN_IT_RX_FIFO0_MSG_PENDING);
  HAL_CAN_ActivateNotification(&hcan, CAN_IT_RX_FIFO0_FULL);
}

typedef union{
        char ui8[8];
        uint16_t ui16[4];
        int in[2];
        float fl[2];
        double df;
}get_Data;


CAN_RxHeaderTypeDef  pHeader;//报头
get_Data Data;//数据
uint32_t pTxMailbox;
int idnumber=0;//设置id



void can_msg_parse(get_Data data)//分析来自can总线的命令
{
  
  
  
  
 // int order_number=0;
  if((data.ui8[0]-128)>=110){direction=1;}//控制前进
  if((data.ui8[0]-128)<=(0-110)){direction=2;}//控制后退
  if((data.ui8[1]-128)>=110){direction=3;}//控制左移
  if((data.ui8[1]-128)<=(0-110)){direction=4;}//控制右移
  if((data.ui8[3]-128)>=110){direction=5;}//控制左旋
  if((data.ui8[3]-128)<=(0-110)){direction=6;}//控制右旋
  //if((data.ui8[4]-128)==7){order_number=6;}//控制停止
  
  
  /*
  switch(order_number)
  {
  case 0:break;
  case 1:break;
  case 2:break;
  case 3:break;
  case 4:break;
  case 5:break;
  case 6:break;
  case 7:break;
    
    
  }*/
}



void HAL_CAN_RxFifo0MsgPendingCallback(CAN_HandleTypeDef *hcan)
{
   //读取消息
   HAL_CAN_GetRxMessage(hcan, CAN_RX_FIFO0, &pHeader,Data.ui8);
   if(pHeader.IDE==CAN_ID_STD)
   {
     if(pHeader.StdId==324)
     {
       can_msg_parse(Data);
     }
     if(pHeader.StdId==325)
     {
     }
       
   }
   if(pHeader.IDE==CAN_ID_EXT)
   {
     if(pHeader.ExtId==324)
     {
      can_msg_parse(Data);
     }
     if(pHeader.ExtId==325)
     {
     }
   }
   
   
   
   
  // Uprint(Data);

}

void HAL_CAN_RxFifo0FullCallback(CAN_HandleTypeDef *hcan)
{
	Uprint("The queue is full\r\n");//队列满时的处理
}

//HAL_CAN_AddTxMessage(hcan,&pHeader, Data[10], &pTxMailbox);

void can_send(char can_id,char id_number,char *letter,uint8_t letter_size)
{
   
  CAN_TxHeaderTypeDef header;
  if(can_id=='0')
  {
    header.IDE=CAN_ID_STD;//标准帧
    header.StdId=atoi(&id_number);
  }
  else
  {header.IDE=CAN_ID_EXT;
  header.ExtId=atoi(&id_number);}
  
  header.DLC=letter_size;
  header.RTR=CAN_RTR_DATA;
  header.TransmitGlobalTime=DISABLE;
  HAL_CAN_AddTxMessage(&hcan,&header, letter, &pTxMailbox);
 // Uprint(letter);

  
}







/*
计算各个轮子速度
*/

ActThreeVell ThreeWheelVellControl2(float Vx, float Vy, float angularVell)
{
#define AFA 60
#define L   2

ActThreeVell vell;
float theta = 0;

vell.v1 = (float)(-cos((AFA + theta) / 180.0f*3.1415926f) * Vx - sin((theta + AFA) / 180.0f*3.1415926f) * Vy + L * angularVell);

vell.v2 = (float)(cos(theta / 180.0f*3.1415926f) * Vx + sin(theta /180.0f*3.1415926f) * Vy      + L * angularVell);

vell.v3 = (float)(-cos((AFA - theta) / 180.0f * 3.1415926f) * Vx + sin((AFA - theta) / 180.0f*3.1415926f) * Vy + L * angularVell);

return vell;





}


typedef union{
        char ch[8];
        char ui8[8];
        uint16_t ui16[4];
        int in[2];
        float fl[2];
        double df;
}can_change_msg;


/*向can发送关于轮子的信息*/

int v1_id=1;
int v2_id=2;
int v3_id=3;


void send_wheel_msg(ActThreeVell vell)
{
  can_change_msg can_msg;
  

  can_msg.in[0]=1;
  can_msg.in[1]=(int)vell.v1;
  can_send(0,v1_id,can_msg.ui8,8);
  can_msg.in[1]=(int)vell.v2;
  can_send(0,v2_id,can_msg.ui8,8);
  can_msg.in[1]=(int)vell.v3;
  can_send(0,v3_id,can_msg.ui8,8);
  


}
/*
运动方向函数

*/


void forward()
{
  ActThreeVell v=ThreeWheelVellControl2(0, Vy, 0);
  send_wheel_msg(v);
  
}

void backward()
{
  ActThreeVell v=ThreeWheelVellControl2(0, 0-Vy,0);
  send_wheel_msg(v);
}

void left_translation()
{
  ActThreeVell v=ThreeWheelVellControl2(0-Vx,0,0);
  send_wheel_msg(v);
}

void right_translation()
{
  ActThreeVell v=ThreeWheelVellControl2(Vx, 0,0);
  send_wheel_msg(v);
}

void left_spin()
{
  ActThreeVell v=ThreeWheelVellControl2(0,0, angularVell);
  send_wheel_msg(v);
}

void right_spin()
{
  ActThreeVell v=ThreeWheelVellControl2(0,0, 0-angularVell);
  send_wheel_msg(v);
}

void stop()
{
  ActThreeVell v=ThreeWheelVellControl2(0,0,0);
  send_wheel_msg(v);
  
}

/* USER CODE END 1 */

/**
  * @}
  */

/**
  * @}
  */

/************************ (C) COPYRIGHT STMicroelectronics *****END OF FILE****/
