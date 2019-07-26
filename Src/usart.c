/**
  ******************************************************************************
  * File Name          : USART.c
  * Description        : This file provides code for the configuration
  *                      of the USART instances.
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
#include "usart.h"

#include "gpio.h"

/* USER CODE BEGIN 0 */
#include "main.h"
#include <math.h>
#include "tim.h"
#include "string.h"
#include <stdlib.h>
#include "stdint.h"
#include "can.h"


UART_HandleTypeDef huart1;
//int show_speed=0;
//int pid=0;


/* USER CODE END 0 */

UART_HandleTypeDef huart1;

/* USART1 init function */

void MX_USART1_UART_Init(void)
{

  huart1.Instance = USART1;
  huart1.Init.BaudRate = 115200;
  huart1.Init.WordLength = UART_WORDLENGTH_8B;
  huart1.Init.StopBits = UART_STOPBITS_1;
  huart1.Init.Parity = UART_PARITY_NONE;
  huart1.Init.Mode = UART_MODE_TX_RX;
  huart1.Init.HwFlowCtl = UART_HWCONTROL_NONE;
  huart1.Init.OverSampling = UART_OVERSAMPLING_16;
  if (HAL_UART_Init(&huart1) != HAL_OK)
  {
    _Error_Handler(__FILE__, __LINE__);
  }

}

void HAL_UART_MspInit(UART_HandleTypeDef* uartHandle)
{

  GPIO_InitTypeDef GPIO_InitStruct;
  if(uartHandle->Instance==USART1)
  {
  /* USER CODE BEGIN USART1_MspInit 0 */

  /* USER CODE END USART1_MspInit 0 */
    /* USART1 clock enable */
    __HAL_RCC_USART1_CLK_ENABLE();
  
    /**USART1 GPIO Configuration    
    PA9     ------> USART1_TX
    PA10     ------> USART1_RX 
    */
    GPIO_InitStruct.Pin = GPIO_PIN_9;
    GPIO_InitStruct.Mode = GPIO_MODE_AF_PP;
    GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_HIGH;
    HAL_GPIO_Init(GPIOA, &GPIO_InitStruct);

    GPIO_InitStruct.Pin = GPIO_PIN_10;
    GPIO_InitStruct.Mode = GPIO_MODE_INPUT;
    GPIO_InitStruct.Pull = GPIO_NOPULL;
    HAL_GPIO_Init(GPIOA, &GPIO_InitStruct);

    /* USART1 interrupt Init */
    HAL_NVIC_SetPriority(USART1_IRQn, 1, 0);
    HAL_NVIC_EnableIRQ(USART1_IRQn);
  /* USER CODE BEGIN USART1_MspInit 1 */

  /* USER CODE END USART1_MspInit 1 */
  }
}

void HAL_UART_MspDeInit(UART_HandleTypeDef* uartHandle)
{

  if(uartHandle->Instance==USART1)
  {
  /* USER CODE BEGIN USART1_MspDeInit 0 */

  /* USER CODE END USART1_MspDeInit 0 */
    /* Peripheral clock disable */
    __HAL_RCC_USART1_CLK_DISABLE();
  
    /**USART1 GPIO Configuration    
    PA9     ------> USART1_TX
    PA10     ------> USART1_RX 
    */
    HAL_GPIO_DeInit(GPIOA, GPIO_PIN_9|GPIO_PIN_10);

    /* USART1 interrupt Deinit */
    HAL_NVIC_DisableIRQ(USART1_IRQn);
  /* USER CODE BEGIN USART1_MspDeInit 1 */

  /* USER CODE END USART1_MspDeInit 1 */
  }
} 

/* USER CODE BEGIN 1 */


uint8_t cmd[50] = {0};
char detail[10][50] = {0,0};
float figure[10] = {0};
//char s[22]={'b','y',16,6};//用在示波器里面
//char letter[8]={'1','2','3'};

/*
//用在示波器里面
void send_wave(float arg1,float arg2,float arg3,float arg4){

  s[20]='\r';
  s[21]='\n';
  memcpy(s+4,&arg1,sizeof(arg1));
  memcpy(s+8,&arg2,sizeof(arg1));
  memcpy(s+12,&arg3,sizeof(arg1));
  memcpy(s+16,&arg4,sizeof(arg1));
  HAL_UART_Transmit(&huart1,(uint8_t *)s,sizeof(s),2000);
  }

*/




void char_to_number(int num)//将收到的字符数组转为数字
{
  int p=0;
  for(p=0;p<=num;p++)
  {
  if(strlen((char *)detail[p])!=0)
  {
    figure[p]=atof((char *)detail[p]);
  
  }
  }
}


void do_cmd()
{
  
  int cmd_number;

  cmd_number=0;
  if(strcmp((char *)cmd,"setmode")==0)
      {cmd_number=1;}
  if(strcmp((char *)cmd,"forward")==0)
      {cmd_number=2;}
  if(strcmp((char *)cmd,"backward")==0)
      {cmd_number=3;}
  if(strcmp((char *)cmd,"lefttranslation")==0)
      {cmd_number=4;}
  if(strcmp((char *)cmd,"righttraslation")==0)
      {cmd_number=5;}
  if(strcmp((char *)cmd,"leftspin")==0)
      {cmd_number=6;}
  if(strcmp((char *)cmd,"rightspin")==0)
      {cmd_number=7;}
  if(strcmp((char *)cmd,"stop")==0)
      {cmd_number=8;}
  if(strcmp((char *)cmd,"setidnumber")==0)
      {cmd_number=9;}
  if(strcmp((char *)cmd,"setvy")==0)
      {cmd_number=10;}
  if(strcmp((char *)cmd,"setvx")==0)
      {cmd_number=11;}
  if(strcmp((char *)cmd,"setangularVell")==0)
      {cmd_number=12;}
  if(strcmp((char *)cmd,"showidnumber")==0)
      {cmd_number=13;}
  if(strcmp((char *)cmd,"help")==0)
      {cmd_number=14;}
  if(strcmp((char *)cmd,"showwheelid")==0)
      {cmd_number=15;}
  if(strcmp((char *)cmd,"vaid")==0)
      {cmd_number=16;}
  if(strcmp((char *)cmd,"vbid")==0)
      {cmd_number=17;}
  if(strcmp((char *)cmd,"vcid")==0)
      {cmd_number=18;}
  
  
  switch(cmd_number)
  {
    case 0:Uprint("wrong cmd!   \r\n");break;
    case 1:move_mode=figure[0];break;
    case 2:direction=1;break;
    case 3:direction=2;break;
    case 4:direction=3;break;
    case 5:direction=4;break;
    case 6:direction=5;break;
    case 7:direction=6;break;
    case 8:direction=7;break;
    case 9:idnumber=figure[0];break;
    case 10:Vy=figure[0];break;
    case 11:Vx=figure[0];break;
    case 12:angularVell=figure[0];break;
    case 13:Uprint("id number is %d \r\n",idnumber);break;
    case 14:Uprint("setmode\r\nforward\r\nbackward\r\nlefttranslation\r\nrighttraslation\r\nleftspin\r\nrightspin\r\nstop\r\nsetidnumber (idnumber)\r\nsetvy (Vy)\r\nsetvx (Vx)\r\nsetangularVell (angularVell)\r\nshowidnumber\r\nhelp\r\nshowwheelid\r\nvaid (v1id)\r\nvbid(v2id)\r\nvcid (v3id)");
    case 15:
    

}
}

void cmd_phase(uint8_t buffer_context)
{
   static int i=0,line=-1,crow=0;
   if((buffer_context>='A' && buffer_context <= 'Z')||(buffer_context>='a' && buffer_context<='z'))
    {
      cmd[i]=buffer_context;
      i++;
    }
    if(buffer_context==' ')
    {
        detail[line][crow]='\0';
        line++;
        crow=0;
    }
    if((buffer_context>='0' && buffer_context<='9')||(buffer_context=='.'))
    {
      detail[line][crow]=buffer_context;
      crow++;
    }
   if(buffer_context=='\r')
   {
     cmd[i]='\0';
     detail[line][crow]='\0';
     char_to_number(line);
     do_cmd();//放置需要执行的函数
    i=0;
    line=-1;
    crow=0;
   }
   if(buffer_context=='\n')
   {
     Uprint("Done!\r\n");
   }
}


//串口中断
void HAL_UART_RxCpltCallback(UART_HandleTypeDef *huart){
  if(huart->Instance==USART1){  
    
	cmd_phase(buffer_rx_temp[0]);
	// 开中断，否则只能进入一次中断
HAL_UART_Receive_IT(&huart1,(uint8_t *)&buffer_rx_temp,1);
  }
}


/* USER CODE END 1 */

/**
  * @}
  */

/**
  * @}
  */

/************************ (C) COPYRIGHT STMicroelectronics *****END OF FILE****/
