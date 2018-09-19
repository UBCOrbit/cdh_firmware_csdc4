
#include "pymem.h"
#include "string.h"
#include "stm32f4xx_hal.h"
#include "main.h"


extern UART_HandleTypeDef huart2;
extern UART_HandleTypeDef huart1;

/**
  * @brief  Requests memory form python memory emulator
  * @param  data the buffer where data will be stored
  * @param  address which section of the memory gets read
  * @param  Size Amount of data to be read
  * @retval none
  */
void request_pkt(uint8_t *data,uint16_t address,int size){
  uint8_t out[4];
  memset((char*)data,'\n',size);


  out[0]='r';//settup request command
  out[1]=(uint8_t)(address>>8);
  out[2]=(uint8_t)(0xFF&address);
  out[3]=(uint8_t)(0xFF&size);

  HAL_UART_Transmit(&huart2, out,4, 100);

  HAL_UART_Receive(&huart2, data,size, 1000);

  return;
}

/**
  * @brief  Sends memory to python memory emulator
  * @param  data the buffer with the data that needs to be written
  * @param  address which section of the memory gets written to
  * @param  Size Amount of data to be sent
  * @retval none
  */
void write_pkt(uint8_t *data, uint16_t address, int size){
  uint8_t out[size+4];
  int i;
  
  out[0]='w';
  out[1]=(uint8_t)(address>>8);
  out[2]=(uint8_t)(0xFF&address);
  out[3]=(uint8_t)(size);

  for (i=0;i<size;i++){
    out[4+i]=data[i];
  }

  HAL_UART_Transmit(&huart2, out,size+4, 100);
  
  HAL_Delay(100);// give the computer some time to write the data to a file

  return;
}


/**
  * @brief  Uses the memory emulator as a serial monitor
  * @param  data the buffer with the data that needs to be printed
  * @param  Size Amount of data to be sent
  * @retval none
  */
void ser_print(uint8_t *data, int size){

  uint8_t out[size+2];
  int i;

  out[0]='p';//settup request command
  out[1]=(uint8_t)(size);

  for (i=0;i<size;i++){
    out[2+i]=data[i];
  }

  HAL_UART_Transmit(&huart2, out,sizeof(out), 100);

  return;

}

/**
  * @brief  Generic python command interface
  * @param  cmd the command to be sent to the python script
  * @param  data the buffer with the data that needs to be printed
  * @param  Size Amount of data to be sent
  * @retval none
  */
void py_cmd(char cmd, uint8_t *data, int size){

  uint8_t out[size+3];
  int i;

  out[0]=cmd;//settup request command
  out[1]=(uint8_t)(size>>8);
  out[2]=(uint8_t)(0xFF&size);

  for (i=0;i<size;i++){
    out[3+i]=data[i];
  }

  HAL_UART_Transmit(&huart2, out,sizeof(out), 100);
  
  HAL_Delay(500);

  return;



}

void wcsdc(uint8_t *head, uint8_t *data, uint8_t size){

  uint8_t out[size+3];
  int i;

  HAL_Delay(3000);
  out[0]=(size);//settup request command


  for (i=0;i<2;i++){
    out[1+i]=head[i];
  }
  for (i=0;i<size;i++){
    out[3+i]=data[i];
  }

  HAL_UART_Transmit(&huart2, out,sizeof(out), 100);
  

  return;
}
/**
  * @brief  Receives packet from gnd
  * @param  data the data grounds sends
  * @retval size of data
  */
uint8_t rcsdc(uint8_t *data){

  uint8_t size;

  while(HAL_UART_Receive(&huart2, &size,1, 500)!=HAL_OK) 
    HAL_GPIO_TogglePin(LD2_GPIO_Port, LD2_Pin);
  HAL_UART_Receive(&huart2, data,size, 100);

  return size;
}


void sertest(){

  uint8_t in[256];
  uint8_t size[1];
  while(HAL_UART_Receive(&huart2, size,1, 500)!=HAL_OK) 
   HAL_GPIO_TogglePin(LD2_GPIO_Port, LD2_Pin);
  HAL_UART_Transmit(&huart2,size,1,10000);
  HAL_UART_Receive(&huart2, in,size[0], 10000);
  HAL_Delay(500);
  HAL_UART_Transmit(&huart2, in,size[0], 10000);

}

void csdcdemo(){

  uint8_t rbuff[256];
  uint8_t cdhead[2];
  uint8_t cdhbuff[254];
  uint8_t rsize;

  HAL_GPIO_WritePin(LD2_GPIO_Port, LD2_Pin, 1);

  // sertest();

  rsize=rcsdc(rbuff); //wait for packet from gnd
  wcsdc(rbuff,rbuff+2,rsize);
  HAL_UART_Transmit(&huart1, rbuff,rsize, 100);//send data from gnd to cdh
  
  memset((char*)rbuff,'\0',sizeof(rbuff));
  rsize=rcsdc(rbuff); //wait for packet from gnd
  wcsdc(rbuff,rbuff+2,rsize);
  HAL_UART_Transmit(&huart1, rbuff,rsize, 100);//send data from gnd to cdh
  

  memset((char*)rbuff,'\0',sizeof(rbuff));
  rsize=rcsdc(rbuff); //wait for packet from gnd
  wcsdc(rbuff,rbuff+2,rsize);
  HAL_UART_Transmit(&huart1, rbuff,rsize, 100);//send data from gnd to cdh
  
  HAL_GPIO_WritePin(LD2_GPIO_Port, LD2_Pin, 1);

  while(1){  


    memset((char*)cdhbuff,'y',sizeof(cdhbuff));

    while(HAL_UART_Receive(&huart1,cdhead,sizeof(cdhead),100)!=HAL_OK);//wait for cdh's reply
    rsize=cdhead[1];  
    // HAL_GPIO_TogglePin(LD2_GPIO_Port, LD2_Pin);
    HAL_GPIO_WritePin(LD2_GPIO_Port, LD2_Pin, 0);

    if(HAL_UART_Receive(&huart1,cdhbuff,rsize,100)==HAL_OK)
      HAL_GPIO_WritePin(LD2_GPIO_Port, LD2_Pin, 1);

    wcsdc(cdhead,cdhbuff,rsize+2);//return cdh reply to ground
  }

}