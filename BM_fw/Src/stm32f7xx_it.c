/**
  ******************************************************************************
  * @file    stm32f7xx_it.c
  * @brief   Interrupt Service Routines.
  ******************************************************************************
  *
  * COPYRIGHT(c) 2018 STMicroelectronics
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
#include "stm32f7xx_hal.h"
#include "stm32f7xx.h"
#include "stm32f7xx_it.h"

/* USER CODE BEGIN 0 */

#include "fifo.h"
#include "storage.h"
#include "fatfs.h"
#include "usbd_audio.h"



extern SPI_HandleTypeDef hspi2;
extern USBD_HandleTypeDef hUsbDeviceFS;


int16_t storageBuffer[STORAGE_BUFFER_SIZE];


extern int16_t receiveBuffer5[BUFFER_SIZE];
extern int16_t receiveBuffer4[BUFFER_SIZE];
extern int16_t receiveBuffer6[BUFFER_SIZE];
extern int16_t receiveBuffer3[BUFFER_SIZE];


extern SPI_HandleTypeDef hspi3;
extern SPI_HandleTypeDef hspi4;
extern SPI_HandleTypeDef hspi5;
extern SPI_HandleTypeDef hspi6;

int flag5 = NOT_STARTED,flag4 = NOT_STARTED, flag6 = NOT_STARTED, flag3 = NOT_STARTED;


int packetCounter = 0;  //10 msec per packet

int rec_on = 1;

UINT *dataWriten;
extern FIL File;


/* USER CODE END 0 */

/* External variables --------------------------------------------------------*/
extern PCD_HandleTypeDef hpcd_USB_OTG_FS;
extern DMA_HandleTypeDef hdma_sdmmc2_tx;
extern DMA_HandleTypeDef hdma_spi3_rx;
extern DMA_HandleTypeDef hdma_spi4_rx;
extern DMA_HandleTypeDef hdma_spi5_rx;
extern DMA_HandleTypeDef hdma_spi6_rx;
extern TIM_HandleTypeDef htim14;

/******************************************************************************/
/*            Cortex-M7 Processor Interruption and Exception Handlers         */ 
/******************************************************************************/

/**
* @brief This function handles Non maskable interrupt.
*/
void NMI_Handler(void)
{
  /* USER CODE BEGIN NonMaskableInt_IRQn 0 */

  /* USER CODE END NonMaskableInt_IRQn 0 */
  /* USER CODE BEGIN NonMaskableInt_IRQn 1 */

  /* USER CODE END NonMaskableInt_IRQn 1 */
}

/**
* @brief This function handles Hard fault interrupt.
*/
void HardFault_Handler(void)
{
  /* USER CODE BEGIN HardFault_IRQn 0 */

  /* USER CODE END HardFault_IRQn 0 */
  while (1)
  {
    /* USER CODE BEGIN W1_HardFault_IRQn 0 */
    /* USER CODE END W1_HardFault_IRQn 0 */
  }
  /* USER CODE BEGIN HardFault_IRQn 1 */

  /* USER CODE END HardFault_IRQn 1 */
}

/**
* @brief This function handles Memory management fault.
*/
void MemManage_Handler(void)
{
  /* USER CODE BEGIN MemoryManagement_IRQn 0 */

  /* USER CODE END MemoryManagement_IRQn 0 */
  while (1)
  {
    /* USER CODE BEGIN W1_MemoryManagement_IRQn 0 */
    /* USER CODE END W1_MemoryManagement_IRQn 0 */
  }
  /* USER CODE BEGIN MemoryManagement_IRQn 1 */

  /* USER CODE END MemoryManagement_IRQn 1 */
}

/**
* @brief This function handles Pre-fetch fault, memory access fault.
*/
void BusFault_Handler(void)
{
  /* USER CODE BEGIN BusFault_IRQn 0 */

  /* USER CODE END BusFault_IRQn 0 */
  while (1)
  {
    /* USER CODE BEGIN W1_BusFault_IRQn 0 */
    /* USER CODE END W1_BusFault_IRQn 0 */
  }
  /* USER CODE BEGIN BusFault_IRQn 1 */

  /* USER CODE END BusFault_IRQn 1 */
}

/**
* @brief This function handles Undefined instruction or illegal state.
*/
void UsageFault_Handler(void)
{
  /* USER CODE BEGIN UsageFault_IRQn 0 */

  /* USER CODE END UsageFault_IRQn 0 */
  while (1)
  {
    /* USER CODE BEGIN W1_UsageFault_IRQn 0 */
    /* USER CODE END W1_UsageFault_IRQn 0 */
  }
  /* USER CODE BEGIN UsageFault_IRQn 1 */

  /* USER CODE END UsageFault_IRQn 1 */
}

/**
* @brief This function handles System service call via SWI instruction.
*/
void SVC_Handler(void)
{
  /* USER CODE BEGIN SVCall_IRQn 0 */

  /* USER CODE END SVCall_IRQn 0 */
  /* USER CODE BEGIN SVCall_IRQn 1 */

  /* USER CODE END SVCall_IRQn 1 */
}

/**
* @brief This function handles Debug monitor.
*/
void DebugMon_Handler(void)
{
  /* USER CODE BEGIN DebugMonitor_IRQn 0 */

  /* USER CODE END DebugMonitor_IRQn 0 */
  /* USER CODE BEGIN DebugMonitor_IRQn 1 */

  /* USER CODE END DebugMonitor_IRQn 1 */
}

/**
* @brief This function handles Pendable request for system service.
*/
void PendSV_Handler(void)
{
  /* USER CODE BEGIN PendSV_IRQn 0 */

  /* USER CODE END PendSV_IRQn 0 */
  /* USER CODE BEGIN PendSV_IRQn 1 */

  /* USER CODE END PendSV_IRQn 1 */
}

/**
* @brief This function handles System tick timer.
*/
void SysTick_Handler(void)
{
  /* USER CODE BEGIN SysTick_IRQn 0 */

  /* USER CODE END SysTick_IRQn 0 */
  HAL_IncTick();
  HAL_SYSTICK_IRQHandler();
  /* USER CODE BEGIN SysTick_IRQn 1 */

  /* USER CODE END SysTick_IRQn 1 */
}

/******************************************************************************/
/* STM32F7xx Peripheral Interrupt Handlers                                    */
/* Add here the Interrupt Handlers for the used peripherals.                  */
/* For the available peripheral interrupt handler names,                      */
/* please refer to the startup file (startup_stm32f7xx.s).                    */
/******************************************************************************/

/**
* @brief This function handles DMA1 stream0 global interrupt.
*/
void DMA1_Stream0_IRQHandler(void)
{
  /* USER CODE BEGIN DMA1_Stream0_IRQn 0 */

  /* USER CODE END DMA1_Stream0_IRQn 0 */
  HAL_DMA_IRQHandler(&hdma_spi3_rx);
  /* USER CODE BEGIN DMA1_Stream0_IRQn 1 */

  /* USER CODE END DMA1_Stream0_IRQn 1 */
}

/**
* @brief This function handles EXTI line[15:10] interrupts.
*/
void EXTI15_10_IRQHandler(void)
{
  /* USER CODE BEGIN EXTI15_10_IRQn 0 */

  HAL_NVIC_DisableIRQ(EXTI15_10_IRQn);
  if (rec_on == 1)
  {
    rec_on = 0;
    close_session();
    HAL_GPIO_WritePin(YELLOW_GPIO_Port, YELLOW_Pin, GPIO_PIN_RESET);
  }else
  {
    
    rec_on = 1;
    HAL_GPIO_WritePin(YELLOW_GPIO_Port, YELLOW_Pin, GPIO_PIN_SET);
    //start_new_session();
  }

  /* USER CODE END EXTI15_10_IRQn 0 */
  HAL_GPIO_EXTI_IRQHandler(GPIO_PIN_13);
  HAL_GPIO_EXTI_IRQHandler(GPIO_PIN_15);
  /* USER CODE BEGIN EXTI15_10_IRQn 1 */

  HAL_NVIC_EnableIRQ(EXTI15_10_IRQn);

  /* USER CODE END EXTI15_10_IRQn 1 */
}

/**
* @brief This function handles TIM8 trigger and commutation interrupts and TIM14 global interrupt.
*/
void TIM8_TRG_COM_TIM14_IRQHandler(void)
{
  /* USER CODE BEGIN TIM8_TRG_COM_TIM14_IRQn 0 */

  /* USER CODE END TIM8_TRG_COM_TIM14_IRQn 0 */
  HAL_TIM_IRQHandler(&htim14);
  /* USER CODE BEGIN TIM8_TRG_COM_TIM14_IRQn 1 */


  /* USER CODE END TIM8_TRG_COM_TIM14_IRQn 1 */
}

/**
* @brief This function handles DMA2 stream0 global interrupt.
*/
void DMA2_Stream0_IRQHandler(void)
{
  /* USER CODE BEGIN DMA2_Stream0_IRQn 0 */

  /* USER CODE END DMA2_Stream0_IRQn 0 */
  HAL_DMA_IRQHandler(&hdma_spi4_rx);
  /* USER CODE BEGIN DMA2_Stream0_IRQn 1 */

  /* USER CODE END DMA2_Stream0_IRQn 1 */
}

/**
* @brief This function handles DMA2 stream3 global interrupt.
*/
void DMA2_Stream3_IRQHandler(void)
{
  /* USER CODE BEGIN DMA2_Stream3_IRQn 0 */

  /* USER CODE END DMA2_Stream3_IRQn 0 */
  HAL_DMA_IRQHandler(&hdma_spi5_rx);
  /* USER CODE BEGIN DMA2_Stream3_IRQn 1 */

  /* USER CODE END DMA2_Stream3_IRQn 1 */
}

/**
* @brief This function handles USB On The Go FS global interrupt.
*/
void OTG_FS_IRQHandler(void)
{
  /* USER CODE BEGIN OTG_FS_IRQn 0 */

  /* USER CODE END OTG_FS_IRQn 0 */
  HAL_PCD_IRQHandler(&hpcd_USB_OTG_FS);
  /* USER CODE BEGIN OTG_FS_IRQn 1 */

  /* USER CODE END OTG_FS_IRQn 1 */
}

/**
* @brief This function handles DMA2 stream5 global interrupt.
*/
void DMA2_Stream5_IRQHandler(void)
{
  /* USER CODE BEGIN DMA2_Stream5_IRQn 0 */

  /* USER CODE END DMA2_Stream5_IRQn 0 */
  HAL_DMA_IRQHandler(&hdma_sdmmc2_tx);
  /* USER CODE BEGIN DMA2_Stream5_IRQn 1 */

  /* USER CODE END DMA2_Stream5_IRQn 1 */
}

/**
* @brief This function handles DMA2 stream6 global interrupt.
*/
void DMA2_Stream6_IRQHandler(void)
{
  /* USER CODE BEGIN DMA2_Stream6_IRQn 0 */

  /* USER CODE END DMA2_Stream6_IRQn 0 */
  HAL_DMA_IRQHandler(&hdma_spi6_rx);
  /* USER CODE BEGIN DMA2_Stream6_IRQn 1 */

  /* USER CODE END DMA2_Stream6_IRQn 1 */
}

/* USER CODE BEGIN 1 */



void HAL_SPI_RxHalfCpltCallback(SPI_HandleTypeDef * hspi)
{
  int i;

  

  if (hspi->Instance == SPI3)
  {
    for (i=0; i<BUFFER_SIZE/2; i++)
    {
      storageBuffer[i+3*BUFFER_SIZE] = receiveBuffer3[i];
    }
    flag3 = HALF_READY;
  }
  if (hspi->Instance == SPI4)
  {
    for (i=0; i<BUFFER_SIZE/2; i++)
    {
      storageBuffer[i+BUFFER_SIZE] = receiveBuffer4[i];
    }
    flag4 = HALF_READY;
  }
  if (hspi->Instance == SPI5)
  {
    for (i=0; i<BUFFER_SIZE/2; i++)
    {
      storageBuffer[i] = receiveBuffer5[i];
    }
    flag5 = HALF_READY;
  }
  if (hspi->Instance == SPI6)
  {
    for (i=0; i<BUFFER_SIZE/2; i++)
    {
      storageBuffer[i+2*BUFFER_SIZE] = receiveBuffer6[i];
    }
    flag6 = HALF_READY;
  }

/*   if (((flag3==HALF_READY)||(flag3==NOT_STARTED))&&
    ((flag4==HALF_READY)||(flag4==NOT_STARTED))&&
    ((flag5==HALF_READY)||(flag5==NOT_STARTED))&&
    ((flag6==HALF_READY)||(flag6==NOT_STARTED)))
  {
      for (i=0; i<BUFFER_SIZE/2; i++)
      {
        storageBuffer[i+3*BUFFER_SIZE] = receiveBuffer3[i];
        storageBuffer[i+BUFFER_SIZE] = receiveBuffer4[i];
        storageBuffer[i] = receiveBuffer5[i];
        storageBuffer[i+2*BUFFER_SIZE] = receiveBuffer6[i];
      }
  } */

  
  fifo_put(receiveBuffer6, 2);

}

void HAL_SPI_RxCpltCallback(SPI_HandleTypeDef * hspi)
{
  int i;

  

  if (hspi->Instance == SPI3)
  {
/*     for (i=BUFFER_SIZE/2; i<BUFFER_SIZE; i++)
    {
      storageBuffer[i+3*BUFFER_SIZE] = receiveBuffer3[i];
    } */
    flag3 = FULL_READY;
  }
  if (hspi->Instance == SPI4)
  {
/*     for (i=BUFFER_SIZE/2; i<BUFFER_SIZE; i++)
    {
      storageBuffer[i+BUFFER_SIZE] = receiveBuffer4[i];
    } */
    flag4 = FULL_READY;
  }
  if (hspi->Instance == SPI5)
  {
/*     for (i=BUFFER_SIZE/2; i<BUFFER_SIZE; i++)
    {
      storageBuffer[i] = receiveBuffer5[i];
    } */
    flag5 = FULL_READY;
  }
  if (hspi->Instance == SPI6)
  {
/*     for (i=BUFFER_SIZE/2; i<BUFFER_SIZE; i++)
    {
      storageBuffer[i+2*BUFFER_SIZE] = receiveBuffer6[i];
    } */
    flag6 = FULL_READY;
  }


  packetCounter++;

  

  if (((flag3==FULL_READY)||(flag3==NOT_STARTED))&&
      ((flag4==FULL_READY)||(flag4==NOT_STARTED))&&
      ((flag5==FULL_READY)||(flag5==NOT_STARTED))&&
      ((flag6==FULL_READY)||(flag6==NOT_STARTED)))
  {
    if (rec_on == 1)
    {
      HAL_GPIO_WritePin(RED_GPIO_Port, RED_Pin, GPIO_PIN_SET);
      for (i=BUFFER_SIZE/2; i<BUFFER_SIZE; i++)
      {
        storageBuffer[i+3*BUFFER_SIZE] = receiveBuffer3[i];
        storageBuffer[i+BUFFER_SIZE] = receiveBuffer4[i];
        storageBuffer[i] = receiveBuffer5[i];
        storageBuffer[i+2*BUFFER_SIZE] = receiveBuffer6[i];
      }
      f_write(&File, &storageBuffer[0], sizeof(storageBuffer),dataWriten);
      flag3 = NOT_STARTED;
      flag4 = NOT_STARTED;
      flag5 = NOT_STARTED;
      flag6 = NOT_STARTED;
      HAL_GPIO_WritePin(RED_GPIO_Port, RED_Pin, GPIO_PIN_RESET);
    }
  }

  
  

/*   if (packetCounter >= 500)
  {
    //HAL_SPI_DMAPause(hspi);
    HAL_GPIO_WritePin(GREEN_GPIO_Port, GREEN_Pin, GPIO_PIN_SET);
<<<<<<< HEAD
    close_session();
  } */
=======
    f_close(&File);
  }
>>>>>>> parent of 4250c54... log_added

  fifo_put(&receiveBuffer6[BUFFER_SIZE/2], 2);
  
}

/* USER CODE END 1 */
/************************ (C) COPYRIGHT STMicroelectronics *****END OF FILE****/
