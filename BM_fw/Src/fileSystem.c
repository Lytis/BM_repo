#include "fileSystem.h"

#include "main.h"
#include "stm32f7xx_hal.h"
#include "fatfs.h"
#include "usb_device.h"

#include "fifo.h"
#include "usb_device.h"
#include "storage.h"
#include "ff.h"




FATFS mynewdiskFatFs;
FIL File, logFile;
UINT *dataWr;
char mynewdiskPath[4];
char dataFileName[] = "data_00.hex";
char logFileName[] = "logfile.log";
char logMsg[] = "log:data_00.hex\n";

int file_no = 0;
int file_size = 0, logs = 0;


void init_file_system()
{

    
  int file_no = 0;
  int file_size = 0, logs = 0;

  if(f_mount(&mynewdiskFatFs, (TCHAR const*)mynewdiskPath,0) == FR_OK)
  {
    file_no = 0;
    HAL_Delay(500); // time to sellte the SD card init
  }

}


void start_new_session()
{
    if (f_open(&logFile, logFileName, FA_OPEN_ALWAYS|FA_WRITE) == FR_OK)
    {
        file_size = f_size(&logFile);
        logs = file_size / sizeof(logMsg);
        file_no = logs + 1;
      
        HAL_GPIO_WritePin(YELLOW_GPIO_Port, YELLOW_Pin, GPIO_PIN_SET);
    }else
    {
      HAL_GPIO_WritePin(RED_GPIO_Port, RED_Pin, GPIO_PIN_SET);
    }

    HAL_Delay(500);
    HAL_GPIO_WritePin(YELLOW_GPIO_Port, YELLOW_Pin, GPIO_PIN_RESET);
    HAL_GPIO_WritePin(RED_GPIO_Port, RED_Pin, GPIO_PIN_RESET);

    sprintf(dataFileName, "data_%.2d.hex", file_no);
    sprintf(logMsg, "log: data_%.2d.hex\n", file_no);

    if (f_open(&File, dataFileName, FA_CREATE_ALWAYS|FA_WRITE) == FR_OK)
    {
      HAL_GPIO_WritePin(YELLOW_GPIO_Port, YELLOW_Pin, GPIO_PIN_SET);
      f_lseek(&logFile, file_size);
      f_write(&logFile, logMsg, sizeof(logMsg), dataWr);

    }else
    {
      HAL_GPIO_WritePin(RED_GPIO_Port, RED_Pin, GPIO_PIN_SET);
    }

    f_close(&logFile);

    HAL_Delay(500);
    HAL_GPIO_WritePin(YELLOW_GPIO_Port, YELLOW_Pin, GPIO_PIN_RESET);
    HAL_GPIO_WritePin(RED_GPIO_Port, RED_Pin, GPIO_PIN_RESET);

}


void close_session()
{
    f_close(&File);
}




