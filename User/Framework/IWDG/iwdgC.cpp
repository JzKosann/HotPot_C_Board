//
// Created by ShiF on 2023/9/12.
//
/* Incude---------------------------------------------------------------------*/
#include "iwdgC.h"

/* Private define-------------------------------------------------------------*/

/* Private variables----------------------------------------------------------*/

/* Private function prototypes------------------------------------------------*/
//static void FeedDog(void);
extern IWDG_HandleTypeDef hiwdg;
/* Public variables-----------------------------------------------------------*/
/**
	* @name   FeedDog
	* @brief  喂狗
*/
void FeedDog(void)
{
    HAL_IWDG_Refresh(&hiwdg); //喂狗
}
/********************************************************
  End Of File
********************************************************/