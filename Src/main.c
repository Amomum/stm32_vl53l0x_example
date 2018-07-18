/* Includes ------------------------------------------------------------------*/
#include "main.h"
#include "stm32l1xx_hal.h"

#include "vl53l0/ranging_vl53l0x.h"
#include "vl53l0x_api.h"

/* Private variables ---------------------------------------------------------*/
I2C_HandleTypeDef hi2c2;



/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
static void MX_GPIO_Init(void);
static void MX_I2C2_Init(void);


/* Private function prototypes -----------------------------------------------*/

/**
  * @brief  The application entry point.
  *
  * @retval None
  */
int main(void)
{
    /* MCU Configuration----------------------------------------------------------*/

    /* Reset of all peripherals, Initializes the Flash interface and the Systick. */
    HAL_Init();


    SystemClock_Config();


    MX_GPIO_Init();
    MX_I2C2_Init();

    static VL53L0X_Dev_t device;
    
    static volatile uint16_t res = 0;
    
    // дефолты
    device.I2cHandle=&hi2c2;
    device.I2cDevAddr=0x52;                                                   
    device.Present=0;
    device.Id=0; 
  
  
    static VL53L0X_Error Status;                           //под хранение кода ошибки
    static uint32_t refSpadCount;                      //для процесса конфигурации датчиков
    static uint8_t  isApertureSpads;                    //для процесса конфигурации датчиков
    static uint8_t  VhvSettings;                          //для процесса конфигурации датчиков
    static uint8_t  PhaseCal;                             //для процесса конфигурации датчиков


    refSpadCount =    0;   
    isApertureSpads = 0;
    VhvSettings =     0;      
    PhaseCal =        0;        


    Status=VL53L0X_ERROR_NONE;                                                  //сбрасываем код ошибки        
   
    if (Status == VL53L0X_ERROR_NONE) 
    {                                                         
        Status=VL53L0X_DataInit(&device);
    }
    
    if (Status == VL53L0X_ERROR_NONE) 
    {
        Status=VL53L0X_StaticInit(&device);	
    }
    
    if (Status == VL53L0X_ERROR_NONE) 
    {
        Status = VL53L0X_PerformRefSpadManagement(&device, &refSpadCount, &isApertureSpads);
    }
    
    if (Status == VL53L0X_ERROR_NONE) 
    {
        Status = VL53L0X_PerformRefCalibration(&device, &VhvSettings, &PhaseCal);
    }
    
    if (Status == VL53L0X_ERROR_NONE) 
    {
        Status=VL53L0X_SetReferenceSpads(&device, refSpadCount, isApertureSpads);
    }      
    
    if (Status == VL53L0X_ERROR_NONE) 
    {
        Status=VL53L0X_SetRefCalibration(&device, VhvSettings, PhaseCal);
    }
    
    if (Status == VL53L0X_ERROR_NONE) 
    {
        Status=VL53L0X_SetDeviceMode(&device, VL53L0X_DEVICEMODE_CONTINUOUS_RANGING);	
    }
    
    if (Status == VL53L0X_ERROR_NONE) 
    {
        Status = VL53L0X_SetLimitCheckValue(&device, VL53L0X_CHECKENABLE_SIGNAL_RATE_FINAL_RANGE, (FixPoint1616_t)(0.25*65536));
    }
    
    if (Status == VL53L0X_ERROR_NONE) 
    {
        Status = VL53L0X_SetLimitCheckValue(&device, VL53L0X_CHECKENABLE_SIGMA_FINAL_RANGE, (FixPoint1616_t)(32*65536));
    }
    
    if (Status == VL53L0X_ERROR_NONE) 
    {
        Status =VL53L0X_SetMeasurementTimingBudgetMicroSeconds(&device,	20000);
    }
    
    if (Status == VL53L0X_ERROR_NONE) 
    {
        Status=VL53L0X_StartMeasurement(&device);
    }
            
          
    static uint8_t data_ready;      //флаг готовности результата измерений       
    static VL53L0X_RangingMeasurementData_t result;
    
    while(1)
    {
        Status=VL53L0X_GetMeasurementDataReady(&device, &data_ready);
        
        if( Status == VL53L0X_ERROR_NONE )
        {
            Status=VL53L0X_GetRangingMeasurementData(&device, &result);            

            if (Status == VL53L0X_ERROR_NONE) 
            {
                Status=VL53L0X_ClearInterruptMask(&device,0);
            }
        }
    }
  
  
  
//  uint8_t status = IR_Init(&hi2c2);
//  
//  if( status != 0 )
//  {
//    __BKPT(0xAA);
//  }
//  
//  
//  
//  static volatile uint16_t res = 0;

//  while (1)
//  {
//      IR_Process();
//      
//      res=IR_GetRange(0);

//  }

}

/**
  * @brief System Clock Configuration
  * @retval None
  */
void SystemClock_Config(void)
{

  RCC_OscInitTypeDef RCC_OscInitStruct;
  RCC_ClkInitTypeDef RCC_ClkInitStruct;

    /**Configure the main internal regulator output voltage 
    */
  __HAL_PWR_VOLTAGESCALING_CONFIG(PWR_REGULATOR_VOLTAGE_SCALE1);

    /**Initializes the CPU, AHB and APB busses clocks 
    */
  RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSE;
  RCC_OscInitStruct.HSEState = RCC_HSE_ON;
  RCC_OscInitStruct.PLL.PLLState = RCC_PLL_ON;
  RCC_OscInitStruct.PLL.PLLSource = RCC_PLLSOURCE_HSE;
  RCC_OscInitStruct.PLL.PLLMUL = RCC_PLL_MUL8;
  RCC_OscInitStruct.PLL.PLLDIV = RCC_PLL_DIV3;
  if (HAL_RCC_OscConfig(&RCC_OscInitStruct) != HAL_OK)
  {
    _Error_Handler(__FILE__, __LINE__);
  }

    /**Initializes the CPU, AHB and APB busses clocks 
    */
  RCC_ClkInitStruct.ClockType = RCC_CLOCKTYPE_HCLK|RCC_CLOCKTYPE_SYSCLK
                              |RCC_CLOCKTYPE_PCLK1|RCC_CLOCKTYPE_PCLK2;
  RCC_ClkInitStruct.SYSCLKSource = RCC_SYSCLKSOURCE_PLLCLK;
  RCC_ClkInitStruct.AHBCLKDivider = RCC_SYSCLK_DIV1;
  RCC_ClkInitStruct.APB1CLKDivider = RCC_HCLK_DIV1;
  RCC_ClkInitStruct.APB2CLKDivider = RCC_HCLK_DIV1;

  if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_1) != HAL_OK)
  {
    _Error_Handler(__FILE__, __LINE__);
  }

    /**Configure the Systick interrupt time 
    */
  HAL_SYSTICK_Config(HAL_RCC_GetHCLKFreq()/1000);

    /**Configure the Systick 
    */
  HAL_SYSTICK_CLKSourceConfig(SYSTICK_CLKSOURCE_HCLK);

  /* SysTick_IRQn interrupt configuration */
  HAL_NVIC_SetPriority(SysTick_IRQn, 0, 0);
}

/* I2C2 init function */
static void MX_I2C2_Init(void)
{

  hi2c2.Instance = I2C2;
  hi2c2.Init.ClockSpeed = 100000;
  hi2c2.Init.DutyCycle = I2C_DUTYCYCLE_2;
  hi2c2.Init.OwnAddress1 = 0;
  hi2c2.Init.AddressingMode = I2C_ADDRESSINGMODE_7BIT;
  hi2c2.Init.DualAddressMode = I2C_DUALADDRESS_DISABLE;
  hi2c2.Init.OwnAddress2 = 0;
  hi2c2.Init.GeneralCallMode = I2C_GENERALCALL_DISABLE;
  hi2c2.Init.NoStretchMode = I2C_NOSTRETCH_DISABLE;
  if (HAL_I2C_Init(&hi2c2) != HAL_OK)
  {
    _Error_Handler(__FILE__, __LINE__);
  }

}

/** Configure pins as 
        * Analog 
        * Input 
        * Output
        * EVENT_OUT
        * EXTI
*/
static void MX_GPIO_Init(void)
{

  /* GPIO Ports Clock Enable */
  __HAL_RCC_GPIOH_CLK_ENABLE();
  __HAL_RCC_GPIOB_CLK_ENABLE();
  
  GPIO_InitTypeDef GPIO_InitStruct;


  GPIO_InitStruct.Pin = GPIO_PIN_0 | GPIO_PIN_1 | GPIO_PIN_5 | GPIO_PIN_8;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  
  HAL_GPIO_Init(GPIOB, &GPIO_InitStruct);
  
    /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(GPIOB, GPIO_PIN_0 | GPIO_PIN_1 | GPIO_PIN_5 | GPIO_PIN_8, GPIO_PIN_SET);


}

/* USER CODE BEGIN 4 */

/* USER CODE END 4 */

/**
  * @brief  This function is executed in case of error occurrence.
  * @param  file: The file name as string.
  * @param  line: The line in file as a number.
  * @retval None
  */
void _Error_Handler(char *file, int line)
{
  /* USER CODE BEGIN Error_Handler_Debug */
  /* User can add his own implementation to report the HAL error return state */
  while(1)
  {
  }
  /* USER CODE END Error_Handler_Debug */
}

#ifdef  USE_FULL_ASSERT
/**
  * @brief  Reports the name of the source file and the source line number
  *         where the assert_param error has occurred.
  * @param  file: pointer to the source file name
  * @param  line: assert_param error line source number
  * @retval None
  */
void assert_failed(uint8_t* file, uint32_t line)
{ 
  /* USER CODE BEGIN 6 */
  /* User can add his own implementation to report the file name and line number,
     tex: printf("Wrong parameters value: file %s on line %d\r\n", file, line) */
  /* USER CODE END 6 */
}
#endif /* USE_FULL_ASSERT */

/**
  * @}
  */

/**
  * @}
  */

/************************ (C) COPYRIGHT STMicroelectronics *****END OF FILE****/
