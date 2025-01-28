/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file           : main.c
  * @brief          : Main program body
  ******************************************************************************
  * @attention
  *
  * Copyright (c) 2025 STMicroelectronics.
  * All rights reserved.
  *
  * This software is licensed under terms that can be found in the LICENSE file
  * in the root directory of this software component.
  * If no LICENSE file comes with this software, it is provided AS-IS.
  *
  ******************************************************************************
  */
/* USER CODE END Header */
/* Includes ------------------------------------------------------------------*/
#include "main.h"

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */

/* USER CODE END Includes */

/* Private typedef -----------------------------------------------------------*/
/* USER CODE BEGIN PTD */
enum fw_update_type
{
  FW_UPDATE_APP = 1,
  FW_UPDATE_BOOT = 2,
};

typedef struct
{
  uint8_t version[3];
  uint8_t fw_type;
  uint32_t magic;
} fw_info;

/* USER CODE END PTD */

/* Private define ------------------------------------------------------------*/
/* USER CODE BEGIN PD */

/* USER CODE END PD */

/* Private macro -------------------------------------------------------------*/
/* USER CODE BEGIN PM */

/* USER CODE END PM */

/* Private variables ---------------------------------------------------------*/
UART_HandleTypeDef huart1;

/* USER CODE BEGIN PV */
extern const uint32_t __BootFlash_start;
extern const uint32_t __BootFlash_end;
extern const uint32_t __AppFlash_start;
extern const uint32_t __AppFlash_end;
extern const uint32_t __FWUpdateFlash_start;
extern const uint32_t __FWUpdateFlash_end;
extern const fw_info __FWInfo_addr;

const uint32_t* boot_start_addr = &__BootFlash_start;
const uint32_t* boot_end_addr = &__BootFlash_end;
const uint32_t* app_start_addr = &__AppFlash_start;
const uint32_t* app_end_addr = &__AppFlash_end;
const uint32_t* update_start_addr = &__FWUpdateFlash_start;
const uint32_t* update_end_addr = &__FWUpdateFlash_end;

const fw_info* fw_header = &__FWInfo_addr;

#define STM32_FLASH_SIZE 64  // Kbytes
  #if STM32_FLASH_SIZE < 256
    #define STM_SECTOR_SIZE 1024 //1K bytes
  #else 
    #define STM_SECTOR_SIZE 2048 //2K bytes
#endif

/* Function pointer for jumping to user application. */
typedef void (*app_ptr)(void);

/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
static void MX_GPIO_Init(void);
static void MX_USART1_UART_Init(void);
/* USER CODE BEGIN PFP */

/* USER CODE END PFP */

/* Private user code ---------------------------------------------------------*/
/* USER CODE BEGIN 0 */
char* u8_to_str(uint8_t n, char* str){
  uint8_t digits[3];

  digits[0] = n / 100;
  digits[1] = (n - digits[0]) / 10;
  digits[2] = (n - digits[1]);

  if (digits[0] == 0){
    if (digits[1] == 0){
      str[0] = digits[2] | 0x30;
      str[1] = 0;
    }
    else{
      str[0] = digits[1] | 0x30;
      str[1] = digits[2] | 0x30;
      str[2] = 0;
    }
  }
  else{
    str[0] = digits[0] | 0x30;
    str[1] = digits[1] | 0x30;
    str[2] = digits[2] | 0x30;
    str[3] = 0;
  }
  return str;
}

int uart0_write_str_blocking(char* buffer)
{
	int len = 0;
	while (*buffer){
		HAL_UART_Transmit(&huart1, (uint8_t *)buffer, 1, 0xFFFF);
		buffer++;
		len++;
	}
	return len;
}

uint8_t get_page_number(const uint32_t* flash_addr){
  return ((uint8_t)((uint32_t)flash_addr - (uint32_t)boot_start_addr) / STM_SECTOR_SIZE);
}

uint32_t* get_address(const uint8_t page_num){
  return ( uint32_t*)(boot_start_addr+page_num*STM_SECTOR_SIZE);
}

void copy_flash_page(const uint8_t src_page, const uint8_t dest_page, uint8_t page_len)
{
  uint32_t buffer[STM_SECTOR_SIZE/4];
  uint32_t* src_addr = get_address(src_page);
  uint32_t* dest_addr = get_address(dest_page);

  for (uint32_t i = 0; i < page_len; i++){
    for (uint32_t j = 0; j < STM_SECTOR_SIZE/4; j++){
      buffer[j] = *(uint32_t*)(src_addr+i*STM_SECTOR_SIZE+j*4);
    }
    for (uint32_t j = 0; j < STM_SECTOR_SIZE/4; j++){
      if(HAL_FLASH_Program(FLASH_TYPEPROGRAM_WORD, (uint32_t)(dest_addr+i*STM_SECTOR_SIZE+j*4), buffer[j]) != HAL_OK){
        /*Error occurred while write.*/
        uart0_write_str_blocking("[BOOTLOADER] Page write ERROR");
        return;
      }
      /* Read back the content of the memory. If it is wrong, then report an error. */
      if (buffer[j] != *(volatile uint32_t*)(dest_addr+i*STM_SECTOR_SIZE+j*4)){
        /*Error occurred while read back check. */
        uart0_write_str_blocking("[BOOTLOADER] Page read back and check ERROR");
        return;
      }
    }
  }
}

/**
 * @brief   Actually jumps to the user application.
 * @param   void
 * @return  void
 */
void flash_jump_to_app(void)
{
  /* Function pointer to the address of the user application. */
  app_ptr jump_to_app;
  jump_to_app = (app_ptr)(*(volatile uint32_t*) (app_start_addr+4u));
  // HAL_DeInit();
  /* Change the main stack pointer. */
  __set_MSP(*(volatile uint32_t*)app_start_addr);
  jump_to_app();
}

void update(enum fw_update_type ftype)
{
  static FLASH_EraseInitTypeDef EraseInitStruct;
  uint32_t PAGEError;
  uint8_t target_start_page;
  uint8_t target_end_page;
  uint8_t target_page_size;
  const uint8_t update_start_page = get_page_number(update_start_addr);

  if (ftype == FW_UPDATE_APP)
  {
      target_start_page = get_page_number(app_start_addr);
      target_end_page = get_page_number(app_end_addr);
      target_page_size = target_end_page - target_start_page + 1;
  }
  else if (ftype == FW_UPDATE_BOOT)
  {
      target_start_page = get_page_number(boot_start_addr);
      target_end_page = get_page_number(boot_end_addr);
      target_page_size = target_end_page - target_start_page + 1;
  }
  else return;

  /* Unlock the flash to enable the flash control register access */
  HAL_FLASH_Unlock(); 

  /* Erase flash firmware app or bootloader section */
  EraseInitStruct.TypeErase = FLASH_TYPEERASE_PAGES;
  EraseInitStruct.PageAddress = target_start_page;
  EraseInitStruct.NbPages = target_page_size; 
	if(HAL_FLASHEx_Erase(&EraseInitStruct, &PAGEError) != HAL_OK){
		/*Error occurred while page erase.*/
    uart0_write_str_blocking("[BOOTLOADER] Page erase ERROR");
		return;
	}

  copy_flash_page(update_start_page, target_start_page, target_page_size);

  /* Erase flash firmware update section */
  EraseInitStruct.TypeErase = FLASH_TYPEERASE_PAGES;
  EraseInitStruct.PageAddress = update_start_page;
  EraseInitStruct.NbPages = target_page_size; 
	if(HAL_FLASHEx_Erase(&EraseInitStruct, &PAGEError) != HAL_OK){
		/*Error occurred while page erase.*/
    uart0_write_str_blocking("[BOOTLOADER] Page erase ERROR for update section");
		return;
	}

	/* Lock the Flash to disable the flash control register access
	 * Recommended to protect the FLASH MEMORY against possible unwanted operation
	 */
	HAL_FLASH_Lock();
  /* Jump to application code */
  flash_jump_to_app();
}

/* USER CODE END 0 */

/**
  * @brief  The application entry point.
  * @retval int
  */
int main(void)
{

  /* USER CODE BEGIN 1 */

  /* USER CODE END 1 */

  /* MCU Configuration--------------------------------------------------------*/

  /* Reset of all peripherals, Initializes the Flash interface and the Systick. */
  HAL_Init();

  /* USER CODE BEGIN Init */

  /* USER CODE END Init */

  /* Configure the system clock */
  SystemClock_Config();

  /* USER CODE BEGIN SysInit */

  /* USER CODE END SysInit */

  /* Initialize all configured peripherals */
  MX_GPIO_Init();
  MX_USART1_UART_Init();
  /* USER CODE BEGIN 2 */
  uart0_write_str_blocking("[BOOTLOADER] Compiled: ");
  uart0_write_str_blocking(__DATE__);
  uart0_write_str_blocking(" ");
  uart0_write_str_blocking(__TIME__);
  uart0_write_str_blocking("\r\n");
  if (fw_header->magic == 0xAAAAAAAA){
    char tmp[128];
    uart0_write_str_blocking("[BOOTLOADER] DO NOT TURN OFF WHILE UPDATING!\r\n");
    if (fw_header->fw_type == 1){
      uart0_write_str_blocking("[BOOTLOADER] New app firmware update found!\r\nUpdating to ");
      uart0_write_str_blocking(u8_to_str(fw_header->version[0], tmp));
      uart0_write_str_blocking(".");
      uart0_write_str_blocking(u8_to_str(fw_header->version[1], tmp));
      uart0_write_str_blocking(".");
      uart0_write_str_blocking(u8_to_str(fw_header->version[2], tmp));
      uart0_write_str_blocking("...\r\n");

      update(FW_UPDATE_APP);
    }
    else if(fw_header->fw_type == 2){
      uart0_write_str_blocking("[BOOTLOADER] New bootloader firmware update found!\r\nUpdating to ");
      uart0_write_str_blocking(u8_to_str(fw_header->version[0], tmp));
      uart0_write_str_blocking(".");
      uart0_write_str_blocking(u8_to_str(fw_header->version[1], tmp));
      uart0_write_str_blocking(".");
      uart0_write_str_blocking(u8_to_str(fw_header->version[2], tmp));
      uart0_write_str_blocking("...\r\n");

      update(FW_UPDATE_BOOT);
    }
    else{
      uart0_write_str_blocking("[BOOTLOADER] ERROR: Unknown fw_type ");
      uart0_write_str_blocking(u8_to_str(fw_header->fw_type, tmp));
      uart0_write_str_blocking(" !\r\n Jumping to application code...\r\n");
    }
  }
  /* Jump to application code */
  flash_jump_to_app();

  /* USER CODE END 2 */

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */
  while (1)
  {
    /* USER CODE END WHILE */

    /* USER CODE BEGIN 3 */
  }
  /* USER CODE END 3 */
}

/**
  * @brief System Clock Configuration
  * @retval None
  */
void SystemClock_Config(void)
{
  RCC_OscInitTypeDef RCC_OscInitStruct = {0};
  RCC_ClkInitTypeDef RCC_ClkInitStruct = {0};

  /** Initializes the RCC Oscillators according to the specified parameters
  * in the RCC_OscInitTypeDef structure.
  */
  RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSE;
  RCC_OscInitStruct.HSEState = RCC_HSE_BYPASS;
  RCC_OscInitStruct.HSEPredivValue = RCC_HSE_PREDIV_DIV1;
  RCC_OscInitStruct.HSIState = RCC_HSI_ON;
  RCC_OscInitStruct.PLL.PLLState = RCC_PLL_ON;
  RCC_OscInitStruct.PLL.PLLSource = RCC_PLLSOURCE_HSE;
  RCC_OscInitStruct.PLL.PLLMUL = RCC_PLL_MUL9;
  if (HAL_RCC_OscConfig(&RCC_OscInitStruct) != HAL_OK)
  {
    Error_Handler();
  }

  /** Initializes the CPU, AHB and APB buses clocks
  */
  RCC_ClkInitStruct.ClockType = RCC_CLOCKTYPE_HCLK|RCC_CLOCKTYPE_SYSCLK
                              |RCC_CLOCKTYPE_PCLK1|RCC_CLOCKTYPE_PCLK2;
  RCC_ClkInitStruct.SYSCLKSource = RCC_SYSCLKSOURCE_PLLCLK;
  RCC_ClkInitStruct.AHBCLKDivider = RCC_SYSCLK_DIV1;
  RCC_ClkInitStruct.APB1CLKDivider = RCC_HCLK_DIV2;
  RCC_ClkInitStruct.APB2CLKDivider = RCC_HCLK_DIV1;

  if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_2) != HAL_OK)
  {
    Error_Handler();
  }
}

/**
  * @brief USART1 Initialization Function
  * @param None
  * @retval None
  */
static void MX_USART1_UART_Init(void)
{

  /* USER CODE BEGIN USART1_Init 0 */

  /* USER CODE END USART1_Init 0 */

  /* USER CODE BEGIN USART1_Init 1 */

  /* USER CODE END USART1_Init 1 */
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
    Error_Handler();
  }
  /* USER CODE BEGIN USART1_Init 2 */

  /* USER CODE END USART1_Init 2 */

}

/**
  * @brief GPIO Initialization Function
  * @param None
  * @retval None
  */
static void MX_GPIO_Init(void)
{
/* USER CODE BEGIN MX_GPIO_Init_1 */
/* USER CODE END MX_GPIO_Init_1 */

  /* GPIO Ports Clock Enable */
  __HAL_RCC_GPIOD_CLK_ENABLE();
  __HAL_RCC_GPIOA_CLK_ENABLE();

/* USER CODE BEGIN MX_GPIO_Init_2 */
/* USER CODE END MX_GPIO_Init_2 */
}

/* USER CODE BEGIN 4 */

/* USER CODE END 4 */

/**
  * @brief  This function is executed in case of error occurrence.
  * @retval None
  */
void Error_Handler(void)
{
  /* USER CODE BEGIN Error_Handler_Debug */
  /* User can add his own implementation to report the HAL error return state */
  __disable_irq();
  while (1)
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
void assert_failed(uint8_t *file, uint32_t line)
{
  /* USER CODE BEGIN 6 */
  /* User can add his own implementation to report the file name and line number,
     ex: printf("Wrong parameters value: file %s on line %d\r\n", file, line) */
  /* USER CODE END 6 */
}
#endif /* USE_FULL_ASSERT */
