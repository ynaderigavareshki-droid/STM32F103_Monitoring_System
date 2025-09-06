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
#include "FreeRTOS.h"
#include "task.h"
#include "queue.h"
#include "semphr.h"
#include "stdio.h"
#include "string.h"
/* USER CODE END Includes */

/* Private typedef -----------------------------------------------------------*/
/* USER CODE BEGIN PTD */
#define UART_TIMEOUT_MS 500            // UART transmission timeout in milliseconds
#define MAX_TASKS 10                   // Maximum number of tasks for monitoring
#define CRITICAL_STACK_THRESHOLD 100   // Stack warning threshold in bytes
#define CRITICAL_HEAP_THRESHOLD  1024  // Heap warning threshold in bytes
#define MAX_UART_MSG_LEN 120           // Maximum UART message length
#define ADC_BUFFER_SIZE 10             // ADC buffer size for DMA

/**
  * @brief UART message structure
  */
typedef struct {
    char data[MAX_UART_MSG_LEN];
    size_t len;
} UART_Message_t;

/**
  * @brief Log type enumeration
  */
typedef enum {
    LOG_TYPE_SYS,     // System log
    LOG_TYPE_ADC,     // ADC log	
    LOG_TYPE_WARNING, // Warning log
    LOG_TYPE_ERROR    // Error log
} LogType_t;

/**
  * @brief Error information structure
  */
typedef struct {
    uint8_t error_code;    // Error code
    char task_name[20];    // Task name where error occurred
} ErrorInfo_t;
/* USER CODE END PTD */

/* Private define ------------------------------------------------------------*/
/* USER CODE BEGIN PD */
#define DEBUG_MONITORRING    // Enable system monitoring debug
#define DEBUG_ERROR          // Enable error debugging
#define DEBUG_MODE           // Enable debug mode (comment to disable)
/* USER CODE END PD */

/* Private macro -------------------------------------------------------------*/
/* USER CODE BEGIN PM */
// LED GPIO Definitions
#define LED_ERROR_GPIO_Port			GPIOC
#define LED_ERROR_Pin       		GPIO_PIN_9

#define LED_Activity_GPIO_Port 	GPIOC
#define LED_Activity_Pin       	GPIO_PIN_7

#define LED_ADC_GPIO_Port 			GPIOC
#define LED_ADC_Pin       			GPIO_PIN_8
/* USER CODE END PM */

/* Private variables ---------------------------------------------------------*/
RTC_HandleTypeDef hrtc;

/* USER CODE BEGIN PV */
// Peripheral Handlers
UART_HandleTypeDef huart1;
IWDG_HandleTypeDef hiwdg;
ADC_HandleTypeDef hadc1;
DMA_HandleTypeDef hdma_adc1;
DMA_HandleTypeDef hdma_usart1_tx;

// FreeRTOS Objects
QueueHandle_t xErrorQueue;            // Error message queue
QueueHandle_t xUARTQueue;             // UART transmission queue
SemaphoreHandle_t xDMASemaphore;      // DMA access semaphore
SemaphoreHandle_t xUARTMutex;         // UART access mutex
TaskHandle_t xAdcTaskHandle;          // ADC task handle

// ADC Variables
__IO uint16_t adc_buffer[ADC_BUFFER_SIZE] = {0};  // ADC DMA buffer

// UART Transmission State
typedef struct {
    UART_Message_t message;
    uint8_t is_in_progress;
} UART_TransmissionState_t;

static UART_TransmissionState_t uart_tx_state = {0};
/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
static void MX_GPIO_Init(void);
static void MX_RTC_Init(void);
/* USER CODE BEGIN PFP */
// Peripheral Initialization
void MX_USART1_UART_Init(void); 
void MX_IWDG_Init(void);
static void MX_ADC1_Init(void);
static void MX_DMA_Init(void);

// FreeRTOS Tasks
void vUARTTask(void *pvParameters);
void vSystemMonitorTask(void *pvParameters);
void vErrorHandlerTask(void *pvParameters);
void ADC_Task(void *argument);

// Utility Functions
void get_rtc_time(char *buffer, size_t size);
void uart_send(LogType_t log_type, const char *data);
/* USER CODE END PFP */

/* Private user code ---------------------------------------------------------*/
/* USER CODE BEGIN 0 */

/* USER CODE END 0 */

/**
  * @brief  The application entry point.
  * @retval int
  */
int main(void)
{
  /* MCU Configuration--------------------------------------------------------*/
  HAL_Init();
  SystemClock_Config();

  /* Initialize all configured peripherals */
  MX_GPIO_Init();
  MX_RTC_Init();
  MX_IWDG_Init();
  MX_DMA_Init();  
  MX_USART1_UART_Init();
  MX_ADC1_Init();
  
  /* USER CODE BEGIN 2 */ 
  // Create DMA semaphore
  xDMASemaphore = xSemaphoreCreateBinary();
  if(xDMASemaphore != NULL) {
    xSemaphoreGive(xDMASemaphore);
  }
  
  // Create UART mutex
  xUARTMutex = xSemaphoreCreateMutex();
  
  // Create FreeRTOS queues
  xErrorQueue = xQueueCreate(10, sizeof(ErrorInfo_t));
  xUARTQueue = xQueueCreate(20, sizeof(UART_Message_t));
  
  // Create system tasks with priorities
  // Priority 6: Highest priority for error handling
  xTaskCreate(vErrorHandlerTask, "Error", 256, NULL, 50, NULL);   
  
  // Priority 7: UART communication
  xTaskCreate(vUARTTask, "UART", 512, NULL, 49, NULL);            
  
  // Priority 8: System monitoring (includes watchdog refresh)
  xTaskCreate(vSystemMonitorTask, "SysMon", 256, NULL, 48, NULL); 
  
  // Priority 9: ADC processing (lowest priority)
  xTaskCreate(ADC_Task, "ADC", 256, NULL, 47, &xAdcTaskHandle);   

  // Start ADC with DMA
  if(HAL_ADC_Start_DMA(&hadc1, (uint32_t*)adc_buffer, ADC_BUFFER_SIZE) != HAL_OK) {
    Error_Handler();
  }
  
  // Start FreeRTOS scheduler
  vTaskStartScheduler();
  /* USER CODE END 2 */

  /* Infinite loop */
  while (1)
  {
    // Should never reach here if scheduler is running
  }
}

/* USER CODE BEGIN 4 */
/**
  * @brief Get current RTC time as formatted string
  * @param buffer: Output buffer for time string
  * @param size: Size of output buffer
  */
void get_rtc_time(char *buffer, size_t size) {
    RTC_DateTypeDef sDate = {0};  
    RTC_TimeTypeDef sTime = {0};

    // Must call GetDate before GetTime per STM32 documentation
    HAL_RTC_GetDate(&hrtc, &sDate, RTC_FORMAT_BIN);    
    HAL_RTC_GetTime(&hrtc, &sTime, RTC_FORMAT_BIN);
    
    // Format time as HH:MM:SS
    snprintf(buffer, size, "%02d:%02d:%02d", 
             sTime.Hours, sTime.Minutes, sTime.Seconds);
}

/**
  * @brief Send log message via UART
  * @param log_type: Type of log message
  * @param data: Log message content
  */
void uart_send(LogType_t log_type, const char *data) {
    UART_Message_t msg = {0};
    char timeStr[10] = {0};      
    const char *prefix = "";
    
    // Set log prefix based on type
    switch (log_type) {
        case LOG_TYPE_SYS:     prefix = "SYS";   break;
        case LOG_TYPE_ADC:     prefix = "ADC";   break;			
        case LOG_TYPE_WARNING: prefix = "WARN";  break;
        case LOG_TYPE_ERROR:   prefix = "ERR";   break;
        default:               prefix = "UNK";   break;
    }
          
    // Get current time
    get_rtc_time(timeStr, sizeof(timeStr));

    // Format message: [TIME] [PREFIX] message
    int written = snprintf(msg.data, sizeof(msg.data), "\r\n[%s][%s]%s", timeStr, prefix, data);
    
    // Calculate length and prevent buffer overflow
    if (written < 0) {
        // Error in formatting
        return;
    }
    
    msg.len = (size_t)written;
    if(msg.len >= MAX_UART_MSG_LEN) {
        msg.len = MAX_UART_MSG_LEN - 1;
        msg.data[msg.len] = '\0';
    }
    
    // Send to UART queue (block indefinitely if queue is full)
    if(xQueueSendToBack(xUARTQueue, &msg, portMAX_DELAY) != pdPASS) {
        // Signal error if queue is full
        HAL_GPIO_WritePin(LED_ERROR_GPIO_Port, LED_ERROR_Pin, GPIO_PIN_SET);
    }
}

/**
  * @brief UART transmission task
  * @param pvParameters: Task parameters (unused)
  */
void vUARTTask(void *pvParameters) {
  UART_Message_t msg;
  
  for(;;) {
    // Refresh watchdog timer 
    HAL_IWDG_Refresh(&hiwdg);
    
    // Wait for message in queue
    if(xQueueReceive(xUARTQueue, &msg, portMAX_DELAY) == pdTRUE) {
      // Acquire UART mutex to ensure exclusive access
      if(xSemaphoreTake(xUARTMutex, portMAX_DELAY) == pdTRUE) {
        // Use interrupt-based transmission for better reliability
        if(HAL_UART_Transmit(&huart1, (uint8_t*)msg.data, msg.len, UART_TIMEOUT_MS) != HAL_OK) {
          // Transmission failed
          ErrorInfo_t error_info = {.error_code = 0xE0, .task_name = "UART"};
          xQueueSend(xErrorQueue, &error_info, 0);
        }
        
        // Release UART mutex
        xSemaphoreGive(xUARTMutex);
      } else {
        // Semaphore take failed
        ErrorInfo_t error_info = {.error_code = 0xE1, .task_name = "UART"};
        xQueueSend(xErrorQueue, &error_info, 0);
      }
      
      // Small delay to allow UART to complete transmission
      vTaskDelay(pdMS_TO_TICKS(10));
    }
  }
}

/**
  * @brief System monitoring task (includes watchdog refresh)
  * @param pvParameters: Task parameters (unused)
  */
void vSystemMonitorTask(void *pvParameters) {
    const TickType_t xFrequency = pdMS_TO_TICKS(2000);
    TickType_t xLastWakeTime = xTaskGetTickCount();
    
    static TaskStatus_t taskStats[MAX_TASKS];
    static uint8_t criticalTasks[MAX_TASKS] = {0};
    static uint8_t firstRun = 1;

    for(;;) {
        // Refresh watchdog timer 
        HAL_IWDG_Refresh(&hiwdg);
      
        // Toggle activity indicator LED 
        HAL_GPIO_TogglePin(LED_Activity_GPIO_Port, LED_Activity_Pin);
        
        #ifdef DEBUG_MONITORRING
            // Get system health information
            uint32_t currentHeap = xPortGetFreeHeapSize();
            UBaseType_t numTasks = uxTaskGetNumberOfTasks();
            
            // Limit to max tasks we can handle
            if(numTasks > MAX_TASKS) numTasks = MAX_TASKS;
            
            // Get task state information
            numTasks = uxTaskGetSystemState(taskStats, numTasks, NULL);
            
            uint8_t anyCritical = 0;
            for(int i = 0; i < numTasks; i++) {
                // Calculate free stack space
                UBaseType_t stackFree = taskStats[i].usStackHighWaterMark * sizeof(StackType_t);
                
                // Check for critical stack usage
                if(stackFree < CRITICAL_STACK_THRESHOLD) {
                    if(!criticalTasks[i] || firstRun) {
                        char alert[80];
                      
                        snprintf(alert, sizeof(alert),"CRITICAL TASK: %s | Stack: %lub < %ub",
                          taskStats[i].pcTaskName,(unsigned long)stackFree,CRITICAL_STACK_THRESHOLD);
                        #ifdef DEBUG_MODE
                          uart_send(LOG_TYPE_ERROR, alert);
                        #endif
                      
                        criticalTasks[i] = 1;
                        anyCritical = 1;
                    }
                } else if(criticalTasks[i]) {
                    // Task recovered from critical state
                    char recovery[70];
                  
                    snprintf(recovery, sizeof(recovery),"RECOVERED: %s | Stack: %lub",
                        taskStats[i].pcTaskName,(unsigned long)stackFree);
                    #ifdef DEBUG_MODE
                      uart_send(LOG_TYPE_SYS, recovery);
                    #endif
                    
                    criticalTasks[i] = 0;
                }
            }
            
            // Check for critical heap usage
            if(currentHeap < CRITICAL_HEAP_THRESHOLD) {
                char alert[50];
              
                snprintf(alert, sizeof(alert), "CRITICAL HEAP! %ub < %ub", 
                    currentHeap,CRITICAL_HEAP_THRESHOLD);
                
                #ifdef DEBUG_MODE
                  uart_send(LOG_TYPE_ERROR, alert);
                #endif
            }
            
            // Periodically send health report
            static TickType_t lastHealthReport = 0;
            if(firstRun || anyCritical || (xTaskGetTickCount() - lastHealthReport > pdMS_TO_TICKS(1000))) {
                char health[80];
              
                snprintf(health, sizeof(health),"System Health: Tasks:%lu | Heap:%ub/%ub", 
                    (unsigned long)numTasks,currentHeap,CRITICAL_HEAP_THRESHOLD);
                #ifdef DEBUG_MODE
                  uart_send(LOG_TYPE_SYS, health);
                #endif
                
                lastHealthReport = xTaskGetTickCount();
                firstRun = 0;
            }
        #endif
        
        // Maintain fixed execution period
        vTaskDelayUntil(&xLastWakeTime, xFrequency);
    }
}

/**
  * @brief ADC processing task
  * @param argument: Task parameters (unused)
  */
void ADC_Task(void *argument) {
    uint32_t running_sum = 0;  // Moving sum of ADC values
    uint16_t average = 0;      // Calculated average
    uint32_t last_value = 0;   // Last value for change detection
 
    const TickType_t xDelayADC = pdMS_TO_TICKS(10000); // 10 second delay
    TickType_t xLastWakeTime = xTaskGetTickCount();
  
    for(;;) {
        // Refresh watchdog timer 
        HAL_IWDG_Refresh(&hiwdg);
        // Wait for conversion complete notification
        ulTaskNotifyTake(pdTRUE, portMAX_DELAY);
        
        // Calculate sum of ADC values
        running_sum = 0;
        for(int i = 0; i < ADC_BUFFER_SIZE; i++) {
            running_sum += adc_buffer[i];
        }
        
        // Calculate moving average
        average = running_sum / ADC_BUFFER_SIZE;
        
        #ifdef DEBUG_MODE
          // Send ADC value every 10 seconds
          if(xTaskGetTickCount() - xLastWakeTime >= xDelayADC) {
            xLastWakeTime = xTaskGetTickCount();
            char ADC_String[30];
            snprintf(ADC_String, sizeof(ADC_String), "Value: %u", average);
            uart_send(LOG_TYPE_ADC, ADC_String);
          }
        #endif
        
        
        // Check if ADC value has changed
        if(average != last_value) {
            last_value = average;
            // Clear error indicator 
            HAL_GPIO_WritePin(LED_ADC_GPIO_Port, LED_ADC_Pin, GPIO_PIN_RESET);
        } else {
            // Set error indicator if value is static
            HAL_GPIO_WritePin(LED_ADC_GPIO_Port, LED_ADC_Pin, GPIO_PIN_SET);
        }
    }
}

/**
  * @brief Error handling task
  * @param pvParameters: Task parameters (unused)
  */
void vErrorHandlerTask(void *pvParameters) {
    ErrorInfo_t errorInfo;
    char text[MAX_UART_MSG_LEN] = {0};
    
    for(;;) {
        // Refresh watchdog timer 
        HAL_IWDG_Refresh(&hiwdg);
        // Wait for error notifications
        if(xQueueReceive(xErrorQueue, &errorInfo, portMAX_DELAY)) {
            // Visual indicator (LED on)
            HAL_GPIO_WritePin(LED_ERROR_GPIO_Port, LED_ERROR_Pin, GPIO_PIN_SET);
            
            #ifdef DEBUG_ERROR
                // Send error code to UART
                snprintf(text, sizeof(text), "Error Code: 0x%02X in Task: %s", 
                  errorInfo.error_code, errorInfo.task_name);
                #ifdef DEBUG_MODE
                  uart_send(LOG_TYPE_ERROR, text);
                #endif
            #endif
            
            // Keep error state for 2 seconds
            vTaskDelay(pdMS_TO_TICKS(2000));
            
            // Clear error indicator
            HAL_GPIO_WritePin(LED_ERROR_GPIO_Port, LED_ERROR_Pin, GPIO_PIN_RESET);
        }
    }
}

/**
  * @brief FreeRTOS stack overflow hook
  * @param xTask: Handle of the offending task
  * @param pcTaskName: Name of the offending task
  */
void vApplicationStackOverflowHook(TaskHandle_t xTask, char *pcTaskName) {
    ErrorInfo_t error = {.error_code = 0xFF};  // Stack overflow error code
    strncpy(error.task_name, pcTaskName, sizeof(error.task_name)-1);
    error.task_name[sizeof(error.task_name)-1] = '\0';
    
    // Send error info to queue from ISR context
    xQueueSendFromISR(xErrorQueue, &error, NULL);  
    
    // Critical error - blink LED rapidly
    for(;;) {
        HAL_GPIO_TogglePin(LED_ERROR_GPIO_Port, LED_ERROR_Pin);
        for(int i = 0; i < 1000000; i++); //Delay
    }
}

/**
  * @brief FreeRTOS idle hook - puts CPU to sleep
  */
void vApplicationIdleHook(void) {
    static TickType_t xLastIdleTime;
    TickType_t xNow = xTaskGetTickCount();
    TickType_t xIdleTime = xNow - xLastIdleTime;
    xLastIdleTime = xNow;
    
    // Enter sleep mode after 10 ticks of idle
    if(xIdleTime > 10) {
        __WFI(); // Wait for interrupt
    }
}


/* Callbacks ----------------------------------------------------------*/

/**
  * @brief ADC conversion complete callback
  * @param hadc: ADC handle
  */
void HAL_ADC_ConvCpltCallback(ADC_HandleTypeDef* hadc) {
    BaseType_t xHigherPriorityTaskWoken = pdFALSE;
    
    if(hadc->Instance == ADC1 && xAdcTaskHandle != NULL) {
        // Notify ADC task
        vTaskNotifyGiveFromISR(xAdcTaskHandle, &xHigherPriorityTaskWoken);
        
        // Perform context switch if needed
        portYIELD_FROM_ISR(xHigherPriorityTaskWoken);
    }
}

/**
  * @brief UART Tx complete callback
  * @param huart: UART handle
  */
void HAL_UART_TxCpltCallback(UART_HandleTypeDef *huart) {
    BaseType_t xHigherPriorityTaskWoken = pdFALSE;
    
    if(huart->Instance == USART1) {
        // Release DMA semaphore
        xSemaphoreGiveFromISR(xDMASemaphore, &xHigherPriorityTaskWoken);
        
        // Perform context switch if needed
        portYIELD_FROM_ISR(xHigherPriorityTaskWoken);
    }
}

/**
  * @brief UART error callback
  * @param huart: UART handle
  */
void HAL_UART_ErrorCallback(UART_HandleTypeDef *huart) {
    BaseType_t xHigherPriorityTaskWoken = pdFALSE;
    
    if(huart->Instance == USART1) {
        // Release DMA semaphore on error
        xSemaphoreGiveFromISR(xDMASemaphore, &xHigherPriorityTaskWoken);
        
        // Perform context switch if needed
        portYIELD_FROM_ISR(xHigherPriorityTaskWoken);
    }
}

/* Peripheral Initialization Functions ---------------------------------------*/

/**
  * @brief System Clock Configuration
  * @retval None
  */
void SystemClock_Config(void) {
  RCC_OscInitTypeDef RCC_OscInitStruct = {0};
  RCC_ClkInitTypeDef RCC_ClkInitStruct = {0};
  RCC_PeriphCLKInitTypeDef PeriphClkInit = {0};

  // Configure oscillator
  RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_LSI|RCC_OSCILLATORTYPE_HSE;
  RCC_OscInitStruct.HSEState = RCC_HSE_ON;
  RCC_OscInitStruct.LSIState = RCC_LSI_ON;
  RCC_OscInitStruct.PLL.PLLState = RCC_PLL_NONE;
  if (HAL_RCC_OscConfig(&RCC_OscInitStruct) != HAL_OK) {
    Error_Handler();
  }

  // Configure CPU clocks
  RCC_ClkInitStruct.ClockType = RCC_CLOCKTYPE_HCLK|RCC_CLOCKTYPE_SYSCLK
                              |RCC_CLOCKTYPE_PCLK1|RCC_CLOCKTYPE_PCLK2;
  RCC_ClkInitStruct.SYSCLKSource = RCC_SYSCLKSOURCE_HSE;
  RCC_ClkInitStruct.AHBCLKDivider = RCC_SYSCLK_DIV1;
  RCC_ClkInitStruct.APB1CLKDivider = RCC_HCLK_DIV2;
  RCC_ClkInitStruct.APB2CLKDivider = RCC_HCLK_DIV1;

  if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_0) != HAL_OK) {
    Error_Handler();
  }

  // Configure peripheral clocks
  PeriphClkInit.PeriphClockSelection = RCC_PERIPHCLK_RTC | RCC_PERIPHCLK_ADC;
  PeriphClkInit.RTCClockSelection = RCC_RTCCLKSOURCE_LSI;
  PeriphClkInit.AdcClockSelection = RCC_ADCPCLK2_DIV6; // 12MHz ADC clock
  if (HAL_RCCEx_PeriphCLKConfig(&PeriphClkInit) != HAL_OK) {
    Error_Handler();
  }
}

/**
  * @brief GPIO initialization
  */
static void MX_GPIO_Init(void) {
  GPIO_InitTypeDef GPIO_InitStruct = {0};

  // Enable GPIO clocks
  __HAL_RCC_GPIOA_CLK_ENABLE();
  __HAL_RCC_GPIOB_CLK_ENABLE();
  __HAL_RCC_GPIOC_CLK_ENABLE();

  /* USER CODE BEGIN MX_GPIO_Init_2 */
  
  // Configure error LED (PC9)
  GPIO_InitStruct.Pin = LED_ERROR_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_HIGH;
  HAL_GPIO_Init(LED_ERROR_GPIO_Port, &GPIO_InitStruct);
  HAL_GPIO_WritePin(LED_ERROR_GPIO_Port, LED_ERROR_Pin, GPIO_PIN_RESET);
  
  // Configure Activity LED (PC7) and ADC error LED (PC8)
  GPIO_InitStruct.Pin = LED_Activity_Pin| LED_ADC_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(LED_Activity_GPIO_Port, &GPIO_InitStruct);
  
  // Initialize ADC LEDs
  HAL_GPIO_WritePin(LED_Activity_GPIO_Port, LED_Activity_Pin, GPIO_PIN_RESET); // Activity off
  HAL_GPIO_WritePin(LED_ADC_GPIO_Port, LED_ADC_Pin, GPIO_PIN_RESET); // Error off
  
  // Configure ADC input pin (PB0 - Channel 8)
  GPIO_InitStruct.Pin = GPIO_PIN_0;
  GPIO_InitStruct.Mode = GPIO_MODE_ANALOG;
  HAL_GPIO_Init(GPIOB, &GPIO_InitStruct);

  // Configure TX pin (PA9)
  GPIO_InitStruct.Pin = GPIO_PIN_9;
  GPIO_InitStruct.Mode = GPIO_MODE_AF_PP;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_HIGH;
  HAL_GPIO_Init(GPIOA, &GPIO_InitStruct);

  // Configure RX pin (PA10)
  GPIO_InitStruct.Pin = GPIO_PIN_10;
  GPIO_InitStruct.Mode = GPIO_MODE_INPUT;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  HAL_GPIO_Init(GPIOA, &GPIO_InitStruct);	
  /* USER CODE END MX_GPIO_Init_2 */
}

/**
  * @brief RTC initialization
  */
static void MX_RTC_Init(void) {
  hrtc.Instance = RTC;
  hrtc.Init.AsynchPrediv = RTC_AUTO_1_SECOND;
  hrtc.Init.OutPut = RTC_OUTPUTSOURCE_ALARM;
  if (HAL_RTC_Init(&hrtc) != HAL_OK) {
    Error_Handler();
  }
}

/**
  * @brief DMA initialization
  */
static void MX_DMA_Init(void) {
    __HAL_RCC_DMA1_CLK_ENABLE();
    
    // Configure ADC DMA
    hdma_adc1.Instance = DMA1_Channel1;
    hdma_adc1.Init.Direction = DMA_PERIPH_TO_MEMORY;
    hdma_adc1.Init.PeriphInc = DMA_PINC_DISABLE;
    hdma_adc1.Init.MemInc = DMA_MINC_ENABLE;
    hdma_adc1.Init.PeriphDataAlignment = DMA_PDATAALIGN_HALFWORD;
    hdma_adc1.Init.MemDataAlignment = DMA_MDATAALIGN_HALFWORD;
    hdma_adc1.Init.Mode = DMA_CIRCULAR;
    hdma_adc1.Init.Priority = DMA_PRIORITY_HIGH;
    
    if (HAL_DMA_Init(&hdma_adc1) != HAL_OK) {
        Error_Handler();
    }
    
    // Link DMA to ADC
    __HAL_LINKDMA(&hadc1, DMA_Handle, hdma_adc1);
    
    // Configure DMA interrupts
    HAL_NVIC_SetPriority(DMA1_Channel1_IRQn, 10, 0);
    HAL_NVIC_EnableIRQ(DMA1_Channel1_IRQn);
    
    /* USER CODE BEGIN MX_DMA_Init_2 */
    hdma_usart1_tx.Instance = DMA1_Channel4;
    hdma_usart1_tx.Init.Direction = DMA_MEMORY_TO_PERIPH;
    hdma_usart1_tx.Init.PeriphInc = DMA_PINC_DISABLE;
    hdma_usart1_tx.Init.MemInc = DMA_MINC_ENABLE;
    hdma_usart1_tx.Init.PeriphDataAlignment = DMA_PDATAALIGN_BYTE;
    hdma_usart1_tx.Init.MemDataAlignment = DMA_MDATAALIGN_BYTE;
    hdma_usart1_tx.Init.Mode = DMA_NORMAL;
    hdma_usart1_tx.Init.Priority = DMA_PRIORITY_LOW;

    if (HAL_DMA_Init(&hdma_usart1_tx) != HAL_OK) {
        Error_Handler();
    }

    __HAL_LINKDMA(&huart1, hdmatx, hdma_usart1_tx);

    HAL_NVIC_SetPriority(DMA1_Channel4_IRQn, 3, 0);
    HAL_NVIC_EnableIRQ(DMA1_Channel4_IRQn);
}

/**
  * @brief ADC1 initialization
  */
static void MX_ADC1_Init(void) {
    ADC_ChannelConfTypeDef sConfig = {0};
    
    __HAL_RCC_ADC1_CLK_ENABLE();
    
    hadc1.Instance = ADC1;
    hadc1.Init.ScanConvMode = DISABLE;
    hadc1.Init.ContinuousConvMode = ENABLE;
    hadc1.Init.DiscontinuousConvMode = DISABLE;
    hadc1.Init.ExternalTrigConv = ADC_SOFTWARE_START;
    hadc1.Init.DataAlign = ADC_DATAALIGN_RIGHT;
    hadc1.Init.NbrOfConversion = 1;
    
    if (HAL_ADC_Init(&hadc1) != HAL_OK) {
        Error_Handler();
    }
    
    // Configure ADC channel (PB0 - Channel 8)
    sConfig.Channel = ADC_CHANNEL_8;
    sConfig.Rank = 1;
    sConfig.SamplingTime = ADC_SAMPLETIME_71CYCLES_5;
    if (HAL_ADC_ConfigChannel(&hadc1, &sConfig) != HAL_OK) {
        Error_Handler();
    }
}

/**
  * @brief USART1 initialization
  */
void MX_USART1_UART_Init(void) {
  huart1.Instance = USART1;
  huart1.Init.BaudRate = 115200;
  huart1.Init.WordLength = UART_WORDLENGTH_8B;
  huart1.Init.StopBits = UART_STOPBITS_1;
  huart1.Init.Parity = UART_PARITY_NONE;
  huart1.Init.Mode = UART_MODE_TX_RX;
  huart1.Init.HwFlowCtl = UART_HWCONTROL_NONE;
  huart1.Init.OverSampling = UART_OVERSAMPLING_16;

  __HAL_RCC_USART1_CLK_ENABLE();	
  
  HAL_UART_Init(&huart1);
  
  __HAL_RCC_DMA1_CLK_ENABLE();

  // Configure UART DMA
  hdma_usart1_tx.Instance = DMA1_Channel4;
  hdma_usart1_tx.Init.Direction = DMA_MEMORY_TO_PERIPH;
  hdma_usart1_tx.Init.PeriphInc = DMA_PINC_DISABLE;
  hdma_usart1_tx.Init.MemInc = DMA_MINC_ENABLE;
  hdma_usart1_tx.Init.PeriphDataAlignment = DMA_PDATAALIGN_BYTE;
  hdma_usart1_tx.Init.MemDataAlignment = DMA_MDATAALIGN_BYTE;
  hdma_usart1_tx.Init.Mode = DMA_NORMAL;
  hdma_usart1_tx.Init.Priority = DMA_PRIORITY_LOW;
  
  __HAL_RCC_USART1_CLK_ENABLE(); 
  
  if (HAL_DMA_Init(&hdma_usart1_tx) != HAL_OK) {
    Error_Handler();
  }

  HAL_NVIC_SetPriority(USART1_IRQn, 5, 0);
  HAL_NVIC_EnableIRQ(USART1_IRQn);
}

/**
  * @brief Independent watchdog initialization
  */
void MX_IWDG_Init(void) {
  hiwdg.Instance = IWDG;
  hiwdg.Init.Prescaler = IWDG_PRESCALER_256;
  hiwdg.Init.Reload = 4095; // ~26s timeout
  HAL_IWDG_Init(&hiwdg);
}

/* Interrupt Handlers --------------------------------------------------------*/

/**
  * @brief DMA1 Channel1 interrupt handler (ADC)
  */
void DMA1_Channel1_IRQHandler(void) {
  HAL_DMA_IRQHandler(&hdma_adc1);
}

/**
  * @brief DMA1 Channel4 interrupt handler (UART TX)
  */
void DMA1_Channel4_IRQHandler(void) {
  HAL_DMA_IRQHandler(&hdma_usart1_tx);
}

/**
  * @brief USART1 interrupt handler
  */
void USART1_IRQHandler(void) {
  HAL_UART_IRQHandler(&huart1);
}

/**
  * @brief Period elapsed callback for system tick
  */
void HAL_TIM_PeriodElapsedCallback(TIM_HandleTypeDef *htim) {
  if (htim->Instance == TIM3) {
    HAL_IncTick();
  }
}

/* Error Handling ------------------------------------------------------------*/

/**
  * @brief Error handler - traps system on critical error
  */
void Error_Handler(void) {
  __disable_irq();
  while (1) {
    // Critical error - flash LEDs rapidly
    HAL_GPIO_TogglePin(GPIOC, GPIO_PIN_13 | GPIO_PIN_14 | LED_ERROR_Pin);
    HAL_Delay(100);
  }
}

#ifdef USE_FULL_ASSERT
/**
  * @brief Assert failed callback
  */
void assert_failed(uint8_t *file, uint32_t line) {
  // User can add assert handling here
}
#endif /* USE_FULL_ASSERT */
/* USER CODE END 4 */