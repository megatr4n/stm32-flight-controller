#include "stm32f1xx_hal.h"
#include "hal/uart_driver.h"
#include "FreeRTOS.h"
#include "task.h"

using namespace HAL;

extern "C" void HardFault_Handler(void) {
    const char* msg = "\r\n[FATAL] HARD FAULT! (System Crashed)\r\n";
    while(*msg) {
        while(!(USART1->SR & USART_SR_TXE));
        USART1->DR = *msg++;
    }
    while(1);
}

extern "C" void xPortSysTickHandler(void);
extern "C" void SysTick_Handler(void) {
    HAL_IncTick();
    
    if (xTaskGetSchedulerState() != taskSCHEDULER_NOT_STARTED) {
        extern void xPortSysTickHandler(void);
        xPortSysTickHandler(); 
    }
}

static void SystemClock_Config(void) {
    RCC_OscInitTypeDef RCC_OscInitStruct = {0};
    RCC_ClkInitTypeDef RCC_ClkInitStruct = {0};

    RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSE;
    RCC_OscInitStruct.HSEState = RCC_HSE_ON;
    RCC_OscInitStruct.HSEPredivValue = RCC_HSE_PREDIV_DIV1;
    RCC_OscInitStruct.PLL.PLLState = RCC_PLL_ON;
    RCC_OscInitStruct.PLL.PLLSource = RCC_PLLSOURCE_HSE;
    RCC_OscInitStruct.PLL.PLLMUL = RCC_PLL_MUL9;
    HAL_RCC_OscConfig(&RCC_OscInitStruct);

    RCC_ClkInitStruct.ClockType = RCC_CLOCKTYPE_HCLK | RCC_CLOCKTYPE_SYSCLK |
                                  RCC_CLOCKTYPE_PCLK1 | RCC_CLOCKTYPE_PCLK2;
    RCC_ClkInitStruct.SYSCLKSource = RCC_SYSCLKSOURCE_PLLCLK;
    RCC_ClkInitStruct.AHBCLKDivider = RCC_SYSCLK_DIV1;
    RCC_ClkInitStruct.APB1CLKDivider = RCC_HCLK_DIV2;
    RCC_ClkInitStruct.APB2CLKDivider = RCC_HCLK_DIV1;
    HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_2);
}

void vBlinkTask(void *pvParameters) {
    (void)pvParameters;
    
    UART_Print("\r\n[TASK] Hello from FreeRTOS Task!\r\n");

    for (;;) {
        UART_Print("[TASK] Working...\r\n");
        for(volatile uint32_t i = 0; i < 2000000; i++) {} 
    }
}

extern "C" void SVC_Handler(void);
extern "C" void PendSV_Handler(void);

int main(void) {
    volatile void* ptr1 = (void*)&SVC_Handler;
    volatile void* ptr2 = (void*)&PendSV_Handler;
    (void)ptr1;
    (void)ptr2;

    HAL_Init();
    SystemClock_Config();
    
    HAL_NVIC_SetPriorityGrouping(NVIC_PRIORITYGROUP_4); 

    UART1_Init();
    UART_Print("\r\nFreeRTOS Final Linker Test\r\n");

    SCB->VTOR = FLASH_BASE | 0x00;

    UART_Print("[MAIN] Creating Task...\r\n");
    BaseType_t taskStatus = xTaskCreate(vBlinkTask, "Blink", 256, NULL, configMAX_PRIORITIES - 1, NULL);
    
    if (taskStatus == pdPASS) {
        UART_Print("[MAIN] Task OK. Handing over to RTOS...\r\n");
    } else {
        UART_Print("[MAIN] Task Creation Failed!\r\n");
    }

    vTaskStartScheduler();

    while (1) {
    }
}