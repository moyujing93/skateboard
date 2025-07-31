#include "./SYSTEM/sys/sys.h"
#include "./SYSTEM/usart/usart.h"
#include "./BSP/bldc.h"
#include "stm32f1xx_it.h"
#include <string.h>


/* 如果使用os,则包括下面的头文件即可 */
#if SYS_SUPPORT_OS
#include "includes.h"                               /* os 使用 */
#endif

/******************************************************************************************/
/* 加入以下代码, 支持printf函数, 而不需要选择use MicroLIB */

#if 1
#if (__ARMCC_VERSION >= 6010050)                    /* 使用AC6编译器时 */
__asm(".global __use_no_semihosting\n\t");          /* 声明不使用半主机模式 */
__asm(".global __ARM_use_no_argv \n\t");            /* AC6下需要声明main函数为无参数格式，否则部分例程可能出现半主机模式 */

#else
/* 使用AC5编译器时, 要在这里定义__FILE 和 不使用半主机模式 */
#pragma import(__use_no_semihosting)

struct __FILE
{
    int handle;
    /* Whatever you require here. If the only file you are using is */
    /* standard output using printf() for debugging, no file handling */
    /* is required. */
};

#endif

/* 不使用半主机模式，至少需要重定义_ttywrch\_sys_exit\_sys_command_string函数,以同时兼容AC6和AC5模式 */
int _ttywrch(int ch)
{
    ch = ch;
    return ch;
}

/* 定义_sys_exit()以避免使用半主机模式 */
void _sys_exit(int x)
{
    x = x;
}

char *_sys_command_string(char *cmd, int len)
{
    return NULL;
}

/* FILE 在 stdio.h里面定义. */
FILE __stdout;

/* 重定义fputc函数, printf函数最终会通过调用fputc输出字符串到串口 */
int fputc(int ch, FILE *f)
{
    while ((USART1->SR & 0X40) == 0);               /* 等待上一个字符发送完成 */

    USART1->DR = (uint8_t)ch;                       /* 将要发送的字符 ch 写入到DR寄存器 */
    return ch;
}
#endif
/***********************************************END*******************************************/
    

// 全局状态变量
#define RX_BUFFER_SIZE 200
uint8_t rx_buffer[RX_BUFFER_SIZE];  // DMA接收缓冲区
volatile uint16_t rx_data_len = 0;   // 当前接收数据长度

volatile uint8_t rx_fifo_sta = 0;  
volatile uint8_t rx_fifo_len = 0;  
uint8_t g_usart_rx_buf[2][RX_BUFFER_SIZE];//双缓冲区，因为只需要保证数据是最新状态，就不用环形缓冲区了

DMA_HandleTypeDef hdma_usart1_rx;


volatile struct_read   g_esp32_struct = {0};
volatile uint8_t g_esp32_sta = 0;

UART_HandleTypeDef g_uart1_handle;  /* UART句柄 */

/**
 * @brief       串口X初始化函数
 * @param       
 * @note        串口加DMA搬运数据，在UART空闲中断中复制数据
 *              这样可以支持更高的发送频率
 * @retval      无
 */
void MX_USART1_UART_Init(void)
{
    // 1. 启用时钟
    __HAL_RCC_GPIOB_CLK_ENABLE();    // GPIOB时钟
    __HAL_RCC_USART1_CLK_ENABLE();   // UART1时钟
    __HAL_RCC_AFIO_CLK_ENABLE();     // AFIO时钟（重映射需要）
    __HAL_RCC_DMA1_CLK_ENABLE();     // DMA1时钟

    // 2. 配置UART1重映射（PB6=TX, PB7=RX）
	__HAL_AFIO_REMAP_USART1_ENABLE();// 重映射到PB6/PB7

    // 3. 配置PB6（TX）引脚：复用推挽输出
    GPIO_InitTypeDef GPIO_InitStruct = {0};
    GPIO_InitStruct.Pin = UART_TX_PIN;
    GPIO_InitStruct.Mode = GPIO_MODE_AF_PP;
    GPIO_InitStruct.Pull = GPIO_PULLUP;
    GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_HIGH;
    HAL_GPIO_Init(UART_TX_PORT, &GPIO_InitStruct);

    // 4. 配置PB7（RX）引脚：复用输入（DMA接收需要）
    GPIO_InitStruct.Pin = UART_RX_PIN;
    GPIO_InitStruct.Mode = GPIO_MODE_AF_INPUT;
    GPIO_InitStruct.Pull = GPIO_PULLUP;
    GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_HIGH;
    HAL_GPIO_Init(UART_RX_PORT, &GPIO_InitStruct);

    // 5. 配置UART1基础参数
    g_uart1_handle.Instance = USART1;
    g_uart1_handle.Init.BaudRate = 115200;
    g_uart1_handle.Init.WordLength = UART_WORDLENGTH_8B;
    g_uart1_handle.Init.StopBits = UART_STOPBITS_1;
    g_uart1_handle.Init.Parity = UART_PARITY_NONE;
    g_uart1_handle.Init.Mode = UART_MODE_TX_RX;
    g_uart1_handle.Init.HwFlowCtl = UART_HWCONTROL_NONE;
    g_uart1_handle.Init.OverSampling = UART_OVERSAMPLING_16;
    // 8. 初始化UART（会自动关联DMA）
    HAL_UART_Init(&g_uart1_handle);
	
	
    // 6. 配置DMA参数（UART1_RX使用DMA1通道5）
    hdma_usart1_rx.Instance = DMA1_Channel5;    // UART1_RX固定使用通道5
    hdma_usart1_rx.Init.Direction = DMA_PERIPH_TO_MEMORY;  // 外设→内存
    hdma_usart1_rx.Init.PeriphInc = DMA_PINC_DISABLE;       // 外设地址固定（UART->DR）
    hdma_usart1_rx.Init.MemInc = DMA_MINC_ENABLE;           // 内存地址递增
    hdma_usart1_rx.Init.PeriphDataAlignment = DMA_PDATAALIGN_BYTE;  // 8位对齐
    hdma_usart1_rx.Init.MemDataAlignment = DMA_MDATAALIGN_BYTE;     // 8位对齐
    hdma_usart1_rx.Init.Mode = DMA_NORMAL;                 // 循环模式（持续接收）
    hdma_usart1_rx.Init.Priority = DMA_PRIORITY_MEDIUM;      // 中等优先级
    HAL_DMA_Init(&hdma_usart1_rx);

    // 7. 关联UART和DMA句柄
    __HAL_LINKDMA(&g_uart1_handle, hdmarx, hdma_usart1_rx);

	
    // 10. 启用UART空闲中断（新增）
    __HAL_UART_ENABLE_IT(&g_uart1_handle, UART_IT_IDLE);    // 启用空闲中断
    HAL_NVIC_SetPriority(USART1_IRQn, 1, 0);        // 空闲中断优先级（高于DMA中断）
    HAL_NVIC_EnableIRQ(USART1_IRQn);                // 启用UART中断向量

    // 11. 启动DMA接收（假设接收缓冲区为rx_buffer，大小128字节）
    HAL_UART_Receive_DMA(&g_uart1_handle, rx_buffer, sizeof(rx_buffer));
}


// UART1中断服务函数（处理空闲中断）
void USART1_IRQHandler(void)
{
	DWT->CYCCNT = 0;
    // 处理空闲中断（关键）
    if(__HAL_UART_GET_FLAG(&g_uart1_handle, UART_FLAG_IDLE))
    {
        __HAL_UART_CLEAR_IDLEFLAG(&g_uart1_handle);  // 清除空闲标志（必须操作）
        
        // 计算当前接收数据长度（DMA剩余计数法）
        rx_data_len = RX_BUFFER_SIZE - hdma_usart1_rx.Instance->CNDTR;
        
        if(rx_fifo_sta == 1)
        {
            memcpy(&g_usart_rx_buf[0][0],rx_buffer,rx_data_len);//读data[1]数据时，写data[0]数据
            rx_fifo_sta = 0;
        }else
        {
            memcpy(&g_usart_rx_buf[1][0],rx_buffer,rx_data_len);//读data[0]数据时，写data[1]数据
            rx_fifo_sta = 1;
        }
        if(rx_fifo_len < 2)rx_fifo_len++;
		
		// 直接操作寄存器实现极速重启
        hdma_usart1_rx.Instance->CCR &= ~DMA_CCR_EN;         // 1. 关闭DMA（必要操作）
		hdma_usart1_rx.Instance->CMAR = (uint32_t)rx_buffer;
        hdma_usart1_rx.Instance->CNDTR = RX_BUFFER_SIZE;    // 重置传输计数器
        hdma_usart1_rx.Instance->CCR |= DMA_CCR_EN;   // 直接使能DMA（约2周期）
		HAL_UART_Receive_DMA(&g_uart1_handle, rx_buffer, sizeof(rx_buffer));
    }
	
    HAL_UART_IRQHandler(&g_uart1_handle);  // 调用HAL库UART中断处理函数
	
	if(TEST_SYS_TICK < DWT->CYCCNT) TEST_SYS_TICK = DWT->CYCCNT;
}




/**
 * @brief       返回接收到的数据
 * @param       你的缓冲区首地址
 * @param       你的缓冲区大小
 * @retval      返回本次接受到的字节数,为0则是接受数据失败
 */
uint8_t usart_receive_data(void)
{
    uint8_t sta = 0;
    if (rx_fifo_len > 0)
    {
        rx_fifo_len--;
        uint8_t num = rx_fifo_sta;
        
        for(uint16_t i = 0; i < RX_BUFFER_SIZE;i++)
        {
            if (g_usart_rx_buf[num][i] == 0x7b)
            {
                g_esp32_struct = *(struct_read*)&g_usart_rx_buf[num][i];
                if(g_esp32_struct.bb == 0x7d)  //接收到帧尾,保证数据使用完
                {
                    g_esp32_sta = 1;
                    return 1;
                }
                
            }
        }
    }
    return sta;
}
/**
 * @brief       发送len个字节(需要自己加上结束符\n)
 * @param       buf     : 发送区首地址
 * @param       len     : 发送的字节数(最大200字节)
 * @retval      无
 */
void usart_send_data(uint8_t *buf, uint8_t len)
{
    HAL_UART_Transmit(&g_uart1_handle, buf, len, 1000); /* 发送数据 */
    while(__HAL_UART_GET_FLAG(&g_uart1_handle,UART_FLAG_TC) != SET );     /* 等待发送完成 */
}


