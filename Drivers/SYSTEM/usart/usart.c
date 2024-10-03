/**
 ****************************************************************************************************
 * @file        usart.c
 * @author      ����ԭ���Ŷ�(ALIENTEK)
 * @version     V1.0
 * @date        2020-04-20
 * @brief       ���ڳ�ʼ������(һ���Ǵ���1)��֧��printf
 * @license     Copyright (c) 2020-2032, ������������ӿƼ����޹�˾
 ****************************************************************************************************
 * @attention
 *
 * ʵ��ƽ̨:����ԭ�� STM32F103������
 * ������Ƶ:www.yuanzige.com
 * ������̳:www.openedv.com
 * ��˾��ַ:www.alientek.com
 * �����ַ:openedv.taobao.com
 *
 * �޸�˵��
 * V1.0 20211103
 * ��һ�η���
 *
 ****************************************************************************************************
 */

#include "./SYSTEM/sys/sys.h"
#include "./SYSTEM/usart/usart.h"
#include "./BSP/bldc.h"

/******************************************************************************************/
/* �������´���, ֧��printf����, ������Ҫѡ��use MicroLIB */

#if 1

#if (__ARMCC_VERSION >= 6010050)            /* ʹ��AC6������ʱ */
__asm(".global __use_no_semihosting\n\t");  /* ������ʹ�ð�����ģʽ */
__asm(".global __ARM_use_no_argv \n\t");    /* AC6����Ҫ����main����Ϊ�޲�����ʽ�����򲿷����̿��ܳ��ְ�����ģʽ */

#else
/* ʹ��AC5������ʱ, Ҫ�����ﶨ��__FILE �� ��ʹ�ð�����ģʽ */
#pragma import(__use_no_semihosting)

struct __FILE
{
    int handle;
    /* Whatever you require here. If the only file you are using is */
    /* standard output using printf() for debugging, no file handling */
    /* is required. */
};

#endif

/* ��ʹ�ð�����ģʽ��������Ҫ�ض���_ttywrch\_sys_exit\_sys_command_string����,��ͬʱ����AC6��AC5ģʽ */
int _ttywrch(int ch)
{
    ch = ch;
    return ch;
}

/* ����_sys_exit()�Ա���ʹ�ð�����ģʽ */
void _sys_exit(int x)
{
    x = x;
}

char *_sys_command_string(char *cmd, int len)
{
    return NULL;
}


/* FILE �� stdio.h���涨��. */
FILE __stdout;

/* MDK����Ҫ�ض���fputc����, printf�������ջ�ͨ������fputc����ַ��������� */
int fputc(int ch, FILE *f)
{
    while ((USART_UX->SR & 0X40) == 0);     /* �ȴ���һ���ַ�������� */

    USART_UX->DR = (uint8_t)ch;             /* ��Ҫ���͵��ַ� ch д�뵽DR�Ĵ��� */
    return ch;
}
#endif
/******************************************************************************************/

#if USART_EN_RX /*���ʹ���˽���*/


volatile struct_send   g_esp32_struct = {0};
/* ���ջ���, ���USART_REC_LEN���ֽ�. */
volatile uint8_t g_usart_rx_buf[USART_REC_LEN];

/*  ����״̬
 *  bit7��      ������ɱ�־
 *  bit3��      ���յ�0x0d
*/
volatile uint8_t g_usart_rx_sta = 0;
/*  ���յ������ֽ� */
volatile uint8_t g_usart_rx_num = 0;
volatile uint32_t esp32_lose_time = 0;
volatile uint8_t g_rx_buffer[RXBUFFERSIZE];  /* HAL��ʹ�õĴ��ڽ��ջ��� */

UART_HandleTypeDef g_uart1_handle;  /* UART��� */

/**
 * @brief       ����X��ʼ������
 * @param       baudrate: ������, �����Լ���Ҫ���ò�����ֵ
 * @note        ע��: ����������ȷ��ʱ��Դ, ���򴮿ڲ����ʾͻ������쳣.
 *              �����USART��ʱ��Դ��sys_stm32_clock_init()�������Ѿ����ù���.
 * @retval      ��
 */
void usart_init(uint32_t baudrate)
{
    /*UART ��ʼ������*/
    g_uart1_handle.Instance = USART_UX;                                       /* USART_UX */
    g_uart1_handle.Init.BaudRate = baudrate;                                  /* ������ */
    g_uart1_handle.Init.WordLength = UART_WORDLENGTH_8B;                      /* �ֳ�Ϊ8λ���ݸ�ʽ */
    g_uart1_handle.Init.StopBits = UART_STOPBITS_1;                           /* һ��ֹͣλ */
    g_uart1_handle.Init.Parity = UART_PARITY_NONE;                            /* ����żУ��λ */
    g_uart1_handle.Init.HwFlowCtl = UART_HWCONTROL_NONE;                      /* ��Ӳ������ */
    g_uart1_handle.Init.Mode = UART_MODE_TX_RX;                               /* �շ�ģʽ */
    HAL_UART_Init(&g_uart1_handle);                                           /* HAL_UART_Init()��ʹ��UART1 */

    /* �ú����Ὺ�������жϣ���־λUART_IT_RXNE���������ý��ջ����Լ����ջ��������������� */
    HAL_UART_Receive_IT(&g_uart1_handle, (uint8_t *)g_rx_buffer, RXBUFFERSIZE); 
}

/**
 * @brief       UART�ײ��ʼ������
 * @param       huart: UART�������ָ��
 * @note        �˺����ᱻHAL_UART_Init()����
 *              ���ʱ��ʹ�ܣ��������ã��ж�����
 * @retval      ��
 */
void HAL_UART_MspInit(UART_HandleTypeDef *huart)
{
    GPIO_InitTypeDef gpio_init_struct;

    if (huart->Instance == USART_UX)
    {
        USART_TX_GPIO_CLK_ENABLE();                             /* ʹ�ܴ���TX��ʱ�� */
        USART_RX_GPIO_CLK_ENABLE();                             /* ʹ�ܴ���RX��ʱ�� */
        USART_UX_CLK_ENABLE();                                  /* ʹ�ܴ���ʱ�� */
        __HAL_RCC_AFIO_CLK_ENABLE();                           /* ʹ��AFIOʱ�� */
        __HAL_AFIO_REMAP_USART1_ENABLE();                       /* USART1��ӳ��AFIO��GPIOB6,7 */

        gpio_init_struct.Pin = USART_TX_GPIO_PIN;               /* ���ڷ������ź� */
        gpio_init_struct.Mode = GPIO_MODE_AF_PP;                /* ����������� */
        gpio_init_struct.Pull = GPIO_PULLUP;                    /* ���� */
        gpio_init_struct.Speed = GPIO_SPEED_FREQ_HIGH;          /* IO�ٶ�����Ϊ���� */
        HAL_GPIO_Init(USART_TX_GPIO_PORT, &gpio_init_struct);
                
        gpio_init_struct.Pin = USART_RX_GPIO_PIN;               /* ����RX�� ģʽ���� */
        gpio_init_struct.Mode = GPIO_MODE_AF_INPUT;    
        HAL_GPIO_Init(USART_RX_GPIO_PORT, &gpio_init_struct);   /* ����RX�� �������ó�����ģʽ */
        
#if USART_EN_RX
        HAL_NVIC_EnableIRQ(USART_UX_IRQn);                      /* ʹ��USART1�ж�ͨ�� */
        HAL_NVIC_SetPriority(USART_UX_IRQn, 2, 0);             
#endif
    }
}

/**
 * @brief       �������ݽ��ջص�����
                ���ݴ������������
 * @param       huart:���ھ��
 * @retval      ��
 */
void HAL_UART_RxCpltCallback(UART_HandleTypeDef *huart)
{
    if (huart->Instance == USART_UX)                    /* ����Ǵ���1 */
    {
        if (g_rx_buffer[0] == 0x7b || g_usart_rx_num > 0)                /* ���յ���0x0d�����س����� */
        {
            g_usart_rx_buf[g_usart_rx_num] = g_rx_buffer[0];
            if(g_usart_rx_num == sizeof(struct_send)-1)
            {
                if(g_rx_buffer[0] == 0x7d && g_esp32_struct.sta == 0)  //���յ�֡β
                {
                    g_esp32_struct = *(struct_send*)&g_usart_rx_buf[0];
                    g_esp32_struct.sta = 1;
                    esp32_lose_time = time_num;
                }
                g_usart_rx_num = 0;             //��������ʱΪ��������
            }else
            {
                g_usart_rx_num++;
            }
            
        }else
        {
            g_usart_rx_num = 0;    //��������
        }
    }
}

/**
 * @brief       ����X�жϷ�����
                ע��,��ȡUSARTx->SR�ܱ���Ī������Ĵ���
 * @param       ��
 * @retval      ��
 */
void USART_UX_IRQHandler(void)
{

    HAL_UART_IRQHandler(&g_uart1_handle);                               /* ����HAL���жϴ����ú��� */
    
       /*   ���Է����и��ʳ����ж��������   */
    if(__HAL_UART_GET_FLAG(&g_uart1_handle,UART_FLAG_ORE) != RESET)
    {
        __HAL_UART_CLEAR_OREFLAG(&g_uart1_handle);
    }
    
    HAL_UART_Receive_IT(&g_uart1_handle, (uint8_t *)g_rx_buffer, RXBUFFERSIZE);
}

/**
 * @brief       ���ؽ��յ�������
 * @param       ��Ļ������׵�ַ
 * @param       ��Ļ�������С
 * @retval      ���ر��ν��ܵ����ֽ���,Ϊ0���ǽ�������ʧ��
 */
uint8_t usart_receive_data(uint8_t *buf, uint16_t buf_size)
{
    uint8_t sta;
    if(g_usart_rx_sta == 0x80)
    {
        if(buf_size < g_usart_rx_num)
        {
            return 0;
        }else
        {
            uint16_t i;
            for(i=0;i<g_usart_rx_num;i++)
            {
                buf[i] = g_usart_rx_buf[i];           /*  ��������  */
            }
            sta = g_usart_rx_num;
        }
    }else
    {
        return 0;
    }
    
    return sta;
}
/**
 * @brief       ����len���ֽ�(��Ҫ�Լ����Ͻ�����\n)
 * @param       buf     : �������׵�ַ
 * @param       len     : ���͵��ֽ���(���200�ֽ�)
 * @retval      ��
 */
void usart_send_data(uint8_t *buf, uint8_t len)
{
    HAL_UART_Transmit(&g_uart1_handle, buf, len, 1000); /* �������� */
    while(__HAL_UART_GET_FLAG(&g_uart1_handle,UART_FLAG_TC) != SET );     /* �ȴ�������� */
}
#endif
