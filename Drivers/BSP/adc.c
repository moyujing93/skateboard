/**
  ******************************************************************************
  * @file    adc.c
  * @author  MYJ
  * @brief   获取所有的ADC,通过DMA并存入相关结构体中,
             读取时判断DMA是否搬运完成后直接使用.
  @verbatim
  ==============================================================================
                        ##### How to use this driver #####
  ==============================================================================
  1.先初始化:adc_dma_init,使用的时DMA循环模式
  2.判断DMA是否搬运完成后直接使用.
  @endverbatim
  ******************************************************************************
  **/
#include "./SYSTEM/delay/delay.h"
#include "./BSP/adc.h"
#include "./BSP/bldc.h"
#include <math.h>

/***************************************** 结构体和变量 *************************************************/
DMA_HandleTypeDef g_dma_adc_handle = {0};                                   /* 定义要搬运ADC数据的DMA句柄 */
ADC_HandleTypeDef g_adc_dma_handle = {0};                                   /* 定义ADC（DMA读取）句柄 */


    /*   DMA使用的结构体    */
GET_ADC_typedef  GetADC_temp = {0};


/***************************************** 函数部分 *************************************************/
/**
 * @brief       ADC1 DMA 初始化函数
 * @param       mar         : 存储器地址
 * @retval      无
 */
void adc1_dma_init(void)
{
    /*   初始化结构体    */
    GPIO_InitTypeDef gpio_init_struct;
    RCC_PeriphCLKInitTypeDef adc_clk_init = {0};
    ADC_ChannelConfTypeDef adc_ch_conf = {0};
    
    /*      时钟使能     */
    __HAL_RCC_ADC1_CLK_ENABLE();                                          /* 使能ADCx时钟 */
    __HAL_RCC_GPIOA_CLK_ENABLE();                                         /* 开启GPIO时钟 */
    __HAL_RCC_DMA1_CLK_ENABLE();                                          /* DMA1时钟使能 */

    /* ADC时钟配置 */
    adc_clk_init.PeriphClockSelection = RCC_PERIPHCLK_ADC;                  /* ADC外设时钟 */
    adc_clk_init.AdcClockSelection = RCC_ADCPCLK2_DIV6;                     /* 分频因子6时钟为72M/6=12MHz */
    HAL_RCCEx_PeriphCLKConfig(&adc_clk_init);                               /* 设置ADC时钟 */

    /* IO引脚配置 */
    gpio_init_struct.Pin   = MA_CUN_PIN | MB_CUN_PIN | V_B_PIN | V_T_PIN;
    gpio_init_struct.Mode  = GPIO_MODE_ANALOG;
    gpio_init_struct.Pull  = GPIO_NOPULL;
    gpio_init_struct.Speed = GPIO_SPEED_FREQ_HIGH;
    HAL_GPIO_Init(GPIOA, &gpio_init_struct);
    
    /* 初始化DMA */
    g_dma_adc_handle.Instance = DMA1_Channel1;                              /* 设置DMA通道 */
    g_dma_adc_handle.Init.Direction = DMA_PERIPH_TO_MEMORY;                 /* 从外设到存储器模式 */
    g_dma_adc_handle.Init.PeriphInc = DMA_PINC_DISABLE;                     /* 外设非增量模式 */
    g_dma_adc_handle.Init.MemInc = DMA_MINC_ENABLE;                         /* 存储器增量模式 */
    g_dma_adc_handle.Init.PeriphDataAlignment = DMA_PDATAALIGN_HALFWORD;    /* 外设数据长度:16位 */
    g_dma_adc_handle.Init.MemDataAlignment = DMA_MDATAALIGN_HALFWORD;       /* 存储器数据长度:16位 */
    g_dma_adc_handle.Init.Mode = DMA_NORMAL;                                /* 循环模式选择 */
    g_dma_adc_handle.Init.Priority = DMA_PRIORITY_MEDIUM;                   /* 中等优先级 */
    HAL_DMA_Init(&g_dma_adc_handle);
    
    /* 初始化ADC */
    g_adc_dma_handle.Instance = ADC1;                                       /* 选择哪个ADC */
    g_adc_dma_handle.Init.DataAlign = ADC_DATAALIGN_RIGHT;                  /* 数据对齐方式：右对齐 */
    g_adc_dma_handle.Init.ScanConvMode = ADC_SCAN_ENABLE;                  /* 是否使用扫描模式 */
    g_adc_dma_handle.Init.ContinuousConvMode = ENABLE;                      /* 是否使能连续转换模式 */
    g_adc_dma_handle.Init.NbrOfConversion = DATA_NUM;                       /* 赋值范围是1~16通道 */
    g_adc_dma_handle.Init.DiscontinuousConvMode = DISABLE;                  /* 禁止间断模式 */
    g_adc_dma_handle.Init.NbrOfDiscConversion = 0;                          /* 配置间断模式通道个数 */
    g_adc_dma_handle.Init.ExternalTrigConv = ADC_SOFTWARE_START;            /* 触发转换方式：软件触发 */
    HAL_ADC_Init(&g_adc_dma_handle);                                        /* 初始化 */
    
    /* 配置DMA数据流请求中断优先级 */
    HAL_NVIC_SetPriority(DMA1_Channel1_IRQn, 7, 0);
    HAL_NVIC_EnableIRQ(DMA1_Channel1_IRQn);
    __HAL_LINKDMA(&g_adc_dma_handle, DMA_Handle, g_dma_adc_handle);         /* 将DMA与adc联系起来 */

    /* 配置ADC通道 */
    adc_ch_conf.Channel = ADC_CHANNEL_0;                                     /* 通道 */
    adc_ch_conf.Rank = ADC_REGULAR_RANK_1;                                  /* 序列 */
    adc_ch_conf.SamplingTime = ADC_SAMPLETIME_239CYCLES_5;                  /* 采样时间，设置最大采样周期:239.5个ADC周期 */
    HAL_ADC_ConfigChannel(&g_adc_dma_handle, &adc_ch_conf);                 /* 通道配置 */
    /* 配置ADC通道 */
    adc_ch_conf.Channel = ADC_CHANNEL_1;                                     /* 通道 */
    adc_ch_conf.Rank = ADC_REGULAR_RANK_2;                                  /* 序列 */
    adc_ch_conf.SamplingTime = ADC_SAMPLETIME_239CYCLES_5;                  /* 采样时间，设置最大采样周期:239.5个ADC周期 */
    HAL_ADC_ConfigChannel(&g_adc_dma_handle, &adc_ch_conf);                 /* 通道配置 */
    /* 配置ADC通道 */
    adc_ch_conf.Channel = ADC_CHANNEL_4;                                     /* 通道 */
    adc_ch_conf.Rank = ADC_REGULAR_RANK_3;                                  /* 序列 */
    adc_ch_conf.SamplingTime = ADC_SAMPLETIME_239CYCLES_5;                  /* 采样时间，设置最大采样周期:239.5个ADC周期 */
    HAL_ADC_ConfigChannel(&g_adc_dma_handle, &adc_ch_conf);                 /* 通道配置 */
    /* 配置ADC通道 */
    adc_ch_conf.Channel = ADC_CHANNEL_5;                                     /* 通道 */
    adc_ch_conf.Rank = ADC_REGULAR_RANK_4;                                  /* 序列 */
    adc_ch_conf.SamplingTime = ADC_SAMPLETIME_239CYCLES_5;                  /* 采样时间，设置最大采样周期:239.5个ADC周期 */
    HAL_ADC_ConfigChannel(&g_adc_dma_handle, &adc_ch_conf);                 /* 通道配置 */
    
    HAL_ADCEx_Calibration_Start(&g_adc_dma_handle);                         /* 校准ADC */
    
    HAL_DMA_Start(&g_dma_adc_handle, (uint32_t)&ADC1->DR, (uint32_t)&GetADC_temp, DATA_NUM);       /* 启动DMA，并开启中断 */
    HAL_ADC_Start_DMA(&g_adc_dma_handle, (uint32_t*)&GetADC_temp, DATA_NUM);                /* 先开启DMA，再开启ADC的DMA，后面两个参数其实没用 */

//    delay_us(100);  //  GD32ADC使能后需要延时20US以上

}
//排序算法
void Sort(uint16_t *pData,uint16_t count)
{
    uint16_t temp;
    uint16_t i,j;
    
    for(i=0;i<(count-1);i++)
    {
        for(j=i+1;j<count;j++)
        {
        if(pData[i]>pData[j])
            {
                temp=pData[j];
                pData[j]=pData[i];
                pData[i]=temp;
            }
        }
    }
}

/**
 * @brief       使能ADC转换
 * @param       取多少次然后平均,值越大,滤波越好
 * @retval      1=成功,0=失败
 */
void adc_dma_conver(uint8_t num)
{
    static uint16_t Get_time = 0;
    uint32_t GetR_temp = 0;
    //ADC计算使用
    uint16_t MA_array[20] = {0};
    uint16_t MB_array[20] = {0};
    uint16_t MA_temp = 0;
    uint16_t MB_temp = 0;
    uint32_t vt_temp = 0;
    uint32_t vb_temp = 0;
    if(num > 20) num = 20;
    if(num > 0)
    {
        for(uint8_t i = 0; i < num; i++ )             /* 叠加所以ADC值 */
        {
            delay_us(50);
            ADC1->CR2 &= ~(1 << 0);                   /* 先关闭ADC */
            DMA1_Channel1->CCR &= ~(1 << 0);          /* 关闭DMA通道 */
            while(DMA1_Channel1->CCR & 1 << 0);       /* 等待关闭成功 */
            DMA1_Channel1->CNDTR = DATA_NUM;          /* 设置传输数量,需要先关闭DMA */
            DMA1_Channel1->CCR |= 1 << 0;             /* 非循环模式,传输数量为0时,开启通道也不会启动传输 */
            ADC1->CR2 |= 1 << 0;                      /* 重新启动ADC */
            ADC1->CR2 |= 1 << 22;                     /* 启动规则转换通道 */
            
            while(DMA1 -> ISR & (1 << 1) == RESET);   /* 等待DMA传输完成 */
            DMA1 -> IFCR |= 1 << 1;            /* 清除DMA1 CHANNEL1完成中断 */
            
            
            MA_array[i] = GetADC_temp.MA_cun;  //电流是方波形，不能直接叠加
            MB_array[i] = GetADC_temp.MB_cun;
            vt_temp += GetADC_temp.V_T;
            vb_temp += GetADC_temp.V_B;
        }
        
        /* 电流计算:运放 = 20，采样电阻5mR,抬升电压 0v 
        I(A)=(U(mV)/20)/R(mR)
        adc = 4096 时 电压 3.3v  电压3.3v时 电流 3.3V/20倍/0.005R = 33A
        33A  =  4096 / x
        x = 4096 / 33A
        X = 0.124f
        */
        
        Sort(MA_array,20);
        MA_temp = MA_array[17];
        Sort(MB_array,20);
        MB_temp = MB_array[17];
        g_bldc_motorA.current = MA_temp / 0.124f;
        g_bldc_motorB.current = MB_temp / 0.124f;
        
         if(Get_time > 100)  /* 电压和温度变化没那么快，且耗时长，没必要太频繁计算， */
        {
            /*
            电压计算:分压/电压 = 0.0517    电阻：44k | 2.4k
            adc = 4096时 电压3.3v   3.3/0.0517 = 63.8v
            63.8v = 4096 / x
            x   =  4096 / 63.8
            */
            vb_temp /= num;
            g_bldc_motorA.v_bus = vb_temp / 0.0642f;  //单位mV
            g_bldc_motorB.v_bus = g_bldc_motorA.v_bus;  //单位mV
            
            /*
            温度计算:
            1,先算出R值  电阻5.1K 热敏电阻 10k=25摄氏度 电压是测电阻的分压
            (5.1K/(R+5.1K))*3300 = v
            16830/(R+5.1K) = V
            R+5.1 = 16830 / V
            R = 16830 / V - 5.1K
            v = num / 4096 * 3300
            R = 16830 / (num / 1.22) - 5.1K
            R = 20532.6 / num  -  5.1K
            假设电阻都为5.1k    adc值则为2048   5.1k = x / 2048 
            2,用R带入公式得到开尔文温度(单位:温度=开尔文,电阻 = 欧姆)
            T = 1/( 1 / T0    +    (1 / B ln(R/R0) ) )
            这里使用的25摄氏度电阻10k,B值是3950
            T = 1/( 0.00335402 +   ( 0.000253165 * ln(R/10000)))
            3,开尔文温度减去273.15得到摄氏度
            */
            
            vt_temp /= num;
            GetR_temp = 40960000 / vt_temp  -  12500;  // 减去偏移值
            /* 这里用100作除数, 把值放大了100 方便以整数的形式发送 */
            vt_temp = (uint16_t)(100.0f/( 0.00335402f +   ( 0.000253165f *  log(GetR_temp/10000.0f))));
            /* 转成摄氏度，温度放大了100倍，所以减27315，而不是273.15 */
            g_bldc_motorA.v_t = vt_temp - 27315;
            g_bldc_motorB.v_t = g_bldc_motorA.v_t;
            
            Get_time = 0;
        }else
        {
            Get_time++;
        }
    }
}



