
# DAC＋DMA＋TIM生成正弦波
> stm32F091RC
---
1. 设置DAC
   <img src="1.png" width=50% height=50%>
2. 设置DMA
   <img src="2.png" width=50% height=50%>
3. 设置定时器
   <img src="3.png" width=50% height=50%>
4. 生成正炫波数据
 ```
 /* num:一个正玄播中采集的点数
 * p:存放数据的数组
 * U:输出电压的峰值(0~1.5V)
 */
#include "math.h"
#define PI (double)3.1415927
void SinWaveData(uint16_t num,uint16_t *p,float U)
{	
	uint16_t i;
	for(i=0;i<num;i++)
		*p++ = (uint16_t)((U * sin(( 1.0*i/(num-1))*2*PI)+U)*4095/3.3);
		//D[i] = (u16)((U*sin(( 1.0*i/(num-1))*2*Pi)+U)*4095/3.3);
}
```
5. 启动DAC
```
    SinWaveData(100,DualSine12bit,1.5);
	HAL_TIM_Base_Start(&htim6);
	HAL_DAC_Start_DMA(&hdac,DAC1_CHANNEL_2,(uint32_t *)DualSine12bit,100,DAC_ALIGN_12B_R);
  
```