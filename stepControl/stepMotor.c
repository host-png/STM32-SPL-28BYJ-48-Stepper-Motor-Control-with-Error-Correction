#include "Tim.h"
#include "stm32f10x.h"
#include "Delay.h"	
#define minStep (360.0f/4096.0f)//最小角度

uint8_t currentStep = 0;//记录当前半步状态的参数 4拍8拍共用
float accumulatedError = 0;//累计误差，即不满足当前最小单步导致的误差
		
uint16_t halfStep[8] = {
    GPIO_Pin_0,          // A
    GPIO_Pin_0 | GPIO_Pin_1,  // A+B
    GPIO_Pin_1,          // B
    GPIO_Pin_1 | GPIO_Pin_2, // B+C
    GPIO_Pin_2,         // C
    GPIO_Pin_2 | GPIO_Pin_3,// C+D
    GPIO_Pin_3,         // D
    GPIO_Pin_3 | GPIO_Pin_0  // D+A
};

void GenericEightstep(uint32_t delyTime,uint8_t Mode,uint8_t StepNum);//通用低精度
void stepOnce(uint8_t stepCurrent, uint32_t delyTime);//单步驱动
void stepMotor_Init(void)
{
	
	//初始化步进电机
	RCC_APB2PeriphClockCmd(RCC_APB2Periph_GPIOA, ENABLE);

	GPIO_InitTypeDef GPIO_InitStructure;
	GPIO_InitStructure.GPIO_Mode = GPIO_Mode_Out_PP;//推挽输出
	GPIO_InitStructure.GPIO_Pin = GPIO_Pin_0|GPIO_Pin_1|GPIO_Pin_2|GPIO_Pin_3;		//pa0pa1pa2pa3
	GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
	GPIO_Init(GPIOA, &GPIO_InitStructure);
	GPIO_ResetBits(GPIOA, GPIO_Pin_0 | GPIO_Pin_1 | GPIO_Pin_2 | GPIO_Pin_3);//拉低防止电机乱动
	
}

// 8 步半步顺序 每一步0.0879度
//8拍模式更平滑
//speed 转一圈需要的时间 单位秒/每圈  s/r 最小是4s/r 最大无上限 
//旋转的角度
//模式，分低精度和高精度模式 低精度以8步为一个单位，高精度记录单步的状态精度最高 （0低精度1高精度）
int halfModeRotate(float speed,float angle,uint8_t Mode)
{
	
	//speed延迟转换
	uint32_t delyTime = (uint32_t)(speed * 1000000 / 4096);  //一个最小角度的时间,单步时间
	if(!Mode)//低精度模式，大角度不考虑精度可用
	{
		if(angle>0)
		{
			for(int halfstepi = 0;halfstepi<angle*512/360;halfstepi++)
			{
				GenericEightstep(delyTime,0,1);
			}
		}
		else
		{
			for(int halfstepi = 0;halfstepi<-angle*512/360;halfstepi++)
			{
				GenericEightstep(delyTime,7,1);
			}
		}
		
	}
	else//高精度模式
	{
		//误差累加（做的其实是浮点数取余）
		accumulatedError += angle - (int)(angle/minStep)*minStep;//angle对minStep取余
		angle+= (int)(accumulatedError/minStep)*minStep;//把可转动误差加到角度中
		accumulatedError = accumulatedError - (int)(accumulatedError/minStep)*minStep;//补偿对minStep取余

		int mustStep =angle/minStep ;
		int state = 1;  
		if(angle<0)
		{
			mustStep =-mustStep;
			state = -1;
		}
		for(int step= 0;step<mustStep;step++)
		{
			currentStep = (currentStep + state +8)%8; //中间+8避免负数
			stepOnce(currentStep,delyTime);
		}
	}
	    GPIO_SetBits(GPIOA, GPIO_Pin_0 | GPIO_Pin_1 | GPIO_Pin_2 | GPIO_Pin_3);
}

//4拍模式 每一步0.0879*2（单步最低时间是3-4ms）
int allModeRotate(float speed,float angle,uint8_t Mode)//注意这里传入的时间并不是真正的转动时间，真正的转动时间等于 输入时间/2
{
		//speed延迟转换
	uint32_t delyTime = (uint32_t)(speed * 1000000 / 4096);  //一个最小角度的时间,单步时间
	
	
	if(!Mode)//低精度模式（不考虑精度的情况下，大角度最顺滑，不适合小角度一点一点动）
	{
		if(angle>0)
		{
			for(int halfstepi = 0;halfstepi<angle*512/360;halfstepi++)
			{
				GenericEightstep(delyTime,0,2);
			}
		}
		else
		{
			for(int halfstepi = 0;halfstepi<-angle*512/360;halfstepi++)
			{
				GenericEightstep(delyTime,7,2);
			}
		}
		
	}
	else//高精度模式 
	{
		accumulatedError += angle - (int)(angle/(minStep*2.0f))*(minStep*2.0f);//angle对minStep取余
		angle+= (int)(accumulatedError/(minStep*2.0f))*(minStep*2.0f);//把可转动误差加到角度中
		accumulatedError = accumulatedError - (int)(accumulatedError/(minStep*2.0f))*(minStep*2.0f);//补偿对minStep取余
		
		if(currentStep == 0)
		{
			currentStep =1;
		}
		int mustStep =angle*4096/360/2;
		int state = 2;  
		if(angle<0)
		{
			mustStep =-mustStep;
			state = -2;
		}
		for(int step= 0;step<mustStep;step++)
		{
			currentStep = (currentStep + state +8)%8; //中间+8避免负数
			stepOnce(currentStep,delyTime);
		}
	}
	    GPIO_SetBits(GPIOA, GPIO_Pin_0 | GPIO_Pin_1 | GPIO_Pin_2 | GPIO_Pin_3);
}


void GenericEightstep(uint32_t delyTime,uint8_t Mode,uint8_t StepNum)//8拍低精度模式专用
{	
		for(int step = Mode;step<8;step+=StepNum)
			{
				stepOnce(step,delyTime);
			}
}

void stepOnce(uint8_t stepCurrent, uint32_t delyTime)//高精度模式通用
{
    GPIO_SetBits(GPIOA, GPIO_Pin_0 | GPIO_Pin_1 | GPIO_Pin_2 | GPIO_Pin_3);
    GPIO_ResetBits(GPIOA, halfStep[stepCurrent]);
    Delay_us(delyTime);
}


/*
void allRoteEightstep(uint32_t delyTime,uint8_t Mode)//4拍低精度模式专用
{	if(Mode)
	{
		for(int step = 1;step<8;step+=2)
			{
				stepOnce(step,delyTime);
			}
	}
	else
	{
		for(int step = 7;step>-1;step-=2)
			{
				stepOnce(step,delyTime);
			}
	}
}
*/
/*
uint16_t allStep[4] = {//4步全步 每一步0.1758
    
    GPIO_Pin_0 | GPIO_Pin_1,  // A+B
    GPIO_Pin_1 | GPIO_Pin_2, // B+C
    GPIO_Pin_2 | GPIO_Pin_3,// C+D
    GPIO_Pin_3 | GPIO_Pin_0  // D+A
};
*/

			/*
			if(currentStep !=0)//进行补全操作
			{
				if((7-currentStep)>mustStep)//补全步数 比 必须步 大
				{
					for(;currentStep<(mustStep+currentStep);currentStep++)
					{
						GPIO_SetBits(GPIOA, GPIO_Pin_0 | GPIO_Pin_1 | GPIO_Pin_2 | GPIO_Pin_3);
						// 再通电对应线圈
						GPIO_ResetBits(GPIOA, halfStep[currentStep]);
						// 延时，控制速度
						Delay_us(delyTime);
					}
					return 0;
				}
				else//补全小于必须步
				{
					mustStep -= (7- currentStep);//减去补全所需的步数
					for(;currentStep<8;currentStep++)
					{
						GPIO_SetBits(GPIOA, GPIO_Pin_0 | GPIO_Pin_1 | GPIO_Pin_2 | GPIO_Pin_3);
						// 再通电对应线圈
						GPIO_ResetBits(GPIOA, halfStep[currentStep]);
						// 延时，控制速度
						Delay_us(delyTime);
					}
					//currentStep =0;
				}
			}
			for(int halfstepi = 0;halfstepi<(mustStep/8)-1;halfstepi++)//8步
			{
				RoteEightstep(delyTime);
			}
			
			currentStep = (mustStep%8)-1;//当前半步状态，即记录位置
			for(int step = 0;step<currentStep+1;step++)//补上最后一个半步的单步
			{
				GPIO_SetBits(GPIOA, GPIO_Pin_0 | GPIO_Pin_1 | GPIO_Pin_2 | GPIO_Pin_3);
				// 再通电对应线圈
				GPIO_ResetBits(GPIOA, halfStep[step]);
				// 延时，控制速度
				Delay_us(delyTime);
			}
			if(currentStep<0)
			{
				currentStep=0;
			}*/
			
			/*
//全步模式每次执行2条线 A+B B+C这样
//参数与半步相同 速度也是4s/r
void allModeRotate(float speed,float angle,uint8_t Mode)
{
		//speed延迟转换
	float delyTime = speed/4096;  //一个最小角度的时间,单步时间
	if(!Mode)
	{
		for(int halfstepi = 0;halfstepi<angle*512/360;halfstepi++)//低精度模式
		{
			for(int step = 0;step<8;step++)
			{
				GPIO_SetBits(GPIOA, GPIO_Pin_0 | GPIO_Pin_1 | GPIO_Pin_2 | GPIO_Pin_3);
			// 再通电对应线圈
				GPIO_ResetBits(GPIOA, halfStep[step]);
			// 延时，控制速度
				Delay_ms(1);
			}
		}
	}
	else//高精度模式
	{
		int mustStep = angle*4096/360;
		currentStep = (mustStep%8)-1;//当前半步状态，即记录位置
		for(int halfstepi = 0;halfstepi<(angle*512/360)-1;halfstepi++)
		{
			for(int step = 0;step<8;step++)
			{
				GPIO_SetBits(GPIOA, GPIO_Pin_0 | GPIO_Pin_1 | GPIO_Pin_2 | GPIO_Pin_3);
				// 再通电对应线圈
				GPIO_ResetBits(GPIOA, halfStep[step]);
				// 延时，控制速度
				Delay_ms(1);
			}
		}
		for(int step = 0;step<currentStep+1;step++)//补上最后一个半步的单步
		{
			GPIO_SetBits(GPIOA, GPIO_Pin_0 | GPIO_Pin_1 | GPIO_Pin_2 | GPIO_Pin_3);
			// 再通电对应线圈
			GPIO_ResetBits(GPIOA, halfStep[step]);
			// 延时，控制速度
			Delay_ms(1);
		}
	}
}
*/
