#include "stm32f10x.h"               
#include "Delay.h"						
#include "stepMotor.h"

int main(void)
{
	stepMotor_Init();

	halfModeRotate(4,-180,1);//旋转-180度
	for(int i =0;i<36000;i++)//用0.01为一步·，步进360度 测试误差修正
	{
		halfModeRotate(4,0.01,1);//8拍
		//allModeRotate(6,0.01,1);//4拍
	}

}	
