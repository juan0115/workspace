#include "dprocess.h"

/*************************************************************
 * 函数名：int main(int argc,char *argv[]) 
 * 说  明：主函数
 * 参  数：argc,argv
 * 返回值：无
 * ***********************************************************/
int main(int argc,char *argv[])
{ 
    int fd, ret;                            
    char serialPort[] = "/dev/ttyUSB0"; //默认串口
    pthread_t thread[10];
    //如果串口不是默认，输入小鸟所用串口
	if(argc>3)                    
		strcpy(serialPort,argv[3]);
	fd=UART0_Open(serialPort);
    if(fd<0) 
		exit(0);
	ret=UART0_Init(fd,115200,0,8,1,'N');
	if(ret<0)
	{
		UART0_Close(fd);
		exit(0);
	}
	pthread_create(&thread[1], NULL, SendComd, (void *)&fd);//创建线程，发送心跳指令
	pthread_create(&thread[2], NULL, Process_data, (void *)&fd);//接收控制板上报数据，并显示时间戳和左右轮脉冲
	pthread_create(&thread[3], NULL, set_server15000, (void *)argv[1]);//服务端程序,发送位置
	//pthread_create(&thread[4], NULL, set_server13000, (void *)argv[1]);//服务端程序,发送位置
	if(control_sweeper(argv[2], fd))  //客户端初始化，接收速度控制扫地机移动
	{
		printf("set client field!!\n");
		return 0;
	}
	UART0_Close(fd);
	return 0;
} 
