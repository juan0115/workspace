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
    char serialPort[] = "/dev/ttyUSB1"; //默认串口
    pthread_t thread[10];
    //如果串口不是默认，输入小鸟所用串口
	if(argc>3)                    
		strcpy(serialPort,argv[3]);
	fd=UART0_Open(serialPort);
    if(fd<0) 
		exit(0);
	ret=UART0_Init(fd,115200,0,8,1,'E');
	if(ret<0)
	{
		UART0_Close(fd);
		exit(0);
	}
	pthread_create(&thread[1], NULL, SendComd, (void *)&fd);//创建线程，发送小鸟数据采集指令
	pthread_create(&thread[2], NULL, Process_data, (void *)&fd);//解析小鸟数据并将数据转换成二维数组
	pthread_create(&thread[4], NULL, set_client, (void *)argv[2]);//客户端程序，接收位置信息
	pthread_create(&thread[5], NULL, send_map, (void *)argv[2]);
	cv_map(argv[2]);  //使用opencv将地图以图片形式呈现并保存于本地
	UART0_Close(fd);
	return 0;
} 
