#include "serial.h"

/*************************************************************
 * 函数名：int UART0_Open(char* port)
 * 说  明：打开串口并返回串口设备文件描述 
 * 参  数：port：串口号(ttyUSB0,ttyUSB1,ttyUSB2) 
 * 返回值：打开串口失败返回-1，成功返回串口fd
 * ***********************************************************/
int UART0_Open(char* port) 
{  
	int fd = -1 ;
	if (!port)
	{
		return -1 ;
	}
	fd = open ( port , O_RDWR|O_NOCTTY|O_NDELAY ) ;
  	if ( fd < 0 )
  	{
		printf("open serial failed!!\n");
    	return -1 ;
  	}

	if ( fcntl(fd, F_SETFL, 0 ) < 0 )
	{
		printf("serial fcntl failed!!\n") ;
		close(fd) ;
		return -1 ;
	}
  	printf("serial open ok!!\n") ;
	return fd;  
}  
/*************************************************************
 * 函数名：void UART0_Close(int fd)
 * 说  明：关闭串口
 * 参  数：fd：串口fd
 * 返回值：无
 * ***********************************************************/
void UART0_Close(int fd)
{  
	if ( !fd ) return ;
  	close( fd ) ;
}  
 
/*************************************************************
 * 函数名：int UART0_Init(int fd, int speed,int flow_ctrl,int databits,int stopbits,int parity)
 * 说  明：串口初始化，设置串口各参数
 * 参  数：fd：串口fd speed：波特率 flow_ctrl：数据流控制 databits：数据位 stopbits：停止位 parity：效验类型 取值为N,E,O,,S 
 * 返回值：设置成功返回0，设置错误返回-1
 * ***********************************************************/
int UART0_Init(int fd, int speed,int flow_ctrl,int databits,int stopbits,int parity)
{  
	int  i;   
	int  speed_arr[] = { B115200, B19200, B9600, B4800, B2400, B1200, B300};//十六进制
	int  name_arr[] = {115200,  19200,  9600,4800,  2400,  1200,  300};  
	struct termios options; //控制非同步通信端口 
	/*tcgetattr(fd,&options)得到与fd指向对象的相关参数，
	并将它们保存于options,该函数还可以测试配置是否正确，
	该串口是否可用等。若调用成功，函数返回值为0，
	若调用失败，函数返回值为1. 
    */  
	if( tcgetattr( fd,&options)  !=  0){  
		perror("SetupSerial 1");      
		return -1;   
	}  
    //设置串口输入波特率和输出波特率  
	for ( i= 0;  i < sizeof(speed_arr) / sizeof(int);  i++){  
		if  (speed == name_arr[i])  {               
			cfsetispeed(&options, speed_arr[i]);//输入波特率   
			cfsetospeed(&options, speed_arr[i]);//输出波特率
		}  
	}       
    //修改控制模式，保证程序不会占用串口 
    options.c_cflag |= CLOCAL;  //c_cflag输出模式
    //修改控制模式，使得能够从串口中读取输入数据  
    options.c_cflag |= CREAD;  
    //设置数据流控制  
    switch(flow_ctrl){
		case 0 ://不使用流控制  
              options.c_cflag &= ~CRTSCTS;  
              break;     
		case 1 ://使用硬件流控制  
              options.c_cflag |= CRTSCTS;  
              break;  
		case 2 ://使用软件流控制  
              options.c_cflag |= IXON | IXOFF | IXANY;  
              break; 
    }  

    //设置数据位  
    //屏蔽其他标志位 
    options.c_cflag &= ~CSIZE;  
    switch (databits){    
		case 5  : 
                     options.c_cflag |= CS5;  
                     break;  
		case 6  :  
                     options.c_cflag |= CS6;  
                     break;  
		case 7  :      
                 options.c_cflag |= CS7; 

                 break;  
		case 8  :      
                 options.c_cflag |= CS8;  
                 break;    
		default:     

                 fprintf(stderr,"Unsupported data size\n");  
                 return -1;  
    }  
    //设置校验位  
    switch (parity){    
		case 'n':  
		case 'N': //无奇偶校验位。  
                 options.c_cflag &= ~PARENB;   
                 options.c_iflag &= ~INPCK;      
                 break;   
		case 'o':    
		case 'O'://设置为奇校验      
                 options.c_cflag |= (PARODD | PARENB);   
                 options.c_iflag |= INPCK;               
                 break;   
		case 'e':   
		case 'E'://设置为偶校验    
                 options.c_cflag |= PARENB;        
                 options.c_cflag &= ~PARODD;         
                 options.c_iflag |= INPCK;        
                 break;  
		case 's':  
		case 'S': //设置为空格   
                 options.c_cflag &= ~PARENB;  
                 options.c_cflag &= ~CSTOPB;  
                 break;   
        default:    
                 fprintf(stderr,"Unsupported parity\n");      
                 return -1;   
    }   

    // 设置停止位   
    switch (stopbits){
		case 1:     
                 options.c_cflag &= ~CSTOPB; break;   
		case 2:     
                 options.c_cflag |= CSTOPB; break;  
		default:     
                       fprintf(stderr,"Unsupported stop bits\n");   
                       return -1;  
    }  
	//修改输出模式，原始数据输出  
	options.c_oflag &= ~OPOST;  
	options.c_lflag &= ~(ICANON | ECHO | ECHOE | ISIG);  
	//options.c_lflag &= ~(ISIG | ICANON);  
    //设置等待时间和最小接收字符  
    options.c_cc[VTIME] = 1; /* 读取一个字符等待1*(1/10)s */    
    options.c_cc[VMIN] = 1; /* 读取字符的最少个数为1 */ 
    //如果发生数据溢出，接收数据，但是不再读取 刷新收到的数据但是不读  
    tcflush(fd,TCIFLUSH);  
    //激活配置 (将修改后的termios数据设置到串口中）  
    if (tcsetattr(fd,TCSANOW,&options) != 0){  
		perror("com set error!\n");    
		return -1;  
	}  
    return 0;
}  
/*************************************************************
 * 函数名：int UART0_Recv(int fd, unsigned char *rcv_buf,int data_len)
 * 说  明：接收串口数据，使用select阻塞监听
 * 参  数：fd：串口fd recv_buf：存放数据数组地址 data_len：预接收数据长度
 * 返回值：返回实际接收数据长度
 * ***********************************************************/
int UART0_Recv(int fd, unsigned char *rcv_buf,int data_len)
{ 	
	int ret;  
    fd_set fs_read;//结构体/数组
    struct timeval time;//超时时间
    FD_ZERO(&fs_read);//清空集合中的描述字

    FD_SET(fd,&fs_read);//设置描述字
    time.tv_sec = 0;//秒
    time.tv_usec = 0;//微秒
    //使用select实现串口的多路通信  超时，检测是否有文件可读
     //传入两个NULL表示不关心文件的写变化和错误异常信息
    ret = select(fd+1,&fs_read,NULL,NULL,&time);
    if(ret)
		ret = read(fd,rcv_buf,data_len);//读取的字节数
	return ret;
}  

/*************************************************************
 * 函数名：int UART0_Send(int fd, unsigned char *send_buf,int data_len) 
 * 说  明：接收串口数据
 * 参  数：fd：串口fd send_buf：要发送数据数组地址 data_len：发送数据长度
 * 返回值：返回发送数据长度
 * ***********************************************************/
int UART0_Send(int fd, unsigned char *send_buf,int data_len)  
{  
    int len = 0; 
    len = write(fd,send_buf,data_len);
    if (len == data_len){
		return len;  
	}
	return -1;  
}
