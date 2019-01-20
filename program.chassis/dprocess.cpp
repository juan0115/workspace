#include"dprocess.h"
//#include "opencv2/core.hpp"
//#include "opencv2/imgproc.hpp"
//#include "opencv2/highgui.hpp"
//#include "opencv2/videoio.hpp"
//using namespace cv;

static int ack=0;
double x, y, theta;
static double last_time=0, last_lpulse=0, last_rpulse=0;
unsigned char speeddata[]={0xa5,0x01,0x00,0x06,0x00,0x00,0x00,0x00,0x06,0x5a};
//存放速度指令，格式：帧头，类型，长度（2位），左速度（2位），右速度（2位），校验位（长度位和数据位异或），帧尾
union data_info             //联合体，用于4字节uchar和int互相转换
{
	u_char data_len[4];
	int len;
};

int get_data(data_info *p,int len)
{
	p->len = len;
	int t = p->len;
	p->data_len[0] = p->data_len[0] ^ p->data_len[3];
	p->data_len[3] = p->data_len[0] ^ p->data_len[3];
	p->data_len[0] = p->data_len[0] ^ p->data_len[3];
	p->data_len[1] = p->data_len[1] ^ p->data_len[2];
	p->data_len[2] = p->data_len[1] ^ p->data_len[2];
	p->data_len[1] = p->data_len[1] ^ p->data_len[2];
	return t;
}
/*************************************************************
 * 函数名：Dead_Recking
 * 说  明：解析收到的数据帧，并根据时间和左右轮变化量进行定位推算，并填充矩阵值
 * 参  数：recvBuff:存放数据帧，size：数据帧长度
 * 返回值：无
 * ***********************************************************/
void Dead_Recking(unsigned char *recvBuff, int size)
{
	double delta_time=0, delta_lpulse=0, delta_rpulse, vl, vr, v, w;
	if(recvBuff!=NULL&&recvBuff[0]==0xa5&&recvBuff[1]==0x01&&recvBuff[79]==0x5a)
	{
		//时间戳和左右轮脉冲数据解析，并计算各变量的变化量
		//printf("%x---%x\n",recvBuff[46],recvBuff[47]);
		int t=(recvBuff[46]<<8)+recvBuff[47];
		if(t>18000)
			t-=(0x8000 * 2) ;
		theta = ((double)t)/100;
		theta = theta > 0 ? theta : 360 + theta ; 
		delta_time=((double)(recvBuff[4]<<24|recvBuff[5]<<16|recvBuff[6]<<8|recvBuff[7]))-last_time;
		last_time=(double)(recvBuff[4]<<24|recvBuff[5]<<16|recvBuff[6]<<8|recvBuff[7]);
		delta_lpulse=((double)(recvBuff[11]<<24|recvBuff[12]<<16|recvBuff[13]<<8|recvBuff[14]))-last_lpulse;
		last_lpulse=(double)(recvBuff[11]<<24|recvBuff[12]<<16|recvBuff[13]<<8|recvBuff[14]);
		delta_rpulse=((double)(recvBuff[15]<<24|recvBuff[16]<<16|recvBuff[17]<<8|recvBuff[18]))-last_rpulse;
		last_rpulse=(double)(recvBuff[15]<<24|recvBuff[16]<<16|recvBuff[17]<<8|recvBuff[18]);
		ack=(int)recvBuff[58];
		if(delta_time)  //根据时间和左右轮变化量进行定位推算
		{
			vl=(delta_lpulse*odo_perimeter/odo_pulse)/delta_time;  //左轮线速度(mm/ms)
			vr=(delta_rpulse*odo_perimeter/odo_pulse)/delta_time;  //右轮线速度(mm/ms)
			v=(vr+vl)/2;   //机器前进线速度
			w=(vr-vl)/odo_axis;      //机器前进角速度（rad/ms），机器前进角度为w*delta_time
			y+=v*delta_time*cos(theta);
			x+=v*delta_time*sin(theta);
			theta+=w*delta_time*57.2957795;
			printf("(***************%10.2lf mm, %0.2lf mm, %0.2lf 度*****************)\n",x,y,theta);
		}
	}
}
/*************************************************************
 * 函数名：void* Process_data(void *arg)
 * 说  明：接收控制板上传数据，调用Dead_Recking函数进行数据处理
 * 参  数：arg：串口fd
 * 返回值：无
 * ***********************************************************/
void* Process_data(void *arg)
{
	unsigned char recvBuff[1024];		  //接收缓冲区
	unsigned char tempbuff[1024];
    int readLength, i, k=0;
	int fd = *(int *)arg;
	x=0.0; y=0.0; theta=0.0;
   	while(1)
	{
		readLength=UART0_Recv(fd, recvBuff, 1024);
		for(i=0;i<readLength;i++)
		{//printf("%02x ",recvBuff[i]);
			if(recvBuff[i]==0xa5)  //如果遇到帧头，清空数组，帧头后的数据入队，直到遇到帧尾
			{
				memset(tempbuff,0,sizeof(tempbuff));
				k=0;
				tempbuff[k++]=recvBuff[i];
			}
			else if(recvBuff[i]==0x5a)  //如果遇到帧尾且队头是帧头，一组数据接收完成，进行数据处理
			{
				tempbuff[k++]=recvBuff[i];
				Dead_Recking(tempbuff,k);   //将收到的一帧数据传给Dead_Recking函数，进行定位推算
			}
			else
			{
				tempbuff[k++]=recvBuff[i];
			}
		}
	}
    return 0;
}
/*************************************************************
 * 函数名：void* SendComd(void* arg)
 * 说  明：每隔1秒向控制板发送一次心跳指令
 * 参  数：arg：串口fd
 * 返回值：无
 * ***********************************************************/
void* SendComd(void* arg)
{
    int ret;
	int fd = *(int *)arg;
	unsigned char heartBeat[6] = {0xa5,0x06,0x00,0x02,0x02,0x5a};  //心跳指令
	while(1)
	{
		ret=UART0_Send(fd,(char*)heartBeat,6);
        if(ret<0)
            printf("Send heartBeat field!!\n");
		sleep(1);
	}
    return 0;
}	
/*************************************************************
 * 函数名：XoR
 * 说  明：生成校验值，校验方式：从start-1开始到end间所有位异或得到校验位
 * 参  数：buffer：存放指令数组，start：要处理数据开始下标，end：要处理数据结束下标
 * 返回值：校验位
 * ***********************************************************/
char XoR(const char buffer[], int start, int end, int bufferSize)
{
	char tempXorValue = buffer[start - 1];
	int i = start;
	for(;i<end;i++)
	{
		tempXorValue = tempXorValue ^ buffer[i];
	}
	return tempXorValue;
}
void move(int speedL, int speedR, int fd)
{
	char one,two,thr,four;
	int tempL, tempR;
	char destBuffer[10]={0};
	if(speedL<0)
	{
		tempL = 0x8000 * 2 + speedL;
		one = tempL / 256;
		two = tempL % 256;
	}
	else if(speedL>=0)
	{
		one = speedL / 256;
		two = speedL % 256;
	}
	if(speedR<0)
	{
		tempR = 0x8000 * 2 + speedR;
		thr = tempR / 256;
		four = tempR % 256;
	}
	else if(speedR>=0)
	{
		thr = speedR / 256;
		four = speedR % 256;
	}
	destBuffer[0] = 0xa5;
	destBuffer[1] = 0x01;
	destBuffer[2] = 0x00;
	destBuffer[3] = 0x06;
	destBuffer[4] = one;
	destBuffer[5] = two;
	destBuffer[6] = thr;
	destBuffer[7] = four;
	destBuffer[9] = 0x5a;
	destBuffer[8] = XoR(destBuffer, 3, 8, 10);
	int i;
	int count=5;
	while(count--)
	{
		for(i=0;i<10;i++)printf("%02x ",destBuffer[i]);printf("\n");
		UART0_Send(fd, destBuffer, 10);
		//sleep(1);
	}
}
//前进s厘米
void run(int s,int fd)
{
	double puls=(last_lpulse+last_rpulse)/2;
	while(1)
	{
		move(100,100,fd);
		if((last_lpulse+last_rpulse)/2-puls>=s*7.29)
		{
			int count=10;
			while(count--)move(0,0,fd);
			return ;
		}
	}
}
//左转90度
void turnleft(double t,int fd)
{
	double thh=theta*57.2957795;
	while(1)
	{
		move(-100,100,fd);
		if(theta*57.2957795-thh>40)
		{
			int count=20;
			while(count--)move(0,0,fd);
			return ;
		}
	}
}
int scanKeyboard()  //读取键盘值
{
	int in;
	struct termios new_settings;
	struct termios stored_settings;
	tcgetattr(0, &stored_settings);
	new_settings = stored_settings;
	new_settings.c_lflag &= (~ICANON);
	new_settings.c_cc[VTIME] = 0;
	tcgetattr(0, &stored_settings);
	new_settings.c_cc[VMIN] = 1;
	tcsetattr(0, TCSANOW, &new_settings);

	in = getchar();

	tcsetattr(0, TCSANOW, &stored_settings);
	return in;
}
void control_sweeper(int fd)  //读取键盘值，控制扫地机移动
{
	while(1)
	{
		char keyBorad = scanKeyboard();
		switch(keyBorad)
		{	
			case 'w'|'W':move(100,100,fd);break;
			case 'a'|'A':move(-100,100,fd);break;
			case 's'|'S':move(-100,-100,fd);break;
			case 'd'|'D':move(100,-100,fd);break;
			case '/':move(0,0,fd);break;
			case  'q'|'Q':exit(0); break;
			default: break;	
		}	 
	}
}
/*************************************************************
 * 函数名：control_sweeper
 * 说  明：建立client，接收速度控制指令，根据指令调用move函数控制扫地机移动，端口号：13001
 * 参  数：serverIP:要连接服务器ip
 * 返回值：无
 * **********************************************************/
/*int control_sweeper(char *serverIP, int fd)
{
    unsigned char recv_buff[10];  //存放速度指令
    int i, j, _port=13001;
    int sock = socket(AF_INET,SOCK_STREAM, 0);
    if(sock < 0)
    {
        perror("socket");
        return 1;
    }
    //需要connect的是对端的地址，因此这里定义服务器端的地址结构体
    struct sockaddr_in server;
    server.sin_family = AF_INET;
    server.sin_port = htons(_port);
    server.sin_addr.s_addr = inet_addr(serverIP);
    socklen_t len = sizeof(struct sockaddr_in);
    while(1)
	{
		if(connect(sock, (struct sockaddr*)&server, len) <0)
		{
			//perror("client connect");
			continue;
		}
		else
		{
			printf("client connect server success!!\n");
			break;
		}
	}
    //连接成功进行收数据
    while(1)
    {
		char pos[10][35];
		int t;
		unsigned char info_buff[4];
		read(sock,info_buff,4);
		int p =(int)(info_buff[3] | info_buff[2] << 8 | info_buff[1] << 16 | info_buff[0] << 24);
		int len = read(sock,recv_buff,p);
		while(len<p)
			len += read(sock,recv_buff+len,p-len);
		recv_buff[len]=0;
		printf("%s\n",recv_buff);
		j=0; t=0;
		memset(pos,0,sizeof(pos));
		for (i = 0;i<len;i++)
		{
			if (recv_buff[i]!=' ')
				pos[t][j++]=recv_buff[i];
			else
			{
				t++;
				j = 0;
			}
			if(t==3)
			{
				t=i;
				break;
			}
		}
		int speedL = atoi(pos[1]);
		int speedR = atoi(pos[2]);
		printf("%d-----%d\n",speedL,speedR);
		move(speedL, speedR, fd);
	}
    close(sock);
	return 0;
}*/
/*************************************************************
 * 函数名：startup
 * 说  明：socket服务器端初始化，socket，bind，listen
 * 参  数：_port：端口号，_ip:本地ip
 * 返回值：返回socket句柄
 * ***********************************************************/
int startup(int _port,const char* _ip)
{
    int sock = socket(AF_INET,SOCK_STREAM,0);
    if(sock < 0)
    {
        perror("socket");
        exit(1);
    }
    struct sockaddr_in local;
    local.sin_family = AF_INET;
    local.sin_port = htons( _port);
    local.sin_addr.s_addr = inet_addr(_ip);
    socklen_t len = sizeof(local);
    if(bind(sock,(struct sockaddr*)&local , len) < 0)
    {
        perror("bind");
        exit(2);
    }
    if(listen(sock, 5) < 0) //允许连接的最大数量为5
    {
        perror("listen");
        exit(3);
    }
    return sock;
}
/*************************************************************
 * 函数名：set_server
 * 说  明：创建服务器，每隔50ms发送位置，端口15000
 * 参  数：arg:本地ip
 * 返回值：无
 * ***********************************************************/
void* set_server15000(void* arg)
{ 
    int _port=15000;
	char *_ip=(char *)arg;
	int listen_sock = startup(_port, _ip);//初始化
    struct sockaddr_in remote;
    socklen_t len = sizeof(struct sockaddr_in);

    while(1)
    {
        int sock = accept(listen_sock, (struct sockaddr*)&remote, &len);
        if(sock < 0)
        {
            perror("accept");
            continue;
        }
        printf("get a client, ip:%s, port:%d\n",inet_ntoa(remote.sin_addr),ntohs(remote.sin_port));
        while(1)
        {
			char splace[]="";
            sprintf(splace,"pos %lf %lf %lf",x/10,y/10,theta*57.2957795);
            data_info info;
			info.len = strlen(splace);
			get_data(&info,strlen(splace));
			fflush(stdin);
			write(sock,info.data_len,4);
			fflush(stdin);
			write(sock,splace,strlen(splace));
			usleep(50);
        }
    }
    return 0;
}


void* set_server13000(void* arg)  //发s，端口13000
{
    int sock;
	char *_ip=(char *)arg;
    int _port=13000;
    int listen_sock = startup(_port, _ip);//初始化

    //用来接收客户端的socket地址结构体
    struct sockaddr_in remote;
    socklen_t len = sizeof(struct sockaddr_in);

    while(1)
    {
        int sock = accept(listen_sock, (struct sockaddr*)&remote, &len);
        if(sock < 0)
        {
            perror("accept");
            continue;
        }
        printf("13000 get a client, ip:%s, port:%d\n",inet_ntoa(remote.sin_addr),ntohs(remote.sin_port));
        while(1)
        {
            char splace[]="";
			int xx=(int)(x/10);
			int yy=(int)(y/10);
			int th=(int)(theta*572.957795);
            sprintf(splace,"pos %d %d %d",xx,yy,th+900);
			data_info info;
			info.len = strlen(splace);
			get_data(&info,strlen(splace));
			fflush(stdin);
			write(sock,info.data_len,4);
			fflush(stdin);
			write(sock,splace,strlen(splace));
			sleep(1);
        }
    }
	close(sock);
    return 0;
}