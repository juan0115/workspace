#include"dprocess.h"
#include "opencv2/core.hpp"
#include "opencv2/imgproc.hpp"
#include "opencv2/highgui.hpp"
#include "opencv2/videoio.hpp"
#define witetime 3
using namespace cv;

int intbuf[1024];   //存放转换成10进制的小鸟数据
int xmap[1024][1024]; //存放地图数据
char placedata[8];
int sxx=512,syy=512;
double pos_x=0.0, pos_y=0.0, theta=0.0;

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
 * 函数名：void Bresenhamline(int x1,int y1,int x2,int y2,int color)
 * 说  明：在点(x1,y1)和点(x2,y2)间画直线
 * 参  数：x1,y1,x2,y2：两点的横纵坐标， color：填充二维数组的值
 * 返回值：无
 * ***********************************************************/
void Bresenhamline(int x1,int y1,int x2,int y2,int color)
{
    int dx,dy,yy,ix,iy,cx,cy,n2dy,n2dydx,d;
    dx = abs(x2 - x1);
    dy = abs(y2 - y1),
    yy = 0;
    if (dx < dy)
    {
        yy = 1;
        int temp;
        temp=x1; x1=y1; y1=temp;
        temp=x2; x2=y2; y2=temp;
        temp=dx; dx=dy; dy=temp;
    }
    ix=(x2 - x1) > 0 ? 1 : -1;
    iy=(y2 - y1) > 0 ? 1 : -1;
    cx=x1; cy=y1;
    n2dy = dy * 2;
    n2dydx = (dy - dx) * 2;
    d = dy * 2 - dx;
    // 如果直线与 x 轴的夹角大于 45 度
    if (yy)
    {
        while (cx != x2)
        {
            if (d < 0)
                d+=n2dy;
            else
            {
                cy += iy;
                d += n2dydx;
            }
            if(xmap[cx][cy]!=4&&xmap[cx][cy]!=9)
            xmap[cy][cx]=color;
            cx += ix;
        }
    }
    // 如果直线与 x 轴的夹角小于 45 度
    else
    {
        while (cx != x2) {
            if (d < 0)
            {
                d += n2dy;
            }
            else
            {
                cy += iy;
                d += n2dydx;
            }
            if(xmap[cx][cy]!=4&&xmap[cx][cy]!=9)
            xmap[cx][cy]=color;
            cx += ix;
        }
    }
}

/*************************************************************
 * 函数名：void Create_2dmap()
 * 说  明：根据intbuf数组创建二维数组
 * 参  数：无
 * 返回值：无
 * ***********************************************************/
void Create_2dmap()
{
    int i;
    double xx,yy;
    for(i = 20; i < 201; i++)               //舍去-45° ~ 45°之外的的测距值
    {
        //2D激光测距模块数据单位：mm，地图中一个像素代表一厘米，intbuf中值四舍五入
        intbuf[i] = (intbuf[i] + 5) / 10; 
        if(intbuf[i] > 300)                 //舍去超出最大测距3m的距离值
        {  
            //激光照射到的点的坐标
            xx=sxx - (sin(tfm(theta + 90.0 + 45.0 - (i-20) / 2.0)) * 300.0);
            yy=syy + (cos(tfm(theta + 90.0 + 45.0 - (i-20) / 2.0)) * 300.0);
            Bresenhamline(xx, yy, sxx, syy, MAP_EXPLORED_WHITE);        //最大默认距离点到激光头画直线
        }
        else
        {   
            //激光照射到的点的坐标
            xx=sxx - (sin(tfm(theta + 90.0 + 45.0 - (i-20) / 2.0)) * (double)intbuf[i]);
            yy=syy + (cos(tfm(theta + 90.0 + 45.0 - (i-20) / 2.0)) * (double)intbuf[i]);
            Bresenhamline(xx, yy, sxx, syy, MAP_EXPLORED_WHITE);        //红外线射到的点到激光头画直线
            xmap[(int)xx][(int)yy] = MAP_BARRIER_BLACK;                 //激光照射到的点标记为障碍物 
        }
    }
    memset(intbuf, 0, sizeof(intbuf));
}

/*************************************************************
 * 函数名：void* Process_data(void *arg)
 * 说  明：接收并处理小鸟数据，先将小鸟数据转换成10进制，然后调用Create_2dmap()函数转换成二维数组
 * 参  数：arg：串口fd
 * 返回值：无
 * ***********************************************************/
void* Process_data(void *arg)
{
	unsigned char recvBuff[1024];		  //接收缓冲区
	int i,j,k,readLength;
	int fd = *(int *)arg;
   	while(1)
	{
		readLength=UART0_Recv(fd, recvBuff, sizeof(recvBuff)-1);
        for(i=0;i<readLength;i++)
        {
            if(recvBuff[i]==0xa5 && recvBuff[i+1]==0x01&&readLength-i>=440) 
            {
                k=0;
                memset(intbuf,0,sizeof(intbuf));
                for(j=i+2;j<readLength;j+=2)
                {
                    //前一个字节为高八位，后一个为低八位
                    intbuf[k++]=(int)(recvBuff[j]<<8 | recvBuff[j+1]); 
                }
                Create_2dmap();  //创建二维数组
                break;
            }
        }
        sleep(1);
	}
    return 0;
}
/*************************************************************
 * 函数名：void* SendComd(void* arg)
 * 说  明：每隔若干秒发向小鸟发送一次数据采样指令
 * 参  数：arg：串口fd
 * 返回值：无
 * ***********************************************************/
void* SendComd(void* arg)
{
	int fd = *(int *)arg;
	unsigned char heartBeat[7] = {0xa5,0x07,0x01,0x36,0x00,0x00,0xe3};  //数据采样指令，见技术文档
	while(1)
	{
		UART0_Send(fd,heartBeat,7);
		sleep(1);
	}
    return 0;
}	

void* set_client(void* arg)  //接收实时位置，端口15000
{
    unsigned char recv_buff[1024];
    char *_ip=(char *)arg;
    int i, j, t,  _port=15000;
    char pos[10][50];
    int sock = socket(AF_INET,SOCK_STREAM, 0);
    if(sock < 0)
    {
        perror("socket");
        return 0;
    }
    //需要connect的是对端的地址，因此这里定义服务器端的地址结构体
    struct sockaddr_in server;
    server.sin_family = AF_INET;
    server.sin_port = htons(_port);
    server.sin_addr.s_addr = inet_addr(_ip);
    socklen_t len = sizeof(struct sockaddr_in);
    while(1)
    {
        if(connect(sock, (struct sockaddr*)&server, len) < 0 )
        {
            perror("15000connect");
            continue;
        }
        //连接成功进行收数据
        while(1)
        {
            unsigned char info_buff[4];
            read(sock,info_buff,4);
            int p =(int)(info_buff[3] | info_buff[2] << 8 | info_buff[1] << 16 | info_buff[0] << 24);
            int len = read(sock,recv_buff,p);
            while(len<p)
                len += read(sock,recv_buff+len,p-len);
            recv_buff[len]=0;
           // printf("%s\n",recv_buff);
            j=0; t=0;
            for (i = 0;i<len;i++)
            {
                if (recv_buff[i]!=' ')
                    pos[t][j++]=recv_buff[i];
                else
                {
                    t++;
                    j = 0;
                }
                if(t==4)
                {
                    t=i;
                    break;
                }
            }
            pos_x = atof(pos[1]);
            pos_y = atof(pos[2]);
            theta = atof(pos[3]);
            sxx=512-(int)pos_y;
            syy=512-(int)pos_x;
            xmap[sxx][syy]=9;
        }
    }
    close(sock);
    return 0;
}
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
void* send_map(void *arg)
{
    int _port=14000,i, j;
    char *_ip=(char *)arg;
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
        printf("get a client, ip:%s, port:%d\n",inet_ntoa(remote.sin_addr),ntohs(remote.sin_port));
        while(1)
        { 
            char *map_buff=new char[1024*1024+20]{};
            sprintf(map_buff,"map -512 -300 512 300 ");
            int k=22;
            for(i=212;i<812;i++)
            for(j=0;j<1024;j++)
                map_buff[k++]=xmap[i][j];
            map_buff[k]=0;
            data_info info;
            info.len = 614422;
            get_data(&info,614422);
            fflush(stdin);
            write(sock,info.data_len,4);   
            fflush(stdin);
            int ret=write(sock,map_buff,614422);
            delete []map_buff;
            sleep(3);
        }
    }
    close(listen_sock);
}
/*************************************************************
 * 函数名：void cv_map()
 * 说  明：向客户端发送地图并把地图用open库显示并保存到本地
 * 参  数：无
 * 返回值：无
 * ***********************************************************/
void cv_map(char *_ip)
{
    int i, j, R, G, B;
    char map_path[50];    //地图图片路径
    Mat image;
    image = Mat(1024,1024,CV_8UC3);//创建1024x1024矩阵
    while(1)
    {
        for(i=0;i<1024;i++)
        for(j=0;j<1024;j++)
        {
            switch(xmap[i][j])
            {
                case MAP_UNEXPLORED_GRAY: R=194; G=194; B=194; break;   //灰色未探索区域
                case MAP_EXPLORED_WHITE: R=255; G=255; B=255; break;    //白色已探索区域
                case MAP_BARRIER_BLACK: R=0; G=0; B=0; break;           //黑色障碍物
                case MAP_LOCUS_RED: R=255; G=0; B=0; break;             //红色轨迹
                default:R=194;G=194;B=194;break;
            }
            image.at<cv::Vec3b>(i, j)[0] = B;
            image.at<cv::Vec3b>(i, j)[1] = G;
            image.at<cv::Vec3b>(i, j)[2] = R;
        }
        imshow("map", image);
        //sprintf(map_path,"/mnt/hgfs/vmshare/mapphotos/map-%04d.png",count++);
        //imwrite(map_path,image);
        waitKey(1000*witetime);
    }
}
