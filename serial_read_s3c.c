#include<stdio.h>
#include<string.h>
#include<sys/types.h>
#include<errno.h>
#include<sys/stat.h>
#include<fcntl.h>
#include<unistd.h>
#include<termios.h>
#include<stdlib.h>
#include<time.h>
#include<malloc.h>

#define L 10

static int set_serial_para(int fd, int baudRate, int nBits, char nEvent, int nStop);
static void write_buffer(char *buf);
static int StrReplace(char strRes[],char from[], char to[]);

int main(int argc, char **argv)
{
	FILE *fp;
	int fd;
	int nread,nwrite;
	int i;
	int receDateBufLen = 10; 		//接收的数据长度
	char getMsg[L];
	char getMsgBak[11];
	char readFileBuf[1024];	//文件读取缓存
	fd_set rd;
	time_t timep;

	int flag=0;
	fd = open("/dev/ttyUSB0",O_RDWR | O_NOCTTY | O_NDELAY);
	do{
		if(flag!=0)
			sleep(5);
		flag=1;
		fd = open("/dev/ttyUSB0",O_RDWR | O_NOCTTY | O_NDELAY);
		
	}while(fd<0);

	if(fcntl(fd,F_SETFL,0)<0)
	{
		printf("fcntl failed!\n");
	}else{
		printf("fcntl=%d\n",fcntl(fd,F_SETFL,0));
	}
	
	if(isatty(STDIN_FILENO)==0)
	{
		printf("standard input is not a terminal device\n");
	}else{
		printf("isatty success!\n");
	}

	if((set_serial_para(fd,115200,8,'N',1)) < 0)
	{
		perror("set_serial_para error");
		printf("set_serial_parameter error\n");
		return -1;
	}

	FD_ZERO(&rd);
	FD_SET(fd,&rd);
	while(FD_ISSET(fd,&rd))
	{
		if(select(fd+1,&rd,NULL,NULL,NULL) < 0)
		{
			perror("select");
		}else{
			while((nread = read(fd,getMsg,L))>0)
			{
				fp = fopen("write.ini","r");
				while(fgets(readFileBuf,1024,fp)!=NULL);
				fclose(fp);
				
				strcpy(getMsgBak,getMsg);
				
				//切换最后一位
				if(getMsg[9] == '1'){
						getMsg[9] == '0';
				}else if(getMsg[9] == '0'){
						getMsg[9] == '1';
				}else{;}
				
				StrReplace(readFileBuf,getMsg,getMsgBak);
				
		    fp = fopen("write.ini","w");

		    fputs(readFileBuf,fp);

		    fclose(fp);
				
			}
			
		}
	}
	close(fd);
	return 0;
	
}

static int set_serial_para(int fd, int baudRate, int nBits, char nEvent, int nStop)
{
	struct termios newtio,oldtio;
	if(tcgetattr(fd,&oldtio)!=0)
	{
		perror("SetupSerial 1");
		return -1;
	}
	bzero(&newtio,sizeof(newtio));
	
	newtio.c_cflag |= CLOCAL | CREAD;
	newtio.c_cflag &= ~CSIZE;
	
	switch(nBits)
	{
		case 7:
			newtio.c_cflag |= CS7;
			break;
		case 8:
			newtio.c_cflag |= CS8;
			break;	
	}
	
	switch(nEvent)
	{
		case 'O':
			newtio.c_cflag |= PARENB;
			newtio.c_cflag |= PARODD;
			newtio.c_iflag |= (INPCK | ISTRIP);
			break;
		case 'E':
			newtio.c_iflag |= (INPCK | ISTRIP);
			newtio.c_cflag |= PARENB;
			newtio.c_cflag &= ~PARODD;
			break;
		case 'N':
			newtio.c_cflag &= ~PARENB;
			break;			
	}
	
	switch(baudRate)
	{
		case 2400:
			cfsetispeed(&newtio,B2400);
			cfsetospeed(&newtio,B2400);
			break;
		case 4800:
			cfsetispeed(&newtio,B4800);
			cfsetospeed(&newtio,B4800);
			break;
		case 9600:
			cfsetispeed(&newtio,B9600);
			cfsetospeed(&newtio,B9600);
			break;
		case 115200:
			cfsetispeed(&newtio,B115200);
			cfsetospeed(&newtio,B115200);
			break;
		case 57600:
			cfsetispeed(&newtio,B57600);
			cfsetospeed(&newtio,B57600);
			break;
		default:
			cfsetispeed(&newtio,B9600);
			cfsetospeed(&newtio,B9600);
			break;		
	}
	if(nStop == 1)
	{
		newtio.c_cflag &= ~CSTOPB;
	}else if(nStop == 2){
		newtio.c_cflag |= CSTOPB;
	}
	newtio.c_cc[VTIME] = 0;
	newtio.c_cc[VMIN] = 0;
	tcflush(fd,TCIFLUSH);
	if((tcsetattr(fd,TCSANOW,&newtio)) != 0)
	{
		perror("com set error");
		return -1;
	}
	printf("set done!\n");
	return 0;
}


static void write_buffer(char *buf)
{
	FILE *filep;
	static unsigned int cnt;
	cnt++;
/*
	if(cnt>30000)
	{
		filep=fopen("write.ini","w");
		cnt=0;
	}
	else	
	{
*/
		filep=fopen("write.ini","w");
		fputs(buf,filep);
		//fputc('\n',filep);
//	}	
	fclose(filep);	
}

int StrReplace(char strRes[],char from[], char to[]) {
    int i,flag = 0;
    char *p,*q,*ts;
    for(i = 0; strRes[i]; ++i) {
        if(strRes[i] == from[0]) {
            p = strRes + i;
            q = from;
            while(*q && (*p++ == *q++));
            if(*q == '\0') {
                ts = (char *)malloc(strlen(strRes) + 1);
                strcpy(ts,p);
                strRes[i] = '\0';
                strcat(strRes,to);
                strcat(strRes,ts);
                free(ts);
                flag = 1;
            }
        }
    }
    return flag;
}