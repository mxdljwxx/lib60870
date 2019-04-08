#include <stdlib.h>
#include <stdbool.h>
#include <stdio.h>
#include <stdarg.h>
#include <string.h>
#include <signal.h>
#include <time.h>
#include <assert.h>

#include <sys/socket.h>

#include <unistd.h>

#include "moxadevice.h"
//#include "../rtc.h"

#include "hal_serial.h"
#include "hal_socket.h"


#include "cs104_slave.h"
#include "cs101_slave.h"

#include "hal_thread.h"
#include "hal_time.h"

static bool running = true;
//#define localserial  true
//#define SERVERIS104  true
#define DEBUG_SOCKET true

// meter data collet 645-2007 begin ------lhq 2018-10-16

#define BYTE  uint8_t
#define WORD  uint16_t
#define DWORD  uint32_t
#define UNSIGNED unsigned

#define UART_PACKET_SIZE  25
#define YX_TABLE_SIZE  500
#define YX_ADD  1 //0001H
#define YC_TABLE_SIZE  500
#define YC_ADD  16385 //4001H
#define YM_TABLE_SIZE  200
#define YM_ADD 25601  //6401H

#define YC_CHANGE_PECENT  5  //6401H

#define PORT104 2404  //104 listen port
#define SERIALPORT101  "/dev/ttyM0"
#define SERIALPORT645  "/dev/ttyM1"

#define FALSE false 
#define TRUE  true
#define PSLEEP  usleep(550000)
#define LPSLEEP  usleep(2*550000)
#define LLPSLEEP  sleep(3)
#define MAXDELAY  2500000

#define MSLEEP  Thread_sleep(10)
#define CYCSLEEP sleep(150)

#define HIBYTE(p) (BYTE)((p & 0xff00) >> 8)
#define LOBYTE(p) (BYTE)(p & 0x00ff)
//#define MAKEWORD(hix,loy) ((WORD)(((WORD)(BYTE)(hix)) << 8) | ((BYTE)(loy)))
#define MAKEWORD(a,b) ((WORD) (((BYTE) (a)) | ((WORD) ((BYTE) (b))) << 8))

#define MAXLINE 256
#define COUNTFILE "count.txt"
#define DMBFILE "ddb-dmb.txt"
#define DDBFILE "ddb.txt"
#define BSH  10
#define BYCD  9
#define BYXD  5
#define BYMD  4
#define RCOUNT 13

typedef struct ycrecord *YcRecord;
typedef struct yxrecord *YxRecord;

static  BYTE	  m_tx_buf[UART_PACKET_SIZE];
static  BYTE	  m_rx_buf[UART_PACKET_SIZE];
static  BYTE	  framebuf[256];//UART_PACKET_SIZE];
static  BYTE	  getaddr_645[6]={0xaa,0xaa,0xaa,0xaa,0xaa,0xaa};

struct ycrecord   yctable[YC_TABLE_SIZE];
struct yxrecord   yxtable[YX_TABLE_SIZE];
struct ycrecord   ymtable[YM_TABLE_SIZE];
static  int32_t   streamcount;

static  int32_t   meterdelay=250000;
static  int32_t   mindelay=150000;
static  int32_t   avgdelay=250000;
static  int32_t   maxdelay=350000;

struct ycrecord{
     int32_t ycvalue;
	 bool ischanger;
};

struct yxrecord{
     bool yxvalue;
	 bool ischanger;
};


struct ddrecord{
	char shjbsh[9];
	float data;
	unsigned int len;
	short int xishu;
	short int rorw;
	short int lch;
	char mingch[30];
};

/*
struct sSerialPort {
    char interfaceName[100];
    int fd;
    int baudRate;
    uint8_t dataBits;
    char parity;
    uint8_t stopBits;
    uint64_t lastSentTime;
    struct timeval timeout;
    SerialPortError lastError;
};*/


struct ddb{
	char xh[10];
	char changm[20];
	char stationame[20];
	BYTE  meterno[6];
	char protname[20];	
	char serialname[20];
	int baudRate;
    uint8_t dataBits;
    char parity[3];
    uint8_t stopBits;
	#ifdef localserial
    	SerialPort port;
	#else
        Socket port;
	#endif
	uint8_t pt;
	uint8_t ct;
	bool isopen;
	char ipadd[16];
	uint32_t  ipport;
	uint32_t  minnetdelay;
	uint32_t  avgnetdelay;
	uint32_t  maxnetdelay;
};

struct ddb ddblist[BSH];


void
sigint_handler(int signalId)
{
    running = false;
}

unsigned char Bcd2Hex(BYTE bcd_data)
{
	unsigned char temp;
	temp=((bcd_data>>8)*100)+((bcd_data>>4)*10)+(bcd_data & 0x0f);
	return temp;
 }

BYTE Hex2Bcd(unsigned char hex_data)
{
	unsigned int bcd_data;
	unsigned char temp;
	temp=hex_data%100;
	bcd_data=((unsigned int)hex_data)/100<<8;
	bcd_data=bcd_data|temp/10<<4;
	bcd_data=bcd_data|temp%10;
	return bcd_data;
}

static BYTE  calc_check_sum(const BYTE *p_data, WORD data_len)
{
	WORD  i;
	BYTE  sum =0;
	for (i =0; i < data_len; i++)
	{
		sum += p_data[i];
	}
	return sum;
}

char* mysubstr(char* srcstr, int offset, int length) {
    int total_length,real_length;
    char * tmp;
    
    assert(length > 0);
    assert(srcstr != NULL);
    total_length = strlen(srcstr);//首先获取srcstr的长度
    real_length = ((total_length - offset) >= length ? length : (total_length - offset)) + 1;
    if (NULL == (tmp=(char*) malloc(real_length * sizeof(char)))) {
        printf("Memory overflow . \n");
        exit(0);
    }
    strncpy(tmp, srcstr+offset, real_length - 1);
    tmp[real_length - 1] = '\0';

    return tmp;
}
/* 
作者：joeywen 
来源：CSDN 
原文：https://blog.csdn.net/wzhg0508/article/details/16865563 
版权声明：本文为博主原创文章，转载请附上博文链接！
*/

uint8_t
initddb2(struct ddb * ddbl,int shu){

	//printf("enter init ddb!!!!!!!!!!!!!!!!!\n");
	uint8_t i,j,i2;
	char fileline[MAXLINE]="";
	char s1[15]="";
	float d3;
	bool nometernumber;
	FILE * fd2;


	fd2=fopen(DDBFILE,"r+");
	fgetc(fd2);
	fseek(fd2,0L,SEEK_SET);
	i=0;
	while(fgets(fileline,MAXLINE-1,fd2)!=0 && !feof(fd2) && i<BSH){

 		printf("get line is %s\n",fileline);
		#ifdef localserial
			i2=sscanf(fileline,"%s %s %s %s %s %s %d %u %s %u %u %u",ddbl[i].xh,ddbl[i].changm,ddbl[i].stationame,s1,ddbl[i].protname,ddbl[i].serialname,&ddbl[i].baudRate,&ddbl[i].dataBits,ddbl[i].parity,&ddbl[i].stopBits,&ddbl[i].pt,&ddbl[i].ct);
 			printf("%s\n%s\n%s\n%s\n%s\n%s\n%d\n%u\n%s\n%u\n%u\n%d\n",ddbl[i].xh,ddbl[i].changm,ddbl[i].stationame,s1,ddbl[i].protname,ddbl[i].serialname,ddbl[i].baudRate,ddbl[i].dataBits,ddbl[i].parity,ddbl[i].stopBits,ddbl[i].pt,ddbl[i].ct);
		#else
			i2=sscanf(fileline,"%s %s %s %s %s %s %d %u %s %u %u %u %s %d",ddbl[i].xh,ddbl[i].changm,ddbl[i].stationame,s1,ddbl[i].protname,ddbl[i].serialname,&ddbl[i].baudRate,&ddbl[i].dataBits,ddbl[i].parity,&ddbl[i].stopBits,&ddbl[i].pt,&ddbl[i].ct,ddbl[i].ipadd,&ddbl[i].ipport);
 			printf("%s\n%s\n%s\n%s\n%s\n%s\n%d\n%u\n%s\n%u\n%u\n%u\n%s\n%d\n",ddbl[i].xh,ddbl[i].changm,ddbl[i].stationame,s1,ddbl[i].protname,ddbl[i].serialname,ddbl[i].baudRate,ddbl[i].dataBits,ddbl[i].parity,ddbl[i].stopBits,ddbl[i].pt,ddbl[i].ct,ddbl[i].ipadd,ddbl[i].ipport);
	
		#endif
			
	// meterno asylia
	nometernumber=false;
	for(j=0;j<6;j++){
		ddbl[i].meterno[j]=atoi(mysubstr(s1,j*2,2));
		printf("meterno is %u\n",ddbl[i].meterno[j]);
    } 
    	
	// serialport set
	i++;
	printf("i is %u\n",i);
	}

	fclose(fd2);
	free(fd2);
	return 0;
}

static bool frame645check(BYTE *framebuf, WORD data_noh,WORD data_nol,uint8_t rbsize){
	 
	/* 检查异常应答帧 */

	if ( framebuf[0] !=0x68 || framebuf[7] != 0x68 || framebuf[rbsize-1] != 0x16 || framebuf[rbsize-2]!=calc_check_sum(framebuf,rbsize-2) )
        {
            printf("rev[0] is fails!\n");
            return FALSE;
        }
        if (framebuf[8] == 0xC1 || framebuf[0] !=0x68 )
        {
            printf("rev[8] is fails!\n");
            return FALSE;
        }
        /* 检查数据编号 */
/*      if((framebuf[10]-0x33) != LOBYTE(data_nol) || (framebuf[11]-0x33) != HIBYTE(data_nol) || (framebuf[12]-0x33) != LOBYTE(data_noh) || (framebuf[13]-0x33) != HIBYTE(data_noh))
        {
            printf("m_rx_buf is %02x, %02x, %02x, %02x!\n",frmaebuf[10]-0x33,framebuf[11],framebuf[12]-0x33,framebuf[13]-0x33);

            printf("rev meter_no is %02x, %02x, %02x, %02x!\n",LOBYTE(data_nol),HIBYTE(data_nol),LOBYTE(data_noh),HIBYTE(data_noh));
            printf("rev meter_no is fails!\n");
            return FALSE;
        }*/

	return TRUE;
}


#ifdef localserial
	int8_t call_pulse(SerialPort self)
#else
	int8_t call_pulse(Socket self)
#endif
{
	WORD wLow, wMi,wHi,wHHi;
        int i,j,revbyte=0,sendbyte=0,nexti=0;
	int socketfd;
	/*BYTE framebuf[UART_PACKET_SIZE];
	BYTE m_tx_buf[UART_PACKET_SIZE];
	BYTE m_rx_buf[UART_PACKET_SIZE];*/
        
	//BYTE i =0;
	printf("enter serail call_pulse!\n");
	m_tx_buf[0] =0xfe;
	m_tx_buf[1] =0xfe;
	m_tx_buf[2] =0xfe;
	m_tx_buf[3] =0xfe;

    #ifdef localserial
		sendbyte=SerialPort_write(self, m_tx_buf, 0, 4);
	#else
		sendbyte=Socket_write(self, m_tx_buf, 4);
	#endif
	for(j=0;j<4;j++)
		printf("%02x  ",m_tx_buf[j]);
	printf("\n");

	if(sendbyte==4){
		printf("serial send succs!\n");
		streamcount+=sendbyte;
	}
	else{
	   printf("serial send %d fails!\n",sendbyte);
	   return -1;
	}
}


#ifdef localserial
	int8_t get_meternumber(SerialPort self,BYTE *meter_no, uint8_t rbsize)
#else
    int8_t get_meternumber(Socket self,BYTE *meter_no, uint8_t rbsize)
#endif
{
	WORD wLow, wMi,wHi,wHHi;
    int8_t i,j,readtimes;
	int revbyte=0,sendbyte=0,nexti=0;
	int socketfd;
	/*BYTE framebuf[UART_PACKET_SIZE];
	BYTE m_tx_buf[UART_PACKET_SIZE];
	BYTE m_rx_buf[UART_PACKET_SIZE];*/
	m_tx_buf[0] =0x68;
	m_tx_buf[1] =0xaa;
	m_tx_buf[2] =0xaa;
	m_tx_buf[3] =0xaa;
	m_tx_buf[4] =0xaa;
	m_tx_buf[5] =0xaa;
	m_tx_buf[6] =0xaa;
	m_tx_buf[7] =0x68;
	m_tx_buf[8] =0x13;
	m_tx_buf[9] =0x00;
	m_tx_buf[10]=calc_check_sum(&m_tx_buf[0],10);
	m_tx_buf[11]=0x16;
    i=0;
sendbegin:
    #ifdef localserial
		sendbyte=SerialPort_write(self, m_tx_buf, 0, 12);
	#else
		sendbyte=Socket_write(self, m_tx_buf, 12);
	#endif
	for(j=0;j<12;j++)
		printf("%02x  ",m_tx_buf[j]);
	printf("\n");

	if(sendbyte==12){
		printf("serial send succs!\n");
		streamcount+=sendbyte;
		i++;
	}
	else{
	   printf("serial send %d fails!\n",sendbyte);
	   return -1;
	}
        
    #ifdef  localserial 
    revbyte=SerialPort_readByte(self);
  	if (revbyte== -1){
	    printf("seria  rev no %d  fail!\n",i);
	    return -2;
	}
	#else
	nexti=0;
	readtimes=0;
repeatread:
	usleep(maxdelay);
	revbyte=Socket_read(self,&framebuf[nexti],rbsize);
    printf("revbyte====================== %d  is  ",revbyte);
    if(revbyte==-1)
		return -1;
	for(j=0;j<revbyte;j++)
   		printf("%02x  ",framebuf[j]);
    printf("\n");
	if(revbyte==0){
		if(i==3)
			return -1;
		switch(i){
	   		case 2:usleep(avgdelay);
	   		case 3:usleep(maxdelay);
	   		default:usleep(mindelay);
	   	}
	   	i++;
		goto repeatread;
	}else
		i=1;
    
	if((revbyte+nexti)>=rbsize){
		goto socketread_end;
    }else{
		 nexti+=revbyte;
	    //printf("tcp  rev no %d=%d  fail!\n",i,revbyte);
	     if(i<4){
	  		switch(i){
	    		case 2:usleep(avgdelay);
	    		case 3:usleep(maxdelay);
	    		default:usleep(mindelay);
	    	}
	    	i++;
            goto repeatread;
	    }
	    printf("socket rev fail!\n");
	    return -2;
		}	
socketread_end:
		nexti=0;
		for(j=0;j<rbsize;j++)
	        m_rx_buf[j]=framebuf[j];
    #endif
condecode:
  if(m_rx_buf[8]==0x93 && m_rx_buf[9]==0x06){
		for(i=0;i<6;i++)
			meter_no[i]=m_rx_buf[i+1];
		return 0;
  }
  return -1;
  
}





bool pecentchanger(int32_t a,int32_t b){
	if( b==0 || ((a-b)/b*100)>4.9)
		return true;
	else
		return false;
}



/* return -1 is send fail ,-2 is rev fail ,-3 is frame error,0 is right */ 
#ifdef localserial
int8_t ws_get_pulse(SerialPort self,BYTE *meter_no, WORD data_noh,WORD data_nol, YcRecord p_val,uint8_t rbsize)
#else
int8_t ws_get_pulse(Socket self,BYTE *meter_no, WORD data_noh,WORD data_nol, YcRecord p_val,uint8_t rbsize)
#endif
{
	WORD wLow, wMi,wHi,wHHi;
    uint8_t i,j,nexti=0;
	int8_t revbyte=0,sendbyte=0,orderbyters;
	int8_t socketfd;
	int32_t newvalue;
	uint8_t readtimes=0;
	/*BYTE framebuf[UART_PACKET_SIZE];
	BYTE m_tx_buf[UART_PACKET_SIZE];
	BYTE m_rx_buf[UART_PACKET_SIZE];*/
        
	printf("enter serail ws_get_pulse!\n");
	//*p_val =0;
	m_tx_buf[0] =0x68;
	m_tx_buf[1] =(BYTE) Hex2Bcd(meter_no[5]);
	m_tx_buf[2] =(BYTE) Hex2Bcd(meter_no[4]);
	m_tx_buf[3] =(BYTE) Hex2Bcd(meter_no[3]);
	m_tx_buf[4] =(BYTE) Hex2Bcd(meter_no[2]);
	m_tx_buf[5] =(BYTE) Hex2Bcd(meter_no[1]);
	m_tx_buf[6] =(BYTE) Hex2Bcd(meter_no[0]);
	m_tx_buf[7] =0x68;
	m_tx_buf[8] =0x11;
	m_tx_buf[9] =0x04;
    m_tx_buf[10]=LOBYTE(data_nol) + 0x33;
	m_tx_buf[11]=HIBYTE(data_nol) + 0x33;
	m_tx_buf[12]=LOBYTE(data_noh) + 0x33;
	m_tx_buf[13]=HIBYTE(data_noh) + 0x33;
	m_tx_buf[14]=calc_check_sum(&m_tx_buf[0],14);
	m_tx_buf[15]=0x16;
    i=0;
	orderbyters=16;
sendbegin:
    #ifdef localserial
		sendbyte=SerialPort_write(self, m_tx_buf, 0, orderbyters);
	#else
		sendbyte=Socket_write(self, m_tx_buf, orderbyters);
	#endif
	for(j=0;j<orderbyters;j++)
		printf("%02x  ",m_tx_buf[j]);
	printf("\n");

	if(sendbyte==orderbyters){
		printf("serial send succs!\n");
		streamcount+=sendbyte;
		i++;
	}
	else{
	   printf("serial send %d fails!\n",sendbyte);
	   return -1;
	}
	switch(i){
	    case 2:usleep(avgdelay);
	    case 3:usleep(maxdelay);
	    default:usleep(mindelay);
	}

    #ifdef  localserial 
    revbyte=SerialPort_readByte(self);
  	if (revbyte== -1){
	    printf("seria  rev no %d  fail!\n",i);
	    if(i<4)
		  goto sendbegin;  
	    return -2;
	}
    i=0;
	while(revbyte!=-1){
		m_rx_buf[i]=revbyte;
		i=i+1;
		revbyte=SerialPort_readByte(self);
	}
	#else
	nexti=0;
	readtimes=0;
repeatread:
	revbyte=Socket_read(self,&framebuf[nexti],rbsize);
    printf("revbyte====================== %d  is  ",revbyte);
    if(revbyte==-1)
		return -1;
	for(j=0;j<revbyte;j++)
   		printf("%02x  ",framebuf[j]);
    printf("\n");
	if(revbyte==0){
		if(i==4)
			return -1;
		switch(i){
	   		case 2:usleep(avgdelay);
	   		case 3:usleep(maxdelay);
	   		default:usleep(mindelay);
	   	}
	   	i++;
		goto repeatread;
	}else
		i=1;
    
	if((revbyte+nexti)>=rbsize){
		goto socketread_end;
    }else{
		 nexti+=revbyte;
	    //printf("tcp  rev no %d=%d  fail!\n",i,revbyte);
	     if(i<4){
	  		switch(i){
	    		case 2:usleep(avgdelay);
	    		case 3:usleep(maxdelay);
	    		default:usleep(mindelay);
	    	}
	    	i++;
            goto repeatread;
	    }
	    printf("socket rev fail!\n");
	    return -2;
		}
		
	socketread_end:
		nexti=0;
		for(j=0;j<rbsize;j++)
	        m_rx_buf[j]=framebuf[j];
    #endif
condecode:
	streamcount+=rbsize;

	/* 检查异常应答帧 */
    if(frame645check(m_rx_buf,data_noh,data_nol,rbsize)!=TRUE)
		return -3;
	
	for(j=0;j<rbsize;j++)
	  printf("%02x  ",m_rx_buf[j]);
	  printf("\n");

        switch(m_rx_buf[9]){
             case 0x06:
	             newvalue=Bcd2Hex(m_rx_buf[15]-0x33)*100+Bcd2Hex(m_rx_buf[14]-0x33);
				 if(newvalue == p_val->ycvalue)
					 p_val->ischanger=false;
				 else
					 p_val->ischanger=pecentchanger(newvalue,p_val->ycvalue);
			 	 p_val->ycvalue = newvalue;
	             printf("rev is %4.1d succes!\n",newvalue);
		     	 break;
             case 0x07:
	             newvalue=Bcd2Hex(m_rx_buf[16]-0x33)*10000+Bcd2Hex(m_rx_buf[15]-0x33)*100+Bcd2Hex(m_rx_buf[14]-0x33);
				 if(newvalue == p_val->ycvalue)
					 p_val->ischanger=false;
				 else
					 p_val->ischanger=pecentchanger(newvalue,p_val->ycvalue);
			 	 p_val->ycvalue = newvalue;
		     	 break;
             case 0x08:
	             newvalue=Bcd2Hex(m_rx_buf[17]-0x33)*1000000+Bcd2Hex(m_rx_buf[16]-0x33)*10000+Bcd2Hex(m_rx_buf[15]-0x33)*100+Bcd2Hex(m_rx_buf[14]-0x33);
				 if(newvalue == p_val->ycvalue)
					 p_val->ischanger=false;
				 else
					 p_val->ischanger=pecentchanger(newvalue,p_val->ycvalue);
			 	 p_val->ycvalue = newvalue;
		     	 break;
             case 0x0a:
			     if (rbsize==25)  
				     goto dl_input;
				 for(i=0;i<3;i++){
	                 newvalue=Bcd2Hex(m_rx_buf[15+i*2]-0x33)*100+Bcd2Hex(m_rx_buf[14+i*2]-0x33);
					 if(newvalue == (p_val+i)->ycvalue)
						 (p_val+i)->ischanger=false;
					 else
						 (p_val+i)->ischanger=pecentchanger(newvalue,(p_val+i)->ycvalue);
			 		 (p_val+i)->ycvalue = newvalue;
				 }
				break;
             case 0x0d:
                 dl_input: 
				 for(i=0;i<3;i++){
	                 newvalue=Bcd2Hex(m_rx_buf[16+i*3]-0x33)*10000+Bcd2Hex(m_rx_buf[15+i*3]-0x33)*100+Bcd2Hex(m_rx_buf[14+i*3]-0x33);
					 if(newvalue == (p_val+i)->ycvalue)
						 (p_val+i)->ischanger=false;
					 else
						 (p_val+i)->ischanger=pecentchanger(newvalue,(p_val+i)->ycvalue);
			 		 (p_val+i)->ycvalue = newvalue;
				 }
				 break;
             default:break;		     
	}	

    printf("rev is ws_get succes!  rev is %i succes!\n",*p_val);
    return 0;
}

static bool getnetdelay(uint8_t listno){
	 
	 FILE * fddelay;
     char cmdstr[100];
     char *tmpstr;
     char *tmpstr1;
     uint32_t tmdelay;
     uint8_t i;

	 strcat(cmdstr,"ping -c 10 ");
	 strcat(cmdstr,ddblist[listno].ipadd);
	 strcat(cmdstr," > delay.txt");
	 system(cmdstr);
         
         if(fddelay=fopen("delay.txt","r")){
	     while(fgets(cmdstr,100,fddelay)>0){
	      /*printf("bottom line is %s\n",cmdstr);*/
	     }
	 }else
		 return FALSE;
	
	 printf("bottom line is %s\n",cmdstr);

	 /*get bottom line,have min/avg/max network delay */
	 //tmpstr=strcat(cmdstr,"=",strlen(cmdstr));
	 tmpstr=strtok(cmdstr,"=");
	 tmpstr=strtok(NULL,"=");
	 printf("strtok str is %s\n",tmpstr);
	 strcpy(cmdstr,tmpstr);
	 /*del spaceback*/
	 tmpstr=strtok(cmdstr," ");
	 printf("del space  str is %s\n",tmpstr);
	 
	 /*get  min/avg/max network delay */
	 tmdelay=atoi(strtok(tmpstr,"/"));
	 ddblist[listno].minnetdelay=tmdelay*1000+meterdelay;
	 
	 ddblist[listno].avgnetdelay=atoi(strtok(NULL,"/"))*1000+meterdelay;

	 if((tmdelay=atoi(strtok(NULL,"/"))*1000+meterdelay)>MAXDELAY)
	      ddblist[listno].maxnetdelay=tmdelay;
	 else
	      ddblist[listno].maxnetdelay=MAXDELAY;

	/* ddblist[listno].minnetdelay=atoi(strtok(NULL,"/"))*1000+meterdelay;*/
	 
	 printf("trtok finish!!!\n"); 
         if(fddelay)
		fclose(fddelay);	 
	 free(fddelay);
	 printf("quit getnetdelay!!!\n"); 
         return TRUE;
}

//void* meter_acq(int32_t *ycb, uint32_t *ymb)
void* meter_acq()
//void* meter_acq()
{
    uint8_t i=0,j,k,n,w;
    uint32_t delayms;
    int8_t comok;
    char cmdstr[100];
    #ifdef localserial
    	SerialPort port1;
    #else
	Socket port1;
    #endif   	
    printf("WORD IS %d\n",MAKEWORD(43,21));

    signal(SIGINT, sigint_handler);
    //  int32_t *ycb=yctable;
    // uint32_t *ymb=ymtable;

    for(i=0;i<BSH;i++){ 
    //SerialPort port = SerialPort_create(serialPort, 2400, 8, 'E', 1);
    #ifdef localserial
		printf("enter serial connect!\n");
    	port1 = SerialPort_create(ddblist[i].serialname,ddblist[i].baudRate,ddblist[i].dataBits,*ddblist[i].parity,ddblist[i].stopBits);
		ddblist[i].port=port1;
    	if(SerialPort_open(port1))
     	{
        	ddblist[i].isopen=true;
	    	ddblist[i].minnetdelay=mindelay+meterdelay;
	    	ddblist[i].avgnetdelay=avgdelay+meterdelay;
	    	ddblist[i].maxnetdelay=maxdelay+meterdelay;
         	printf("serail port is creat!\n");
     	}else{
         	ddblist[i].isopen=false;
         	printf("serail port is fail!\n");
    	}
    #else
		printf("enter tcp connect!\n");
    	port1=TcpSocket_create();
		ddblist[i].port=port1;
		if(Socket_connect(ddblist[i].port,ddblist[i].ipadd,ddblist[i].ipport))
     		{
         	printf("tcp port is open!\n");
	 		if(getnetdelay(i))
		 		printf("getnetdelay succes!!! min=%d,avg=%d,max=%d\n",ddblist[i].minnetdelay,ddblist[i].avgnetdelay,ddblist[i].maxnetdelay);
	 		else
		 		printf("getnetdelay fail!!!");

         	ddblist[i].isopen=true;
         	printf("tcp port is creat!\n");

     	}else{
         	ddblist[i].isopen=false;
         	printf("tcp port is fail!\n");
    	}
    #endif 
    if(ddblist[i].meterno[0]==0xaa){
		if(get_meternumber(port1,ddblist[i].meterno,18)==0)
			printf("get meter number succes!!!/n");
		else
			printf("get meter number fails!!!/n");
	}


    }
       w=0;
       printf("enter runing!\n");
       while(running)
       {
	   comok=0;
           
	   for(i=0;i<BSH;i++)
	    { 
            /*printf("meter begin ******************************************************************\n");	
    	    for(w=0;w<6;w++)
		    printf("meter[%i,%i]=%i \n",i,w,ddblist[i].meterno[w]);*/
	    if(ddblist[i].isopen) {
    	    j=i*9;
	    	k=i*4;
	    	mindelay=ddblist[i].avgnetdelay;
	    	avgdelay=ddblist[i].avgnetdelay;
	    	maxdelay=ddblist[i].maxnetdelay;
            call_pulse(ddblist[i].port);

	     	/*printf("yc,start=%i,ym start=%i \n",j,k);
	    	comok=ws_get_pulse(ddblist[i].port,ddblist[i].meterno, 0x0201,0xff00, &yctable[j],22);
    	    printf("I IS %i   and comok is %i,meter no is %s\n",i,comok,ddblist[i].meterno);*/
	    	//if (comok==0){
        		/*printf(" no=%i   ws_get_pulse is succ!\n",i);*/
	        	comok=ws_get_pulse(ddblist[i].port,ddblist[i].meterno, 0x0201,0xff00, &yctable[j],22);
				usleep(mindelay);
	    	//}
			/*
			//a b c 三相电压
			ws_get_pulse(ddblist[i].port,ddblist[i].meterno, 0x0201,0xff00, &yctable[j+1],22);
			usleep(mindelay);
			*/
			// a b c 三相电流
	    	//if (comok==0){
				comok=ws_get_pulse(ddblist[i].port,ddblist[i].meterno, 0x0202,0xff00, &yctable[j+3],25);
				usleep(mindelay);
	    	//}
			//瞬时总有功
	    	//if (comok==0){
				comok=ws_get_pulse(ddblist[i].port,ddblist[i].meterno, 0x0203,0x0000, &yctable[j+6],19);
				usleep(mindelay);
	    	//}
			//瞬时总无功
	    	//if (comok==0){
				comok=ws_get_pulse(ddblist[i].port,ddblist[i].meterno, 0x0204,0x0000, &yctable[j+7],19);
				usleep(mindelay);
	    	//}
			//瞬时总功率因数
	    	//if (comok==0){
				comok=ws_get_pulse(ddblist[i].port,ddblist[i].meterno, 0x0206,0x0000, &yctable[j+8],18);
				usleep(mindelay);
	    	//}  
	    	if((w % 10)==0 && comok==0){	
				//正向有功总电能
				ws_get_pulse(ddblist[i].port,ddblist[i].meterno, 0x0001,0x0000, &ymtable[k],20);
				usleep(mindelay);
		        //反向有功总电能
        		ws_get_pulse(ddblist[i].port,ddblist[i].meterno, 0x0002,0x0000, &ymtable[k+1],20);
				usleep(mindelay);
               
	    		//正向无功总电能
        		ws_get_pulse(ddblist[i].port,ddblist[i].meterno, 0x0005,0x0000, &ymtable[k+2],20);
				usleep(mindelay);

        		//反向无功总电能
        		ws_get_pulse(ddblist[i].port,ddblist[i].meterno, 0x0007,0x0000, &ymtable[k+3],20);
				usleep(mindelay);
                w=0; 
	       }

            for(n=j;n<(j+9);n++)
               	printf("meter yctable  %d  is  %d, value changer=%u\n",n,yctable[n].ycvalue,yctable[n].ischanger);
            for(n=k;n<(k+4);n++)
                printf("meter ymtable  %d  is  %d,value changer=%u\n",n,ymtable[n].ycvalue,ymtable[n].ischanger);
			MSLEEP;
       	    if (comok!=0){
                #ifdef localserial
  	            	SerialPort_close(ddblist[i].port);
                //#else
  	            //Socket_destroy(ddblist[i].port);
		    	#endif*/
                printf("port being close!!!!!!!!!!!!!!!!!!!!!!\n");
		        ddblist[i].isopen=false;
	        }
	       }else{
     		#ifdef localserial
        		if(SerialPort_open(ddblist[i].port)){
            		ddblist[i].isopen=true;
					printf("port open beging!!!!!!!!!!!!!!!!!!!!!!\n");
				}
    		#else
        	    if(Socket_connect(ddblist[i].port,ddblist[i].ipadd,ddblist[i].ipport))
				{
					ddblist[i].isopen=true;
                	printf("port open beging!!!!!!!!!!!!!!!!!!!!!!\n");
   			    }
			#endif
     		}
        /*    printf("meter end ******************************************************************\n");	*/
    }
      CYCSLEEP;
      w++;
    }

  
  //  else
  //  printf("serial open is fials!\n");

 for(i=0;i<BSH;i++)
  { 
  /*if(ddblist[i].isopen==true)
  {*/
  #ifdef localserial
  	SerialPort_close(ddblist[i].port);
  	printf("serial port is close!\n");
  	SerialPort_destroy(ddblist[i].port);
  #else
  	Socket_destroy(ddblist[i].port);
  #endif
  /*printf("serial port is destory!\n");
  }*/
  }
  return 0;
}






// meter data collet 645-2007 end ------lhq 2018-10-16
static bool
insertusedata1que(CS101_Slave slave1)
{
    uint8_t i,k;
    bool asdufull=false;
    CS101_AppLayerParameters alParams;
    CS101_ASDU newAsdu;
    InformationObject io;
   // printf("payload size is  %i\n", CS101_ASDU_getNumberOfElements(asdu));

    alParams = CS101_Slave_getAppLayerParameters(slave1);
    newAsdu = CS101_ASDU_create(alParams, false, CS101_COT_PERIODIC,0, 1, false, false);
	for(i=0;i<BSH;i++)
		{
        	for(k=0;k<BYCD;k++)
			{	
	        	if(yxtable[i*BYCD+k].ischanger==1){
					io= (InformationObject) SinglePointInformation_create(NULL,YX_ADD+i*BYXD+k,yxtable[i*BYXD+k].yxvalue, IEC60870_QUALITY_GOOD);
                    io = (InformationObject) SinglePointInformation_create(NULL, 104, true, IEC60870_QUALITY_GOOD);
	        		printf("send yxtable[%i] === %d\n",YX_ADD+i*BYXD+k,yxtable[i*BYXD+k].yxvalue);
        			CS101_ASDU_addInformationObject(newAsdu, io);
                	InformationObject_destroy(io);
				}
			}
			asdufull=true;
            if(i!=0  && (i+1)%4==0){
		    	printf("usedata1 have cont!!!!!!!!!!!!!!!!\n");	
			    CS101_Slave_enqueueUserDataClass1(slave1, newAsdu);
        	    CS101_ASDU_destroy(newAsdu);
        		newAsdu = CS101_ASDU_create(alParams, false, CS101_COT_PERIODIC,0, 1, false, false);
				asdufull=false;
			}
        }

        if(asdufull)
			CS101_Slave_enqueueUserDataClass1(slave1, newAsdu);

        CS101_ASDU_destroy(newAsdu);
    free(alParams);
    return true;
}


static bool
insertusedata2que(CS101_Slave slave1)
{
    uint8_t i,k;
    bool asdufull=false;
    CS101_AppLayerParameters alParams;
    CS101_ASDU newAsdu;
    InformationObject io;
    printf("enter usedata2quere!!!!!!!!!!!!!!!!!!!!!!!!!!!!!\n");

    alParams = CS101_Slave_getAppLayerParameters(slave1);
    newAsdu = CS101_ASDU_create(alParams, false, CS101_COT_PERIODIC,0, 1, false, false);
	for(i=0;i<BSH;i++)
		{
        	for(k=0;k<BYCD;k++)
			{	
	        	if(yctable[i*BYCD+k].ischanger==1){
					io= (InformationObject) MeasuredValueScaled_create(NULL,YC_ADD+i*BYCD+k,yctable[i*BYCD+k].ycvalue, IEC60870_QUALITY_GOOD);
	        		printf("add yctable[%i] === %d\n",YC_ADD+i*BYCD+k,yctable[i*BYCD+k].ycvalue);
        			CS101_ASDU_addInformationObject(newAsdu, io);
                	InformationObject_destroy(io);
			        asdufull=true;
				}
			}
            if(i!=0  && (i+1)%4==0 && asdufull==true ){
		    	printf("send usedata2 !!!!!!!!!!!!!!!!\n");	
			    CS101_Slave_enqueueUserDataClass2(slave1, newAsdu);
        	    CS101_ASDU_destroy(newAsdu);
        		newAsdu = CS101_ASDU_create(alParams, false, CS101_COT_PERIODIC,0, 1, false, false);
				asdufull=false;
			}
        }

    if(asdufull)
			CS101_Slave_enqueueUserDataClass2(slave1, newAsdu);
    if(newAsdu)
    	CS101_ASDU_destroy(newAsdu);
    free(alParams);
    return true;
}



void
printCP56Time2a(CP56Time2a time)
{
    struct tm tm1;
    time_t rtm;
	struct timeval tv1;
	printf("%02i:%02i:%02i %02i/%02i/%04i", CP56Time2a_getHour(time),
                             CP56Time2a_getMinute(time),
                             CP56Time2a_getSecond(time),
                             CP56Time2a_getDayOfMonth(time),
                             CP56Time2a_getMonth(time),
                             CP56Time2a_getYear(time) + 2000);
	/*fd_rtc=srtc_open();
	if(fd_rtc<0){
	    printf("real time clock init faild !\n");
    }else{*/	
	tm1.tm_sec=CP56Time2a_getSecond(time);
	tm1.tm_min= CP56Time2a_getMinute(time);
	tm1.tm_hour=CP56Time2a_getHour(time);
	tm1.tm_mday=CP56Time2a_getDayOfMonth(time);
	tm1.tm_mon=CP56Time2a_getMonth(time);
	tm1.tm_year=CP56Time2a_getYear(time) + 2000;
	rtm=mktime(&tm1);
	tv1.tv_sec=rtm;
	tv1.tv_usec=0;

	if (settimeofday(&tv1, (struct timezone *)0) < 0) 
        printf("\nset time error !\n");
}


/* Callback handler to log sent or received messages (optional) */
static void
raw101MessageHandler(void* parameter, uint8_t* msg, int msgSize, bool sent)
{
    uint8_t i;
    if (sent)
        printf("SEND: ");
    else
        printf("RCVD: ");

    for (i = 0; i < msgSize; i++) {
        printf("%02x ", msg[i]);
    }

    printf("\n");
}


/* Callback handler to log sent or received messages (optional) */
static void
rawMessageHandler(void* parameter, IMasterConnection conneciton, uint8_t* msg, int msgSize, bool sent)
{
    uint8_t i;
    if (sent)
        printf("SEND: ");
    else
        printf("RCVD: ");

    for (i = 0; i < msgSize; i++) {
        printf("%02x ", msg[i]);
    }

    printf("\n");
}

static bool
clockSyncHandler (void* parameter, IMasterConnection connection, CS101_ASDU asdu, CP56Time2a newTime)
{
    printf("Process time sync command with time "); printCP56Time2a(newTime); printf("\n");

    return true;
}

static bool
interrogationHandler(void* parameter, IMasterConnection connection, CS101_ASDU asdu, uint8_t qoi)
{
    uint8_t i,k;
    bool asdufull=false;
    CS101_AppLayerParameters alParams;
    CS101_ASDU newAsdu;
    InformationObject io;
    printf("Received interrogation for group %i\n", qoi);
 //   printf("payload size is  %i\n", CS101_ASDU_getNumberOfElements(asdu));

    if (qoi == CS101_COT_INTERROGATED_BY_STATION || qoi == CS101_COT_INTERROGATED_BY_GROUP_1 || qoi==0) { /* only handle station interrogation  qoi=20 or 21 */

        alParams = IMasterConnection_getApplicationLayerParameters(connection);

        IMasterConnection_sendACT_CON(connection, asdu, false);

        /* The CS101 specification only allows information objects without timestamp in GI responses */

        //newAsdu = CS101_ASDU_create(alParams, false, CS101_COT_INTERROGATED_BY_STATION,0, 1, false, false);
        newAsdu = CS101_ASDU_create(alParams, true, CS101_COT_INTERROGATED_BY_STATION,0, 1, false, false);

        
//	InformationObject io;
		for(i=0;i<BSH;i++)
		{
        	for(k=0;k<BYCD;k++)
			{	
	        	io= (InformationObject) MeasuredValueScaled_create(NULL,YC_ADD+i*BYCD+k,yctable[i*BYCD+k].ycvalue, IEC60870_QUALITY_GOOD);
	        	printf("add asdu  yctable[%i] === %d\n",YC_ADD+i*BYCD+k,yctable[i*BYCD+k].ycvalue);
        		CS101_ASDU_addInformationObject(newAsdu, io);
                InformationObject_destroy(io);
				asdufull=true;
			}
            if(i!=0  && (i+1)%4==0 && asdufull==true){
//		    	printf("have cont!!!!!!!!!!!!!!!!\n");	
                IMasterConnection_sendASDU(connection, newAsdu);
        	    CS101_ASDU_destroy(newAsdu);
                newAsdu = CS101_ASDU_create(alParams, true, CS101_COT_INTERROGATED_BY_STATION,0, 1, false, false);
				asdufull=false;
			}
        }
        /*InformationObject_destroy(io);*/

        if(asdufull)
			IMasterConnection_sendASDU(connection, newAsdu);

        if(newAsdu)
			CS101_ASDU_destroy(newAsdu);

/*	InformationObject io = (InformationObject) MeasuredValueScaled_create(NULL, 100, -1, IEC60870_QUALITY_GOOD);

        CS101_ASDU_addInformationObject(newAsdu, io);

        CS101_ASDU_addInformationObject(newAsdu, (InformationObject)
            MeasuredValueScaled_create((MeasuredValueScaled) io, 101, 23, IEC60870_QUALITY_GOOD));

        InformationObject_destroy(io);

        IMasterConnection_sendASDU(connection, newAsdu);

        CS101_ASDU_destroy(newAsdu);

        newAsdu = CS101_ASDU_create(alParams, false, CS101_COT_INTERROGATED_BY_STATION,
                    0, 1, false, false);

        io = (InformationObject) SinglePointInformation_create(NULL, 104, true, IEC60870_QUALITY_GOOD);

        CS101_ASDU_addInformationObject(newAsdu, io);

        InformationObject_destroy(io);

        IMasterConnection_sendASDU(connection, newAsdu);

        CS101_ASDU_destroy(newAsdu);

        CS101_ASDU_addInformationObject(newAsdu, (InformationObject) SinglePointInformation_create((SinglePointInformation) io, 301, false, IEC60870_QUALITY_GOOD));

        InformationObject_destroy(io);

        IMasterConnection_sendASDU(connection, newAsdu);

        CS101_ASDU_destroy(newAsdu);
*/
        IMasterConnection_sendACT_TERM(connection, asdu);
  
    }
    free(alParams);
    return true;
}

static bool
asduHandler(void* parameter, IMasterConnection connection, CS101_ASDU asdu)
{
    if (CS101_ASDU_getTypeID(asdu) == C_SC_NA_1) {
        printf("received single command\n");

        if  (CS101_ASDU_getCOT(asdu) == CS101_COT_ACTIVATION) {
            InformationObject io = CS101_ASDU_getElement(asdu, 0);

            if (InformationObject_getObjectAddress(io) == 5000) {
                SingleCommand sc = (SingleCommand) io;

                printf("IOA: %i switch to %i\n", InformationObject_getObjectAddress(io),
                        SingleCommand_getState(sc));

                CS101_ASDU_setCOT(asdu, CS101_COT_ACTIVATION_CON);
            }
            else
                CS101_ASDU_setCOT(asdu, CS101_COT_UNKNOWN_IOA);

            InformationObject_destroy(io);
        }
        else
            CS101_ASDU_setCOT(asdu, CS101_COT_UNKNOWN_COT);

        IMasterConnection_sendASDU(connection, asdu);

        return true;
    }

    return false;
}

static bool
counterInterrogationHandler(void* parameter, IMasterConnection connection, CS101_ASDU asdu, uint8_t qcc)
{
    uint8_t i,k;
    bool asdufull=false;
    CS101_AppLayerParameters alParams;
    CS101_ASDU newAsdu;
    InformationObject io;
    printf("Received count interrogation for group %i\n", qcc);
    printf("payload size is  %i\n", CS101_ASDU_getNumberOfElements(asdu));
            
	io = CS101_ASDU_getElement(asdu, 0);

            printf ("value is %i\n",InformationObject_getObjectAddress(io));
    
    //for(i=0;i<9;i++)
	//		printf((asdu->asdu)[i]);


    /*if(qoi == CS101_COT_REQUESTED_BY_GENERAL_COUNTER){  //qcc=37*/

    if ((qcc == 5) ) { /* only handle station interrogation  qcc=5*/
		//CS101_AppLayerParameters alParams = IMasterConnection_getApplicationLayerParameters(connection);
		alParams = IMasterConnection_getApplicationLayerParameters(connection);

        IMasterConnection_sendACT_CON(connection, asdu, false);

        /* The CS101 specification only allows information objects without timestamp in GI responses */

        //CS101_ASDU newAsdu = CS101_ASDU_create(alParams, false,CS101_COT_REQUESTED_BY_GENERAL_COUNTER,
        newAsdu = CS101_ASDU_create(alParams,true,CS101_COT_REQUESTED_BY_GENERAL_COUNTER,0, 1, false, false);
        asdufull=false;
		for(i=0;i<BSH;i++)
		{
        	for(k=0;k<BYMD;k++)
        	{		
            	io= (InformationObject) IntegratedTotals_create((IntegratedTotals)io,YM_ADD+i*BYMD+k,(BinaryCounterReading)&ymtable[i*BYMD+k].ycvalue);
//	        	printf("add ymtable[%i] === %d\n",YM_ADD+i*BYMD+k,ymtable[i*BYMD+k].ycvalue);
            	CS101_ASDU_addInformationObject(newAsdu, io);
            	InformationObject_destroy(io);
                asdufull=true;
        	}	
                if( i!=0 && (i+1)%9==0 && asdufull==true){
		    		printf("have cont!!!!!!!!!!!!!!!!\n");	
                	IMasterConnection_sendASDU(connection, newAsdu);
        	    	CS101_ASDU_destroy(newAsdu);
                    //newAsdu = CS101_ASDU_create(alParams, false,CS101_COT_REQUESTED_BY_GENERAL_COUNTER,
                    newAsdu = CS101_ASDU_create(alParams, true,CS101_COT_REQUESTED_BY_GENERAL_COUNTER,0, 1, false, false);
                    asdufull=false;
				}
		}	
        /*InformationObject_destroy(io);*/

        if(asdufull)
			IMasterConnection_sendASDU(connection, newAsdu);
       if(newAsdu)
        	CS101_ASDU_destroy(newAsdu);
	
	}else{
	    IMasterConnection_sendACT_CON(connection, asdu, true);
       }
   
    free(alParams);
    return true;
}

static bool
connectionRequestHandler(void* parameter, const char* ipAddress)
{
    printf("New connection request from %s\n", ipAddress);

#if 0
    if (strcmp(ipAddress, "127.0.0.1") == 0) {
        printf("Accept connection\n");
        return true;
    }
    else {
        printf("Deny connection\n");
        return false;
    }
#else
    return true;
#endif
}

static void
connectionEventHandler(void* parameter, IMasterConnection con, CS104_PeerConnectionEvent event)
{
    if (event == CS104_CON_EVENT_CONNECTION_OPENED) {
        printf("Connection opened (%p)\n", con);
    }
    else if (event == CS104_CON_EVENT_CONNECTION_CLOSED) {
        printf("Connection closed (%p)\n", con);
    }
    else if (event == CS104_CON_EVENT_ACTIVATED) {
        printf("Connection activated (%p)\n", con);
    }
    else if (event == CS104_CON_EVENT_DEACTIVATED) {
        printf("Connection deactivated (%p)\n", con);
    }
}

static void
resetCUHandler(void* parameter)
{
    printf("Received reset CU\n");

    CS101_Slave_flushQueues((CS101_Slave) parameter);
}

static void
linkLayerStateChanged(void* parameter, int address, LinkLayerState state)
{
    printf("Link layer state: ");

    switch (state) {
    case LL_STATE_IDLE:
        printf("IDLE\n");
        break;
    case LL_STATE_ERROR:
        printf("ERROR\n");
        break;
    case LL_STATE_BUSY:
        printf("BUSY\n");
        break;
    case LL_STATE_AVAILABLE:
        printf("AVAILABLE\n");
        break;
    }
}


void*
server101()
{
    uint8_t i,j;
    int16_t scaledValue = 0;
    uint64_t lastMessageSent = 0;
    
    SerialPort port;
    
    CS101_Slave slave;
    CS101_AppLayerParameters alParameters;
    LinkLayerParameters llParameters;
    CS101_ASDU newAsdu;
    InformationObject io;
    
    const char* serialPort = SERIALPORT101;//"/dev/ttyM0";
    
      /* Add Ctrl-C handler */
    signal(SIGINT, sigint_handler);
    port = SerialPort_create(serialPort, 9600, 8, 'E', 1);

    /* create a new slave/server instance with default link layer and application layer parameters */
      slave = CS101_Slave_create(port, NULL, NULL, IEC60870_LINK_LAYER_UNBALANCED);
    //slave = CS101_Slave_create(port, NULL, NULL, IEC60870_LINK_LAYER_BALANCED);

    CS101_Slave_setLinkLayerAddress(slave, 1);
    CS101_Slave_setLinkLayerAddressOtherStation(slave, 1);

    /* get the application layer parameters - we need them to create correct ASDUs */
    alParameters = CS101_Slave_getAppLayerParameters(slave);
    alParameters->sizeOfCOT = 1;
    alParameters->sizeOfCA = 1;
    alParameters->sizeOfIOA = 2;

    llParameters = CS101_Slave_getLinkLayerParameters(slave);
    llParameters->timeoutForAck = 500;


    /* set the callback handler for the clock synchronization command */
    CS101_Slave_setClockSyncHandler(slave, clockSyncHandler, NULL);

    /* set the callback handler for the interrogation command */
    CS101_Slave_setInterrogationHandler(slave, interrogationHandler, NULL);
    
    /* set the callback handler for the counterinterrogation command */
    CS101_Slave_setCounterInterrogationHandler(slave,counterInterrogationHandler,NULL);



    /* set handler for other message types */
    CS101_Slave_setASDUHandler(slave, asduHandler, NULL);

    /* set handler for reset CU (reset communication unit) message */
    CS101_Slave_setResetCUHandler(slave, resetCUHandler, (void*) slave);

    /* set timeout for detecting connection loss */
    CS101_Slave_setIdleTimeout(slave, 1500);

    /* set handler for link layer state changes */
    CS101_Slave_setLinkLayerStateChanged(slave, linkLayerStateChanged, NULL);

    /* uncomment to log messages */
    /*rawMessageHandler(void* parameter, IMasterConnection conneciton, uint8_t* msg, int msgSize, bool sent)*/
    CS101_Slave_setRawMessageHandler(slave,raw101MessageHandler, NULL);


    SerialPort_open(port);
    printf("101 server running!\n");
    
//    CS101_Slave_start(slave);
/*
    if (slave->isRunning == false) {
        printf("Starting server failed!\n");
        goto exit_program;
    }
*/
    lastMessageSent = Hal_getTimeInMs();
    while (running) {

        /* has to be called periodically */
        CS101_Slave_run(slave);

        /* Enqueue a measurement every second */
        if (Hal_getTimeInMs() > (lastMessageSent + 90000)) {
            insertusedata2que(slave);
            lastMessageSent = Hal_getTimeInMs();
        }

       // Thread_sleep(300);
    }

    CS101_Slave_stop(slave);

exit_program:
    CS101_Slave_destroy(slave);

    SerialPort_close(port);
    SerialPort_destroy(port);
}




void*
server104()
{
    uint8_t i,j;
    CS104_Slave slave;
    CS101_AppLayerParameters alParams;
    int16_t scaledValue = 0;
    CS101_ASDU newAsdu;

    InformationObject io;
    /* Add Ctrl-C handler */
    signal(SIGINT, sigint_handler);


    /* create a new slave/server instance with default connection parameters and
     * default message queue size */
    //CS104_Slave slave = CS104_Slave_create(100, 100);
    slave = CS104_Slave_create(100, 100);

    CS104_Slave_setLocalAddress(slave, "0.0.0.0");


    /*lhq 2018-10-15 add tcpport=PORT104 */
    CS104_Slave_setLocalPort(slave, PORT104);

    /* Set mode to a single redundancy group
     * NOTE: library has to be compiled with CONFIG_CS104_SUPPORT_SERVER_MODE_SINGLE_REDUNDANCY_GROUP enabled (=1)
     */
    CS104_Slave_setServerMode(slave, CS104_MODE_SINGLE_REDUNDANCY_GROUP);

    /* get the connection parameters - we need them to create correct ASDUs */
    //CS101_AppLayerParameters alParams = CS104_Slave_getAppLayerParameters(slave);
    alParams = CS104_Slave_getAppLayerParameters(slave);
    //alParams->sizeOfCOT = 1;
    //alParams->sizeOfCA = 1;
    //alParams->sizeOfIOA = 2;

    /* set the callback handler for the clock synchronization command */
    CS104_Slave_setClockSyncHandler(slave, clockSyncHandler, NULL);

    /* set the callback handler for the interrogation command */
    CS104_Slave_setInterrogationHandler(slave, interrogationHandler, NULL);

    /* set the callback handler for the counterinterrogation command */
    CS104_Slave_setCounterInterrogationHandler(slave,counterInterrogationHandler,NULL);


    /* set handler for other message types */
    CS104_Slave_setASDUHandler(slave, asduHandler, NULL);

    /* set handler to handle connection requests (optional) */
    CS104_Slave_setConnectionRequestHandler(slave, connectionRequestHandler, NULL);

    /* set handler to track connection events (optional) */
    CS104_Slave_setConnectionEventHandler(slave, connectionEventHandler, NULL);

    /* uncomment to log messages */
    CS104_Slave_setRawMessageHandler(slave, rawMessageHandler, NULL);

    CS104_Slave_start(slave);

    if (CS104_Slave_isRunning(slave) == false) {
        printf("Starting server failed!\n");
        goto exit_program;
    }

    printf("104 server running!\n");

    while (running) {

/*        Thread_sleep(1000);

	//printf("this time is %d\n",time(NULL));

        newAsdu = CS101_ASDU_create(alParams, false, CS101_COT_PERIODIC, 0, 1, false, false);

        io = (InformationObject) MeasuredValueScaled_create(NULL, 110, scaledValue, IEC60870_QUALITY_GOOD);

        scaledValue++;

        CS101_ASDU_addInformationObject(newAsdu, io);

        InformationObject_destroy(io);*/

        /* Add ASDU to slave event queue - don't release the ASDU afterwards!
         * The ASDU will be released by the Slave instance when the ASDU
         * has been sent.
         */
//        CS104_Slave_enqueueASDU(slave, newAsdu);

     //   CS101_ASDU_destroy(newAsdu);
    }

    CS104_Slave_stop(slave);

exit_program:
    CS104_Slave_destroy(slave);
    free(alParams);

    Thread_sleep(500);
    return 0;
}


int
main(int argc, char** argv)
{
    uint8_t i,j;
    char string1[25];
    Thread th645;

    #ifdef SERVERIS104
      	Thread th104;
    #else
    	Thread th101;
	#endif
 
    int fd_wtd;//handle of watch dog 
	FILE * fd1;

	fd_wtd=swtd_open();
	if(fd_wtd<0){
	    printf("watch dog init faild !\n");
		return -1;
    }	
	swtd_enable(fd_wtd,60000);

	fd1=fopen(COUNTFILE,"r");
	if(fgets(string1,25,fd1)>0)
            streamcount=atoi(string1);
	if(streamcount != -1)
		printf("count sum is %d \n",streamcount);
	fclose(fd1);

    if(initddb2(ddblist,BSH)!=0){
	    printf("init ddb faild ! exit!!!\n");
		return -1;
    }	

    //Thread th645,th104;
    /* Add Ctrl-C handler */
    signal(SIGINT, sigint_handler);

   /* for(i=0;i<YC_TABLE_SIZE;i++)
          yctable[i]=0;
    for(i=0;i<YM_TABLE_SIZE;i++)
          ymtable[i]=0;
   */
    th645=Thread_create(meter_acq,NULL,true);
    Thread_start(th645);
    MSLEEP;
    #ifdef SERVERIS104
    	th104=Thread_create(server104,NULL,true);
        Thread_start(th104);
    #else
    	th101=Thread_create(server101,NULL,true);
        Thread_start(th101);
    #endif


    while(running)
    {
		swtd_ack(fd_wtd);
		MSLEEP;
     }

    fd1=fopen(COUNTFILE,"w+");
	sprintf(string1,"%d",streamcount);
	if(fputs(string1,fd1)!=0) 
	printf("count sum is %s \n",string1);
    fflush(fd1);	
    fclose(fd1);
    free(fd1);

    if(fd_wtd>0)
			swtd_close(fd_wtd);

   Thread_destroy(th645);

   #ifdef SERVERIS104
    //if (server101or104)
        Thread_destroy(th104);
   #else
        Thread_destroy(th101);
   #endif
   /*
   free(ddblist);
   free(m_tx_buf);
   free(m_rx_buf);
   free(yctable);
   free(ymtable);
   */
}
