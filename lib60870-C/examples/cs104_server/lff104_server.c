#include <stdlib.h>
#include <stdbool.h>
#include <stdio.h>
#include <stdarg.h>
#include <string.h>
#include <signal.h>
#include <time.h>
#include <assert.h>

#include <unistd.h>
#include "hal_serial.h"

#include "cs104_slave.h"

#include "hal_thread.h"
#include "hal_time.h"

static bool running = true;
#define localserial  false
#define DEBUG_SOCKET true

// meter data collet 645-2007 begin ------lhq 2018-10-16

#define BYTE  uint8_t
#define WORD  uint16_t
#define DWORD  uint32_t
#define UNSIGNED unsigned

#define UART_PACKET_SIZE  20
#define YC_TABLE_SIZE  800
#define YC_ADD  16385 //4001H
#define YM_TABLE_SIZE  800
#define YM_ADD 25601  //6401H

#define FALSE false
#define TRUE true
#define PSLEEP  usleep(250000)

#define MSLEEP  sleep(50)
#define CYCSLEEP sleep(300)

#define HIBYTE(p) (BYTE)((p & 0xff00) >> 8)
#define LOBYTE(p) (BYTE)(p & 0x00ff)
//#define MAKEWORD(hix,loy) ((WORD)(((WORD)(BYTE)(hix)) << 8) | ((BYTE)(loy)))
#define MAKEWORD(a,b) ((WORD) (((BYTE) (a)) | ((WORD) ((BYTE) (b))) << 8))

#define MAXLINE 256
#define DMBFILE "ddb-dmb.txt"
#define DDBFILE "ddb.txt"
#define BSH  2
#define BYCD  9
#define RCOUNT 13



static  BYTE	  m_tx_buf[UART_PACKET_SIZE];
static  BYTE	  m_rx_buf[UART_PACKET_SIZE];
static  int32_t   yctable[YC_TABLE_SIZE];
static  uint32_t  ymtable[YM_TABLE_SIZE];


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
	#if localserial
	SerialPort port;
	#else
        Socket port;
	#endif
	uint8_t pt;
	uint8_t ct;
	bool isopen;
	char ipadd[16];
	int  ipport;
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
	temp=((bcd_data>>8)*100)|((bcd_data>>4)*10)|(bcd_data&0x0f);
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
    assert(length > 0);
    assert(srcstr != NULL);

    int total_length = strlen(srcstr);//首先获取srcstr的长度
    int real_length = ((total_length - offset) >= length ? length : (total_length - offset)) + 1;
    char *tmp;
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

	printf("enter init ddb!!!!!!!!!!!!!!!!!\n");
	int i,j,i2;
	char fileline[MAXLINE]="";
	char s1[15]="";
	float d3;
	FILE * fd2;
	//FILE * fd1;

	fd2=fopen(DDBFILE,"r+");
	fgetc(fd2);
	fseek(fd2,0L,SEEK_SET);
	i=0;
	while(fgets(fileline,MAXLINE-1,fd2)!=0 && !feof(fd2) && i<BSH){

 	printf("get line is %s\n",fileline);
	#if localserial
		i2=sscanf(fileline,"%s %s %s %s %s %s %d %u %s %u %u %u",ddbl[i].xh,ddbl[i].changm,ddbl[i].stationame,s1,ddbl[i].protname,ddbl[i].serialname,&ddbl[i].baudRate,&ddbl[i].dataBits,ddbl[i].parity,&ddbl[i].stopBits,&ddbl[i].pt,&ddbl[i].ct);
 	printf("%s\n%s\n%s\n%s\n%s\n%s\n%d\n%u\n%s\n%u\n%u\n%d\n",ddbl[i].xh,ddbl[i].changm,ddbl[i].stationame,s1,ddbl[i].protname,ddbl[i].serialname,ddbl[i].baudRate,ddbl[i].dataBits,ddbl[i].parity,ddbl[i].stopBits,ddbl[i].pt,ddbl[i].ct);
	#else
		i2=sscanf(fileline,"%s %s %s %s %s %s %d %u %s %u %u %u %s %d",ddbl[i].xh,ddbl[i].changm,ddbl[i].stationame,s1,ddbl[i].protname,ddbl[i].serialname,&ddbl[i].baudRate,&ddbl[i].dataBits,ddbl[i].parity,&ddbl[i].stopBits,&ddbl[i].pt,&ddbl[i].ct,ddbl[i].ipadd,&ddbl[i].ipport);
 	printf("%s\n%s\n%s\n%s\n%s\n%s\n%d\n%u\n%s\n%u\n%u\n%u\n%s\n%d\n",ddbl[i].xh,ddbl[i].changm,ddbl[i].stationame,s1,ddbl[i].protname,ddbl[i].serialname,ddbl[i].baudRate,ddbl[i].dataBits,ddbl[i].parity,ddbl[i].stopBits,ddbl[i].pt,ddbl[i].ct,ddbl[i].ipadd,ddbl[i].ipport);
	
	#endif
	// meterno asylia
	for(j=0;j<6;j++){
	ddbl[i].meterno[j]=atoi(mysubstr(s1,j*2,2));
	printf("meterno is %u\n",ddbl[i].meterno[j]);
        } 
	
	// serialport set
	i++;
	printf("i is %u\n",i);
	}

	fclose(fd2);
	return 0;
}



#if localserial
static bool ws_get_pulse(SerialPort self,BYTE *meter_no, WORD data_noh,WORD data_nol, int32_t *p_val)
#else
static bool ws_get_pulse(Socket self,BYTE *meter_no, WORD data_noh,WORD data_nol, int32_t *p_val)
#endif
{
	printf("enter serail ws_get_pulse!\n");
	WORD wLow, wMi,wHi,wHHi;
        BYTE i =0;
        int revbyte=0,sendbyte=0;
	*p_val =0;
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

	//for(int8_t j=0;j<18;j++)
	//  printf("sendbuf is %02x\n",m_tx_buf[j]);
        #if localserial
	sendbyte=SerialPort_write(self, m_tx_buf, 0, 16);
	#else
	sendbyte=Socket_write(self, m_tx_buf, 16);
	#endif

	if(sendbyte==16){
	printf("serial send succs!\n");
	}
	else{
	   printf("serial send %d fails!\n",sendbyte);
	   return FALSE;
	}
        PSLEEP;
        #if localserial 
        revbyte=SerialPort_readByte(self);
  	if (revbyte== -1){
	    printf("rev fail!\n");
	    return FALSE;
	}
        i=0;
	while(revbyte!=-1){
	m_rx_buf[i]=revbyte;
	i=i+1;
	revbyte=SerialPort_readByte(self);
	}
        #else
	i=Socket_read(self,m_rx_buf,UART_PACKET_SIZE);
  	if (i== -1){
	    printf("rev fail!\n");
	    return FALSE;
	}

	#endif

    printf("revbuf  %d  is  ",i);
    for(int8_t j=0;j<i;j++)
	    printf("%02x  ",m_rx_buf[j]);
	printf("\n");

	/* 检查异常应答帧 */
	if (m_rx_buf[8] == 0xC1)
	{
		 printf("rev[8] is fails!\n");
		return FALSE;
	}
	/* 检查数据编号 */
	if((m_rx_buf[10]-0x33) != LOBYTE(data_nol) || (m_rx_buf[11]-0x33) != HIBYTE(data_nol) || (m_rx_buf[12]-0x33) != LOBYTE(data_noh) || (m_rx_buf[13]-0x33) != HIBYTE(data_noh))
	{

		printf("rev meter_no is %02x, %02x, %02x, %02x!\n",LOBYTE(data_nol),HIBYTE(data_nol),LOBYTE(data_noh),HIBYTE(data_noh));
		printf("rev meter_no is fails!\n");
		return FALSE;
	}
	if(m_rx_buf[9]==0x06){
	wLow =Bcd2Hex(m_rx_buf[14]-0x33);
	wHi  =Bcd2Hex(m_rx_buf[15]-0x33);
	*p_val =wHi*100 + wLow;
	printf("rev is %4.1d succes!\n",*p_val);
    }else{
	if(m_rx_buf[9]==0x07){
	wLow =Bcd2Hex(m_rx_buf[14]-0x33);
        wMi  =Bcd2Hex(m_rx_buf[15]-0x33);
	wHi  =Bcd2Hex(m_rx_buf[16]-0x33);
	*p_val =wHi*10000 + wMi*100 + wLow;
	}else{
	if(m_rx_buf[9]==0x08){
	wLow =Bcd2Hex(m_rx_buf[14]-0x33);
        wMi  =Bcd2Hex(m_rx_buf[15]-0x33);
	wHi  =Bcd2Hex(m_rx_buf[16]-0x33);
	wHHi  =Bcd2Hex(m_rx_buf[17]-0x33);
	*p_val =wHHi*1000000 + wHi*10000 + wMi*100 + wLow;
	}
	}
    }

    printf("rev is ws_get succes!\n");


   printf("rev is %i succes!\n",*p_val);

	return TRUE;
}

//void* meter_acq(int32_t *ycb, uint32_t *ymb)
void* meter_acq()
//void* meter_acq()
{
    printf("WORD IS %d\n",MAKEWORD(43,21));

    signal(SIGINT, sigint_handler);
    uint8_t i=0,j,k;
    bool comok;
    #if localserial
    	SerialPort port1;
    #else
	Socket port1;
    #endif   	
  //  int32_t *ycb=yctable;
   // uint32_t *ymb=ymtable;
/*
    BYTE meterno[6];
    meterno[0]=0;
    meterno[1]=18;
    meterno[2]=82;
    meterno[3]=99;
    meterno[4]=99;
    meterno[5]=55;
    const char* serialPort = "/dev/ttyUSB0";
*/

    for(i=0;i<BSH;i++){ 
    //SerialPort port = SerialPort_create(serialPort, 2400, 8, 'E', 1);
    #if localserial
    	port1 = SerialPort_create(ddblist[i].serialname,ddblist[i].baudRate,ddblist[i].dataBits,*ddblist[i].parity,ddblist[i].stopBits);
    //port1 = SerialPort_create(ddblist[i].serialname,ddblist[i].baudRate,ddblist[i].dataBits,'E',ddblist[i].stopBits);
    	if(SerialPort_open(port1))
     	{
	 ddblist[i].port=port1;
         ddblist[i].isopen=true;
         printf("serail port is creat!\n");
     	}else{
         ddblist[i].isopen=false;
         printf("serail port is fail!\n");
    	}
    #else
	printf("enter tcp connect!\n");
    	port1=TcpSocket_create();
	if(Socket_connect(port1,ddblist[i].ipadd,ddblist[i].ipport))
     	{
         printf("tcp port is open!\n");
	 ddblist[i].port=port1;
         ddblist[i].isopen=true;
         printf("tcp port is creat!\n");
     	}else{
         ddblist[i].isopen=false;
         printf("tcp port is fail!\n");
    	}
    #endif 

    }

    //comok=true;
    //if(SerialPort_open(port))
    //{
       printf("enter runing!\n");
       while(running)
       {
	   comok=true;
           for(i=0;i<BSH;i++)
	    { 
	    if(ddblist[i].isopen == true)
    	    {
    	    comok = ws_get_pulse(ddblist[i].port,ddblist[i].meterno, 0x0201,0x0100, &yctable[i+0]);
    	    if (comok==1){
        	printf("ws_get_pulse is succ!\n");
    	        }
		PSLEEP;
                //printf("ws_ge_pulse 0x02010100 is open!\n");
		// b相电压
		ws_get_pulse(ddblist[i].port,ddblist[i].meterno, 0x0201,0x0200, &yctable[i+1]);
                PSLEEP;

		// c三相电压
		ws_get_pulse(ddblist[i].port,ddblist[i].meterno, 0x0201,0x0300, &yctable[i+2]);
	        PSLEEP;

		// a相电流
		ws_get_pulse(ddblist[i].port,ddblist[i].meterno, 0x0202,0x0100, &yctable[i+3]);
	        PSLEEP;

		// b相电流 /
		ws_get_pulse(ddblist[i].port,ddblist[i].meterno, 0x0202,0x0100, &yctable[i+4]);
                PSLEEP;

		// c三相电流
		ws_get_pulse(ddblist[i].port,ddblist[i].meterno, 0x0202,0x0300, &yctable[i+5]);
		PSLEEP;
		//瞬时总有功
		ws_get_pulse(ddblist[i].port,ddblist[i].meterno, 0x0203,0x0000, &yctable[i+6]);
		PSLEEP;

		//  瞬时总无功
		ws_get_pulse(ddblist[i].port,ddblist[i].meterno, 0x0204,0x0000, &yctable[i+7]);
                PSLEEP;
 
		//瞬时总功率因数
		ws_get_pulse(ddblist[i].port,ddblist[i].meterno, 0x0206,0x0000, &yctable[i+8]);
                PSLEEP;
	        
		//正向有功总电能
		ws_get_pulse(ddblist[i].port,ddblist[i].meterno, 0x0001,0x0000, &ymtable[i]);
                PSLEEP;

                //反向有功总电能
                ws_get_pulse(ddblist[i].port,ddblist[i].meterno, 0x0002,0x0000, &ymtable[i+1]);
                PSLEEP;
               
	       	//正向无功总电能
                ws_get_pulse(ddblist[i].port,ddblist[i].meterno, 0x0005,0x0000, &ymtable[i+2]);
                PSLEEP;

                //反向无功总电能
                ws_get_pulse(ddblist[i].port,ddblist[i].meterno, 0x0007,0x0000, &ymtable[i+3]);
                PSLEEP;

                for(j=0;j<9;j++)
            	printf("meter yctable  %d  is  %d              ymtalbe is %d\n",j,yctable[j],ymtable[j]);
        	MSLEEP;
       }
	}
    //SerialPort_close(port);
    //printf("serial port is close!\n");

    }

  
  //  else
  //  printf("serial open is fials!\n");

 for(i=0;i<BSH;i++)
  { 
  if(ddblist[i].isopen==true)
  {
  #if localserial
  	SerialPort_close(ddblist[i].port);
  	printf("serial port is close!\n");
  	SerialPort_destroy(ddblist[i].port);
  #else
  	Socket_destroy(ddblist[i].port);
  #endif
  printf("serial port is destory!\n");
  }
  }
  return 0;
}






// meter data collet 645-2007 end ------lhq 2018-10-16




void
printCP56Time2a(CP56Time2a time)
{
    printf("%02i:%02i:%02i %02i/%02i/%04i", CP56Time2a_getHour(time),
                             CP56Time2a_getMinute(time),
                             CP56Time2a_getSecond(time),
                             CP56Time2a_getDayOfMonth(time),
                             CP56Time2a_getMonth(time),
                             CP56Time2a_getYear(time) + 2000);
}

/* Callback handler to log sent or received messages (optional) */
static void
rawMessageHandler(void* parameter, IMasterConnection conneciton, uint8_t* msg, int msgSize, bool sent)
{
    if (sent)
        printf("SEND: ");
    else
        printf("RCVD: ");

    uint8_t i;
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
    printf("Received interrogation for group %i\n", qoi);

    if (qoi == CS101_COT_INTERROGATED_BY_STATION) { /* only handle station interrogation  qoi=20*/

        CS101_AppLayerParameters alParams = IMasterConnection_getApplicationLayerParameters(connection);

        IMasterConnection_sendACT_CON(connection, asdu, false);

        /* The CS101 specification only allows information objects without timestamp in GI responses */

        CS101_ASDU newAsdu = CS101_ASDU_create(alParams, false, CS101_COT_INTERROGATED_BY_STATION,
                0, 1, false, false);

        
	InformationObject io;
	for(uint8_t i=0;i<BSH;i++)
	{
        for(uint8_t k=0;k<BYCD;k++)
	{
	        io= (InformationObject) MeasuredValueScaled_create(NULL,YC_ADD+i*BYCD+k,yctable[i*BYCD+k], IEC60870_QUALITY_GOOD);
        	CS101_ASDU_addInformationObject(newAsdu, io);
        }
	}
        InformationObject_destroy(io);

        IMasterConnection_sendASDU(connection, newAsdu);

        CS101_ASDU_destroy(newAsdu);
   
     


/*	InformationObject io = (InformationObject) MeasuredValueScaled_create(NULL, 100, -1, IEC60870_QUALITY_GOOD);

        CS101_ASDU_addInformationObject(newAsdu, io);

        CS101_ASDU_addInformationObject(newAsdu, (InformationObject)
            MeasuredValueScaled_create((MeasuredValueScaled) io, 101, 23, IEC60870_QUALITY_GOOD));

        CS101_ASDU_addInformationObject(newAsdu, (InformationObject)
            MeasuredValueScaled_create((MeasuredValueScaled) io, 102, 2300, IEC60870_QUALITY_GOOD));

        InformationObject_destroy(io);

        IMasterConnection_sendASDU(connection, newAsdu);

        CS101_ASDU_destroy(newAsdu);

        newAsdu = CS101_ASDU_create(alParams, false, CS101_COT_INTERROGATED_BY_STATION,
                    0, 1, false, false);

        io = (InformationObject) SinglePointInformation_create(NULL, 104, true, IEC60870_QUALITY_GOOD);

        CS101_ASDU_addInformationObject(newAsdu, io);

        CS101_ASDU_addInformationObject(newAsdu, (InformationObject)
            SinglePointInformation_create((SinglePointInformation) io, 105, false, IEC60870_QUALITY_GOOD));

        InformationObject_destroy(io);

        IMasterConnection_sendASDU(connection, newAsdu);

        CS101_ASDU_destroy(newAsdu);

        newAsdu = CS101_ASDU_create(alParams, true, CS101_COT_INTERROGATED_BY_STATION,
                0, 1, false, false);

        CS101_ASDU_addInformationObject(newAsdu, io = (InformationObject) SinglePointInformation_create(NULL, 300, true, IEC60870_QUALITY_GOOD));
        CS101_ASDU_addInformationObject(newAsdu, (InformationObject) SinglePointInformation_create((SinglePointInformation) io, 301, false, IEC60870_QUALITY_GOOD));
        CS101_ASDU_addInformationObject(newAsdu, (InformationObject) SinglePointInformation_create((SinglePointInformation) io, 302, true, IEC60870_QUALITY_GOOD));
        CS101_ASDU_addInformationObject(newAsdu, (InformationObject) SinglePointInformation_create((SinglePointInformation) io, 303, false, IEC60870_QUALITY_GOOD));
        CS101_ASDU_addInformationObject(newAsdu, (InformationObject) SinglePointInformation_create((SinglePointInformation) io, 304, true, IEC60870_QUALITY_GOOD));
        CS101_ASDU_addInformationObject(newAsdu, (InformationObject) SinglePointInformation_create((SinglePointInformation) io, 305, false, IEC60870_QUALITY_GOOD));
        CS101_ASDU_addInformationObject(newAsdu, (InformationObject) SinglePointInformation_create((SinglePointInformation) io, 306, true, IEC60870_QUALITY_GOOD));
        CS101_ASDU_addInformationObject(newAsdu, (InformationObject) SinglePointInformation_create((SinglePointInformation) io, 307, false, IEC60870_QUALITY_GOOD));

        InformationObject_destroy(io);

        IMasterConnection_sendASDU(connection, newAsdu);

        CS101_ASDU_destroy(newAsdu);
*/
        IMasterConnection_sendACT_TERM(connection, asdu);
  
    }
    else {
        if(qoi == CS101_COT_REQUESTED_BY_GENERAL_COUNTER){  //qoi=37
	CS101_AppLayerParameters alParams = IMasterConnection_getApplicationLayerParameters(connection);

        IMasterConnection_sendACT_CON(connection, asdu, false);

        /* The CS101 specification only allows information objects without timestamp in GI responses */

        CS101_ASDU newAsdu = CS101_ASDU_create(alParams, false,CS101_COT_REQUESTED_BY_GENERAL_COUNTER,
                0, 1, false, false);

        InformationObject io;
        for(uint8_t k=0;k<4;k++)
        {
                io= (InformationObject) IntegratedTotals_create((IntegratedTotals)io,YM_ADD+k,(BinaryCounterReading)&ymtable[k]);
                CS101_ASDU_addInformationObject(newAsdu, io);
        }
        InformationObject_destroy(io);

        IMasterConnection_sendASDU(connection, newAsdu);
       
        CS101_ASDU_destroy(newAsdu);
	
	}else{
	    IMasterConnection_sendACT_CON(connection, asdu, true);
       }
    }

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

void*
server104()
{
    uint8_t i,j;
    /* Add Ctrl-C handler */
    signal(SIGINT, sigint_handler);


    /* create a new slave/server instance with default connection parameters and
     * default message queue size */
    CS104_Slave slave = CS104_Slave_create(100, 100);

    CS104_Slave_setLocalAddress(slave, "0.0.0.0");


    /*lhq 2018-10-15 add tcpport=5010 */
    CS104_Slave_setLocalPort(slave, 5010);

    /* Set mode to a single redundancy group
     * NOTE: library has to be compiled with CONFIG_CS104_SUPPORT_SERVER_MODE_SINGLE_REDUNDANCY_GROUP enabled (=1)
     */
    CS104_Slave_setServerMode(slave, CS104_MODE_SINGLE_REDUNDANCY_GROUP);

    /* get the connection parameters - we need them to create correct ASDUs */
    CS101_AppLayerParameters alParams = CS104_Slave_getAppLayerParameters(slave);

    /* set the callback handler for the clock synchronization command */
    CS104_Slave_setClockSyncHandler(slave, clockSyncHandler, NULL);

    /* set the callback handler for the interrogation command */
    CS104_Slave_setInterrogationHandler(slave, interrogationHandler, NULL);

    /* set handler for other message types */
    CS104_Slave_setASDUHandler(slave, asduHandler, NULL);

    /* set handler to handle connection requests (optional) */
    CS104_Slave_setConnectionRequestHandler(slave, connectionRequestHandler, NULL);

    /* set handler to track connection events (optional) */
    CS104_Slave_setConnectionEventHandler(slave, connectionEventHandler, NULL);

    /* uncomment to log messages */
    //CS104_Slave_setRawMessageHandler(slave, rawMessageHandler, NULL);

    CS104_Slave_start(slave);

    if (CS104_Slave_isRunning(slave) == false) {
        printf("Starting server failed!\n");
        goto exit_program;
    }

    int16_t scaledValue = 0;

    while (running) {

        Thread_sleep(1000);

	//printf("this time is %d\n",time(NULL));

        CS101_ASDU newAsdu = CS101_ASDU_create(alParams, false, CS101_COT_PERIODIC, 0, 1, false, false);

        InformationObject io = (InformationObject) MeasuredValueScaled_create(NULL, 110, scaledValue, IEC60870_QUALITY_GOOD);

        scaledValue++;

        CS101_ASDU_addInformationObject(newAsdu, io);

        InformationObject_destroy(io);

        /* Add ASDU to slave event queue - don't release the ASDU afterwards!
         * The ASDU will be released by the Slave instance when the ASDU
         * has been sent.
         */
//        CS104_Slave_enqueueASDU(slave, newAsdu);

        CS101_ASDU_destroy(newAsdu);
    }

    CS104_Slave_stop(slave);

exit_program:
    CS104_Slave_destroy(slave);

    Thread_sleep(500);
    return 0;
}


int
main(int argc, char** argv)
{
    uint8_t i,j;
//    struct ddb ddblist[BSH];

    if(initddb2(ddblist,BSH)<0){
	printf("init ddb faild ! exit!!!\n");
	return -1;
    }	

    Thread th645,th104;
    /* Add Ctrl-C handler */
    signal(SIGINT, sigint_handler);

   /* for(i=0;i<YC_TABLE_SIZE;i++)
          yctable[i]=0;
    for(i=0;i<YM_TABLE_SIZE;i++)
          ymtable[i]=0;
   */
    th104=Thread_create(server104,NULL,true);
//	printf("th104 thread is %i\n",104);
    Thread_start(th104);
    th645=Thread_create(meter_acq,NULL,true);
//    th645=Thread_create(meter_acq(yctable,ymtable),NULL,true);
    Thread_start(th645);
//        for(int8_t j=0;j<8;j++)
//            printf("yctable  %d  is  %d \n",j,yctable[j]);
    
//    th104=Thread_create(server104,NULL,true);
//	printf("th104 thread is %i\n",104);

//    Thread_start(th104);
    while(running)
    {
        //for(int8_t j=0;j<8;j++)
        //    printf("main yctable  %d  is  %d \n",j,yctable[j]);
	MSLEEP;
    }
   Thread_destroy(th645);
   Thread_destroy(th104);

}
