#include <string.h>
#include <ctype.h>
#include <stdio.h>
#include <stdlib.h>
#include <msp430.h>
#include <ctl_api.h>
#include <terminal.h>
#include <ARCbus.h>
#include <UCA1_uart.h>
#include <Error.h>
#include <i2c.h>
#include "sensor-interface.h"
#include "Error_src.h"

typedef struct{
  const char *name;
  unsigned char addr;
}SYM_ADDR;

const SYM_ADDR busAddrSym[]= {{"LEDL",BUS_ADDR_LEDL},
                              {"ACDS",BUS_ADDR_ACDS},
                              {"COMM",BUS_ADDR_COMM},
                              {"IMG",BUS_ADDR_IMG},
                              {"CDH",BUS_ADDR_CDH},
                              {"GC",BUS_ADDR_GC},
                              {NULL,0}};
              
const SYM_ADDR magAddrSym[]= {{"X+",0x14},
                              {"X-",0x16},
                              {"Y+",0x26},
                              {"Y-",0x34},
                              {"Z+",0x25},
                              {"Z-",0x24},
                              {NULL,0}};
                                                           
const SYM_ADDR tempAddrSym[]={{"X+",0x48},
                              {"X-",0x4A},
                              {"Y+",0x49},
                              {"Y-",0x4C},
                              {"Z+",0x4E},
                              {"Z-",0x4D},
                              {"Tb",0x4F},
                              {NULL,0}};

//helper function to parse I2C address
//if res is true reject reserved addresses
unsigned char getI2C_addr(char *str,short res,const SYM_ADDR *syms){
  unsigned long addr;
  unsigned char tmp;\
  int i;
  char *end;
  //attempt to parse a numeric address
  addr=strtol(str,&end,0);
  //check for errors
  if(end==str){
    //check for symbolic matches
    /*if(!strcmp(str,"LEDL")){
      return BUS_ADDR_LEDL;
    }else if(!strcmp(str,"ACDS")){
      return BUS_ADDR_ACDS;
    }else if(!strcmp(str,"COMM")){
      return BUS_ADDR_COMM;
    }else if(!strcmp(str,"IMG")){
      return BUS_ADDR_IMG;
    }else if(!strcmp(str,"CDH")){
      return BUS_ADDR_CDH;
    }else if(!strcmp(str,"GC")){
      return BUS_ADDR_GC;
    }*/
    if(syms!=NULL){
      for(i=0;syms[i].name!=NULL && syms[i].addr!=0;i++){
        if(!strcmp(str,syms[i].name)){
          return syms[i].addr;
        }
      }
    }
    //not a known address, error
    printf("Error : could not parse address \"%s\".\r\n",str);
    return 0xFF;
  }
  if(*end!=0){
    printf("Error : unknown sufix \"%s\" at end of address\r\n",end);
    return 0xFF;
  }
  //check address length
  if(addr>0x7F){
    printf("Error : address 0x%04lX is not 7 bits.\r\n",addr);
    return 0xFF;
  }
  //check for reserved address
  tmp=0x78&addr;
  if((tmp==0x00 || tmp==0x78) && res){
    printf("Error : address 0x%02lX is reserved.\r\n",addr);
    return 0xFF;
  }
  //return address
  return addr;
}

//reset a MSP430 on command
int restCmd(char **argv,unsigned short argc){
  unsigned char buff[10];
  unsigned char addr;
  unsigned short all=0;
  int resp;
  //force user to pass no arguments to prevent unwanted resets
  if(argc>1){
    puts("Error : too many arguments\r");
    return -1;
  }
  if(argc!=0){
    if(!strcmp(argv[1],"all")){
      all=1;
      addr=BUS_ADDR_GC;
    }else{
      //get address
      addr=getI2C_addr(argv[1],0,busAddrSym);
      if(addr==0xFF){
        return 1;
      }
    }
    //setup packet 
    BUS_cmd_init(buff,CMD_RESET);
    resp=BUS_cmd_tx(addr,buff,0,0,BUS_I2C_SEND_FOREGROUND);
    switch(resp){
      case 0:
        puts("Command Sent Sucussfully.\r");
      break;
      case ERR_TIMEOUT:
        puts("IIC timeout Error.\r");
      break;
    }
  }
  //reset if no arguments given or to reset all boards
  if(argc==0 || all){
    //wait for UART buffer to empty
    while(UCA1_CheckBusy());
    //write to WDTCTL without password causes PUC
    reset(ERR_LEV_INFO,SENP_ERR_SRC_CMD,CMD_ERR_RESET,0);
    //Never reached due to reset
    puts("Error : Reset Failed!\r");
  }
  return 0;
}

//set priority for tasks on the fly
int priorityCmd(char **argv,unsigned short argc){
  extern CTL_TASK_t *ctl_task_list;
  int i,found=0;
  CTL_TASK_t *t=ctl_task_list;
  if(argc<1 || argc>2){
    printf("Error: %s takes one or two arguments, but %u are given.\r\n",argv[0],argc);
    return -1;
  }
  while(t!=NULL){
    if(!strcmp(t->name,argv[1])){
      found=1;
      //match found, break
      break;
    }
    t=t->next;
  }
  //check that a task was found
  if(found==0){
      //no task found, return
      printf("Error: could not find task named %s.\r\n",argv[1]);
      return -3;
  }
  //print original priority
  printf("\"%s\" priority = %u\r\n",t->name,t->priority);
  if(argc==2){
      unsigned char val=atoi(argv[2]);
      if(val==0){
        printf("Error: invalid priority.\r\n");
        return -2;
      }
      //set priority
      ctl_task_set_priority(t,val);
      //print original priority
      printf("new \"%s\" priority = %u\r\n",t->name,t->priority);
  }
  return 0;
}

//get/set ctl_timeslice_period
int timesliceCmd(char **argv,unsigned short argc){
  if(argc>1){
    printf("Error: too many arguments.\r\n");
    return 0;
  }
  //if one argument given then set otherwise get
  if(argc==1){
    int en;
    CTL_TIME_t val=atol(argv[1]);
    //check value
    if(val==0){
      printf("Error: bad value.\r\n");
      return -1;
    }
    //disable interrupts so that opperation is atomic
    en=ctl_global_interrupts_set(0);
    ctl_timeslice_period=val;
    ctl_global_interrupts_set(en);
  }
  printf("ctl_timeslice_period = %ul\r\n",ctl_timeslice_period);
  return 0;
}

//return state name
const char *stateName(unsigned char state){
  switch(state){
    case CTL_STATE_RUNNABLE:
      return "CTL_STATE_RUNNABLE";
    case CTL_STATE_TIMER_WAIT:
      return "CTL_STATE_TIMER_WAIT";
    case CTL_STATE_EVENT_WAIT_ALL:
      return "CTL_STATE_EVENT_WAIT_ALL";
    case CTL_STATE_EVENT_WAIT_ALL_AC:
      return "CTL_STATE_EVENT_WAIT_ALL_AC";
    case CTL_STATE_EVENT_WAIT_ANY:
      return "CTL_STATE_EVENT_WAIT_ANY";
    case CTL_STATE_EVENT_WAIT_ANY_AC:
      return "CTL_STATE_EVENT_WAIT_ANY_AC";
    case CTL_STATE_SEMAPHORE_WAIT:
      return "CTL_STATE_SEMAPHORE_WAIT";
    case CTL_STATE_MESSAGE_QUEUE_POST_WAIT:
      return "CTL_STATE_MESSAGE_QUEUE_POST_WAIT";
    case CTL_STATE_MESSAGE_QUEUE_RECEIVE_WAIT:
      return "CTL_STATE_MESSAGE_QUEUE_RECEIVE_WAIT";
    case CTL_STATE_MUTEX_WAIT:
      return "CTL_STATE_MUTEX_WAIT";
    case CTL_STATE_SUSPENDED:
      return "CTL_STATE_SUSPENDED";
    default:
      return "unknown state";
  }
}

//print the status of all tasks in a table
int statsCmd(char **argv,unsigned short argc){
  extern CTL_TASK_t *ctl_task_list;
  int i;
  CTL_TASK_t *t=ctl_task_list;
  //format string
  const char *fmt="%-10s\t%u\t\t%c%-28s\t%lu\r\n";
  //print out nice header
  printf("\r\nName\t\tPriority\tState\t\t\t\tTime\r\n--------------------------------------------------------------------\r\n");
  //loop through tasks and print out info
  while(t!=NULL){
    printf(fmt,t->name,t->priority,(t==ctl_task_executing)?'*':' ',stateName(t->state),t->execution_time);
    t=t->next;
  }
  //add a blank line after table
  printf("\r\n");
  return 0;
}

//transmit command over I2C
int txCmd(char **argv,unsigned short argc){
  unsigned char buff[10],*ptr,id;
  unsigned char addr;
  unsigned short len;
  unsigned int e;
  char *end;
  int i,resp,nack=BUS_CMD_FL_NACK;
  if(!strcmp(argv[1],"noNACK")){
    nack=0;
    //shift arguments
    argv[1]=argv[0];
    argv++;
    argc--;
  }
  //check number of arguments
  if(argc<2){
    printf("Error : too few arguments.\r\n");
    return 1;
  }
  if(argc>sizeof(buff)){
    printf("Error : too many arguments.\r\n");
    return 2;
  }
  //get address
  addr=getI2C_addr(argv[1],0,busAddrSym);
  if(addr==0xFF){
    return 1;
  }
  //get packet ID
  id=strtol(argv[2],&end,0);
  if(end==argv[2]){
      printf("Error : could not parse element \"%s\".\r\n",argv[2]);
      return 2;
  }
  if(*end!=0){
    printf("Error : unknown sufix \"%s\" at end of element \"%s\"\r\n",end,argv[2]);
    return 3;
  }
  //setup packet 
  ptr=BUS_cmd_init(buff,id);
  //pares arguments
  for(i=0;i<argc-2;i++){
    ptr[i]=strtol(argv[i+3],&end,0);
    if(end==argv[i+1]){
        printf("Error : could not parse element \"%s\".\r\n",argv[i+3]);
        return 2;
    }
    if(*end!=0){
      printf("Error : unknown sufix \"%s\" at end of element \"%s\"\r\n",end,argv[i+3]);
      return 3;
    }
  }
  len=i;
  resp=BUS_cmd_tx(addr,buff,len,nack,BUS_I2C_SEND_FOREGROUND);
  switch(resp){
    case RET_SUCCESS:
      printf("Command Sent Sucussfully.\r\n");
    break;
  }
  //check if an error occured
  if(resp<0){
    printf("Error : unable to send command\r\n");
  }
  printf("Resp = %i\r\n",resp);
  return 0;
}

//Send data over SPI
int spiCmd(char **argv,unsigned short argc){
  unsigned char addr;
  char *end;
  unsigned short crc;
  //static unsigned char rx[2048+2];
  unsigned char *rx=NULL;
  int resp,i,len=100;
  if(argc<1){
    printf("Error : too few arguments.\r\n");
    return 3;
  }
  //get address
  addr=getI2C_addr(argv[1],0,busAddrSym);
  if(addr==0xFF){
    return 1;
  }
  if(argc>=2){
    //Get packet length
    len=strtol(argv[2],&end,0);
    if(end==argv[2]){
        printf("Error : could not parse length \"%s\".\r\n",argv[2]);
        return 2;
    }
    if(*end!=0){
      printf("Error : unknown sufix \"%s\" at end of length \"%s\"\r\n",end,argv[2]);
      return 3;
    }    
    if(len+2>BUS_get_buffer_size()){
      printf("Error : length is too long. Maximum Length is %u\r\n",BUS_get_buffer_size());
      return 4;
    }
  }
  //get buffer, set a timeout of 2 secconds
  rx=BUS_get_buffer(CTL_TIMEOUT_DELAY,2048);
  //check for error
  if(rx==NULL){
    printf("Error : Timeout while waiting for buffer.\r\n");
    return -1;
  }
  //fill buffer with "random" data
  for(i=0;i<len;i++){
    rx[i]=i;
  }
  //send data
  //TESTING: set pin high
  P8OUT|=BIT0;
  //send SPI data
  resp=BUS_SPI_txrx(addr,rx,rx,len);
  //TESTING: wait for transaction to fully complete
  while(UCB0STAT&UCBBUSY);
  //TESTING: set pin low
  P8OUT&=~BIT0;
  //check return value
  if(resp==RET_SUCCESS){
      //print out data message
      printf("SPI data recived\r\n");
      //print out data
      for(i=0;i<len;i++){
        //printf("0x%02X ",rx[i]);
        printf("%03i ",rx[i]);
      }
      printf("\r\n");
  }else{
    printf("%s\r\n",BUS_error_str(resp));
  }
  //free buffer
  BUS_free_buffer();
  return 0;
}

int printCmd(char **argv,unsigned short argc){
  unsigned char buff[40],*ptr,id;
  unsigned char addr;
  unsigned short len;
  int i,j,k;
  //check number of arguments
  if(argc<2){
    printf("Error : too few arguments.\r\n");
    return 1;
  }
  //get address
  addr=getI2C_addr(argv[1],0,busAddrSym);
  if(addr==0xFF){
    return 1;
  }
  //setup packet 
  ptr=BUS_cmd_init(buff,6);
  //coppy strings into buffer for sending
  for(i=2,k=0;i<=argc && k<sizeof(buff);i++){
    j=0;
    while(argv[i][j]!=0){
      ptr[k++]=argv[i][j++];
    }
    ptr[k++]=' ';
  }
  //get length
  len=k;
  //TESTING: set pin high
  P8OUT|=BIT0;
  //send command
  BUS_cmd_tx(addr,buff,len,0,BUS_I2C_SEND_FOREGROUND);
  //TESTING: set pin low
  P8OUT&=~BIT0;
  return 0;
}

int tstCmd(char **argv,unsigned short argc){
  unsigned char buff[40],*ptr,*end;
  unsigned char addr;
  unsigned short len;
  int i,j,k;
  //check number of arguments
  if(argc<2){
    printf("Error : too few arguments.\r\n");
    return 1;
  }
  if(argc>2){
    printf("Error : too many arguments.\r\n");
    return 1;
  }
  //get address
  addr=getI2C_addr(argv[1],0,busAddrSym);
  len = atoi(argv[2]);
  /*if(len<0){
    printf("Error : bad length");
    return 2;
  }*/
  //setup packet 
  ptr=BUS_cmd_init(buff,7);
  //fill packet with dummy data
  for(i=0;i<len;i++){
    ptr[i]=i;
  }
  //TESTING: set pin high
  P8OUT|=BIT0;
  //send command
  BUS_cmd_tx(addr,buff,len,0,BUS_I2C_SEND_FOREGROUND);
  //TESTING: wait for transaction to fully complete
  while(UCB0STAT&UCBBUSY);
  //TESTING: set pin low
  P8OUT&=~BIT0;
  return 0;
}

//print current time
int timeCmd(char **argv,unsigned short argc){
  printf("time ticker = %li\r\n",get_ticker_time());
  return 0;
}

int asyncCmd(char **argv,unsigned short argc){
   //TODO: do this better
   extern int remote;
   char c;
   int err;
   CTL_EVENT_SET_t e=0,evt;
   unsigned char addr;
   if(argc>1){
    printf("Error : %s takes 0 or 1 arguments\r\n",argv[0]);
    return -1;
  }
  if(argc==1){
    addr=getI2C_addr(argv[1],0,busAddrSym);
    if(addr==0xFF){
      return -1;
    }
    //print out address
    printf("Using Address 0x%02X\r\n",addr);
    //try to open async connection
    if((err=async_open(addr))){
      //print error
      printf("Error : opening async\r\n%s\r\n",BUS_error_str(err));
      //return error
      return -2;
    }
    //Tell the user that async is open
    printf("async open use ^C to force quit\r\n");
    //setup events
    async_setup_events(&e,0,1<<0);
    UCA1_setup_events(&e,0,1<<1);
    async_setup_close_event(&e,1<<2);
    //stop printf from printing to UART
    remote=1;
    for(;;){
      //wait for event
      evt=ctl_events_wait(CTL_EVENT_WAIT_ANY_EVENTS,&e,0x07,CTL_TIMEOUT_NONE,0);
      //check for char from UART
      if(evt&(1<<1)){
        //get char
        c=UCA1_Getc();
        //check for ^C
        if(c==0x03){
          //close connection
          async_close();
          //allow printf to work
          remote=0;
          //print message
          printf("\r\nConnection terminated by user\r\n");
          //exit loop
          break;
        }
        //send char over async
        async_TxChar(c);
      }
      //check for char from async
      if(evt&(1<<0)){
        //get char from async
        c=async_Getc(); 
        //print char to UART
        UCA1_TxChar(c);
      }
      if(evt&(1<<2)){
        //allow printf to work
        remote=0;
        //print message
        printf("\r\nconnection closed remotely\r\n");
        //exit loop
        break;
      }
    }
    //stop monitoring events
    async_setup_events(NULL,0,0);
    UCA1_setup_events(NULL,0,0);
    async_setup_close_event(NULL,0);
  }else{
    if(async_close()){
      printf("Error : async_close() failed.\r\n");
    }
  }
}

int readCmd(char **argv,unsigned short argc){
  trigger_read();
  return 0;
}

//run magnetometer
//read data from the magnetometer and print to terminal
int magCmd(char **argv,unsigned short argc){
  signed short set[2],reset[2],os[2],val[2];
  unsigned short single=0,gauss=0,addr=0x14;
  unsigned char c;
  float time=0;
  int i,res;

  //parse arguments
  for(i=1;i<=argc;i++){
    if(!strcmp("single",argv[i])){
      single=1;
    }else if(!strcmp("gauss",argv[i])){
      gauss=1;
    }else if((addr=getI2C_addr(argv[i],0,magAddrSym))!=0xFF){
      mag_addr=addr;
    }else{
      
      printf("Error Unknown argument \'%s\'.\r\n",argv[i]);
      return -1;
    }
  }
  //run until abort is detected
  do{
    res=do_conversion();
    if(res!=0){
      printf("Error encountered. Aborting\r\n");
      break;
    }
    if(gauss){
      printf("%f %f\r\n",ADCtoGauss(magMem[0])/2,ADCtoGauss(magMem[1])/2);
    }else{
      printf("%li %li\r\n",magMem[0],magMem[1]);
    }
    c=UCA1_CheckKey();
  }while(!(c==0x03  || c=='Q' || c=='q' || single));
  return 0;
}

//start taking sensor data and sending to ACDS
int run_sensors_Cmd(char **argv,unsigned short argc){
  unsigned short time=32768,count=0;
  //check for arguments
  if(argc==2){
    time=atoi(argv[1]);
    count=atoi(argv[2]);
  }else if(argc!=0){
      printf("Error : %s takes 0 or 2 arguments\r\n",argv[0]);
      return -1;
  }
  //setup sensors
  run_sensors(time,count);
  return 0;
}
  
//stop taking sensor data
int stop_sensors_Cmd(char **argv,unsigned short argc){
  stop_sensors();
  return 0;
}

//set which errors are logged
int logCmd(char **argv,unsigned short argc){
  const unsigned char logLevels[]={ERR_LEV_DEBUG,ERR_LEV_INFO,ERR_LEV_WARNING,ERR_LEV_ERROR,ERR_LEV_CRITICAL};
  const char *(levelNames[])={"debug","info","warn","error","critical"};
  int found,i;
  unsigned char level;
  //check for too many arguments
  if(argc>1){
    printf("Error : %s takes 0 or 1 arguments\r\n",argv[0]);
    return -1;
  }
  //check if argument given
  if(argc==1){
    if(!strcmp("levels",argv[1])){
      //print a list of level names
      for(i=0;i<sizeof(logLevels)/sizeof(logLevels[0]);i++){
        printf("% 3u - %s\r\n",logLevels[i],levelNames[i]);
       }
       return 0;
    }
    //check for matching level names
    for(i=0;i<sizeof(logLevels)/sizeof(logLevels[0]);i++){
      if(!strcmp(levelNames[i],argv[1])){
        //match found
        found=1;
        //set log level
        level=logLevels[i];
        //done
        break;
      }
    }
    //check if there was a matching name
    if(!found){
      //get convert to integer
      level=atoi(argv[1]);
    }
    //set log level
    set_error_level(level);
  }
  //print (new) log level
  printf("Log level = %u\r\n",get_error_level());
  return 0;
}

int SR_Cmd(char **argv,unsigned short argc){
  unsigned char addr=0x14;
  long dat[4];
  if(argc>1){
    printf("Error : Too many arguments\r\n");
    return -1;
  }
  if(argc==1){
    addr=getI2C_addr(argv[1],0,magAddrSym);
    if(addr==0xFF){
      return -1;
    }
  }
  //generate set pulse
  MAG_SR_OUT|=MAG_SR_PIN;
  //delay for pulse
  ctl_timeout_wait(ctl_get_current_time()+2);
  //take a sample
  if(!single_sample(addr,dat)){
    printf("Error with read. Aborting\r\n");
    return -1;
  }
  //generate set pulse
  MAG_SR_OUT&=~MAG_SR_PIN;
  //delay for pulse
  ctl_timeout_wait(ctl_get_current_time()+2);
  //take a sample
  if(!single_sample(addr,dat+2)){
    printf("Error reading sample\r\n");
    return -2;
  }
  //print sample 
  printf("%li\t%li\t%li\t%li\r\n",dat[0],dat[1],dat[2],dat[3]);
  return 0;
}


int I2C_scan_Cmd(char **argv,unsigned short argc){
  int i,res,found=0;
  unsigned char rx[3];
  for(i=0;i<256;i++){
    res=i2c_rx(i,rx,3);
    if(res>=0){
      printf("Device Found at : %i\r\n",i);
      found++;
    }else if(res!=I2C_ERR_NACK){
        printf("Error Encountered \"%s\". Aborting at %i\r\n",I2C_error_str(res),i);
        return 1;
    }
  }
  if(!found){
    printf("Scan Complete, No Devices found\r\n");
  }else{
    printf("Scan Complete, %i Devices found\r\n",found);
  }
  return 0;
}

//table of commands with help
const CMD_SPEC cmd_tbl[]={{"help"," [command]\r\n\t""get a list of commands or help on a spesific command.",helpCmd},
                         {"priority"," task [priority]\r\n\t""Get/set task priority.",priorityCmd},
                         {"timeslice"," [period]\r\n\t""Get/set ctl_timeslice_period.",timesliceCmd},
                         {"stats","\r\n\t""Print task status",statsCmd},
                         {"reset","\r\n\t""reset the msp430.",restCmd},
                         {"tx"," [noACK] [noNACK] addr ID [[data0] [data1]...]\r\n\t""send data over I2C to an address",txCmd},
                         {"SPI","addr [len]\r\n\t""Send data using SPI.",spiCmd},
                         {"print"," addr str1 [[str2] ... ]\r\n\t""Send a string to addr.",printCmd},
                         {"tst"," addr len\r\n\t""Send test data to addr.",tstCmd},
                         {"time","\r\n\t""Return current time.",timeCmd},
                         {"async","[addr]\r\n\t""Open connection if address given. otherwise close connection.",asyncCmd},
                         {"read","\r\n\t""Trigger a sensor read",readCmd},
                         {"mag","\r\n\t""Read From The magnetomitor",magCmd},
                         {"run","[time count]\r\n\t""Start taking sensor readings",run_sensors_Cmd},
                         {"stop","\r\n\t""stop taking sensor readings",stop_sensors_Cmd},
                         {"log","[level]\r\n\t""get/set log level",logCmd},
                         {"SR","\r\n\t""Set pulse, sample, reset pulse, sample",SR_Cmd},
                         {"I2Cscan","\r\nScan I2C bus for devices",I2C_scan_Cmd},
                         //end of list
                         {NULL,NULL,NULL}};
