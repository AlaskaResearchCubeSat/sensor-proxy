#include <msp430.h>
#include <ctl.h>
#include <stdio.h>
#include <string.h>
#include <ctype.h>
#include <ARCbus.h>
#include <UCA1_uart.h>
#include "timerA.h"
#include "sensor-interface.h"
#include <terminal.h>
#include <i2c.h>
#include <Error.h>
#include "Z:\Software\ADCS\ACDS-flight\SensorDataInterface.h"

CTL_TASK_t tasks[4];

//stacks for tasks
unsigned stack1[1+150+1];          
unsigned stack2[1+260+1];
unsigned stack3[1+200+1];   
unsigned stack4[1+420+1];   

CTL_EVENT_SET_t cmd_parse_evt;

unsigned char buffer[80];

//set printf and friends to send chars out UCA1 uart
int __putchar(int c){
  if(async_isOpen()){
     return EOF;
  }
  return UCA1_TxChar(c);
}

//set scanf and friends to read chars from UAC1 uart
int __getchar(void){
    return UCA1_Getc();
}

//handle subsystem specific commands
int SUB_parseCmd(unsigned char src,unsigned char cmd,unsigned char *dat,unsigned short len){
  int i;
  unsigned short time,count;
  switch(cmd){
    case CMD_MAG_SAMPLE_CONFIG:
      //check command
      switch(*dat){
        case MAG_SAMPLE_START:
          if(len!=4){
            return ERR_PK_LEN;
          }
          //get time parameter
          time=dat[1]<<8;
          time|=((unsigned short)dat[2]);
          count=dat[3];
          //run sensor collection
          run_sensors(time,count);
          //success!
          return RET_SUCCESS;
        case MAG_SAMPLE_STOP:
          //check packet length
          if(len!=1){
            //incorrect length
            return ERR_PK_LEN;
          }
          //stop sensors
          stop_sensors();
          //success!
          return RET_SUCCESS;
        case MAG_SINGLE_SAMPLE:
          //check packet length
          if(len!=1){
            //incorrect length
            return ERR_PK_LEN;
          }
          sensors_single_sample();
          //success!
          return RET_SUCCESS;
        case MAG_TEST_MODE_ON:
          //check packet length
          if(len!=1){
            //incorrect length
            return ERR_PK_LEN;
          }
          mag_tx_addr=BUS_ADDR_CDH;
          return RET_SUCCESS;  
        case MAG_TEST_MODE_OFF:
          //check packet length
          if(len!=1){
            //incorrect length
            return ERR_PK_LEN;
          }
          mag_tx_addr=BUS_ADDR_ACDS;
          return RET_SUCCESS;
      }
  }
  //Return Error
  return ERR_UNKNOWN_CMD;
}

void cmd_parse(void *p) __toplevel{
  unsigned int e;
  //init event
  ctl_events_init(&cmd_parse_evt,0);
  for(;;){
    e=ctl_events_wait(CTL_EVENT_WAIT_ANY_EVENTS_WITH_AUTO_CLEAR,&cmd_parse_evt,0x01,CTL_TIMEOUT_NONE,0);
    if(e&0x01){

    }
  }
}

void sub_events(void *p) __toplevel{
  unsigned int e,len;
  int i;
  unsigned char buf[10],*ptr;
  extern unsigned char async_addr;
  for(;;){
    e=ctl_events_wait(CTL_EVENT_WAIT_ANY_EVENTS_WITH_AUTO_CLEAR,&SUB_events,SUB_EV_ALL|SUB_EV_ASYNC_OPEN|SUB_EV_ASYNC_CLOSE,CTL_TIMEOUT_NONE,0);
    if(e&SUB_EV_PWR_OFF){
      //print message
      puts("System Powering Down\r");
    }
    if(e&SUB_EV_PWR_ON){
      //print message
      puts("System Powering Up\r");
    }
    if(e&SUB_EV_SEND_STAT){
      //send status
      //puts("Sending status\r");
      //setup packet 
      //TODO: put actual command for subsystem response
      ptr=BUS_cmd_init(buf,20);
      //TODO: fill in telemitry data
      //send command
      BUS_cmd_tx(BUS_ADDR_CDH,buf,0,0,BUS_I2C_SEND_FOREGROUND);
    }
    if(e&SUB_EV_SPI_DAT){
      //get length
      len=arcBus_stat.spi_stat.len;
      switch(arcBus_stat.spi_stat.rx[0]){
          case SPI_ERROR_DAT:
            //print errors
            print_spi_err(arcBus_stat.spi_stat.rx,len);
          break;
          default:
              puts("Unknown SPI data recived:\r");
              //print out data
              for(i=0;i<len;i++){
                //printf("0x%02X ",rx[i]);
                printf("%03i ",arcBus_stat.spi_stat.rx[i]);
              }
              printf("\r\n");
      }
      //free buffer
      BUS_free_buffer_from_event();
    }
    if(e&SUB_EV_SPI_ERR_CRC){
      puts("SPI bad CRC\r");
    }
    if(e&SUB_EV_ASYNC_OPEN){
      //async connection not possible, close connection
      async_close();
    }
  }
}

int main(void){
  //DO this first
  ARC_setup(); 
  
  //setup system specific peripherals

  //setup mmc interface
  //mmcInit_msp();
 
  //setup magnetomitor
  mag_init();
 
  //setup UCA1 UART
  UCA1_init_UART();
  //setup baud rate for Bluetooth
  UCA1_BR57600();

  //init I2C interface
  initI2C();
  
  //setup error reporting library
  error_init();
  //TESTING: set log level to report everything by default
  set_error_level(0);
     
  //disable timesliceing so that communication is possible without CDH
  BUS_set_test_mode(BUS_TM_NO_TIMESLICE);
  
  //setup P7 for LED's
  P7OUT=0x80;
  P7DIR=0xFF;

  //setup bus interface
  initARCbus(BUS_ADDR_LEDL);

  //initialize stacks
  memset(stack1,0xcd,sizeof(stack1));  // write known values into the stack
  stack1[0]=stack1[sizeof(stack1)/sizeof(stack1[0])-1]=0xfeed; // put marker values at the words before/after the stack
  
  memset(stack2,0xcd,sizeof(stack2));  // write known values into the stack
  stack2[0]=stack2[sizeof(stack2)/sizeof(stack2[0])-1]=0xfeed; // put marker values at the words before/after the stack
    
  memset(stack3,0xcd,sizeof(stack3));  // write known values into the stack
  stack3[0]=stack3[sizeof(stack3)/sizeof(stack3[0])-1]=0xfeed; // put marker values at the words before/after the stack
      
  memset(stack4,0xcd,sizeof(stack4));  // write known values into the stack
  stack4[0]=stack4[sizeof(stack4)/sizeof(stack4[0])-1]=0xfeed; // put marker values at the words before/after the stack

  //create tasks
  ctl_task_run(&tasks[0],1,cmd_parse,NULL,"cmd_parse",sizeof(stack1)/sizeof(stack1[0])-2,stack1+1,0);
  ctl_task_run(&tasks[1],2,terminal,"ACDS Sensor Proxy","terminal",sizeof(stack2)/sizeof(stack2[0])-2,stack2+1,0);
  ctl_task_run(&tasks[2],10,sub_events,NULL,"sub_events",sizeof(stack3)/sizeof(stack3[0])-2,stack3+1,0);
  ctl_task_run(&tasks[3],15,ACDS_sensor_interface,NULL,"ACDS_sensor",sizeof(stack4)/sizeof(stack4[0])-2,stack4+1,0);
  
  mainLoop();
}

