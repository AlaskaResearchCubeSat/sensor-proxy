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
#include <commandLib.h>
#include "sensor-interface.h"
#include "Error_src.h"


int readCmd(char **argv,unsigned short argc){
  trigger_read();
  return 0;
}

//run magnetometer
//read data from the magnetometer and print to terminal
int magCmd(char **argv,unsigned short argc){
  signed short set[2],reset[2],os[2],val[2];
  unsigned short single=0,gauss=0,addr=0x14,all=0;
  unsigned char c,mag_addr=0x14;
  long result[2];
  float time=0;
  int i,res;

  //parse arguments
  for(i=1;i<=argc;i++){
    if(!strcmp("single",argv[i])){
        single=1;
    }else if(!strcmp("gauss",argv[i])){
        gauss=1;
    }else if(!strcmp("all",argv[i])){
        all=1;
    }else if((addr=getI2C_addr(argv[i],0,magAddrSym))!=0xFF){
        mag_addr=addr;
    }else{
      
      printf("Error Unknown argument \'%s\'.\r\n",argv[i]);
      return -1;
    }
  }
  //run until abort is detected
    do{
        if(all){
            //read from all sensors
            res=do_conversion();
        }else{
            //read from one sensor
            res=single_sample(mag_addr,result);
        }
        //check for error
        if(res!=0){
            printf("Error encountered. Aborting\r\n");
            break;
        }
        
        if(all){
            for(i=0;i<6;i++){
                //check if reading is valid
                if(magFlags&(1<<(2*i+0)) &&magFlags&(1<<(2*i+1))){
                    //valid, print it
                    if(gauss){
                        printf("%f %f",ADCtoGauss(magMem[i].c.a)/2,ADCtoGauss(magMem[i].c.b)/2);
                    }else{
                        printf("(%li %li)\t",magMem[i].c.a,magMem[i].c.b);
                    }
                }else{
                    //invalid, print invalid
                    printf("(### ###)\t");
                }
            }
            //print newline
            printf("\r\n");    
        }else{
            if(gauss){
                printf("%f %f\r\n",ADCtoGauss(result[0])/2,ADCtoGauss(result[1])/2);
            }else{
                printf("%li %li\r\n",result[0],result[1]);
            }
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

int gain_Cmd(char **argv,unsigned short argc){
  #ifdef MAG_ADC_GAIN
    if(argc>0){
      printf("Error : gain has been hard coded %s takes no arguments\r\n",argv[0]);
      return 1;
    }
    switch(MAG_ADC_GAIN){
      case LTC24xx_GAIN1:
        printf("ADC gain = 1\r\n");
      break;
      case LTC24xx_GAIN4:
        printf("ADC gain = 4\r\n");
      break;
      case LTC24xx_GAIN8:
        printf("ADC gain = 8\r\n");
      break;
      case LTC24xx_GAIN16:
        printf("ADC gain = 16\r\n");
      break;
      case LTC24xx_GAIN32:
        printf("ADC gain = 32\r\n");
      break;
      case LTC24xx_GAIN64:
        printf("ADC gain = 64\r\n");
      break;
      case LTC24xx_GAIN128:
        printf("ADC gain = 128\r\n");
      break;
      case LTC24xx_GAIN264:
        printf("ADC gain = 264\r\n");
      break;
      default:
        printf("Error : unknown hardcoded gain\r\n");
      break;
    }
  #else
    int gain;
    if(argc>1){
      printf("Error : Too many Arguments\r\n");
      return 2;
    }
    if(argc==1){
      gain=atoi(argv[1]);
      switch(gain){
        case 1:
        case 4:
        case 8:
        case 16:
        case 32:
        case 64:
        case 128:
        case 264:
        break;
        default:
          printf("Error : %i is not a valid gain fo the LTC2487\r\n");
          return 3;
      }
      mag_ADC_gain=gain;
   }
   printf("ADC gain = %u\r\n",mag_ADC_gain);
  #endif  
}

//table of commands with help
const CMD_SPEC cmd_tbl[]={{"help"," [command]\r\n\t""get a list of commands or help on a spesific command.",helpCmd},
                         CTL_COMMANDS,ARC_COMMANDS,ARC_ASYNC_PROXY_COMMAND,REPLAY_ERROR_COMMAND,ERROR_LOG_LEVEL_COMMAND,I2C_SCAN_COMMAND,ARC_SPI_DREAD,
                         {"read","\r\n\t""Trigger a sensor read",readCmd},
                         {"mag","\r\n\t""Read From The magnetometer",magCmd},
                         {"run","[time count]\r\n\t""Start taking sensor readings",run_sensors_Cmd},
                         {"stop","\r\n\t""stop taking sensor readings",stop_sensors_Cmd},
                         {"SR","\r\n\t""Set pulse, sample, reset pulse, sample",SR_Cmd},
                         {"gain","[gain]\r\n\t""get/set ADC gain for magnetometer",gain_Cmd},
                         //end of list
                         {NULL,NULL,NULL}};
