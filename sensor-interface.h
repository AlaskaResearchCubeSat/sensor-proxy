#ifndef __SENSOR_INTERFCE_H
#define __SENSOR_INTERFCE_H


  //Defines for set reset pin
  #define MAG_SR_PIN         BIT4
  #define MAG_SR_OUT         P8OUT
  #define MAG_SR_DIR         P8DIR
  #define MAG_SR_SEL         P8SEL
  #define MAG_SR_REN         P8REN

  //sensor read event
  #define SENS_EV_READ    0x0001


  #define MAG_A_CH        LTC24xx_CH_1
  #define MAG_B_CH        LTC24xx_CH_0
  
  //#define MAG_ADC_GAIN    LTC24xx_GAIN64
  //#define MAG_ADC_GAIN    LTC24xx_GAIN128
  
  //gain of magnetomitor amplifier
  //#define AMP_GAIN    (2.49e6/5.1e3)    // V/V0
  #define AMP_GAIN    (64)    // V/V
  //#define AMP_GAIN    (128)    // V/V
  //#define AMP_GAIN    (5.11e6/5.1e3)    // V/V
  //sensitivity of magnetomitor
  #define MAG_SENS    (1e-3)            // mV/V/Gauss
  
  //error codes for sensor functions
  enum{SENS_ERR_SETUP,SENS_ERR_CONV_READ,SENS_ERR_BAD_GAIN,SENS_ERR_INSUFFICIENT_DATA} ;
    
  //index:    0    1    2    3    4    5
  //board:    X+   X-   Y+   Y-   Z+   Z- 
  //a-axis:   Y+   Y-   X-   X+   Y-   Y+
  //b-axis:   Z+   Z+   Z+   Z+   X-   X+
    
  //flags for magFLags
  #define MAG_FLAGS_X0      (1<<(2*2+0))      //Y+ board a-axis
  #define MAG_FLAGS_X1      (1<<(2*3+0))      //Y- board a-axis
  #define MAG_FLAGS_X2      (1<<(2*4+1))      //Z+ board b-axis
  #define MAG_FLAGS_X3      (1<<(2*5+1))      //Z- board b-axis
  #define MAG_FLAGS_X       (MAG_FLAGS_X0|MAG_FLAGS_X1|MAG_FLAGS_X2|MAG_FLAGS_X3)
  
  #define MAG_FLAGS_Y0      (1<<(2*0+0))      //X+ board a-axis
  #define MAG_FLAGS_Y1      (1<<(2*1+0))      //X- board a-axis
  #define MAG_FLAGS_Y2      (1<<(2*4+0))      //Z+ board a-axis
  #define MAG_FLAGS_Y3      (1<<(2*5+0))      //Z- board a-axis
  #define MAG_FLAGS_Y       (MAG_FLAGS_Y0|MAG_FLAGS_Y1|MAG_FLAGS_Y2|MAG_FLAGS_Y3)
  
  #define MAG_FLAGS_Z0      (1<<(2*0+1))      //X+ board b-axis    
  #define MAG_FLAGS_Z1      (1<<(2*1+1))      //X- board b-axis
  #define MAG_FLAGS_Z2      (1<<(2*2+1))      //Y+ board b-axis
  #define MAG_FLAGS_Z3      (1<<(2*3+1))      //Y- board b-axis
  #define MAG_FLAGS_Z       (MAG_FLAGS_Z0|MAG_FLAGS_Z1|MAG_FLAGS_Z2|MAG_FLAGS_Z3)

  
  //magnetometer point
    typedef union{
      struct {
        short a,b;
      }c;
      short elm[2];
    } MAG_POINT;
  
  //TESTING: trigger a sensor read
  void trigger_read(void);
  
  void ACDS_sensor_interface(void *p);
  
  void mag_init(void);
  
  float ADCtoGauss(long adc);

  short do_conversion(void);

  extern MAG_POINT magMem[6];
  extern unsigned short magFlags;


  void run_sensors(unsigned short time,short count);

  void stop_sensors(void);

  void sensors_single_sample(void);

  short single_sample(unsigned short addr,long *dest);

  extern unsigned short mag_ADC_gain;

#endif
