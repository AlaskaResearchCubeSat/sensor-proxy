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
    
  //flags for magFLags
  #define MAG_FLAGS_X0      0x0001
  #define MAG_FLAGS_X1      0x0002
  #define MAG_FLAGS_X2      0x0004
  #define MAG_FLAGS_X3      0x0008
  #define MAG_FLAGS_X       (MAG_FLAGS_X0|MAG_FLAGS_X1|MAG_FLAGS_X2|MAG_FLAGS_X3)
  
  #define MAG_FLAGS_Y0      0x0010
  #define MAG_FLAGS_Y1      0x0020
  #define MAG_FLAGS_Y2      0x0040
  #define MAG_FLAGS_Y3      0x0080
  #define MAG_FLAGS_Y       (MAG_FLAGS_Y0|MAG_FLAGS_Y1|MAG_FLAGS_Y2|MAG_FLAGS_Y3)
  
  #define MAG_FLAGS_Z0      0x0100
  #define MAG_FLAGS_Z1      0x0200
  #define MAG_FLAGS_Z2      0x0400
  #define MAG_FLAGS_Z3      0x0800
  #define MAG_FLAGS_Z       (MAG_FLAGS_Z0|MAG_FLAGS_Z1|MAG_FLAGS_Z2|MAG_FLAGS_Z3)

  
  //TESTING: trigger a sensor read
  void trigger_read(void);
  
  void ACDS_sensor_interface(void *p);
  
  void mag_init(void);
  
  float ADCtoGauss(long adc);

  short do_conversion(void);

  extern unsigned long magMem[12];
  extern unsigned short magFlags;


  void run_sensors(unsigned short time,short count);

  void stop_sensors(void);

  short single_sample(unsigned short addr,long *dest);

  extern unsigned short mag_ADC_gain;

#endif
