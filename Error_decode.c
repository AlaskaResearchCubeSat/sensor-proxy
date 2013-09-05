#include <Error.h>
#include <ARCBus.h>
#include <I2C.h>
#include "Error_src.h"
#include "sensor-interface.h"


//decode errors from ACDS system
char *err_decode(char buf[150], unsigned short src,int err, unsigned short arg){
  switch(src){
    case SENP_ERR_SRC_ACDS_I2C:
      //NOTE: argument is unused
      sprintf(buf,"ACDS I2C : %s",BUS_error_str(err));
    return buf;
    case SENP_ERR_SRC_SENSOR_I2C:
      switch(err){
        case SENS_ERR_SETUP:
          sprintf(buf,"Sensor I2C : Setup %s",I2C_error_str(arg));
        return buf;
        case SENS_ERR_CONV_READ:
          sprintf(buf,"Sensor I2C : Conversion Result %s",I2C_error_str(arg));
        return buf;
      }
    break;
    case ERR_SRC_CMD:
      switch(err){
        case CMD_ERR_RESET:
          return "Command Line : Commanded reset";
        break;
      }
  }
  sprintf(buf,"source = %i, error = %i, argument = %i",src,err,arg);
  return buf;
}
