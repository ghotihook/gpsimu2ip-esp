# gpsimu2ip
uses an M5 Stack ESP32 controller and GNSS module to provide GPS (RMC), Heel and Pitch data via NMEA over UDP. 

![Unit](images/unit.jpeg)

**Initial configuration is**
- GPS output at 5hz
- IMU internal refresh (100hz)
- UDP output at 5hz
- Display update 1hz



# parts
[M5 Stack Core S3 SE](https://shop.m5stack.com/products/m5stack-cores3-esp32s3-lotdevelopment-kit?srsltid=AfmBOooBjrRcmQfX4Ls7f4-QoIkYDgNVxXmuVoQinpfh3KtxQ-GVJX3A)

[GNSS Module with IMU](https://shop.m5stack.com/products/gnss-module-with-barometric-pressure-imu-magnetometer-sensors?srsltid=AfmBOorUYsZstzVM0wU1aUoTbz5d52dBKPwS9WHpqgLIYkI7fgVtVrVr)


# Example output
```
$GNRMC,203043.00,A,3251.73631,S,15220.32322,E,0.938,,240125,,,A,V*0B
$IIXDR,A,-1,D,PITCH*10
$IIXDR,A,20,D,HEEL*7D
```

# GPS configuration
By default the NEO-M9N ublox GPS chip in the GNSS module operates at 1hz and outputs lots of additional messages. in the tools directory use the 'serialpassthrough' to configure the M5 Stack to pass any data from the internal UART (GPS) to the external UART (USB Port). Once compriled and uploaded, you can then simply connect the module to a computer and using u-center configure the GPS module using the GUI. I have configured 5hz updates and removed all messages from NMEA except for RMC. 


# Installation
Use the Arduino IDE and install the library install M5Module_GNSS library (which contains the bosch BMI270 driver). I am not sure why this is required but it works.

default install gives this error:

```In file included from /Users/alex060/Documents/Arduino/gps_imu_2ip/gps_imu_2ip.ino:3:
/Users/alex060/Documents/Arduino/libraries/M5Module-GNSS/src/BMI270.h:39:31: error: invalid conversion from 'int' to 'i2c_port_t' [-fpermissive]
   39 |         i2c_port_t i2c_port = 0;
      |                               ^
      |                               |
      |                               int
```

open this file BMI270.h

and replace this line 
```i2c_port_t i2c_port = 0;```

with this 
```i2c_port_t i2c_port = I2C_NUM_1; // Default to I2C_NUM_0 or I2C_NUM_1```



then tweak the paramaters, easiest done in the BMI270.cpp file although the defaults will do.

```
        struct bmi2_sens_config sens_cfg[2];
        sens_cfg[0].type = BMI2_ACCEL;
        sens_cfg[0].cfg.acc.bwp = BMI2_ACC_OSR2_AVG2;
        sens_cfg[0].cfg.acc.odr = BMI2_ACC_ODR_200HZ;
        sens_cfg[0].cfg.acc.filter_perf = BMI2_PERF_OPT_MODE;
        sens_cfg[0].cfg.acc.range = BMI2_ACC_RANGE_4G;
        sens_cfg[1].type = BMI2_GYRO;
        sens_cfg[1].cfg.gyr.filter_perf = BMI2_PERF_OPT_MODE;
        sens_cfg[1].cfg.gyr.bwp = BMI2_GYR_OSR2_MODE;
        sens_cfg[1].cfg.gyr.odr = BMI2_ACC_ODR_200HZ;
        sens_cfg[1].cfg.gyr.range = BMI2_GYR_RANGE_2000;
        sens_cfg[1].cfg.gyr.ois_range = BMI2_GYR_OIS_2000;
```

# ToDo
- Code should self-configure the GPS unit to desired output
- Code should self-configure the IMU
- Address issue requiring manual adjustment to library