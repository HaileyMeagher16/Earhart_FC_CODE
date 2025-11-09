#include <Arduino.h>
#include "peripherals.h"
#include "lsm6dsv80x_reg.h"
#include "driver_w25qxx.h"
#include <Wire.h>
#include <SPI.h>
#include "lps22hh_reg.h"
#include <LPS22HHSensor.h>

#define esp32dev

stmdev_ctx_t imu; // creating stm class
w25qxx_handle_t flash; // creating stm class

int button_input;
int sense_1;

// Setup SPI 
SPIClass *SPI_2 = NULL;
SPIClass *SPI_3 = NULL;
const int dataOrder = MSBFIRST;
const int speedMaximum = 1000000;

int CS;

//Setup IMU

int16_t acceleration_raw[3];
int16_t acceleration_raw_c_x[3];
int16_t acceleration_raw_y[3];
int16_t acceleration_raw_z[3];
float acceleration_ms2[3];
int16_t acceleration_adjusted[3];
float acceleration_g[3];
float acceleration_offset[3];

// creating gyro variables
int16_t gyro_raw[3];
float gyro_dps[3];
int16_t gyro_raw_c[3];
float gyro_offset[3];
int16_t gyro_adjusted[3];

// Function initialization
// IMU platform functions
int32_t platform_write_imu(void *handle, uint8_t reg, const uint8_t *bufp, uint16_t len);
int32_t platform_read_imu(void *handle, uint8_t reg, uint8_t *bufp, uint16_t len);

// Flash platform functions
int32_t platform_write_flash(void *handle, uint8_t reg, const uint8_t *bufp, uint16_t len);
int32_t platform_read_flash(void *handle, uint8_t reg, uint8_t *bufp, uint16_t len);

// /** Optional (may be required by driver) **/
// void platform_delay_imu(uint32_t millisec);
// void platform_dely_flash(uint32_t millisec);

void platform_delay(uint32_t ms);

uint8_t tx_buffer[1000]; //buffer variable

int i;

//setup baro 
int LPS22HH_I2C_SDA = I2C_SDA;
int LPS22HH_I2C_SCL = I2C_SCL;

//Setting up baro communication
LPS22HHSensor PressTemp(&Wire);


void setup() {

  // put your setup code here, to run once:
 Serial.begin(115200);

 //Setting pinModes

 SPI_2 = new SPIClass(HSPI);
  SPI_2->begin(SPI_CLK_2, MISO_2, MOSI_2, IMU_CS);

  pinMode(FLASH_CS, OUTPUT);

  uint8_t read1;
  const char regIMU = 15; //who am i register is 0f=15

  //Checking IMU WhoAmI
  
  pinMode(IMU_CS, OUTPUT);

  SPI_2->beginTransaction(SPISettings(speedMaximum, MSBFIRST, SPI_MODE0));
    digitalWrite(IMU_CS, LOW);
    SPI_2->transfer(regIMU | 0x80);
    read1 = SPI_2->transfer(0x00);
    SPI_2->endTransaction();

    digitalWrite(IMU_CS, HIGH);

    Serial.println(read1);

    if (read1 != 106) {
      Serial.println("IMU not found");
    };

    //IMU setup 
  int32_t LSM_Init(void);
  {
  imu.write_reg = platform_write_imu; // lsm is that stmdev_ctx_t typed struct
  imu.read_reg = platform_read_imu;
  imu.mdelay = platform_delay;
  imu.handle = &SPI_2;

  

   uint8_t lsm_device_id;
    lsm6dsv80x_device_id_get(&imu, &lsm_device_id); // ensure that you can do it thru the ST driver
    if (lsm_device_id != LSM6DSV80X_ID)
    { // should be 106 
      printf("lsm_device_id %u does not match expected %u\n", lsm_device_id, LSM6DSV80X_ID);
    }

    // restore default config
    lsm6dsv80x_reset_set(&imu, LSM6DSV80X_READY);
    lsm6dsv80x_reset_t rst = LSM6DSV80X_GLOBAL_RST; // idk this is a dumb move
    while (rst)
    {
      lsm6dsv80x_reset_get(&imu, &rst);
    }

    lsm6dsv80x_block_data_update_set(&imu, PROPERTY_ENABLE); // enable block data update

    lsm6dsv80x_xl_full_scale_set(&imu, LSM6DSV80X_2g);
    lsm6dsv80x_gy_full_scale_set(&imu, LSM6DSV80X_250dps);
    lsm6dsv80x_xl_data_rate_set(&imu, LSM6DSV80X_ODR_HA03_AT_104Hz);
    lsm6dsv80x_gy_data_rate_set(&imu, LSM6DSV80X_ODR_HA03_AT_104Hz);
    printf("IMU initialized\n");

    Serial.println("Hello from calibration");
    float est[3] = {0.0, 0.0, 0.0};

    for (int i = 0; i < 100; i++)
    {
      memset(acceleration_raw_c_x, 0x00, 3 * sizeof(int16_t));
      lsm6dsv80x_acceleration_raw_get(&imu, acceleration_raw_c_x);

      float alpha = 0.5;

      est[0] = alpha * est[0] + (1 - alpha) * acceleration_raw_c_x[0];
      est[1] = alpha * est[1] + (1 - alpha) * acceleration_raw_c_x[1];
      est[2] = alpha * est[2] + (1 - alpha) * acceleration_raw_c_x[2];

      //        printf("Read: %d, %d, %d Avg: %.0f, %.0f, %.0f\r\n", mpu->gyro_raw[0], mpu->gyro_raw[1], mpu->gyro_raw[2], est[0], est[1], est[2]);

      delay(10); // max sensor polling rate 100Hz
    }

    // Set the offset such that they entirely cancel the average reading
    acceleration_offset[0] = (int16_t)(-est[0]);
    acceleration_offset[1] = (int16_t)(-est[1]);
    acceleration_offset[2] = (int16_t)(-((est[2]))); // Z axis, when calibrated in +-2g mode, should be 16384

    Serial.println("offset:");
    Serial.print(acceleration_offset[0]);
    Serial.print(", ");
    Serial.print(acceleration_offset[1]);
    Serial.print(", ");
    Serial.print(acceleration_offset[2]);
    Serial.println();

    Serial.println("end of calibration");

    Serial.println("Hello from gyro calibration");
    float est_g[3] = {0.0, 0.0, 0.0};

    for (int i = 0; i < 100; i++)
    {
      memset(gyro_raw_c, 0x00, 3 * sizeof(int16_t));
      lsm6dsv80x_angular_rate_raw_get(&imu, gyro_raw_c);

      float alpha = 0.5;

      est_g[0] = alpha * est_g[0] + (1 - alpha) * gyro_raw_c[0];
      est_g[1] = alpha * est_g[1] + (1 - alpha) * gyro_raw_c[1];
      est_g[2] = alpha * est_g[2] + (1 - alpha) * gyro_raw_c[2];

      //        printf("Read: %d, %d, %d Avg: %.0f, %.0f, %.0f\r\n", mpu->gyro_raw[0], mpu->gyro_raw[1], mpu->gyro_raw[2], est[0], est[1], est[2]);

      delay(10); // max sensor polling rate 100Hz
    }

    // Set the offset such that they entirely cancel the average reading
    gyro_offset[0] = (int16_t)(-est_g[0]);
    gyro_offset[1] = (int16_t)(-est_g[1]);
    gyro_offset[2] = (int16_t)(-((est_g[2]))); // Z axis, when calibrated in +-2g mode, should be 16384

    Serial.println("gyro offset:");
    Serial.print(gyro_offset[0]);
    Serial.print(", ");
    Serial.print(gyro_offset[1]);
    Serial.print(", ");
    Serial.print(gyro_offset[2]);
    Serial.println();

    Serial.println("end of gyro calibration");

    return;


      // Baro Setup
    Wire.begin(I2C_SCL, I2C_SDA);
    

  // Initlialize component i think 
  
  PressTemp.begin();
  PressTemp.Enable();
  
}



  

//     uint64_t read2;
//     //Checking Flash WhoAmI
//   SPI_2->beginTransaction(SPISettings(speedMaximum, MSBFIRST, SPI_MODE0));
//     digitalWrite(FLASH_CS, LOW);
//     SPI_2->transfer(144 | 0x24 ); // Flash_reg = 90h = 144
//     read2 = SPI_2->transfer(0x00);
//     SPI_2->endTransaction();

//     digitalWrite(FLASH_CS, HIGH);

//     Serial.println(read2);

//     if (read2 != 115) {
//       Serial.println("Flash not found.");
//     };

//     // Flash driver example code

//     uint8_t res;
// uint8_t manufacturer;
// uint8_t device_id;
// uint8_t data[8];

// res = w25qxx_get_manufacturer_device_id(&flash, &manufacturer, &device_id);
// if (res != 0)
// {
//  Serial.println("Flash not found :(");
// };

}

void loop() {
  // put your main code here, to run repeatedly:


  Serial.print("Loop:");
  Serial.println();
  delay(500);

  lsm6dsv80x_data_ready_t drdy;
  lsm6dsv80x_flag_data_ready_get(&imu, &drdy);
  //          printf("xl_drdy: %u\n", xl_drdy);
  if (drdy.drdy_xl)
  {
    memset(acceleration_raw, 0x00, 3 * sizeof(int16_t));
    lsm6dsv80x_acceleration_raw_get(&imu, acceleration_raw);
  }

  Serial.print("accel: ");
  Serial.print(acceleration_raw[0]);
  Serial.print(", ");
  Serial.print(acceleration_raw[1]);
  Serial.print(", ");
  Serial.print(acceleration_raw[2]);
  Serial.println();

  Serial.println("acceleration offset:");
  Serial.print(acceleration_offset[0]);
  Serial.print(", ");
  Serial.print(acceleration_offset[1]);
  Serial.print(", ");
  Serial.print(acceleration_offset[2]);
  Serial.println();

  //constants
#define G_MS2 9.80665 // [m/s^2]

  for (int i = 0; i < 3; i++)
  {
    acceleration_adjusted[i] = acceleration_raw[i] + acceleration_offset[i];
    acceleration_g[i] = lsm6dsv80x_from_fs2_to_mg(acceleration_adjusted[i]) / 1000.0;
    acceleration_ms2[i] = lsm6dsv80x_from_fs2_to_mg(acceleration_adjusted[i]) / 1000.0 * G_MS2;
  }

  Serial.print("accel: ");
  Serial.print(acceleration_ms2[0]);
  Serial.print(", ");
  Serial.print(acceleration_ms2[1]);
  Serial.print(", ");
  Serial.print(acceleration_ms2[2]);
  Serial.println();

  // Baro test
  float pressure, temperature;
  PressTemp.GetPressure(&pressure);
  PressTemp.GetTemperature(&temperature);

  Serial.println("Pres[hPa]:");
  Serial.println(pressure, 2);
  Serial.println(", Temp[C]:");
  Serial.println(temperature, 2);
  // print an empty line
  Serial.println();

  // gyroscope
  // lsm6dsv80x_data_ready_t drdy;
  lsm6dsv80x_flag_data_ready_get(&imu, &drdy); // detect if new data

  if (drdy.drdy_gy)
  {                                               // if new data ready
    memset(gyro_raw, 0, 3 * sizeof(int16_t));     // create gyro data holder
    lsm6dsv80x_angular_rate_raw_get(&imu, gyro_raw); // get gyro
  }

  for (int i = 0; i < 3; i++)
  {
    gyro_adjusted[i] = gyro_raw[i] + gyro_offset[i];
    gyro_dps[i] = lsm6dsv80x_from_fs250_to_mdps(gyro_adjusted[i]) / 1000.0;

  }

  Serial.print("gyro raw: ");
  Serial.print(gyro_raw[0]);
  Serial.print(", ");
  Serial.print(gyro_raw[1]);
  Serial.print(", ");
  Serial.print(gyro_raw[2]);
  Serial.println();

  Serial.println("gyro offset:");
  Serial.print(gyro_offset[0]);
  Serial.print(", ");
  Serial.print(gyro_offset[1]);
  Serial.print(", ");
  Serial.print(gyro_offset[2]);
  Serial.println();

  Serial.print("gyro dps: ");
  Serial.print(gyro_dps[0]);
  Serial.print(", ");
  Serial.print(gyro_dps[1]);
  Serial.print(", ");
  Serial.print(gyro_dps[2]);
  Serial.println();
  //Pyro test code

  // button_input = !digitalRead(2);
  // sense_1 = analogRead(13);
  // if (button_input){
  //   digitalWrite(5, LOW);
  //   digitalWrite(16, HIGH);
  // } else {
  //   digitalWrite(5, HIGH);
  //   digitalWrite(16, LOW);
  // }

  // Serial.println(sense_1);
  
}

//Function definitions
int32_t platform_read_imu(void *handle, uint8_t reg, uint8_t *bufp, uint16_t len)
{

  uint8_t dataRead;
  // uint8_t *data[len];
  for (int i = 0; i < len; i++)
  {

    
    
    SPI_2->beginTransaction(SPISettings(speedMaximum, MSBFIRST, SPI_MODE3));

    digitalWrite(IMU_CS, LOW);

    SPI_2->transfer(reg | 0x80);
    dataRead = SPI_2->transfer(0x00);
    SPI_2->endTransaction();

    digitalWrite(IMU_CS, HIGH);

    bufp[i] = dataRead;

    reg = reg + 1;
    
    }
    
  return 0;
}

int32_t platform_write_imu(void *handle, uint8_t reg, const uint8_t *bufp, uint16_t len)
{

  uint8_t dataWrite;

  SPI_2->beginTransaction(SPISettings(speedMaximum, MSBFIRST, SPI_MODE0));
  digitalWrite(IMU_CS, LOW);
  SPI_2->transfer(reg | 0);
  SPI_2->transfer(*bufp);
  SPI_2->endTransaction();
  digitalWrite(IMU_CS, HIGH);

  return 0;
}

void platform_delay(uint32_t ms)
{
  delay(1000);
}

// int32_t platform_read_flash(void *handle, uint8_t reg, uint8_t *bufp, uint16_t len)
// {
//   int i;

//   uint8_t dataRead;
//   // uint8_t *data[len];
//   for (int i = 0; i < len; i++)
//   {

    
    
//     SPI_2->beginTransaction(SPISettings(speedMaximum, MSBFIRST, SPI_MODE3));

//     digitalWrite(FLASH_CS, LOW);

//     SPI_2->transfer(reg | 0x80);
//     dataRead = SPI_2->transfer(0x00);
//     SPI_2->endTransaction();

//     digitalWrite(FLASH_CS, HIGH);

//     bufp[i] = dataRead;

//     reg = reg + 1;

//     }
  
//   return 0;
// }


// int32_t platform_write_flash(void *handle, uint8_t reg, const uint8_t *bufp, uint16_t len)
// {

//   uint8_t dataWrite;

//   SPI_2->beginTransaction(SPISettings(speedMaximum, MSBFIRST, SPI_MODE3));
//   digitalWrite(FLASH_CS, LOW);
//   SPI_2->transfer(reg | 0);
//   SPI_2->transfer(*bufp);
//   SPI_2->endTransaction();
//   digitalWrite(FLASH_CS, HIGH);

//   return 0;
// }