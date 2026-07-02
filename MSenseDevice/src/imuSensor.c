

#include "imuSensor.h"
#include "common.h"
#include "zephyrfilesystem.h"
#include "BLEService.h"
#include "ppgSensor.h"
#include <zephyr/drivers/spi.h>
#include <zephyr/logging/log.h>
#include "arm_const_structs.h"
#include <stdio.h>


LOG_MODULE_REGISTER(IMUSensor, CONFIG_LOG_LEVEL_IMU_COLLECTION);

float32_t runningMeanGyro=0.0f, runningSquaredMeanGyro=0.0f;
uint16_t counterGyro=0,counterAcc=0;
float enmo_store[32];
uint16_t testcounter = 0;
int log_counter = 0;

int16_t dataReadGyroX, dataReadGyroY, dataReadGyroZ;
const float accThreshold= 0.001f;
const float gyroThreshold= 5.0f;

uint8_t blePktMotion[ble_motionPktLength];
float quaternionResult_1[4] = {0.0, 0.0, 0.0, 1.0};
struct bleDataPacket my_motionData;


struct accel_config accelConfig = {
  .isEnabled = true, 
  .txPacketEnable = true,
  .sample_bw = IMU_FIXED_ACCEL_DLPFCFG,
  .sensitivity = ACCEL_FS_SEL_4g
};

struct gyro_config gyroConfig = {
  .isEnabled = true,
  .txPacketEnable = true,
  .tot_samples = IMU_FIXED_ACCEL_REPORT_DIVISOR,
  .sensitivity = GYRO_FS_SEL_500
};


struct accData currentAccData;
struct gyroData current_gyro_data;

struct bleDataPacket enmoThreshold;
// Magnometer variables
 

#define IMU_USER_CTRL_VALUE I2C_IF_DIS
#define IMU_LP_CONFIG_VALUE 0x00

void spiReadWriteIMU(uint8_t * tx_buffer, uint8_t txLen, 
uint8_t * rx_buffer, uint8_t rxLen){
  int err;
  const struct spi_buf tx_buf = {
    .buf = tx_buffer,
    .len = txLen
  };
  const struct spi_buf_set tx = {
    .buffers = &tx_buf,
    .count = 1
  };
	
  struct spi_buf rx_buf = {
    .buf = rx_buffer,
    .len = rxLen
  };
  const struct spi_buf_set rx = {
    .buffers = &rx_buf,
    .count = 1
  };

  //there's some area where we might be able to use spi_transceive_dt, with SPI_DT_SPEC_GET to simplify code, but for now, we will just use the previous version.

  err = spi_transceive(spi_dev_imu, &spi_cfg_imu, &tx, &rx);
  if (err)
    printk("SPI error: %d\n", err);
}

// Reads the gyroscope raw data at the global tick rate and accumulates samples
// into a quaternion for the lower-rate motion record. This function also checks if
// the sensor is moving based on a threshold on the angular velocity.
static void gyroscope_measurement(float * quaternionResult){
  uint8_t temp3[8];
  uint8_t burst_tx_gyro[13] = {
    READMASTER | IMU_GYRO_XOUT_H,SPI_FILL,SPI_FILL,
    SPI_FILL,SPI_FILL,SPI_FILL,SPI_FILL,
  };// Burst read acc & gyro regs (0x3B-0x48).
  uint8_t txLen=7,rxLen=7;
  uint8_t burst_rx[23];	// SPI burst read holders.
  float newMagGyro= 0.0f;
  const float pi =(float) 3.14159;
  const float deg_rad = (float)(2.0/360.0)*pi;
  float angularVelX,angularVelY,angularVelZ;
  float thetaRate;
  const float deltaT = 1.0f / (float)GLOBAL_TICK_HZ;
  float quaternions[4];
  float temp,temp1;
  float stdGyro;
  float dividerGyro = 1.0/65.5;
  if(counterGyro == 0) {
    runningMeanGyro=0.0;
    runningSquaredMeanGyro =0.0;
  }

  if(gyroConfig.sensitivity == 0x00) 
    dividerGyro = 1.0/131.0;
  else if(gyroConfig.sensitivity == 0x02 )
    dividerGyro = 1.0/65.5;
  else if(gyroConfig.sensitivity == 0x04 )
    dividerGyro = 1.0/32.8;
  else if(gyroConfig.sensitivity == 0x06 )
    dividerGyro = 1.0/16.4;
  // SPI burst read.
  spiReadWriteIMU(burst_tx_gyro,txLen, burst_rx,rxLen);	
  for(int i = 0; i < 6; i++)
    temp3[i] = burst_rx[i+1];
				
				
  dataReadGyroX = (temp3[0] << 8) | temp3[1];
  if((temp3[0] & 0x80) == 0x80)
    dataReadGyroX = -(~(dataReadGyroX) + 1);
  
  dataReadGyroY = (temp3[2] << 8) | temp3[3];
  if((temp3[2] & 0x80) == 0x80)
    dataReadGyroY = -(~(dataReadGyroY) + 1);
  
  dataReadGyroZ = (temp3[4] << 8) | temp3[5];
  if((temp3[4] & 0x80) == 0x80)
    dataReadGyroZ = -(~(dataReadGyroZ) + 1);
  
  angularVelX = (float)dataReadGyroX*dividerGyro;
  angularVelY = (float)dataReadGyroY*dividerGyro;
  angularVelZ = (float)dataReadGyroZ*dividerGyro;
  
				
  arm_sqrt_f32(angularVelX*angularVelX+angularVelY*angularVelY+
    angularVelZ*angularVelZ,&newMagGyro);
  angularVelX = angularVelX*deg_rad;
  angularVelY = angularVelY*deg_rad;
  angularVelZ = angularVelZ*deg_rad;
				
  runningMeanGyro = newMagGyro/(4*timeWindow) + runningMeanGyro;
  runningSquaredMeanGyro = newMagGyro*newMagGyro/((4*timeWindow)-1.0f) 
   + runningSquaredMeanGyro;
			
  counterGyro++;
  //printf("counterGyro=%d,timeWindow=%d\n",counterGyro,timeWindow);

  if(counterGyro >4*timeWindow ){
    counterGyro=0;
    arm_sqrt_f32(runningSquaredMeanGyro - runningMeanGyro*runningMeanGyro
      *4*timeWindow/(4*timeWindow-1.0f),&stdGyro);
   //printf("stdGyro=%f,gyroThreshold=%f,movingFlag=%d\n",stdGyro,gyroThreshold,current_gyro_data.movingFlag);

    if(stdGyro <= gyroThreshold) 
      current_gyro_data.movingFlag=false;
    else 
      current_gyro_data.movingFlag=true;
  }
  //printf("movingFlag=%d\n",current_gyro_data.movingFlag);
    arm_sqrt_f32(angularVelX*angularVelX +angularVelY*angularVelY
    +angularVelZ*angularVelZ, &thetaRate );
  temp1 = thetaRate*deltaT;
  
  if(thetaRate > (float) 0.0000001){
    angularVelX = angularVelX/thetaRate;
    angularVelY = angularVelY/thetaRate;
    angularVelZ = angularVelZ/thetaRate;
  }
  else{
    angularVelX = 0;
    angularVelY = 0;
    angularVelZ = 0;
    thetaRate = 0;
  }

  temp = arm_sin_f32( temp1/(float)2.0);
  quaternions[0] = angularVelX*temp;
  quaternions[1] = angularVelY*temp;
  quaternions[2] = angularVelZ*temp;
  arm_sqrt_f32(1-temp*temp,&(quaternions[3]));
				
  quaternionResult[0] = quaternionResult[3]*quaternions[0] + 
    quaternionResult[0]*quaternions[3] - quaternionResult[1]*quaternions[2]
     + quaternionResult[2]*quaternions[1];

  quaternionResult[1] = quaternionResult[3]*quaternions[1] +
    quaternionResult[0]*quaternions[2] + quaternionResult[1]*quaternions[3]
     - quaternionResult[2]*quaternions[0];
				
  quaternionResult[2] = quaternionResult[3]*quaternions[2] -
    quaternionResult[0]*quaternions[1] + quaternionResult[1]*quaternions[0]
     + quaternionResult[2]*quaternions[3];
				
  quaternionResult[3] = quaternionResult[3]*quaternions[3] - 
    quaternionResult[0]*quaternions[0] - quaternionResult[1]*quaternions[1]
     - quaternionResult[2]*quaternions[2];
  
  arm_sqrt_f32(quaternionResult[0]*quaternionResult[0] + 
  quaternionResult[1]*quaternionResult[1]+quaternionResult[2]*quaternionResult[2]
  +quaternionResult[3]*quaternionResult[3],&temp);
				
  if(temp > (float)0.0000001){
    quaternionResult[0] = quaternionResult[0]/temp;
    quaternionResult[1] = quaternionResult[1]/temp;
    quaternionResult[2] = quaternionResult[2]/temp;
    quaternionResult[3] = quaternionResult[3]/temp;
  }
  else{		
    temp=1;
    quaternionResult[0] = 0.0;
    quaternionResult[1] = 0.0;
    quaternionResult[2] = 0.0;
    quaternionResult[3] = 1.0;
  }
  if(quaternionResult[3] < 0){
    quaternionResult[0] = -quaternionResult[0];
    quaternionResult[1] = -quaternionResult[1];
    quaternionResult[2] = -quaternionResult[2];
    quaternionResult[3] = -quaternionResult[3];
  }
}


// Function that converts the accumulated quarternion 
// back to angular velocity measurements.
static void prepare_gyros(float* quaternionResult){
 
  current_gyro_data.quaternion_1_val = quaternionResult[0];
  current_gyro_data.quaternion_2_val = quaternionResult[1];
  current_gyro_data.quaternion_3_val = quaternionResult[2];
  current_gyro_data.quaternion_4_val = quaternionResult[3];
}


// our global variables needed for enmo calculation
#define enmo_samples_size 600
float second_enmo_arr[enmo_samples_size] = {0.0};
uint32_t enmo_sample_counter = 0;
// samples since the last activated trigger
uint32_t last_activated_trigger_counter = 0;

// need to implement a read for this
uint8_t enmo_threshold_packet[9] = {0};

float fifteen_second_enmo = 0;
const int enmo_update_rate = 2;
#define ENMO_PACKET_ENMO_OFFSET 0U
#define ENMO_PACKET_COUNTER_OFFSET (sizeof(fifteen_second_enmo))
uint8_t enmo_packet[ENMO_DATA_LEN];
BUILD_ASSERT(sizeof(enmo_packet) == 8, "ENMO packet must be 8 bytes");
BUILD_ASSERT(sizeof(enmo_packet) == (sizeof(fifteen_second_enmo) + sizeof(global_counter)),
  "ENMO packet layout must fit float ENMO plus promoted global_counter");
/**@brief Function for calculating and sending the enmo when necessary.
 *
 * This function will return any enmo sent.
 *
 */
void calculate_enmo(float accelX, float accelY, float accelZ){

    // Calculate ENMO
    // Sometimes the device doesn't like parenthesis, maybe something to do with the FPU? So we just do assignments instead
    float AccelX2 = accelX*accelX;
    float AccelY2 = accelY*accelY;
    float AccelZ2 = accelZ*accelZ;
    
    //float enmo = sqrt(AccelX2 + AccelY2 + AccelZ2) - 1;
    float32_t enmo;
    arm_sqrt_f32(AccelX2+AccelY2+AccelZ2,&enmo);
    enmo = enmo-1;
    if(enmo < 0 ) enmo=0;
    // when we send the enmo, we send as an average of 30
    enmo_store[counterAcc] = enmo;
    counterAcc++;
    if (counterAcc >= 32){
      
      counterAcc = 0;
      //calculate the enmo as an average of 30 samples
      enmo = 0;
      for (int x = 0; x <= 31; x++){
          enmo += enmo_store[x];
      }
      enmo /= 32;
      currentAccData.ENMO = enmo;
      // floats are cast to double in print calls
      LOG_INF("%f, %f, %f", (double)accelX, (double)accelY, (double)accelZ);
      LOG_INF("Enmo: %f", (double)enmo*1000);
      //currentAccData.time = get_current_unix_time();
       
      
      // Testing: Make Enmo a random counter that increments instead.
      testcounter++;
      //accData1.ENMO = testcounter;



      enmo_threshold_evaluation(enmo);
      // Submit our data to the bluetooth work thread.

      int enmo_modulo = enmo_sample_counter % enmo_update_rate; 

      if (enmo_modulo == 0 && enmo_sample_counter >= enmo_update_rate){
      
      // compute the 15 second summary
      fifteen_second_enmo = 0;
      for (int x = enmo_sample_counter - enmo_update_rate; x < enmo_sample_counter; x++){
        fifteen_second_enmo += second_enmo_arr[x];
      }
      fifteen_second_enmo /= enmo_update_rate;

      memcpy(&enmo_packet[ENMO_PACKET_ENMO_OFFSET], &fifteen_second_enmo,
        sizeof(fifteen_second_enmo));
      memcpy(&enmo_packet[ENMO_PACKET_COUNTER_OFFSET], &global_counter,
        sizeof(global_counter));
      my_motionData.dataPacket = enmo_packet;
      my_motionData.packetLength = sizeof(enmo_packet);
      LOG_INF("ENMO ble update: %f", (double)currentAccData.ENMO);
      k_work_submit(&my_motionData.work);
      }
    }


}

void enmo_threshold_evaluation(float enmo_number)
{

  second_enmo_arr[enmo_sample_counter] = currentAccData.ENMO;
  enmo_sample_counter++;
  if (enmo_sample_counter >= enmo_samples_size)
  {
    // Perform the threshold evaluation
    enmo_sample_counter = 0;
  }
  // if we have collected more than enmo_samples_size (420 seconds) worth of data
  if (last_activated_trigger_counter >= enmo_samples_size)
  {
    // count how many enmo values have exceeded MPA (g > .1006)
    int total_enmo = 0;
    for (int sample = 0; sample < enmo_samples_size; sample++)
    {
      if ((second_enmo_arr[sample]) > .1006f)
      {
        total_enmo++;
      }
    }

    LOG_INF("total enmo: %i", total_enmo);

    if (total_enmo > 240)
    {

      enmo_threshold_packet[0] = 1;
      /*if (total_enmo > 294){
        enmo_threshold_packet[0] = 2;
      }
      else {
        enmo_threshold_packet[0] = 1;
      }
      */

      uint64_t current_time = get_current_unix_time();
      memcpy(&enmo_threshold_packet[1], &current_time, sizeof(current_time));
      enmoThreshold.dataPacket = &currentAccData.ENMO;
      enmoThreshold.packetLength = sizeof(currentAccData.ENMO);
      enmo_threshold_send(enmo_threshold_packet, sizeof(enmo_threshold_packet));

      // zero out the last activated trigger since that is now.
      last_activated_trigger_counter = 0;
    }
  }
  else{
    last_activated_trigger_counter++;
  }
}

/**@brief Function for handling the motion data timer timeout.
 *
 * @details This function will be called each time the motion data timer expires.
 *
 */

void motion_data_timeout_handler(struct k_work *item)
{
  struct motionInfo *the_device = ((struct motionInfo *)(((char *)(item)) - offsetof(struct motionInfo, work)));
  uint16_t pktCounter = the_device->pktCounter;
  
  uint8_t burst_tx[13] = {
      READMASTER | ACCEL_XOUT_H, SPI_FILL,
      SPI_FILL, SPI_FILL, SPI_FILL, SPI_FILL,
      SPI_FILL, SPI_FILL, SPI_FILL, SPI_FILL,
      SPI_FILL, SPI_FILL, SPI_FILL}; // Burst read acc & gyro regs (0x3B-0x48).
                                       /**< RX buffer. */

  

    
  uint8_t burst_rx[23];                                           // SPI burst read holders.
  uint8_t m_tx_buf[2] = {REG_BANK_SEL | WRITEMASTER, REG_BANK_0}; /**< TX buffer. */
  uint8_t m_rx_buf[15];      

  // Point to register bank 0 for reading the data from sensors.
  spiReadWriteIMU(m_tx_buf, 2, m_rx_buf, 2);
  
  if (the_device->gyro_first_read == 0)
  {
    float_cast float_cast_arr[4];
    int16_t dataReadAccX, dataReadAccY, dataReadAccZ;
    float accelX, accelY, accelZ;
    float dividerAcc = 0;

    if (accelConfig.sensitivity == 0)
      dividerAcc = 1.0 / 16384;
    else if (accelConfig.sensitivity == 2)
      dividerAcc = 1.0 / 8192;
    else if (accelConfig.sensitivity == 4)
      dividerAcc = 1.0 / 4096;
    else if (accelConfig.sensitivity == 6)
      dividerAcc = 1.0 / 2048;


    spiReadWriteIMU(burst_tx, 7, burst_rx, 7);
    
    prepare_gyros(quaternionResult_1);

    dataReadAccX = (burst_rx[1] << 8) | burst_rx[2];
    if ((burst_rx[1] & 0x80) == 0x80)
      dataReadAccX = -(~(dataReadAccX) + 1);
    dataReadAccY = (burst_rx[3] << 8) | burst_rx[4];
    if ((burst_rx[3] & 0x80) == 0x80)
      dataReadAccY = -(~(dataReadAccY) + 1);
    dataReadAccZ = (burst_rx[5] << 8) | burst_rx[6];
    if ((burst_rx[5] & 0x80) == 0x80)
      dataReadAccZ = -(~(dataReadAccZ) + 1);

#if CONFIG_LOG_LEVEL_IMU_COLLECTION >= 4
    log_counter++;
    if (log_counter > 10)
    {
      LOG_DBG("AccelX: %i, AccelY: %i, AccelZ: %i", dataReadAccX, dataReadAccY, dataReadAccZ);
      log_counter = 0;
    }
#endif
    currentAccData.raw_accx = dataReadAccX;
    currentAccData.raw_accy = dataReadAccY;
    currentAccData.raw_accz = dataReadAccZ;

    accelX = dataReadAccX * dividerAcc / 1.0f;
    accelY = dataReadAccY * dividerAcc / 1.0f;
    accelZ = dataReadAccZ * dividerAcc / 1.0f;
    currentAccData.accx_val = accelX;
    currentAccData.accy_val = accelY;
    currentAccData.accz_val = accelZ;
    calculate_enmo(accelX, accelY, accelZ);

    float_cast_arr[0].float_val = quaternionResult_1[0];
    float_cast_arr[1].float_val = quaternionResult_1[1];
    float_cast_arr[2].float_val = quaternionResult_1[2];

    for (uint8_t i = 0; i < 3; i++)
      quaternionResult_1[i] = 0.0;
    quaternionResult_1[3] = 1.0;

    // Seed the next accumulation window after storing the previous one.
    gyroscope_measurement(quaternionResult_1);
    // blePktMotion[6] = ((uint16_t)dataReadGyroX >> 8) & 0xFF;

    // blePktMotion[7] = (uint16_t)dataReadGyroX & 0xFF;
    // blePktMotion[8] = ((uint16_t)dataReadGyroY >> 8) & 0xFF;
    // blePktMotion[9] = (uint16_t)dataReadGyroY & 0xFF;
    // blePktMotion[10] = ((uint16_t)dataReadGyroZ >> 8) & 0xFF;
    // blePktMotion[11] = (uint16_t)dataReadGyroZ & 0xFF;

    // TODO: If needed, store enmo as well through memcpy-> currentAccData.ENMO,
    // int16_t accel_and_gyro[9] = {dataReadAccX, dataReadAccY, dataReadAccZ, dataReadGyroX, dataReadGyroY, dataReadGyroZ, global_counter};
    // memcpy(&accel_and_gyro[7], &currentAccData.ENMO, sizeof(currentAccData.ENMO));
    #if CONFIG_LOG
    static int last_accel_count = 0; 
    if (global_counter - last_accel_count != gyroConfig.tot_samples) {
      LOG_ERR("Detected accel global counter offset: %d", global_counter - last_accel_count);
    }
    last_accel_count = global_counter;
    #endif
    
    int16_t accel_and_gyro[13] = {dataReadAccX, dataReadAccY, dataReadAccZ};

    memcpy(&accel_and_gyro[3], float_cast_arr[0].floatcast, sizeof(float_cast_arr[0].floatcast));
    memcpy(&accel_and_gyro[5], float_cast_arr[1].floatcast, sizeof(float_cast_arr[1].floatcast));
    memcpy(&accel_and_gyro[7], float_cast_arr[2].floatcast, sizeof(float_cast_arr[2].floatcast));

    memcpy(&accel_and_gyro[9], &currentAccData.ENMO, sizeof(currentAccData.ENMO));

    uint32_t global_tick_512hz = global_counter;
    memcpy(&accel_and_gyro[11], &global_tick_512hz, sizeof(global_tick_512hz));

    store_data(accel_and_gyro, sizeof(accel_and_gyro), 1);

    
    // this function seperately fills blePktMotion with the desired size
    // TODO: Make sure packets are in correct size/order

    for (int i = 0; i < 6; i++){
      blePktMotion[i] = burst_rx[i + 1];
    }


    blePktMotion[18] = blePktMotion[18] | ((pktCounter >> 8) & 0x03);

    // collect packet counter
    // blePktMotion[18] = (pktCounter&) >> 14;
    blePktMotion[19] = ((pktCounter) & 0xFF);

#ifdef CONFIG_MSENSE3_BLUETOOTH_DATA_UPDATES
    my_motionData.dataPacket = blePktMotion;
    my_motionData.packetLength = ACC_GYRO_DATA_LEN;
    if (accelConfig.txPacketEnable == true)
    {
      if (counterAcc == -1)
      {
        k_work_submit(&my_motionData.work);
      }
    }
#endif
  }
  else
  {
    gyroscope_measurement(quaternionResult_1);
  }
}



/**
 * @brief Function for configuring the motion processor.
 *
 * @details  Configures registers in the motion processor before data collection.
 */

void motion_config(void){
  LOG_INF("configuring imu..");

  if(gyroConfig.isEnabled){
    static uint8_t m_tx_buf[2] = {0xF5, SPI_FILL};	/**< TX buffer. */
    static uint8_t m_rx_buf[sizeof(m_tx_buf)];  /**< RX buffer. */
    static const uint8_t m_length = sizeof(m_tx_buf); /**< Transfer length. */

    uint8_t imu_config[36] = {
      REG_BANK_SEL, REG_BANK_0,// change register bank 0
      USER_CTRL,IMU_USER_CTRL_VALUE, // SPI mode; I2C master only when magnetometer path is enabled
      PWR_MGMT_1,IMU_TEMP_DISABLE | IMU_CLK_SEL_BEST_SEL, // Disable temparature sensor and select best available clock source
      PWR_MGMT_2,ENABLE_ACC | ENABLE_GYRO, // Not disabling Gyro and accel
      LP_CONFIG,IMU_LP_CONFIG_VALUE,
      INT_PIN_CFG,0x00,
      REG_BANK_SEL,REG_BANK_2, // changing the register bank to 2
      GYRO_SMPLRT_DIV,IMU_FIXED_GYRO_SMPLRT_DIV, // 1125/(1+1) = 562.5 Hz
      GYRO_CONFIG_1,IMU_FIXED_GYRO_DLPFCFG | GYRO_FS_SEL_500,
                            // gyro full scale =500 dps, LPF = 151.8 Hz
      ACCEL_CONFIG, IMU_FIXED_ACCEL_DLPFCFG | ACCEL_FS_SEL_4g |
        ACCEL_FCHOICE_DLPF_ENABLE   , // accel full scale =4g, LPF = 246 Hz
      ACCEL_SMPLRT_DIV_1,IMU_FIXED_ACCEL_SMPLRT_DIV_MSB, // sample rate accel MSB
      ACCEL_SMPLRT_DIV_2,IMU_FIXED_ACCEL_SMPLRT_DIV_LSB, // 1125/(1+1) = 562.5 Hz
      REG_BANK_SEL,REG_BANK_3,
      I2C_MST_ODR_CONFIG, 0x01, // i2c master dutycycle configuration = 550 Hz,
      I2C_MST_DELAY_CTRL, DELAY_ES_SHADOW, // i2c_mst_delay_ctl = delays shadowing of external sensor
      I2C_MST_CTRL_IMU,I2C_MST_P_NSR_STOP_READS |
        I2C_MST_CLK_345KHZ_40DUTY, // setting i2c master clock = 345 Hz and 46.67% duty cycle recommended
      I2C_SLV0_ADDR_IMU, READMASTER| MAGNETOADDRESS, // setting I2c slave address to magnetometer address
      REG_BANK_SEL,REG_BANK_0
    };	/**< IMU configuration commands. */
    size_t imu_config_length = sizeof(imu_config);

#if !MAGNETOMETER_DATA_PATH_ENABLED
    imu_config[24] = REG_BANK_SEL;
    imu_config[25] = REG_BANK_0;
    imu_config_length = 26;
#endif
    
    // edit ACCEL_CONFIG 
    imu_config[19] = ACCEL_FCHOICE_DLPF_ENABLE | IMU_FIXED_ACCEL_DLPFCFG | accelConfig.sensitivity;
    // edit GYRO_CONFIG1
    imu_config[17] = GYRO_FCHOICE_DLPF_EN | IMU_FIXED_GYRO_DLPFCFG | gyroConfig.sensitivity;
    for(int i = 0; i < imu_config_length; i += 2){
      m_tx_buf[0] = imu_config[i];
      m_tx_buf[1] = imu_config[i+1];
      spiReadWriteIMU(m_tx_buf, m_length, m_rx_buf, m_length);
    }
    k_msleep(10);
  }
}

void getIMUID(){
      uint8_t tx_buffer[4],rx_buffer[4];
      tx_buffer[0] = READMASTER | 0x00;
      tx_buffer[1] = 0xFF;
      uint8_t txLen=2,rxLen=2;
      spiReadWriteIMU(tx_buffer, txLen, rx_buffer, rxLen);
      LOG_INF("Chip ID from motion sensor=%x\n",rx_buffer[1]);
}

void motionSensitivitySampling_config(void){
  if(gyroConfig.isEnabled){
    static uint8_t m_tx_buf[2] = {0xF5,SPI_FILL};	/**< TX buffer. */
    static uint8_t m_rx_buf[sizeof(m_tx_buf)];  /**< RX buffer. */
    static const uint8_t m_length = sizeof(m_tx_buf); /**< Transfer length. */

    uint8_t imu_config[12] = {
      REG_BANK_SEL,REG_BANK_2, // changing the register bank to 2
      GYRO_SMPLRT_DIV,IMU_FIXED_GYRO_SMPLRT_DIV, // 1125/(1+1) = 562.5 Hz
      GYRO_CONFIG_1,IMU_FIXED_GYRO_DLPFCFG,
                            // gyro full scale =500 dps, LPF = 151.8 Hz
      ACCEL_CONFIG,ACCEL_FCHOICE_DLPF_ENABLE   , // accel full scale =4g, LPF = 246 Hz
      ACCEL_SMPLRT_DIV_1,IMU_FIXED_ACCEL_SMPLRT_DIV_MSB,
      ACCEL_SMPLRT_DIV_2,IMU_FIXED_ACCEL_SMPLRT_DIV_LSB,
    };
    imu_config[5] = imu_config[5] | gyroConfig.sensitivity;
    imu_config[7] = ACCEL_FCHOICE_DLPF_ENABLE | IMU_FIXED_ACCEL_DLPFCFG
      | accelConfig.sensitivity;

    for(int i = 0; i < sizeof(imu_config); i += 2){
      m_tx_buf[0] = imu_config[i];
      m_tx_buf[1] = imu_config[i+1];
      spiReadWriteIMU(m_tx_buf, m_length, m_rx_buf, m_length);
    }
  }
}

/**
 * @brief Function for configuring the motion processor to sleep mode and put GYRO in standby mode.
 *
 * @details  Configures registers in the motion processor in idle state.
 */
  
void motion_sleep(void){
  if(gyroConfig.isEnabled){
    uint8_t m_tx_buf[2] = {0xF5, 0xFF};		/**< TX buffer. */
    uint8_t m_rx_buf[sizeof(m_tx_buf)];  /**< RX buffer. */
    const uint8_t m_length = sizeof(m_tx_buf); /**< Transfer length. */
 
    const uint8_t imu_config[6] = { 
      REG_BANK_SEL,REG_BANK_0,
      PWR_MGMT_1, IMU_RESET,
      PWR_MGMT_1, IMU_SLEEP | IMU_CLK_SEL_BEST_SEL  // sleep mode enable
    };	/**< IMU configuration commands. */
		
    for(int i = 0; i < sizeof(imu_config); i += 2){
      m_tx_buf[0] = imu_config[i];
      m_tx_buf[1] = imu_config[i+1];
      spiReadWriteIMU(m_tx_buf, m_length, m_rx_buf, m_length);
    }	
  }
}
