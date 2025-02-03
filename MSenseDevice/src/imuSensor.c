

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
float32_t runningMeanAcc=0.0f, runningSquaredMeanAcc=0.0f;
uint16_t counterGyro=0,counterAcc=0;
float enmo_store[32];
uint16_t testcounter = 0;
int log_counter = 0;

int16_t dataReadGyroX, dataReadGyroY, dataReadGyroZ;
const float accThreshold= 0.001f;
const float gyroThreshold= 5.0f;
uint8_t blePktMagneto[ble_magnetometerPktLength];
uint8_t blePktMotion[ble_motionPktLength];
float quaternionResult_1[4] = {0.0, 0.0, 0.0, 1.0};
struct bleDataPacket my_motionData;

struct bleDataPacket enmoThreshold;
// Magnometer variables
 
bool validMeasurement = false;
bool magnoSecondReading = false;
uint8_t burst_tx_magneto[17];	

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

// Reads the gyroscope raw data at 200 Hz and accumulates the samples 
// using the quarternion representaiotn. This function also checks if 
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
  const float deltaT = 1.0/200.0;
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
  
  //printf("angX = %f,angy = %f,angz = %f,counterGyro=%d\n",
   // angularVelX,angularVelY,angularVelZ,counterGyro);
				
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
  /* 
  float quantizerScale; 
  float pi =(float) 3.14159;
  float32_t rad_deg = 180.0f/pi; 
  float bounds1 = 0.0f;
/*  const float coeffs1[8] = {
    1.5707963050f, -0.214598016f, 0.0889789874f, -0.051743046f,
    0.0308918810f, -0.0170881256f, 0.0066700901f, -0.0012624911f  
  }; // coefficients of the polynomial aproximation for acos(x)

  const float coeffsNum[21] = {
    4.999041E-07, 2.834031E-05, -1.490620E-04, -1.771155E-03,
    5.964344E-03, 3.300317E-02, -8.814134E-02, -2.804760E-01,
    6.451019E-01, 1.290628E+00, -2.668552E+00, -3.482593E+00,
    6.642922E+00, 5.667227E+00, -1.014432E+01, -5.467928E+00,
    9.295363E+00, 2.878479E+00, -4.688171E+00, -6.366198E-01,1 
  }; 

  const float coeffsDenom[21] = {
    4.999041E-07, 0, -1.490620E-04, 0, 5.964344E-03, 0, 
    -8.814134e-02, 0, 6.451019e-01, 0, -2.668552E+00, 0,
    6.642922E+00, 0, -1.014432E+01, 0, 9.295363E+00, 0,
    -4.688171E+00, 0, 1
  }; 

  float delta_T;
  if(gyroConfig.tot_samples == 1) 
    delta_T = 1.0f/200.0f;
  else if(gyroConfig.tot_samples == 2) 
    delta_T = 1.0f/ 100.0f;
  else if(gyroConfig.tot_samples == 4) 
    delta_T = 1.0f/ 50.0f;
  else 
    delta_T = 1.0f/ 25.0f;
		
  if(gyroConfig.sensitivity == 0x00){
    quantizerScale = 32767.0f/250.0f;
    bounds1 = 250.0f;
  }
  else if(gyroConfig.sensitivity == 0x02 ){
    bounds1 = 500.0f;
    quantizerScale = 32767.0f/500.0f;
  }
  else if(gyroConfig.sensitivity == 0x04 ){ 
    bounds1 = 1000.0f;
    quantizerScale = 32767.0f/ 1000.0f;
  }
  else if(gyroConfig.sensitivity == 0x06 ){
    quantizerScale = 32767.0f/ 2000.0f;
    bounds1 = 2000.0f;

  }
  */
/* Using Polynomial approximation for acos(x) = 
  (c0 +c1*x + c2*x^2 + ... + c21*x^21)/(d0 +d1*x + d2*x^2 + ... + d21*x^21) */
/* Not using the polynomial expansion and reverting back to Taylor's series */
/*	
  float temp,q3New = 1.0,acosValue=coeffs1[0],temp2,theta_rate,qq;

  float angularX,angularY,angularZ;
  //float acosValueNum=1.0,acosValueDenom=1.0;
  float factorMul = 1.570796;
  
  arm_sqrt_f32(1-quaternionResult[0]*quaternionResult[0] - 
    quaternionResult[1]*quaternionResult[1] - 
    quaternionResult[2]*quaternionResult[2], &qq );
  arm_sqrt_f32(1-qq, &temp );
  
  for(uint8_t i=1;i<8;i++){
    q3New = q3New*qq;
    acosValue=acosValue+coeffs1[i]*q3New;
    //acosValueNum = acosValueNum + coeffsNum[20-i]*q3New;
    //acosValueDenom = acosValueDenom + coeffsDenom[20-i]*q3New;
  }*/
  //acosValue = (float) temp*acosValueNum/acosValueDenom*factorMul;
  /*

  acosValue = (float32_t) temp*acosValue;
  theta_rate = (float)2.0f*acosValue/delta_T;
  temp2 = arm_sin_f32(theta_rate*delta_T/2.0f);
  
  if(temp2 < 0.000001f)	{
    temp2 = 0.0f;
    angularX = 0.0f;
    angularY = 0.0f;
    angularZ = 0.0f;
    theta_rate = 0.0f;
  }				
  else{
    angularX= quaternionResult[0]/ temp2 *theta_rate *rad_deg;
    angularY= quaternionResult[1]/ temp2 *theta_rate*rad_deg;
    angularZ= quaternionResult[2]/ temp2 *theta_rate*rad_deg;
  }
  
  if(angularX > bounds1) 
    angularX = bounds1;
  if(angularY > bounds1) 
    angularY = bounds1;
  if(angularZ > bounds1 )  
    angularZ = bounds1;
  if(angularX < -bounds1) 
    angularX = -bounds1;
  if(angularY < -bounds1 ) 
    angularY = -bounds1;
  if(angularZ < -bounds1 )  
    angularZ = -bounds1;

  current_gyro_data.gyrox_val = angularX;
  current_gyro_data.gyroy_val = angularY;
  current_gyro_data.gyroz_val = angularZ;
	*/
  //printf("angX_final = %f,angy_final = %f,angz_final = %f\n",
   // angularX,angularY,angularZ);

  // Storing only the first three components of the Quaternion. 
  // The fourth component can be computed because the norm of the 
  // quaternion vector is 1.	
  /*

  temp1.float_val =  quaternionResult[0]; // angularX; changed to quaternionComponent 0 
  blePktMotion[6] = temp1.floatcast[0];
  blePktMotion[7] = temp1.floatcast[1];
  blePktMotion[8] = temp1.floatcast[2];
  blePktMotion[9] = temp1.floatcast[3];

  temp1.float_val =  quaternionResult[1]; // angularY;
  blePktMotion[10] = temp1.floatcast[0];
  blePktMotion[11] = temp1.floatcast[1];
  blePktMotion[12] = temp1.floatcast[2];
  blePktMotion[13] = temp1.floatcast[3];

  temp1.float_val =  quaternionResult[2]; // angularZ;
  blePktMotion[14] = temp1.floatcast[0];
  blePktMotion[15] = temp1.floatcast[1];
  blePktMotion[16] = temp1.floatcast[2];
  blePktMotion[17] = temp1.floatcast[3];*/

  current_gyro_data.quaternion_1_val = quaternionResult[0];
  current_gyro_data.quaternion_2_val = quaternionResult[1];
  current_gyro_data.quaternion_3_val = quaternionResult[2];
  current_gyro_data.quaternion_4_val = quaternionResult[3];
}
// Read the magnetometer sample and use it for orientation calculation
static void magnetometer_data_read_send(bool validMeasurement , uint16_t pktCounter){
  uint8_t txLen=9, rxLen=9;
  
  // Burst read external sensor ( Magnetometer)  registers (0x49-0x50).
  uint8_t burst_rx_magneto[9];	// SPI burst read holders.
  
  printk("packet counter: %i", (unsigned int)pktCounter);
  //Magneto Transmitting 0xFF
  
  burst_tx_magneto[0] =(READMASTER | EXT_SENSE_DATA_0_IMU);
  burst_tx_magneto[1] = SPI_FILL;
  burst_tx_magneto[2] = SPI_FILL;
  burst_tx_magneto[3] = SPI_FILL;
  burst_tx_magneto[4] = SPI_FILL;
  burst_tx_magneto[5] = SPI_FILL;
  burst_tx_magneto[6] = SPI_FILL;
  burst_tx_magneto[7] = SPI_FILL;
  burst_tx_magneto[8] = SPI_FILL;
  // Reading magnetometer data
  spiReadWriteIMU(burst_tx_magneto, txLen, burst_rx_magneto, rxLen);
  
  // Sort magneto data.
  if (magnoSecondReading){
  //for(uint8_t i=0;i<6;i++)
    //blePktMagneto[i]=burst_rx_magneto[i+1];
    blePktMagneto[2] = burst_rx_magneto[0+1];
    blePktMagneto[3] = burst_rx_magneto[1+1];
    blePktMagneto[6] = burst_rx_magneto[2+1];
    blePktMagneto[7] = burst_rx_magneto[3+1];
    blePktMagneto[9] = burst_rx_magneto[4+1];
    blePktMagneto[10] = burst_rx_magneto[5+1];
}

  blePktMagneto[15] = (pktCounter&0xFF00) >> 8;
  blePktMagneto[16] = (pktCounter&0x00FF);
  if(magnetoConfig.txPacketEnable == true){
    my_magnetoSensor.dataPacket = blePktMagneto;
    my_magnetoSensor.packetLength = MAGNETOMETER_DATA_LEN;
    #ifdef CONFIG_MSENSE3_BLUETOOTH_DATA_UPDATES
    k_work_submit(&my_magnetoSensor.work);
    #endif
  }
  else{
    blePktMagneto[0] = burst_rx_magneto[0+1];
    blePktMagneto[1] = burst_rx_magneto[1+1];
    blePktMagneto[4] = burst_rx_magneto[2+1];
    blePktMagneto[5] = burst_rx_magneto[3+1];
    blePktMagneto[7] = burst_rx_magneto[4+1];
    blePktMagneto[8] = burst_rx_magneto[5+1];
  }

  if(validMeasurement == true){
    current_magneto_data.Hy = (burst_rx_magneto[2]<<8) + burst_rx_magneto[1];
    if((burst_rx_magneto[2]&0x80) == 0x80)
      current_magneto_data.Hy = -(~(current_magneto_data.Hy) + 1);
    current_magneto_data.Hx = (burst_rx_magneto[4]<<8) + burst_rx_magneto[3];
    if((burst_rx_magneto[4]&0x80) == 0x80)
      current_magneto_data.Hx = -(~(current_magneto_data.Hx) + 1);
    current_magneto_data.Hz = (burst_rx_magneto[6]<<8) + burst_rx_magneto[5];
    if((burst_rx_magneto[6]&0x80) == 0x80)
      current_magneto_data.Hz = -(~(current_magneto_data.Hz) + 1);
    current_magneto_data.Hz = -current_magneto_data.Hz;
    current_magneto_data.Hx_val = current_magneto_data.Hx *0.15;
    current_magneto_data.Hy_val = current_magneto_data.Hy *0.15;
    current_magneto_data.Hz_val = current_magneto_data.Hz *0.15;

  }
  else{
    current_magneto_data.Hy = 0;
    current_magneto_data.Hx = 0;
    current_magneto_data.Hz = 0;
    current_magneto_data.Hx_val = current_magneto_data.Hx *0.15;
    current_magneto_data.Hy_val = current_magneto_data.Hy *0.15;
    current_magneto_data.Hz_val = current_magneto_data.Hz *0.15;
  }
  //printf("H_x=%f,H_y=%f,H_z=%f\n", 
  //  magnetoData1.Hx_val,magnetoData1.Hy_val,magnetoData1.Hz_val);
  motion_data_orientation_timeout_handler(pktCounter);
  validMeasurement = false;
  magnoSecondReading = !magnoSecondReading;
}

// Configure the magnetometer sensor using the external sensor mode
static void magnetometer_read_sample_config(
magneto_sample_config_t magneto_smpl_config){
  static uint8_t m_tx_buf[2] = {WHO_AM_I | READMASTER,SPI_FILL};		/**< TX buffer. */
  static uint8_t m_rx_buf[sizeof(m_tx_buf)];  /**< RX buffer. */
  static uint8_t tx_buf[14];
  uint8_t magnetoConfig_length;
  static const uint8_t m_length = sizeof(m_tx_buf); /**< Transfer length. */

  tx_buf[0] = REG_BANK_SEL | WRITEMASTER;
  tx_buf[1] = REG_BANK_3;
  tx_buf[2] = I2C_SLV0_ADDR_IMU;
  switch(magneto_smpl_config){
    case MAGNETOMETER_SINGLE:
      tx_buf[3] = MAGNETOADDRESS;
      tx_buf[4] = I2C_SLV0_REG_IMU;
      tx_buf[5] = MAGNETOMETER_CNTL2;
      tx_buf[6] = I2C_SLV0_CTRL_IMU;
      tx_buf[7] = I2C_SLV_EN | 0x01; // writing 1 byte
      tx_buf[8] = I2C_SLV0_DO_IMU;
      tx_buf[9] = MAGNETO_SINGLE_MEASUREMENT;
      tx_buf[10] = I2C_SLV0_CTRL_IMU;
      tx_buf[11] = I2C_SLV_EN | 0x01;
      tx_buf[12] = REG_BANK_SEL | WRITEMASTER;
      tx_buf[13] = REG_BANK_0;
      magnetoConfig_length = 14;
      break;
    case MAGNETOMETER_SET_EXT_SENSOR:
      tx_buf[3] = READMASTER | MAGNETOADDRESS;
      tx_buf[4] = I2C_SLV0_REG_IMU;
      tx_buf[5] = MAGNETOMETER_HXL;  //Reading magnetometer data
      tx_buf[6] = I2C_SLV0_CTRL_IMU;
      tx_buf[7] = I2C_SLV_EN | 0x08; //Reading 8 bytes
      tx_buf[8] = REG_BANK_SEL | WRITEMASTER;
      tx_buf[9] = REG_BANK_0; // changing back to register bank 0
      magnetoConfig_length = 10;
      break;
    case MAGNETOMETER_SET_EXT_TOREAD:
      tx_buf[3] = READMASTER | MAGNETOADDRESS;
      tx_buf[4] = I2C_SLV0_REG_IMU;
      tx_buf[5] = MAGNETOMETER_ST1;
      tx_buf[6] = I2C_SLV0_CTRL_IMU;
      tx_buf[7] = I2C_SLV_EN | 0x01; // reading 1 byte
      tx_buf[8] = REG_BANK_SEL | WRITEMASTER;
      tx_buf[9] = REG_BANK_0; // changing back to register bank 0
      magnetoConfig_length = 10;
      break;
    default:
      return;
  }
  for(int i = 0; i < magnetoConfig_length; i += 2){
    m_tx_buf[0] = tx_buf[i];
    m_tx_buf[1] = tx_buf[i+1];	
    spiReadWriteIMU(m_tx_buf, m_length,m_rx_buf, m_length);
  }
}

// our global variables needed for enmo calculation
#define enmo_samples_size 420
float second_enmo_arr[enmo_samples_size] = {0.0};
int enmo_sample_counter = 0;
// samples since the last activated trigger
int last_activated_trigger_counter = 0;

// need to implement a read for this
uint8_t enmo_threshold_packet[9] = {0};

float fifteen_second_enmo = 0;
const int enmo_update_rate = 2;
uint8_t enmo_packet[6];
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
    // when we send the enmo, we send as an average of 25
    enmo_store[counterAcc] = enmo;
    counterAcc++;
    if (counterAcc >= 32){
      
      counterAcc = 0;
      //calculate the enmo as an average of 25 samples
      enmo = 0;
      for (int x = 0; x <= 31; x++){
          enmo += enmo_store[x];
      }
      enmo /= 32;
      currentAccData.ENMO = enmo;
      LOG_WRN("Enmo: %f", enmo*1000);
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

      memcpy(enmo_packet, &fifteen_second_enmo, sizeof(fifteen_second_enmo));
      memcpy(&enmo_packet[4], &global_counter, sizeof(global_counter));
      my_motionData.dataPacket = enmo_packet;
      my_motionData.packetLength = 6;
      LOG_WRN("ENMO ble update: %f", currentAccData.ENMO);
      k_work_submit(&my_motionData.work);
      }
    }


}

void enmo_threshold_evaluation(float enmo_number)
{

  second_enmo_arr[enmo_sample_counter] = currentAccData.ENMO;
  enmo_sample_counter++;
  last_activated_trigger_counter++;
  if (enmo_sample_counter >= enmo_samples_size)
  {
    // Perform the threshold evaluation
    enmo_sample_counter = 0;
  }

  if (last_activated_trigger_counter >= enmo_samples_size)
  {
    int total_enmo = 0;
    for (int sample = 0; sample < enmo_samples_size; sample++)
    {
      if ((second_enmo_arr[sample] * 1000) > 100.6f)
      {
        total_enmo++;
      }
    }

    // total_enmo = total_enmo / 60;
    LOG_INF("total enmo: %i", total_enmo);

    if (total_enmo > 294)
    {

      enmo_threshold_packet[0] = 1;
      /*if (total_enmo > 294){
        enmo_threshold_packet[0] = 1;
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
      // k_work_submit(&my_motionData.work);

      // zero out the last activated trigger since that is now.
      last_activated_trigger_counter = 0;
    }
  }
}

/**@brief Function for handling the motion data timer timeout.
 *
 * @details This function will be called each time the motion data timer expires.
 *
 */

void motion_data_timeout_handler(struct k_work *item)
{

  // start_timer();
  struct motionInfo *the_device = ((struct motionInfo *)(((char *)(item)) - offsetof(struct motionInfo, work)));

  float_cast temp1[4];

  uint16_t pktCounter = the_device->pktCounter;
  uint8_t magneto_first_readTemp = the_device->magneto_first_read;

  // printk("gyro: %i \n", gyro_first_readTemp);
  uint8_t burst_tx_INT_STAT[2] = {INT_STAATUS_1 | READMASTER, SPI_FILL}; // SPI burst read holders.
  uint16_t checkMag = 0;
  uint8_t burst_tx[13] = {
      READMASTER | ACCEL_XOUT_H, SPI_FILL,
      SPI_FILL, SPI_FILL, SPI_FILL, SPI_FILL,
      SPI_FILL, SPI_FILL, SPI_FILL, SPI_FILL,
      SPI_FILL, SPI_FILL, SPI_FILL}; // Burst read acc & gyro regs (0x3B-0x48).
  uint8_t burst_rx[23];                                           // SPI burst read holders.
  uint8_t m_tx_buf[2] = {REG_BANK_SEL | WRITEMASTER, REG_BANK_0}; /**< TX buffer. */
  uint8_t m_rx_buf[15];                                           /**< RX buffer. */

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

  // Point to register bank 0 for reading the data from sensors.
  spiReadWriteIMU(m_tx_buf, 2, m_rx_buf, 2);

  if (magnetoConfig.isEnabled)
  {
    if (magneto_first_readTemp == (GYRO_SAMPLING_RATE / MAGNETO_SAMPLING_RATE) / 2)
    {
      // Checking the magnetometer status bits

      while (checkMag < 40000)
      {
        spiReadWriteIMU(burst_tx_INT_STAT, 2, burst_rx, 2);
        if ((burst_rx[1] & 0x01) == 0x01)
        {
          validMeasurement = true;
          checkMag = 0;

          break;
        }
        checkMag = checkMag + 1;
      }

      magnetometer_read_sample_config(MAGNETOMETER_SET_EXT_SENSOR);
    }

    else if (magneto_first_readTemp == (GYRO_SAMPLING_RATE / MAGNETO_SAMPLING_RATE) / 2 + 1)
      magnetometer_data_read_send(validMeasurement, pktCounter);
    else if (magneto_first_readTemp == (GYRO_SAMPLING_RATE / MAGNETO_SAMPLING_RATE) / 2 + 2)
      magnetometer_read_sample_config(MAGNETOMETER_SINGLE);
    else if (magneto_first_readTemp == (GYRO_SAMPLING_RATE / MAGNETO_SAMPLING_RATE) / 2 + 3)
      magnetometer_read_sample_config(MAGNETOMETER_SET_EXT_TOREAD);
  }

  // Point to register bank 0 for reading the data from sensors.
  spiReadWriteIMU(m_tx_buf, 2, m_rx_buf, 2);
  // printf("q0=%f,q1=%f,q2=%f,q3=%f\n",
  //   quaternionResult_1[0],quaternionResult_1[1],
  //   quaternionResult_1[2],quaternionResult_1[3]);
  if (the_device->gyro_first_read == 0)
  {
    spiReadWriteIMU(burst_tx, 7, burst_rx, 7);
    for (int i = 0; i < 6; i++)
      blePktMotion[i] = burst_rx[i + 1];

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
    currentAccData.accx = dataReadAccX;
    currentAccData.accy = dataReadAccY;
    currentAccData.accz = dataReadAccZ;

    accelX = dataReadAccX * dividerAcc / 1.0;
    accelY = dataReadAccY * dividerAcc / 1.0;
    accelZ = dataReadAccZ * dividerAcc / 1.0;
    currentAccData.accx_val = accelX;
    currentAccData.accy_val = accelY;
    currentAccData.accz_val = accelZ;
    calculate_enmo(accelX, accelY, accelZ);

    temp1[0].float_val = quaternionResult_1[0];
    temp1[1].float_val = quaternionResult_1[1];
    temp1[2].float_val = quaternionResult_1[2];

    for (uint8_t i = 0; i < 3; i++)
      quaternionResult_1[i] = 0.0;
    quaternionResult_1[3] = 1.0;

    // collect gyroscope for values 6-12
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

    int16_t accel_and_gyro[15] = {dataReadAccX, dataReadAccY, dataReadAccZ};

    memcpy(&accel_and_gyro[3], temp1[0].floatcast, sizeof(temp1[0].floatcast));
    memcpy(&accel_and_gyro[5], temp1[1].floatcast, sizeof(temp1[1].floatcast));
    memcpy(&accel_and_gyro[7], temp1[2].floatcast, sizeof(temp1[2].floatcast));
		  
    uint32_t current_time = get_current_unix_time();

    uint64_t ticks = k_uptime_get();
    // int16_t accel_and_gyro[13] = {dataReadAccX, dataReadAccY, dataReadAccZ, dataReadGyroX, dataReadGyroY, dataReadGyroZ, global_counter};

    memcpy(&accel_and_gyro[9], &currentAccData.ENMO, sizeof(currentAccData.ENMO));

    memcpy(&accel_and_gyro[11], &current_time, sizeof(current_time));

    memcpy(&accel_and_gyro[13], &global_counter, sizeof(global_counter));

    store_data(accel_and_gyro, sizeof(accel_and_gyro), 1);

    // this function seperately fills blePktMotion with the desired size
    // TODO: Make sure packets are in correct size/order
    // ppg_bluetooth_fill(blePktMotion);

    blePktMotion[18] = blePktMotion[18] | ((pktCounter >> 8) & 0x03);

    // collect packet counter
    // blePktMotion[18] = (pktCounter&) >> 14;
    blePktMotion[19] = ((pktCounter) & 0xFF);

#ifdef CONFIG_MSENSE3_BLUETOOTH_DATA_UPDATES
    my_motionData.dataPacket = blePktMotion;
    my_motionData.packetLength = ACC_GYRO_DATA_LEN;
    if (accelConfig.txPacketEnable == true || CONFIG_BLUETOOTH_SETTINGS_OVERRIDE)
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
  /*
  int64_t timer_value = stop_timer();
  if (rand() % 100 == 5){
    LOG_WRN("Timer Value: %lli ms", timer_value);
  }
  */
}

void magnetometer_config(void){
  if (magnetoConfig.isEnabled){
    uint8_t m_tx_buf[2] = {READMASTER | WHO_AM_I,SPI_FILL};		/**< TX buffer. */
    uint8_t m_rx_buf[sizeof(m_tx_buf)];  /**< RX buffer. */
    uint8_t tx_buf[14], rx_buf[4];  /**<Tx/ RX buffer. */

    const uint8_t m_length = sizeof(m_tx_buf); /**< Transfer length. */

    static uint8_t magneto_chipId[10] ={ 
      REG_BANK_SEL, REG_BANK_3, //changing register bank to register bank 3
      I2C_SLV0_ADDR_IMU, (READMASTER | MAGNETOADDRESS),
      I2C_SLV0_REG_IMU,MAGNETOMETER_ID, // read from register address 0 in magnetometer
      I2C_SLV0_CTRL_IMU, I2C_SLV_EN | 0x01, // read 2 bytes from the register 
      REG_BANK_SEL, REG_BANK_0, // change back to register bank 0
    };// commands for configuring I2C master in IMU
    
    static uint8_t magnetoConfig[14] = {   
      REG_BANK_SEL,REG_BANK_3, //switching to register bank 3
      I2C_SLV0_ADDR_IMU, MAGNETOADDRESS,
      I2C_SLV0_REG_IMU,MAGNETOMETER_CNTL2, // writing in CNTL2 register to power down mode
      I2C_SLV0_CTRL_IMU, I2C_SLV_EN | 0x01,
      I2C_SLV0_DO_IMU,0x00,
      I2C_SLV0_CTRL_IMU,I2C_SLV_EN | 0x01,
      REG_BANK_SEL, REG_BANK_0
    }; // commands for powering down magnetomete
    
    for(int i = 0; i < sizeof(magneto_chipId); i += 2){
      m_tx_buf[0] = magneto_chipId[i];
      m_tx_buf[1] = magneto_chipId[i+1];
      spiReadWriteIMU(m_tx_buf, m_length,m_rx_buf, m_length);
      
    }

    k_msleep(15);    
    // Going to the User Bank 0
    tx_buf[0] = REG_BANK_SEL;
    tx_buf[1] = REG_BANK_0;
    spiReadWriteIMU(tx_buf, 2,rx_buf, 2);

    tx_buf[0] =(READMASTER | EXT_SENSE_DATA_0_IMU);
    tx_buf[1] = SPI_FILL;
    spiReadWriteIMU(tx_buf, 2,rx_buf, 2);
    printk("magnetometer chip ID=%x,%x\n",rx_buf[0],rx_buf[1]);
    k_msleep(15);  
    // Going to the User Bank 3
    tx_buf[0] = REG_BANK_SEL;
    tx_buf[1] = REG_BANK_3;
    spiReadWriteIMU(tx_buf, 2,rx_buf, 2);
    for(int i = 0; i < sizeof(magnetoConfig); i += 2){
      m_tx_buf[0] = magnetoConfig[i];
      m_tx_buf[1] = magnetoConfig[i+1];
      spiReadWriteIMU(m_tx_buf, m_length, m_rx_buf, m_length);
    }
		
    for(int i = 0; i < sizeof(magnetoConfig); i += 2){
      m_tx_buf[0] = magnetoConfig[i];
      m_tx_buf[1] = magnetoConfig[i+1];
      spiReadWriteIMU(m_tx_buf, m_length,m_rx_buf, m_length);
    }
//single mode
    tx_buf[0] = REG_BANK_SEL ;
    tx_buf[1] = REG_BANK_3; // switching to register bank 3
    tx_buf[2] = I2C_SLV0_ADDR_IMU;
    tx_buf[3] = MAGNETOADDRESS;
    tx_buf[4] = I2C_SLV0_REG_IMU;
    tx_buf[5] = MAGNETOMETER_CNTL2;
    tx_buf[6] = I2C_SLV0_CTRL_IMU;
    tx_buf[7] = I2C_SLV_EN | 0x01;
    tx_buf[8] = I2C_SLV0_DO_IMU;
    tx_buf[9] = MAGNETO_SINGLE_MEASUREMENT;
    tx_buf[10] = I2C_SLV0_CTRL_IMU;
    tx_buf[11] = I2C_SLV_EN | 0x01;
    tx_buf[12] = REG_BANK_SEL;
    tx_buf[13] = REG_BANK_0;
    k_msleep(15);
    for(int i = 0; i < 14; i += 2){
      m_tx_buf[0] = tx_buf[i];
      m_tx_buf[1] = tx_buf[i+1];
      spiReadWriteIMU(m_tx_buf, m_length,m_rx_buf, m_length);
    }
   // Setting up External sensor data to read magnetometer 
    tx_buf[0] = REG_BANK_SEL;
    tx_buf[1] = REG_BANK_3;
    tx_buf[2] = I2C_SLV0_ADDR_IMU;
    tx_buf[3] = READMASTER | MAGNETOADDRESS;
    tx_buf[4] = I2C_SLV0_REG_IMU;
    tx_buf[5] = MAGNETOMETER_ST1; // reading the ST1 register to check DRDY bit
    tx_buf[6] = I2C_SLV0_CTRL_IMU;
    tx_buf[7] = I2C_SLV_EN | 0x01;
    tx_buf[8] = REG_BANK_SEL;
    tx_buf[9] = REG_BANK_0; // changing back to register bank 0	
    k_msleep(15);
    for(int i = 0; i < 10; i += 2){
      m_tx_buf[0] = tx_buf[i];
      m_tx_buf[1] = tx_buf[i+1];
      spiReadWriteIMU(m_tx_buf, m_length, m_rx_buf, m_length);
    }
    k_msleep(15);
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

    static uint8_t imu_config[36] = {
      REG_BANK_SEL, REG_BANK_0,// change register bank 0
      USER_CTRL,I2C_MST_EN | I2C_IF_DIS, // SPI mode and enable I2c master
      PWR_MGMT_1,IMU_TEMP_DISABLE | IMU_CLK_SEL_BEST_SEL, // Disable temparature sensor and select best available clock source
      PWR_MGMT_2,ENABLE_ACC | ENABLE_GYRO, // Not disabling Gyro and accel
      LP_CONFIG,I2C_MST_CYCLE,
      INT_PIN_CFG,0x00,
      REG_BANK_SEL,REG_BANK_2, // changing the register bank to 2
      GYRO_SMPLRT_DIV,0x03, // 1100/(1+GYRO_SMPLRT_DIV) 
                            //  rate of Gyroscope = 275 Hz	
      GYRO_CONFIG_1,GYRO_DLPFCFG_51HZ | GYRO_FS_SEL_500, 
                            // gyro full scale =500 dps, LPF = 119.5 Hz 
      ACCEL_CONFIG, ACCEL_DLPFCFG_12HZ | ACCEL_FS_SEL_4g |
        ACCEL_FCHOICE_DLPF_ENABLE   , // accel full scale =4g, LPF = 11.6 Hz
      ACCEL_SMPLRT_DIV_1,0x00, // sample rate accel MSB
      ACCEL_SMPLRT_DIV_2,0x10, // sampling rate accel = 66 Hz (1125/(1+DIV))
      REG_BANK_SEL,REG_BANK_3,
      I2C_MST_ODR_CONFIG, 0x01, // i2c master dutycycle configuration = 550 Hz,
      I2C_MST_DELAY_CTRL, DELAY_ES_SHADOW, // i2c_mst_delay_ctl = delays shadowing of external sensor
      I2C_MST_CTRL_IMU,I2C_MST_P_NSR_STOP_READS |
        I2C_MST_CLK_345KHZ_40DUTY, // setting i2c master clock = 345 Hz and 46.67% duty cycle recommended
      I2C_SLV0_ADDR_IMU, READMASTER| MAGNETOADDRESS, // setting I2c slave address to magnetometer address
      REG_BANK_SEL,REG_BANK_0
    };	/**< IMU configuration commands. */
    
    // edit ACCEL_CONFIG 
    imu_config[19] = 0x01 | accelConfig.sensitivity | accelConfig.sample_bw;
    // edit GYRO_CONFIG1
    imu_config[17] = 0x11 | gyroConfig.sensitivity;
    for(int i = 0; i < sizeof(imu_config); i += 2){
      m_tx_buf[0] = imu_config[i];
      m_tx_buf[1] = imu_config[i+1];
      spiReadWriteIMU(m_tx_buf, m_length, m_rx_buf, m_length);
    }
    k_msleep(10);
    magnetometer_config();    
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

    static uint8_t imu_config[8] = {
      REG_BANK_SEL,REG_BANK_2, // changing the register bank to 2
      GYRO_SMPLRT_DIV,0x01, // 1100/(1+GYRO_SMPLRT_DIV) 
                            //  rate of Gyroscope = 550 Hz	
      GYRO_CONFIG_1,GYRO_DLPFCFG_51HZ , 
                            // gyro full scale =500 dps, LPF = 119.5 Hz 
      ACCEL_CONFIG,ACCEL_FCHOICE_DLPF_ENABLE   , // accel full scale =4g, LPF = 11.6 Hz
    };
    imu_config[5] = imu_config[5] | gyroConfig.sensitivity;
    imu_config[7] = imu_config[7] | accelConfig.sample_bw 
      | accelConfig.sensitivity;

    for(int i = 0; i < sizeof(imu_config); i += 2){
      m_tx_buf[0] = imu_config[i];
      m_tx_buf[1] = imu_config[i+1];
      spiReadWriteIMU(m_tx_buf, m_length, m_rx_buf, m_length);
    }
  }
}
void magnetometer_sleep(void){
  if(magnetoConfig.isEnabled){
    uint8_t tx_buf[12];  /**<Tx/ RX buffer. */
    uint8_t m_tx_buf[2] = {WHO_AM_I | READMASTER,SPI_FILL};/**< TX buffer. */
    uint8_t m_rx_buf[sizeof(m_tx_buf)];  /**< RX buffer. */
    const uint8_t m_length = sizeof(m_tx_buf); /**< Transfer length. */

    // Powering down magnetometer
    tx_buf[0] = REG_BANK_SEL;
    tx_buf[1] = REG_BANK_3;
    tx_buf[2] = I2C_SLV0_ADDR_IMU;
    tx_buf[3] = MAGNETOADDRESS;
    tx_buf[4] = I2C_SLV0_REG_IMU;
    tx_buf[5] = MAGNETOMETER_CNTL2;
    tx_buf[6] = I2C_SLV0_CTRL_IMU;
    tx_buf[7] = I2C_SLV_EN | 0x01;
    tx_buf[8] = I2C_SLV0_DO_IMU;
    tx_buf[9] = MAGNETOMETER_POWERDOWN;
    tx_buf[10] = I2C_SLV0_CTRL_IMU;
    tx_buf[11] = I2C_SLV_EN | 0x01;
    for(int i = 0; i < 12; i += 2){
      m_tx_buf[0] = tx_buf[i];
      m_tx_buf[1] = tx_buf[i+1];
      spiReadWriteIMU(m_tx_buf, m_length,m_rx_buf, m_length);
    }
    // Returning back to the User Bank 0
    m_tx_buf[0] = REG_BANK_SEL;
    m_tx_buf[1] = REG_BANK_0;
    spiReadWriteIMU(m_tx_buf,2, m_rx_buf, 2);
  }
}

/**
 * @brief Function for configuring the motion processor to sleep mode and put GYRO in standby mode.
 *
 * @details  Configures registers in the motion processor in idle state.
 */
  
void motion_sleep(void){
  if(gyroConfig.isEnabled){
    uint8_t m_tx_buf[2] = {0xF5,0xFF};		/**< TX buffer. */
    uint8_t m_rx_buf[sizeof(m_tx_buf)];  /**< RX buffer. */
    const uint8_t m_length = sizeof(m_tx_buf); /**< Transfer length. */
 
    const uint8_t imu_config[4] = { 
      REG_BANK_SEL,REG_BANK_0,
      PWR_MGMT_1,IMU_SLEEP | IMU_CLK_SEL_BEST_SEL  // sleep mode and temparature disable
    };	/**< IMU configuration commands. */
		
    for(int i = 0; i < sizeof(imu_config); i += 2){
      m_tx_buf[0] = imu_config[i];
      m_tx_buf[1] = imu_config[i+1];
      spiReadWriteIMU(m_tx_buf, m_length, m_rx_buf, m_length);
    }	
  }
}
