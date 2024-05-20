//#include "arm_const_structs.h"
//#include "arm_math.h"

#include "ppgSensor.h"
#include "imuSensor.h"
#include "BLEService.h"
#include "common.h"

#include <math.h>
#include <stdio.h>
#include <zephyr/drivers/spi.h>
#include <zephyr/logging/log.h> 
#include <zephyr/fs/fs.h>


LOG_MODULE_REGISTER(ppg_sensor);

#define LED_GREEN 1

uint32_t timeWindow=50;

float32_t std_ppgThreshold_lower = 120;

uint8_t low_ch1 =0,up_ch1 = 0x3f;
uint8_t low_ch2 =0,up_ch2 = 0xff;
uint8_t adapt_counterCh1=0,adapt_counterCh2=0;
uint8_t adapt_Ch1=0,adapt_Ch2=0;

uint8_t blePktPPG_noFilter[ble_ppg_noFilter_byteLength];
uint8_t blePktPPG_Filter[ble_ppg_Filter_byteLength];


float32_t runningMeanCh1a =0.0f, runningMeanCh1aFil=0.0f, runningSquaredMeanCh1aFil=0.0f;
float32_t runningMeanCh1b=0.0f, runningMeanCh1bFil=0.0f, runningSquaredMeanCh1bFil=0.0f;
float32_t runningMeanCh2a=0.0f, runningMeanCh2aFil=0.0f, runningSquaredMeanCh2aFil=0.0f;
float32_t runningMeanCh2b=0.0f, runningMeanCh2bFil=0.0f, runningSquaredMeanCh2bFil=0.0f;
uint8_t goodCh1=1,goodCh2=1;
uint8_t badDataCounterCh1=0,badDataCounterCh2=0; 
uint32_t chLED_upperBound=390000;
uint32_t chLED_lowerBound=200000;
uint8_t prevFlag= 0,prevFlag2= 0;
uint8_t counterCheck=0;
float32_t std_ppgThreshold =150;

//#endif // #ifdef MOTIONSENSE_MAGNETO
uint8_t adaptIterMax = 30;
uint8_t binarySteps=0;

//static float ppg_num[400];


int ppg_print_counter;
// Channel 1A - IR 1
// Channel 1B - IR 2
// Channel 2A - Green 1
// Channel 2B - Green 2

void spiReadWritePPG(uint8_t * tx_buffer, 
  uint8_t txLen, uint8_t * rx_buffer, uint8_t rxLen){
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
  err = spi_transceive(spi_dev_ppg, &spi_cfg_ppg, &tx, &rx);
  if (err) 
    printk("SPI error: %d\n", err);
}

void spiWritePPG(uint8_t * tx_buffer, uint8_t txLen){
  int err;
  const struct spi_buf tx_buf = {
    .buf = tx_buffer,
    .len = txLen
  };
  const struct spi_buf_set tx = {
    .buffers = &tx_buf,
    .count = 1
  };
  err = spi_transceive(spi_dev_ppg, &spi_cfg_ppg, &tx,NULL);
  if (err) 
    printk("SPI error: %d\n", err);
	
}


void ppg_config(void){
  if (ppgConfig.isEnabled) {
    //fileOpen();
    
    
    uint8_t rxLen,txLen; 
    // Read chip ID 
    uint8_t cmd_array[] = {PPG_CHIP_ID_1, READMASTER, SPI_FILL};
    uint8_t read_array[5] = {0};
    txLen=3;
    rxLen=3;
    spiReadWritePPG(cmd_array, txLen, read_array, rxLen);

    // Resetting PPG sensor
    cmd_array[0] = PPG_SYS_CTRL;
    cmd_array[1] = WRITEMASTER;
    cmd_array[2] = PPG_RESET;
    spiWritePPG(cmd_array, txLen);

    // Shutting down PPG sensor
    cmd_array[2] = PPG_SHUTDOWN;
    spiWritePPG(cmd_array, txLen);

    // Reading interrupt status register 1
    cmd_array[0] = PPG_INT_STAT_1;
    cmd_array[1] = READMASTER;
    spiReadWritePPG(cmd_array, txLen, read_array, rxLen);

    // Reading interrupt status register 2	
    cmd_array[0] = PPG_INT_STAT_2;
    spiReadWritePPG(cmd_array, txLen, read_array, rxLen);

    #if  defined(LED_RED )
    //nrf_delay_ms(5);
		cmd_array[0] = 0x11;		
		cmd_array[1] = 0x00;
		cmd_array[2] = 0x03; // Red LED 0x3F
		nrf_drv_spi_transfer(&spi_ppg, cmd_array, 3, read_array, 3);
    while (!spi_ppg_xfer_done)
    {
        __WFE();
    }
		spi_ppg_xfer_done = false;
    #endif
  // PPG configuration register - ALC enabled + 
  // PPG ADC Range - 4096 nA, 117.3us integration time
    #if  defined(LED_GREEN )
      cmd_array[0] = PPG_CONFIG_1;		
      cmd_array[1] = WRITEMASTER;
      cmd_array[2] = PPG_TINT_117_3us; 
      spiWritePPG(cmd_array, txLen);
    #endif
		
// Change Sampling rate PPG
    cmd_array[0] = PPG_CONFIG_2;
    cmd_array[2] = PPG_SR_400_1 | ppgConfig.sample_avg;
    spiWritePPG(cmd_array, txLen);
      
    //PPG coniguration 3- LED settling time =12us
    cmd_array[0] = PPG_CONFIG_3;
    cmd_array[1] = WRITEMASTER;
    cmd_array[2] = PPG_LED_SETLNG_12us;
    spiWritePPG(cmd_array, txLen);
      
    // Photo-diode Bias 0 to 65pF 
    cmd_array[0] = PPG_PHOTODIODE_BIAS;
    cmd_array[2] = (uint8_t)(PPG_PDBIAS_65pF<<4) | (uint8_t)PPG_PDBIAS_65pF; 
    spiWritePPG(cmd_array, txLen);
      
      // Configuring LED drive 3 (Green) range 124 mA
      //            LED drive 2 (Green) range 124 mA	
      //            LED drive 1 (IR) range 31 mA
    #if  defined(LED_GREEN )
      cmd_array[0] = PPG_LED_RANGE_1; 
      cmd_array[2] = (uint8_t)(PPG_LED_CURRENT_124mA <<4) | (uint8_t)(PPG_LED_CURRENT_124mA <<2)|PPG_LED_CURRENT_31mA; 
      spiWritePPG(cmd_array, txLen);
    #endif
		
	  #if  defined(LED_RED )
		cmd_array[0] = 0x2A;
		cmd_array[2] = 0x00; // change back to x15
    //cmd_array[2] = 0x03;
		nrf_drv_spi_transfer(&spi_ppg, cmd_array, 3, read_array, 3);
    while (!spi_ppg_xfer_done)
    {
        __WFE();
    }
		spi_ppg_xfer_done = false;
	  //nrf_delay_ms(5);
		#endif
                
    // LED 1 Driver current setting (IR )
    cmd_array[0] = PPG_LED1_PA;
    cmd_array[2] = ppgConfig.infraRed_intensity; 
    spiWritePPG(cmd_array, txLen);
    
    // LED 2 Driver current setting (Green )
    cmd_array[0] = PPG_LED2_PA;
    cmd_array[2] = ppgConfig.green_intensity; 
    spiWritePPG(cmd_array, txLen);
    
    // LED 3 Driver current setting (Green )
    cmd_array[0] = PPG_LED3_PA;
    spiWritePPG(cmd_array, txLen);

    // System control reqister - Low Power Mode + shutoddown 
    cmd_array[0] = PPG_SYS_CTRL;
    cmd_array[2] = PPG_LP_MODE | PPG_SHUTDOWN;
    spiWritePPG(cmd_array, txLen);    
    
    // FIFO configuration - 15 samples stored in FIFO
    cmd_array[0] = PPG_FIFO_CONFIG_1;
    cmd_array[2] = 0x0F; 
    spiWritePPG(cmd_array, txLen);    

    // FIFO configuration 2 Push enable when FIFO is full
    cmd_array[0] = PPG_FIFO_CONFIG_2;
    cmd_array[2] = PPG_FIFO_PUSH_ENABLE;
    spiWritePPG(cmd_array, txLen);    

    // Interrupt Enable A_full interrupt is enabled
    cmd_array[0] = PPG_INT_EN_1;
    cmd_array[2] = PPG_INT_A_FULL_EN;
    spiWritePPG(cmd_array, txLen);    

    // Green LED 2 and LED 3 is pulsed simultaneously first 
    // then IR LED is pulsed next
    cmd_array[0] = PPG_LED_SEQ_1;
    cmd_array[2] = PPG_LEDC2_LED2_LED3_SIMULT | PPG_LEDC1_LED1; 
    spiWritePPG(cmd_array, txLen);    

    // System control Dual PPG + Low power mode enabled
    cmd_array[0] = PPG_SYS_CTRL;
    cmd_array[2] = PPG_LP_MODE;
    spiWritePPG(cmd_array, txLen);  
    
    low_ch1 =0; up_ch1 = 0x3f;
    low_ch2 =0; up_ch2 = 0xff;
    adapt_counterCh1=0;
    adapt_counterCh2=0;
    counterCheck =0;
    badDataCounterCh1=0;  
    badDataCounterCh2=0;
    prevFlag=0;prevFlag2=0;
    adapt_Ch1=1;
    adapt_Ch2=1;
    goodCh1=0;
    goodCh2=0;
    ppgData1.bufferIndex=0;
  }
}
void ppg_changeIntensity(void){
  if (ppgConfig.isEnabled) {
    uint8_t rxLen,txLen; 
    // Read chip ID 
    uint8_t cmd_array[] = {PPG_CHIP_ID_1, WRITEMASTER, SPI_FILL};
    uint8_t read_array[5] = {0};
    txLen=3;
    rxLen=3;
   // LED 1 Driver current setting (IR )
    cmd_array[0] = PPG_LED1_PA;
    cmd_array[2] = ppgConfig.infraRed_intensity; 
    spiWritePPG(cmd_array, txLen);
    
    // LED 2 Driver current setting (Green )
    cmd_array[0] = PPG_LED2_PA;
    cmd_array[2] = ppgConfig.green_intensity; 
    spiReadWritePPG(cmd_array, txLen, NULL, 0);
    
    // LED 3 Driver current setting (Green )
    cmd_array[0] = PPG_LED3_PA;
    spiReadWritePPG(cmd_array, txLen, NULL, 0);
  }
}
void ppg_changeSamplingRate(void){
  if (ppgConfig.isEnabled) {
    uint8_t rxLen,txLen; 
    // Read chip ID 
    uint8_t cmd_array[] = {PPG_CHIP_ID_1, WRITEMASTER, SPI_FILL};
    uint8_t read_array[5] = {0};
    txLen=3;
    rxLen=3;
  // Change Sampling rate PPG
    cmd_array[0] = PPG_CONFIG_2;
    cmd_array[2] = PPG_SR_400_1 | ppgConfig.sample_avg;
    spiWritePPG(cmd_array, txLen);
  }
}

/**	
 * @brief Function for configuring the ppg sensor to shutdown mode.
 *
 * @details  Configures shutdown bit in mode registers .
 */
void ppg_sleep(void){
  uint8_t txLen=3; 
  if (ppgConfig.isEnabled) {
    uint8_t cmd_array[] = {0xFF, 0x80, 0xFF};
   
    // System control Shutdown
    cmd_array[0] = PPG_SYS_CTRL;
    cmd_array[1] = WRITEMASTER;
    cmd_array[2] = PPG_SHUTDOWN;
    spiWritePPG(cmd_array, txLen);  
    
  }
}

uint8_t searchStep( uint8_t adapt_counter, float meanCha,float stdCha_fil,
uint8_t* low_ch,uint8_t* up_ch, uint8_t midVal,uint8_t stepSize)
{
  printk("in f search step, mean: %d\n", meanCha);
  if(meanCha < chLED_upperBound-40000 ){
    if(stdCha_fil > std_ppgThreshold ){
	
    }
    else{ // room for improvement so increase led current
      if(adapt_counter <binarySteps ){
        *low_ch = (*low_ch + *up_ch)/2+1;
	midVal = (*low_ch + *up_ch)/2;
      }
      else{
        if(midVal +  stepSize < 0xFF)
          midVal=midVal+ stepSize;
      }
    }						
  }
  else{ // close to saturation so decrease led current
    if(adapt_counter < binarySteps){
      *up_ch = (*low_ch + *up_ch)/2-1;
      midVal = (*low_ch + *up_ch)/2;
    }
    else{
      if(midVal-stepSize >= 0)
        midVal=midVal-stepSize;
    }
  }
return midVal;
}

void ppg_led_currentUpdate(void){
  //Ch1a - IR1, Ch1b - IR2, Ch2a - G1, Ch2b - G2 
  float meanCh1=0,meanCh2=0;	
  float stdCh1_fil=0,stdCh2_fil=0; 
  uint8_t IR_steps = 1, green_steps = 3;
  uint8_t cmd_array[] = {PPG_CHIP_ID_1, WRITEMASTER, SPI_FILL};
  uint8_t txLen=3;
  if (ppgConfig.isEnabled){
    if(counterCheck == timeWindow){
      if(gyroData1.movingFlag ==0){ // Motion is minimal
        arm_sqrt_f32(runningSquaredMeanCh1aFil - timeWindow/(timeWindow-1.0f)
          *runningMeanCh1aFil*runningMeanCh1aFil, &ppgData1.stdChanIR_1);
	arm_sqrt_f32(runningSquaredMeanCh1bFil - timeWindow/(timeWindow-1.0f)
          *runningMeanCh1bFil*runningMeanCh1bFil, &ppgData1.stdChanIR_2);
	arm_sqrt_f32(runningSquaredMeanCh2aFil - timeWindow/(timeWindow-1.0f)
          *runningMeanCh2aFil*runningMeanCh2aFil, &ppgData1.stdChanGreen_1);
	arm_sqrt_f32(runningSquaredMeanCh2bFil - timeWindow/(timeWindow-1.0f)
          *runningMeanCh2bFil*runningMeanCh2bFil, &ppgData1.stdChanGreen_2);

	ppgData1.meanChanIR_1 = runningMeanCh1a;
	ppgData1.meanChanIR_2 = runningMeanCh1b;
	ppgData1.meanChanGreen_1 = runningMeanCh2a;
	ppgData1.meanChanGreen_2 = runningMeanCh2b;

	if(ppgData1.meanChanIR_1>= ppgData1.meanChanIR_2){
          meanCh1 = ppgData1.meanChanIR_1;
          stdCh1_fil = ppgData1.stdChanIR_1;	
	}
        else{
          meanCh1 = ppgData1.meanChanIR_2;
          stdCh1_fil = ppgData1.stdChanIR_2;	
	}
	if(ppgData1.meanChanGreen_1>= ppgData1.meanChanGreen_2){
          meanCh2 = ppgData1.meanChanGreen_1;
          stdCh2_fil = ppgData1.stdChanGreen_1;	
        }
        else{
          meanCh2 = ppgData1.meanChanGreen_2;
          stdCh2_fil = ppgData1.stdChanGreen_2;	
        }		
      }
      // If the adaptation flag is disabled and data quality is bad when the sensor is not moving
      printk("adapt_flag: %d\n", adapt_Ch2);
      printk("bad counter: %d\n", badDataCounterCh2);
      if(gyroData1.movingFlag ==0){
        if(adapt_Ch1==0){
          if(meanCh1 > chLED_upperBound || meanCh1 < chLED_lowerBound )
            badDataCounterCh1++;
          if(badDataCounterCh1 > 10){
            adapt_counterCh1=0;
            adapt_Ch1=1;
            badDataCounterCh1 = 0;
          }						
        }
        if(adapt_Ch2==0){
          if(meanCh2 > chLED_upperBound || meanCh2 < chLED_lowerBound )
            badDataCounterCh2++;
          if(badDataCounterCh2 > 10){
            adapt_counterCh2=0;
            adapt_Ch2=1;
            badDataCounterCh2 = 0;
          }						
	}
      }
      printk("moving flag: %d\n", gyroData1.movingFlag);
      if(gyroData1.movingFlag ==0 && adapt_Ch1 ==1){ // Motion is minimal an adaptation is required
        ppgConfig.infraRed_intensity = searchStep( 
                adapt_counterCh1,meanCh1, stdCh1_fil,
		&low_ch1,&up_ch1,ppgConfig.infraRed_intensity, IR_steps);
			 
	cmd_array[0] = PPG_LED1_PA;
	cmd_array[2] = ppgConfig.infraRed_intensity; // changing it to 0x10 from 0x20
        spiWritePPG(cmd_array, txLen);  
				
	adapt_counterCh1++;
	printk("adapt counter length: %d\n", adapt_counterCh1);
	if(adapt_counterCh1 >adaptIterMax) {
          adapt_counterCh1=adaptIterMax;
          adapt_Ch1=0;
          goodCh1 = 1;
          badDataCounterCh1=0;
        }
      }
      if(gyroData1.movingFlag ==0 && adapt_Ch2 ==1){
        ppgConfig.green_intensity = searchStep( 
          adapt_counterCh2,meanCh2, stdCh2_fil,
          &low_ch2,&up_ch2,ppgConfig.green_intensity, green_steps);
	txLen=3;			
	cmd_array[0] = PPG_LED2_PA;
        printk("new ppg intensity: %d\n", ppgConfig.green_intensity);
	cmd_array[2] = ppgConfig.green_intensity; // Green 49.9mA
        spiWritePPG(cmd_array, txLen);  
				
	cmd_array[0] = PPG_LED3_PA;
        spiWritePPG(cmd_array, txLen);  
	adapt_counterCh2++;
	if(adapt_counterCh2 >adaptIterMax) {
          adapt_counterCh2=adaptIterMax;
          adapt_Ch2=0;
          goodCh2 =1;
          badDataCounterCh2=0;
	}
      }
      if(adapt_counterCh2 ==adaptIterMax && adapt_counterCh2 ==adaptIterMax ){
        
        uint32_t dataFlash = (ppgConfig.green_intensity)<<8 + ppgConfig.infraRed_intensity;
      }
    }
  }
}


void ppg_bluetooth_preprocessing_raw(uint32_t* led1A, uint32_t* led1B, uint32_t* led2A, uint32_t* led2B, uint16_t pktCounter){


  uint32_cast buff_val_raw;

  buff_val_raw.integer = led1A[0];
  blePktPPG_noFilter[0] = ((buff_val_raw.intcast[2]&0x07)<<5)|((buff_val_raw.intcast[1])&0xF8)>>3;
  blePktPPG_noFilter[1] = ((buff_val_raw.intcast[1]&0x07)<<5)|((buff_val_raw.intcast[0]&0xF8)>>3);
  blePktPPG_noFilter[2] =  (buff_val_raw.intcast[0]&0x07)<<5;
        
  buff_val_raw.integer = led1B[0];
  blePktPPG_noFilter[2] = blePktPPG_noFilter[2] | ((buff_val_raw.intcast[2]&0x07)<<2) |((buff_val_raw.intcast[1]&0xC0)>>6);
  blePktPPG_noFilter[3] = ((buff_val_raw.intcast[1]&0x3F)<<2) | ((buff_val_raw.intcast[0]&0xC0) >>6);
  blePktPPG_noFilter[4] = ((buff_val_raw.intcast[0]&0x3F) <<2) ;
        
  buff_val_raw.integer = led2A[0];
  blePktPPG_noFilter[4] = blePktPPG_noFilter[4] | ((buff_val_raw.intcast[2]&0x06)>>1);
  blePktPPG_noFilter[5] = ((buff_val_raw.intcast[2]&0x01)<<7) | ((buff_val_raw.intcast[1]&0xFE)>>1);
  blePktPPG_noFilter[6] = ((buff_val_raw.intcast[1]&0x01)<<7) | ((buff_val_raw.intcast[0]&0xFE)>>1);
  blePktPPG_noFilter[7] = ((buff_val_raw.intcast[0]&0x01)<<7);
        
  buff_val_raw.integer = led2B[0];
  blePktPPG_noFilter[7] = blePktPPG_noFilter[7] | ((buff_val_raw.intcast[2]&0x07) <<4)|((buff_val_raw.intcast[1]&0xF0) >>4);
  blePktPPG_noFilter[8] = ((buff_val_raw.intcast[1]&0x0F) <<4) | ((buff_val_raw.intcast[0]&0xF0)>>4);
  blePktPPG_noFilter[9] =  (buff_val_raw.intcast[0]&0x0F) <<4;
  blePktPPG_noFilter[10] = (pktCounter&0xFF00) >> 8;
  blePktPPG_noFilter[11] = (pktCounter&0x00FF);


}



void read_ppg_fifo_buffer(struct k_work *item){
  struct ppgInfo* the_device=  ((struct ppgInfo *)(((char *)(item)) 
    - offsetof(struct ppgInfo, work)));
  
  uint16_t pktCounter = the_device->pktCounter;
  bool movingFlag = the_device->movingFlag;
  bool ppgTFPass = the_device->ppgTFPass;
  uint8_t cmd_array[] = {PPG_CHIP_ID_1, WRITEMASTER, SPI_FILL};
  uint8_t read_array[128*2*2*3] = {0};
  uint8_t txLen,rxLen;
  uint32_t led1A[32];
  uint32_t led1B[32];
  uint32_t led2A[32];
  uint32_t led2B[32];
  
  uint8_t tag1A, tag1B, tag2A, tag2B, tag;
  float channel1A_in, channel1B_in, channel2A_in, channel2B_in;
  float meanChannel1A, meanChannel1B, meanChannel2A, meanChannel2B;

  float channel1A_out, channel1B_out, channel2A_out, channel2B_out;
  float_cast buff_val_filtered;
  
  counterCheck++;
  if(counterCheck > timeWindow) {
    counterCheck =0;
  }
  
  uint8_t sampleCount[5] = {0};
		
  // Reading the total number of PPG samples first
  cmd_array[0] = PPG_FIFO_DATA_COUNTER;
  cmd_array[1] = READMASTER;
  txLen=3;
  rxLen=3;
  spiReadWritePPG(cmd_array, txLen, sampleCount, rxLen);
    
  // Reading the actual PPG samples
  cmd_array[0] = PPG_FIFO_DATA;
  spiReadWritePPG(cmd_array, txLen, read_array, sampleCount[2]*3+2);


  int i, j, k;
  for (i=0; i<sampleCount[2]; i++){
    for (j=0; j <= 9; j=j+3){
      tag = (read_array[i*12+j+2] & 0xF8) >> 3;
      switch(tag){
        case PPG1_LEDC1_DATA: // IR 1
          led1A[i] = ((read_array[i*12+j+2]<<16) | (read_array[i*12+j+1+2]<<8)
           | (read_array[i*12+j+2+2])) & 0x7ffff;
          break;
        case PPG1_LEDC2_DATA: // Green 1
          led2A[i] = ((read_array[i*12+j+2]<<16) | (read_array[i*12+j+1+2]<<8)
           | (read_array[i*12+j+2+2])) & 0x7ffff;
          break;
        case PPG2_LEDC1_DATA: // IR 2
          led1B[i] = ((read_array[i*12+j+2]<<16) | (read_array[i*12+j+1+2]<<8)
           | (read_array[i*12+j+2+2])) & 0x7ffff;
          break;
        case PPG2_LEDC2_DATA: // Green 2
          led2B[i] = ((read_array[i*12+j+2]<<16) | (read_array[i*12+j+1+2]<<8)
           | (read_array[i*12+j+2+2])) & 0x7ffff;
          break;
      }
    }
    k = i;
    k = 0;
    tag1A = (read_array[k*12+0+2] & 0xF8) >>3;
    tag1B = (read_array[k*12+3+2] & 0xF8) >>3;
    tag2A = (read_array[k*12+6+2] & 0xF8) >>3;
    tag2B = (read_array[k*12+9+2] & 0xF8) >>3;
  }

  if(counterCheck ==0){
    runningMeanCh1a=0.0f;
    runningMeanCh1b=0.0f;
    runningMeanCh2a=0.0f;
    runningMeanCh2b=0.0f;
    runningMeanCh1aFil= 0.0f;
    runningMeanCh1bFil =0.0f;
    runningMeanCh2aFil =0.0f;
    runningMeanCh2bFil =0.0f;
    runningSquaredMeanCh1aFil= 0.0f;
    runningSquaredMeanCh1bFil =0.0f;
    runningSquaredMeanCh2aFil =0.0f;
    runningSquaredMeanCh2bFil =0.0f;
  }
  runningMeanCh1a = runningMeanCh1a +led1A[0]*1.0f/timeWindow;
  runningMeanCh1b = runningMeanCh1b +led1B[0]*1.0f/timeWindow;
  runningMeanCh2a = runningMeanCh2a +led2A[0]*1.0f/timeWindow;
  runningMeanCh2b = runningMeanCh2b +led2B[0]*1.0f/timeWindow;
  
  // TODO: Add log level define
  ppg_print_counter++;
  if (ppg_print_counter >= 12){
    LOG_INF("ppg led1A %d \n 1b %d \n 2a %d 2b %d", led1A[0], led1B[0], led2A[0], led2B[0]);
  }
  #ifdef CONFIG_MSENSE3
  // Transmitting the un-filtered data on BLE 
  if(ppgConfig.txPacketEnable == true){
    my_ppgDataSensor.dataPacket = blePktPPG_noFilter;
    my_ppgDataSensor.packetLength = PPG_DATA_UNFILTER_LEN;
    k_work_submit(&my_ppgDataSensor.work);
  }
  if(ppgTFPass){
    ppgData1.green_ch1_buffer[ppgData1.bufferIndex] = ppgData1.green_ch1;
    ppgData1.green_ch2_buffer[ppgData1.bufferIndex] = ppgData1.green_ch2;
    ppgData1.infraRed_ch1_buffer[ppgData1.bufferIndex] = ppgData1.infraRed_ch1;
    ppgData1.infraRed_ch1_buffer[ppgData1.bufferIndex] = ppgData1.infraRed_ch2;
    ppgData1.bufferIndex = (ppgData1.bufferIndex +1 )%400;
    if(ppgData1.bufferIndex ==0 ){
      ppgData1.dataReadyTF = true;
    }
  }
  #endif


  ppg_led_currentUpdate();
}



/* This function reads and fills bleSendArr with unfiltered ppg according
to the desired packet format */

/* NOTE: DEPRECATED, because this is theoretically the same function as ppg_read_fifo
TODO: figure out one solution for all */
void ppg_bluetooth_fill(uint8_t* bleSendArr){

  uint8_t cmd_array[] = {PPG_CHIP_ID_1, WRITEMASTER, SPI_FILL};
  uint8_t read_array[128*2*2*3] = {0};
  uint8_t txLen,rxLen;
  uint32_t led1A[32];
  uint32_t led1B[32];
  uint32_t led2A[32];
  uint32_t led2B[32];
  uint8_t tag1A, tag1B, tag2A, tag2B, tag;
  float channel1A_in, channel1B_in, channel2A_in, channel2B_in;
  float meanChannel1A, meanChannel1B, meanChannel2A, meanChannel2B;

  float channel1A_out, channel1B_out, channel2A_out, channel2B_out;
  float_cast buff_val_filtered;
  uint32_cast buff_val_raw;
  int i;
  uint8_t j, k;
  uint8_t sampleCnt[5] = {0};
		
  // Reading the total number of PPG samples
  cmd_array[0] = PPG_FIFO_DATA_COUNTER;
  cmd_array[1] = READMASTER;
  txLen=3;
  rxLen=3;
  spiReadWritePPG(cmd_array, txLen, sampleCnt, rxLen);
    
  // Reading the PPG samples
  cmd_array[0] = PPG_FIFO_DATA;

  
  spiReadWritePPG(cmd_array, txLen, read_array,sampleCnt[2]*3+2);
  for (i=0; i<sampleCnt[2]; i++){
    for (j=0; j <= 9; j=j+3){
      tag = (read_array[i*12+j+2] & 0xF8) >>3;
      switch(tag){
        case PPG1_LEDC1_DATA: // IR 1
          led1A[i] = ((read_array[i*12+j+2]<<16) | (read_array[i*12+j+1+2]<<8)
           | (read_array[i*12+j+2+2])) & 0x7ffff;
          break;
        case PPG1_LEDC2_DATA: // Green 1
          led2A[i] = ((read_array[i*12+j+2]<<16) | (read_array[i*12+j+1+2]<<8)
           | (read_array[i*12+j+2+2])) & 0x7ffff;
          break;
        case PPG2_LEDC1_DATA: // IR 2
          led1B[i] = ((read_array[i*12+j+2]<<16) | (read_array[i*12+j+1+2]<<8)
           | (read_array[i*12+j+2+2])) & 0x7ffff;
          break;
        case PPG2_LEDC2_DATA: // Green 2
          led2B[i] = ((read_array[i*12+j+2]<<16) | (read_array[i*12+j+1+2]<<8)
           | (read_array[i*12+j+2+2])) & 0x7ffff;
          break;
      }
    }
    k = i;
    k = 0;
    tag1A = (read_array[k*12+0+2] & 0xF8) >>3;
    tag1B = (read_array[k*12+3+2] & 0xF8) >>3;
    tag2A = (read_array[k*12+6+2] & 0xF8) >>3;
    tag2B = (read_array[k*12+9+2] & 0xF8) >>3;
  }

  if(counterCheck ==0){
    runningMeanCh1a=0.0f;
    runningMeanCh1b=0.0f;
    runningMeanCh2a=0.0f;
    runningMeanCh2b=0.0f;
    runningMeanCh1aFil= 0.0f;
    runningMeanCh1bFil =0.0f;
    runningMeanCh2aFil =0.0f;
    runningMeanCh2bFil =0.0f;
    runningSquaredMeanCh1aFil= 0.0f;
    runningSquaredMeanCh1bFil =0.0f;
    runningSquaredMeanCh2aFil =0.0f;
    runningSquaredMeanCh2bFil =0.0f;
  }
  runningMeanCh1a = runningMeanCh1a +led1A[0]*1.0f/timeWindow;
  runningMeanCh1b = runningMeanCh1b +led1B[0]*1.0f/timeWindow;
  runningMeanCh2a = runningMeanCh2a +led2A[0]*1.0f/timeWindow;
  runningMeanCh2b = runningMeanCh2b +led2B[0]*1.0f/timeWindow;
  /*	
  buff_val_raw.integer = led1A[0];
  blePktPPG_noFilter[0] = (|((buff_val_raw.intcast[1])&0xF8)>>3;
  blePktPPG_noFilter[1] = ((buff_val_raw.intcast[1]&0x07)<<5)|((buff_val_raw.intcast[0]&0xF8)>>3)
  blePktPPG_noFilter[2] =  (buff_val_raw.intcast[0]&0x07)<<5;
        
  buff_val_raw.integer = led1B[0];
  blePktPPG_noFilter[2] = blePktPPG_noFilter[2] | ((buff_val_raw.intcast[2]&0x07)<<2) |((buff_val_raw.intcast[1]&0xC0)>>6);
  blePktPPG_noFilter[3] = ((buff_val_raw.intcast[1]&0x3F)<<2) | ((buff_val_raw.intcast[0]&0xC0) >>6);
  blePktPPG_noFilter[4] = ((buff_val_raw.intcast[0]&0x3F) <<2) ;
        
  buff_val_raw.integer = led2A[0];
  blePktPPG_noFilter[4] = blePktPPG_noFilter[4] | ((buff_val_raw.intcast[2]&0x06)>>1);
  blePktPPG_noFilter[5] = ((buff_val_raw.intcast[2]&0x01)<<7) | ((buff_val_raw.intcast[1]&0xFE)>>1);
  blePktPPG_noFilter[6] = ((buff_val_raw.intcast[1]&0x01)<<7) | ((buff_val_raw.intcast[0]&0xFE)>>1);
  blePktPPG_noFilter[7] = ((buff_val_raw.intcast[0]&0x01)<<7);
        
  buff_val_raw.integer = led2B[0];
  blePktPPG_noFilter[7] = blePktPPG_noFilter[7] | ((buff_val_raw.intcast[2]&0x07) <<4)|((buff_val_raw.intcast[1]&0xF0) >>4);
  blePktPPG_noFilter[8] = ((buff_val_raw.intcast[1]&0x0F) <<4) | ((buff_val_raw.intcast[0]&0xF0)>>4);
  blePktPPG_noFilter[9] =  (buff_val_raw.intcast[0]&0x0F) <<4;
  */
  //blePktPPG_noFilter[10] = (pktCounter&0xFF00) >> 8;
  //blePktPPG_noFilter[11] = (pktCounter&0x00FF);
   
  // Transmitting the un-filtered data on BLE 
    
    //We are sending 18 bit values here that come from 24 bit values (we just chop off 1 from LSB)
    // additionally, these 24 bit values have five zeros, so we treat them as 19 bit values
    buff_val_raw.integer = led2A[0];
    // grab the first 3 bits (because 5 bits are 0s that we don't want)
    bleSendArr[12] = (buff_val_raw.intcast[2]&0x07)<<5;
    //put in the remaining 5 bits
    bleSendArr[12] = bleSendArr[12] | ((buff_val_raw.intcast[1])&0xF8); // 0xF8 = 11111000


    //continue this pattern for the 2nd byte
    
    bleSendArr[13] = ((buff_val_raw.intcast[1]&0x07)<<5)|((buff_val_raw.intcast[0]&0xF8)>>3);

    //continue the pattern, but because we have reached the end of the 24 bit number, we need to shorten
    //this from 19 bits to 18 bits, so we cut off 1
    bleSendArr[14] = ((buff_val_raw.intcast[0]&0x06) << 5); //0x06 = 00000110
    
    buff_val_raw.integer = led2B[0];
    // again, we need to cut off 5 bits
    // this means there are only 3 useable bits in intcast[2]
    // in BleSendArr[14] only 6  bits remain
    
    bleSendArr[14] = bleSendArr[14] | ((buff_val_raw.intcast[2]&0x07)<<3); //there has to be 2 zeros, and we have 5 already, so we only need to shift by 3
    // now only 3 bits remain in BleSendArr[13]
    //place last 3 bits in
    bleSendArr[14] = bleSendArr[14] | ((buff_val_raw.intcast[1]&0x0E)>>5);
    
    //place remaining 5 bits of intcast[1]
    bleSendArr[15] = (buff_val_raw.intcast[1] &0x1F) << 3;
    //place 3 bits of incast[0]
    bleSendArr[15] = bleSendArr[15] | ((buff_val_raw.intcast[0]&0xE0)>>5); //E0 = 11100000

    //we have to again cut off 1 bit. so we will only be placing 4 total bits
    bleSendArr[16] = (buff_val_raw.intcast[0] & 0x1E) << 3; // 1E = 00011110


    buff_val_raw.integer = (led1A[0] + led2A[0]) / 2;


    //we need to place 4 bits in bleSendArr[16] and there are 3 useable bits in
    // buff_val_raw.intcast[2]. So, :( we have to place 1 bit of
    bleSendArr[16] = bleSendArr[16] | ((buff_val_raw.intcast[2] & 0x07) << 1);

    //place the final bit
    bleSendArr[16] = bleSendArr[16] | ((buff_val_raw.intcast[1] & 0x80) >> 7); //0x80 = 10000000


    bleSendArr[17] = (buff_val_raw.intcast[1] &  0x3F) << 1; //3F = 0111111

    bleSendArr[17] = bleSendArr[17] | ((buff_val_raw.intcast[0] & 0x80) >> 7);

    bleSendArr[18] = (buff_val_raw.intcast[0] & 0x3F) << 1;

    // now we write the packet counter
    
  
    ppg_led_currentUpdate();

}

