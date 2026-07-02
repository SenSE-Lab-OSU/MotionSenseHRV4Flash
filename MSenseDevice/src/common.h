// common.h file
#ifndef COMMON_H_
#define COMMON_H_

#define ble_ppg_noFilter_byteLength 12
#define ble_ppg_Filter_byteLength 18
#define ble_motionPktLength 20

extern uint32_t timeWindow;


typedef union{ 
    float float_val;
    uint8_t floatcast[4];
}float_cast;
typedef union{ 
    int16_t int_val;
    uint8_t int16_cast[2];
}int16_cast;
typedef union{ 
    uint32_t integer;
    uint8_t intcast[4];
}uint32_cast;

 // common.h code goes here
extern uint32_t global_counter;
extern uint8_t movingFlag;
extern uint8_t blePktPPG_noFilter[ble_ppg_noFilter_byteLength];
extern uint8_t blePktPPG_Filter[ble_ppg_Filter_byteLength];
extern uint8_t blePktMotion[ble_motionPktLength];



#endif



