#ifndef SELF_BALANCING_V3_H
#define SELF_BALANCING_V3_H

                    /* Including Library */
#include <Wire.h>                                                             //Include the Wire.h library so we can communicate with the gyro
#include<SPI.h>
#include<nRF24L01.h>
#include<RF24.h>


                    /* TYPE DEFINE */
             /*u -> UNSIGNED and '8' -> 8 bit */
             /*u -> UNSIGNED and '16' -> 16 bit */
typedef unsigned char       u8;
typedef unsigned short int u16;
typedef unsigned long      u64;
typedef long               f64;

    /*Macros TO SET AND CLR AND GET BITS*/
#define SET_BIT(BYTE,No_Bit)  BYTE |=(1<<No_Bit)
#define CLR_BIT(BYTE,No_Bit)  BYTE &=~(1<<No_Bit)
#define GET_BIT(BYTE,Bit_NO) ((BYTE>>Bit_NO)&(0x01))


                    /* PREPROCESSOR */
#define servo1_PB         0                                                    //SERVO1 PIN IN PORTx (X)
#define servo2_PB         0                                                    //SERVO2 PIN IN PORTx (X)
#define Servo_Degree_us  1450                                                  //SERVO  REQUIRED DUTY CYCLE IN us
#define Servo_Freq_us    20000                                                 //SERVO  REQUIRED FREQ
#define Stepper_Steps     50                                                   //NUMBER OF REQUIRED STEPS FROM STEPPER MOTOR TO GIVE FIRST PUSH
#define Step1_PB          0                                                    //STEPPER1 MOTOR STEP PIN IN PORTx (x)
#define Step2_PB          0                                                    //STEPPER2 MOTOR STEP PIN IN PORTx (x)
#define Dir1_PB           0                                                    //STEPPER1 MOTOR DIR PIN IN PORTx (x)
#define Dir2_PB           0                                                    //STEPPER2 MOTOR DIR PIN IN PORTx (x)

//#define LED

#define trig1_PD          0                                                    //TRIG1 PIN IN PORTx (X)
#define echo1_PD          0                                                    //ECHO1 PIN IN PORTx (X)
#define trig2_PD          0                                                    //TRIG1 PIN IN PORTx (X)
#define echo2_PD          0                                                    //ECHO1 PIN IN PORTx (X)
#define LED               0                                                    //LED PIN IN PORTx (X)
#define Sound_Speed       0.034/2                                              //(0.034)--> Sound speed in cm/us , dividing by 2 to calculate backward time only

#define Safe_Distance     30                                                   //SAFE DISTANCE FOR ROBOT +30CM
#define Danger_Distance   10                                                   //DAGNER DISTANCE FOR ROBOT -10CM


#define CE_PIN    14
#define CSN_PIN   10
//#define DEBUG
//#define NEW_FILTER
//#define RC
//#define BATTARY
//#define INVERS_R_M
//#define INVERS_L_M
#define STATE     LOW
#define M_ENABLE   9


                    /* NRF */     

RF24 radio(CE_PIN,CSN_PIN);
const uint64_t Pipe = 0xE0E0F0F0FFLL; 
struct DATA_PACKAGE {
  byte received_byte;
  //bool joy_button;
};

DATA_PACKAGE data;
                    /* MACROS */            

#define MASK(x) ((unsigned char)(1 << x))



                    /* Variables */
/* 'u' refere to unsigned 'f' refere to float */
/* '16' refere to 16 bit '64' refere to 64 bit*/       
           
u64 MicroTimer = 0;
bool flag = 0 ;
bool Danger_Flag_1 = 0 , Danger_Flag_2 = 0;
u16 distance_1 = 0 ,distance_2 = 0 ;
f64 duration_1 = 0 , duration_2 = 0;



const int gyro_address = 0x68;                                       //MPU-6050 I2C address (0x68 or 0x69)
int acc_calibration_value = -275;                                    //Enter the accelerometer calibration value




//Various settings
float pid_p_gain = 30;                                       //Gain setting for the P-controller (15)
float pid_i_gain = 0.5;                                      //Gain setting for the I-controller (1.5)
float pid_d_gain = 2;                                       //Gain setting for the D-controller (30)
float turning_speed = 30;                                    //Turning speed (20)
float max_target_speed = 150;                                //Max target speed (100)
void setup_mpu_6050_registers(void);
///////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
//Declaring global variables
///////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
byte start, received_byte = 0, low_bat;

int left_motor, throttle_counter_left_motor, throttle_left_motor_memory;
int right_motor, throttle_counter_right_motor, throttle_right_motor_memory;
volatile int throttle_left_motor, throttle_right_motor;
int battery_voltage;
int gyro_pitch_data_raw, gyro_yaw_data_raw, accelerometer_data_raw;

long gyro_yaw_calibration_value, gyro_pitch_calibration_value;

unsigned long loop_timer, receive_timer, receive_counter;

float angle_gyro, angle_acc, angle, self_balance_pid_setpoint;
float pid_error_temp, pid_i_mem, pid_setpoint, gyro_input, pid_output, pid_last_d_error;
float pid_output_left, pid_output_right;


#endif
