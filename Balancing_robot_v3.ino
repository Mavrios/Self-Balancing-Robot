                    /* Including Library */
#include "Balancing_robot_v3.h"

///////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
//Setup basic functions
///////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
void setup(){
#ifdef DEBUG
  Serial.begin(115200);                                                       //Start the serial port at 9600 kbps
#endif
  Wire.begin();                                                             //Start the I2C bus as master
  TWBR = 12;                                                                //Set the I2C clock speed to 400kHz

  //To create a variable pulse for controlling the stepper motors a timer is created that will execute a piece of code (subroutine) every 20us
  //This subroutine is called TIMER2_COMPA_vect
  TCCR2A = 0;                                                               //Make sure that the TCCR2A register is set to zero
  TCCR2B = 0;                                                               //Make sure that the TCCR2A register is set to zero
  TIMSK2 |= (1 << OCIE2A);                                                  //Set the interupt enable bit OCIE2A in the TIMSK2 register
  TCCR2B |= (1 << CS21);                                                    //Set the CS21 bit in the TCCRB register to set the prescaler to 8
  OCR2A = 39;                                                               //The compare register is set to 39 => 20us / (1s / (16.000.000MHz / 8)) - 1
  TCCR2A |= (1 << WGM21);                                                   //Set counter 2 to CTC (clear timer on compare) mode

  setup_mpu_6050_registers();                                               //Setup the registers of the MPU-6050 (+/- 250dps \\ +/- 4g) and start the gyro

  radio.begin();
  radio.setRetries(1,1);
  radio.setPayloadSize(sizeof(DATA_PACKAGE));
  radio.setDataRate(RF24_1MBPS);
  radio.setPALevel(RF24_PA_MAX);
  radio.openReadingPipe(1,Pipe);
  radio.startListening(); 
  
  pinMode(2, OUTPUT);                                                       //Configure digital poort 2 as output
  pinMode(3, OUTPUT);                                                       //Configure digital poort 3 as output
  pinMode(4, OUTPUT);                                                       //Configure digital poort 4 as output
  pinMode(5, OUTPUT);                                                       //Configure digital poort 5 as output
  //pinMode(13, OUTPUT);                                                    //Configure digital poort 13 as output
  pinMode(M_ENABLE, OUTPUT);
  
  SET_BIT(DDRD,trig1_PD);                                                   //Sets Trig_1 as OUTPUT
  SET_BIT(DDRD,trig2_PD);                                                   //Sets Trig_2 as OUTPUT
  SET_BIT(DDRB,servo1_PB);                                                   //Sets Servo1 as OUTPUT
  SET_BIT(DDRB,servo2_PB);                                                  //Sets Servo2 as OUTPUT

  MicroTimer=micros();                                                      //Start Micro Timer
  digitalWrite(M_ENABLE, !STATE);
    
  for(receive_timer = 0; receive_timer < 500; receive_timer++){             //Create 500 loops
    //if(receive_timer % 15 == 0)digitalWrite(13, !digitalRead(13));          //Change the state of the LED every 15 loops to make the LED blink fast
    Wire.beginTransmission(gyro_address);                                   //Start communication with the gyro
    Wire.write(0x43);                                                       //Start reading the Who_am_I register 75h
    Wire.endTransmission();                                                 //End the transmission
    Wire.requestFrom(gyro_address, 4);                                      //Request 2 bytes from the gyro
    gyro_yaw_calibration_value += Wire.read()<<8|Wire.read();               //Combine the two bytes to make one integer
    gyro_pitch_calibration_value += Wire.read()<<8|Wire.read();             //Combine the two bytes to make one integer
    delayMicroseconds(3700);                                                //Wait for 3700 microseconds to simulate the main program loop time
  }
  gyro_pitch_calibration_value /= 500;                                      //Divide the total value by 500 to get the avarage gyro offset
  gyro_yaw_calibration_value /= 500;                                        //Divide the total value by 500 to get the avarage gyro offset

  loop_timer = micros() + 4000;                                             //Set the loop_timer variable at the next end loop time

}

///////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
//Main program loop
///////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
void loop(){
  #ifdef RC
  if(radio.available()){                                                   //If there is data available
    received_byte = receiving_data();                                          //Load the received data in the received_byte variable
    receive_counter = 0;                                                    //Reset the receive_counter variable
  }
  if(receive_counter <= 25)receive_counter ++;                              //The received byte will be valid for 25 program loops (100 milliseconds)
  else{
    received_byte = 0x00;                                                //After 100 milliseconds the received byte is deleted
  #ifdef DEBUG
    Serial.println("faild");
  #endif
  }
  #endif
  
  #ifdef BATTARY
  //Load the battery voltage to the battery_voltage variable.
  //85 is the voltage compensation for the diode.
  //Resistor voltage divider => (3.3k + 3.3k)/2.2k = 2.5
  //12.5V equals ~5V @ Analog 0.
  //12.5V equals 1023 analogRead(0).
  //(12.5*100) / 1023 = 1.222.
  //The variable battery_voltage holds 1050 if the battery voltage is 10.5V.
  battery_voltage = (analogRead(0) * 1.222) + 85;
  
  if(battery_voltage < 1050 && battery_voltage > 800){                      //If batteryvoltage is below 10.5V and higher than 8.0V
    digitalWrite(13, HIGH);                                                 //Turn on the led if battery voltage is to low
    low_bat = 1;                                                            //Set the low_bat variable to 1
  }
  #endif

  ///////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
  //Angle calculations
  ///////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
  Wire.beginTransmission(gyro_address);                                     //Start communication with the gyro
  Wire.write(0x3F);                                                         //Start reading at register 3F
  Wire.endTransmission();                                                   //End the transmission
  Wire.requestFrom(gyro_address, 2);                                        //Request 2 bytes from the gyro
  accelerometer_data_raw = Wire.read()<<8|Wire.read();                      //Combine the two bytes to make one integer
  accelerometer_data_raw += acc_calibration_value;                          //Add the accelerometer calibration value
  if(accelerometer_data_raw > 8200)accelerometer_data_raw = 8200;           //Prevent division by zero by limiting the acc data to +/-8200;
  if(accelerometer_data_raw < -8200)accelerometer_data_raw = -8200;         //Prevent division by zero by limiting the acc data to +/-8200;
  //57.296 => 1 / (3.142 / 180) The Arduino asin function is in radians
  angle_acc = asin((float)accelerometer_data_raw/8200.0)* 57.296;           //Calculate the current angle according to the accelerometer

  if(start == 0 && angle_acc > -0.5&& angle_acc < 0.5){                     //If the accelerometer angle is almost 0
    angle_gyro = angle_acc;                                                 //Load the accelerometer angle in the angle_gyro variable
    start = 1;                                                              //Set the start variable to start the PID controller
    digitalWrite(M_ENABLE, STATE);
  }
  
  Wire.beginTransmission(gyro_address);                                     //Start communication with the gyro
  Wire.write(0x43);                                                         //Start reading at register 43
  Wire.endTransmission();                                                   //End the transmission
  Wire.requestFrom(gyro_address, 4);                                        //Request 4 bytes from the gyro
  gyro_yaw_data_raw = Wire.read()<<8|Wire.read();                           //Combine the two bytes to make one integer
  gyro_pitch_data_raw = Wire.read()<<8|Wire.read();                         //Combine the two bytes to make one integer
  
  gyro_pitch_data_raw -= gyro_pitch_calibration_value;                      //Add the gyro calibration value
  angle_gyro += gyro_pitch_data_raw * 0.000031;                             //Calculate the traveled during this loop angle and add this to the angle_gyro variable
  //0.000031 => (1/(250Hz*131))
  
  ///////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
  //MPU-6050 offset compensation
  ///////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
  //Not every gyro is mounted 100% level with the axis of the robot. This can be cause by misalignments during manufacturing of the breakout board. 
  //As a result the robot will not rotate at the exact same spot and start to make larger and larger circles.
  //To compensate for this behavior a VERY SMALL angle compensation is needed when the robot is rotating.
  //Try 0.0000003 or -0.0000003 first to see if there is any improvement.

  gyro_yaw_data_raw -= gyro_yaw_calibration_value;                          //Add the gyro calibration value
  //Uncomment the following line to make the compensation active
  //angle_gyro -= gyro_yaw_data_raw * 0.0000003;                            //Compensate the gyro offset when the robot is rotating
#ifdef NEW_FILTER
  angle_gyro = angle_gyro * 0.98 + angle_acc * 0.02;                    //Correct the drift of the gyro angle with the accelerometer angle
#else
  angle_gyro = angle_gyro * 0.9996 + angle_acc * 0.0004;                    //Correct the drift of the gyro angle with the accelerometer angle
#endif

  ///////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
  //                                                        ULTRASONIC SETUP                                                           //
  ///////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
  /*Clears The trig*/
  SET_BIT(PORTD,trig1_PD);                                      // Sets Trig_1 LOW
  SET_BIT(PORTD,trig2_PD);                                      // Sets Trig_2 LOW
  delayMicroseconds(2);                                         //delay 2 us

 /*Sets trig HIGH for 10us*/
  SET_BIT(PORTD,trig1_PD);                                       //Set Trig_1 High
  SET_BIT(PORTD,trig2_PD);                                       //Set Trig_2 High  
  delayMicroseconds(10);                                         //delay 10us
  CLR_BIT(PORTD,trig1_PD);                                       //Set Trig_1 LOW
  CLR_BIT(PORTD,trig2_PD);                                       //Set Trig_2 LOW
  
  /*Reads the return sound wave in us*/
  duration_1 = pulseIn(echo1_PD,HIGH);                           //Calculate The Time of travled sound wave in us
  duration_2 = pulseIn(echo2_PD,HIGH);                           //Calculate The Time of travled sound wave in us
  distance_1 = duration_1 * Sound_Speed;                         //Calculate Distance (0.034)--> Sound speed in cm/us , dividing by 2 to calculate backward time only
  distance_2 = duration_2 * Sound_Speed;                         //Calculate Distance (0.034)--> Sound speed in cm/us , dividing by 2 to calculate backward time only

  ///////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
  //PID controller calculations
  ///////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
  //The balancing robot is angle driven. First the difference between the desired angel (setpoint) and actual angle (process value)
  //is calculated. The self_balance_pid_setpoint variable is automatically changed to make sure that the robot stays balanced all the time.
  //The (pid_setpoint - pid_output * 0.015) part functions as a brake function.
  pid_error_temp = angle_gyro - self_balance_pid_setpoint - pid_setpoint;
  if(pid_output > 10 || pid_output < -10)pid_error_temp += pid_output * 0.015 ;

  pid_i_mem += pid_i_gain * pid_error_temp;                                 //Calculate the I-controller value and add it to the pid_i_mem variable
  if(pid_i_mem > 400)pid_i_mem = 400;                                       //Limit the I-controller to the maximum controller output
  else if(pid_i_mem < -400)pid_i_mem = -400;
  //Calculate the PID output value
  pid_output = pid_p_gain * pid_error_temp + pid_i_mem + pid_d_gain * (pid_error_temp - pid_last_d_error);
  if(pid_output > 400)pid_output = 400;                                     //Limit the PI-controller to the maximum controller output
  else if(pid_output < -400)pid_output = -400;

  pid_last_d_error = pid_error_temp;                                        //Store the error for the next loop

  if(pid_output < 7 && pid_output > -7)pid_output = 0;                      //Create a dead-band to stop the motors when the robot is balanced

  if(angle_gyro > 30 || angle_gyro < -30 || start == 0 || low_bat == 1){    //If the robot tips over or the start variable is zero or the battery is empty
    pid_output = 0;                                                         //Set the PID controller output to 0 so the motors stop moving
    pid_i_mem = 0;                                                          //Reset the I-controller memory
    start = 0;                                                              //Set the start variable to 0
    self_balance_pid_setpoint = 0;                                          //Reset the self_balance_pid_setpoint variable
    digitalWrite(M_ENABLE, !STATE);

    /*                                       STARTING MECHANICSM CONTROL                                                  */
    SET_BIT(PORTB,servo1_PB);
    SET_BIT(PORTB,servo2_PB);
    delayMicroseconds(Servo_Degree_us);
    CLR_BIT(PORTB,servo1_PB);
    CLR_BIT(PORTB,servo2_PB);
    delayMicroseconds(Servo_Freq_us-Servo_Degree_us);
      
    for(u8 i = 0;i < Stepper_Steps ;i++){
      SET_BIT(PORTB,Step1_PB);
      SET_BIT(PORTB,Step2_PB);
      delayMicroseconds(500);
      CLR_BIT(PORTB,Step1_PB);
      CLR_BIT(PORTB,Step2_PB);

    }
  }
  ///////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
  //                                                        AVOIDING CONTROL                                                           //
  ///////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
  /*Check the forward Ultrasnonic distance Safe Distance*/
  if(distance_1>Safe_Distance){
    #ifdef  LED
      CLR_BIT(PORTC,LED);                                      //Turn off The Warning LED
    #endif
    Danger_Flag_1=0;                                           //Danger_Flag_1 To allow the remote to Go forward
  }
   /*Check the forward Ultrasnonic distance*/
  else if(distance_1<Safe_Distance)
  {
    if(pid_setpoint > 1)pid_setpoint -=0.05;
    else pid_setpoint = 0.5 ;
    /*Check Danger Zone (10 CM)*/
    if(distance_1<Danger_Distance)
    {
      pid_setpoint = 0 ;
      Danger_Flag_1 = 1; 
    
      #ifdef  LED
         SET_BIT(PORTC,LED);                                   //Set LED ON For Warrning 
      #endif
    }
  }
  /*Check the backward Ultrasnonic distance Safe Distance*/
  if(distance_2>Safe_Distance){
    #ifdef  LED
      CLR_BIT(PORTC,LED);                                      //Turn off The Warning LED
    #endif
    Danger_Flag_2=0;                                           //Danger_Flag_2 To allow the remote to Go backward
  }
 
  /*Check the backward Ultrasnonic distance*/
  else if(distance_2<Safe_Distance)
  {
    if(pid_setpoint < -1)pid_setpoint +=0.05;  
    else pid_setpoint = 0.5 ; 
    /*Check Danger Zone (10 CM)*/
    if(distance_1<Danger_Distance)
    {
      pid_setpoint = 0 ;                                          //STOP THE ROBOT
      Danger_Flag_2 = 1;

      #ifdef  LED
         SET_BIT(PORTC,LED);                                       //Set LED ON For Warrning 
      #endif
    }
  }
  

  ///////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
  //Control calculations
  ///////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
  pid_output_left = pid_output;                                             //Copy the controller output to the pid_output_left variable for the left motor
  pid_output_right = pid_output;                                            //Copy the controller output to the pid_output_right variable for the right motor

  if(received_byte & B00000001){                                            //If the first bit of the receive byte is set change the left and right variable to turn the robot to the left
    pid_output_left += turning_speed;                                       //Increase the left motor speed
    pid_output_right -= turning_speed;                                      //Decrease the right motor speed
  }
  if(received_byte & B00000010){                                            //If the second bit of the receive byte is set change the left and right variable to turn the robot to the right
    pid_output_left -= turning_speed;                                       //Decrease the left motor speed
    pid_output_right += turning_speed;                                      //Increase the right motor speed
  }

  if((received_byte & B00000100) && !Danger_Flag_1){                         //If the third bit of the receive byte is set change the left and right variable to turn the robot to the right
    if(pid_setpoint > -2.5)pid_setpoint -= 0.05;                            //Slowly change the setpoint angle so the robot starts leaning forewards
    if(pid_output > max_target_speed * -1)pid_setpoint -= 0.005;            //Slowly change the setpoint angle so the robot starts leaning forewards
  }
  if((received_byte & B00001000)&& !Danger_Flag_2){                         //If the forth bit of the receive byte is set change the left and right variable to turn the robot to the right
    if(pid_setpoint < 2.5)pid_setpoint += 0.05;                             //Slowly change the setpoint angle so the robot starts leaning backwards
    if(pid_output < max_target_speed)pid_setpoint += 0.005;                 //Slowly change the setpoint angle so the robot starts leaning backwards
  }   

  if(!(received_byte & B00001100)){                                         //Slowly reduce the setpoint to zero if no foreward or backward command is given
    if(pid_setpoint > 0.5)pid_setpoint -=0.05;                              //If the PID setpoint is larger then 0.5 reduce the setpoint with 0.05 every loop
    else if(pid_setpoint < -0.5)pid_setpoint +=0.05;                        //If the PID setpoint is smaller then -0.5 increase the setpoint with 0.05 every loop
    else pid_setpoint = 0;                                                  //If the PID setpoint is smaller then 0.5 or larger then -0.5 set the setpoint to 0
  }
  
  //The self balancing point is adjusted when there is not forward or backwards movement from the transmitter. This way the robot will always find it's balancing point
  if(pid_setpoint == 0){                                                    //If the setpoint is zero degrees
    if(pid_output < 0)self_balance_pid_setpoint += 0.0015;                  //Increase the self_balance_pid_setpoint if the robot is still moving forewards
    if(pid_output > 0)self_balance_pid_setpoint -= 0.0015;                  //Decrease the self_balance_pid_setpoint if the robot is still moving backwards
  }

  ///////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
  //Motor pulse calculations
  ///////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
  //To compensate for the non-linear behaviour of the stepper motors the folowing calculations are needed to get a linear speed behaviour.
  if(pid_output_left > 0)pid_output_left = 400 - (1/(pid_output_left + 9)) * 6500;
  else if(pid_output_left < 0)pid_output_left = -400 - (1/(pid_output_left - 9)) * 6500;

  if(pid_output_right > 0)pid_output_right = 400 - (1/(pid_output_right + 9)) * 6500;
  else if(pid_output_right < 0)pid_output_right = -400 - (1/(pid_output_right - 9)) * 6500;

  //Calculate the needed pulse time for the left and right stepper motor controllers
  if(pid_output_left > 0)left_motor = 400 - pid_output_left;
  else if(pid_output_left < 0)left_motor = -400 - pid_output_left;
  else left_motor = 0;

  if(pid_output_right > 0)right_motor = 400 - pid_output_right;
  else if(pid_output_right < 0)right_motor = -400 - pid_output_right;
  else right_motor = 0;

  //Copy the pulse time to the throttle variables so the interrupt subroutine can use them
  throttle_left_motor = left_motor;
  throttle_right_motor = right_motor;

  ///////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
  //Loop time timer
  ///////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
  //The angle calculations are tuned for a loop time of 4 milliseconds. To make sure every loop is exactly 4 milliseconds a wait loop
  //is created by setting the loop_timer variable to +4000 microseconds every loop.
  while(loop_timer > micros());
  loop_timer += 4000;
}

///////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
//Interrupt routine  TIMER2_COMPA_vect
///////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
ISR(TIMER2_COMPA_vect){
  //Left motor pulse calculations
  throttle_counter_left_motor ++;                                           //Increase the throttle_counter_left_motor variable by 1 every time this routine is executed
  if(throttle_counter_left_motor > throttle_left_motor_memory){             //If the number of loops is larger then the throttle_left_motor_memory variable
    throttle_counter_left_motor = 0;                                        //Reset the throttle_counter_left_motor variable
    throttle_left_motor_memory = throttle_left_motor;                       //Load the next throttle_left_motor variable
    if(throttle_left_motor_memory < 0){                                     //If the throttle_left_motor_memory is negative
  #ifdef INVERS_L_M
      PORTD &= ~MASK(2);                                                    //Set output 3 LOW to make the spinning direction of the motor in counterclockwise(FORWARD)
  #else
      PORTD |= MASK(2);
  #endif
      throttle_left_motor_memory *= -1;                                     //Invert the throttle_left_motor_memory variable
    }
  #ifdef INVERS_L_M
    else PORTD |= MASK(2);                                                  //Set output 3 HIGH to make the spinning direction of the motor in clockwise(BACKWARD)
  #else
    else PORTD &= ~MASK(2);
  #endif
  }
  else if(throttle_counter_left_motor == 1)PORTD |=  MASK(3);               //Set output 2 high to create a pulse for the stepper controller
  else if(throttle_counter_left_motor == 2)PORTD &= ~MASK(3);               //Set output 2 low because the pulse only has to last for 20us 
  
  //right motor pulse calculations
  throttle_counter_right_motor ++;                                          //Increase the throttle_counter_right_motor variable by 1 every time the routine is executed
  if(throttle_counter_right_motor > throttle_right_motor_memory){           //If the number of loops is larger then the throttle_right_motor_memory variable
    throttle_counter_right_motor = 0;                                       //Reset the throttle_counter_right_motor variable
    throttle_right_motor_memory = throttle_right_motor;                     //Load the next throttle_right_motor variable
    if(throttle_right_motor_memory < 0){                                    //If the throttle_right_motor_memory is negative
  #ifdef INVERS_R_M
      PORTD &= ~MASK(5);                                                    //Set output 5 LOW to make the spinning direction of the motor in counterclockwise(FORWARD)
  #else
      PORTD |= MASK(5);
  #endif     
      throttle_right_motor_memory *= -1;                                    //Invert the throttle_right_motor_memory variable
    }
  #ifdef INVERS_R_M
    else PORTD |= MASK(5);                                                  //Set output 5 HIGH to make the spinning direction of the motor in clockwise(BACKWARD)
  #else
    else PORTD &= ~MASK(5);
  #endif
  }
  else if(throttle_counter_right_motor == 1)PORTD |=  MASK(4);              //Set output 4 high to create a pulse for the stepper controller
  else if(throttle_counter_right_motor == 2)PORTD &= ~MASK(4);              //Set output 4 low because the pulse only has to last for 20us
}

void setup_mpu_6050_registers(void){
  //By default the MPU-6050 sleeps. So we have to wake it up.
  Wire.beginTransmission(gyro_address);                                     //Start communication with the address found during search.
  Wire.write(0x6B);                                                         //We want to write to the PWR_MGMT_1 register (6B hex)
  Wire.write(0x00);                                                         //Set the register bits as 00000000 to activate the gyro
  Wire.endTransmission();                                                   //End the transmission with the gyro.
  //Set the full scale of the gyro to +/- 250 degrees per second
  Wire.beginTransmission(gyro_address);                                     //Start communication with the address found during search.
  Wire.write(0x1B);                                                         //We want to write to the GYRO_CONFIG register (1B hex)
  Wire.write(0x00);                                                         //Set the register bits as 00000000 (250dps full scale)
  Wire.endTransmission();                                                   //End the transmission with the gyro
  //Set the full scale of the accelerometer to +/- 4g.
  Wire.beginTransmission(gyro_address);                                     //Start communication with the address found during search.
  Wire.write(0x1C);                                                         //We want to write to the ACCEL_CONFIG register (1A hex)
  Wire.write(0x08);                                                         //Set the register bits as 00001000 (+/- 4g full scale range)
  Wire.endTransmission();                                                   //End the transmission with the gyro
  //Set some filtering to improve the raw data.
  Wire.beginTransmission(gyro_address);                                     //Start communication with the address found during search
  Wire.write(0x1A);                                                         //We want to write to the CONFIG register (1A hex)
  Wire.write(0x03);                                                         //Set the register bits as 00000011 (Set Digital Low Pass Filter to ~43Hz)
  Wire.endTransmission();                                                   //End the transmission with the gyro 
}

///////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
//Function to read data from decoder
///////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
byte receiving_data(void){
    bool done = false;
    while(!done){
      done = radio.read(&data , sizeof(DATA_PACKAGE));   
    }
  #ifdef DEBUG
    Serial.println("sucess");
    Serial.print("received: ");
    Serial.println(data.received_byte,BIN);
  #endif
    return data.received_byte;
}
