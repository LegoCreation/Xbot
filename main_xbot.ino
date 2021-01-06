#include "se8r01.h"
#include <Servo.h>
#include <NewPing.h>
#include <Wire.h>
#define TX_ADR_WIDTH    4   
#define TX_PLOAD_WIDTH  7 
#define servo_x_pin 5
#define servo_y_pin 4
#define laser_pin 6
 
byte mode ='r'; 
unsigned char rx_buf[TX_PLOAD_WIDTH] = {0};
unsigned char tx_buf[TX_PLOAD_WIDTH] = {0};
unsigned char TX_ADDRESS[TX_ADR_WIDTH]  = 
{
  0x34,0x43,0x10,0x10
};
byte gtemp[5];


float ccx;
float ccy;
int T_val,A_val,servo_x_val,servo_y_val,roll_val,pitch_val,Tr_val;
int a=800;
int value = 600; 
int esc1,esc2,esc3,esc4;
int readystart,readystart2,readystart3;
long counter;
Servo firstESC, secondESC, thirdESC, fourthESC, servo_x, servo_y;

#define MAX_DISTANCE 200
int sensor_time_taken[4]; 
NewPing ultra_sensor[4]={
  NewPing (23,22,MAX_DISTANCE),
  NewPing (25,24,MAX_DISTANCE),
  NewPing (27,26,MAX_DISTANCE),
  NewPing (29,28,MAX_DISTANCE),
  
};
 
int gyro_roll, gyro_pitch, gyro_yaw;
long acc_x, acc_y, acc_z, acc_total_vector;
int temperature;
long gyro_roll_cal, gyro_pitch_cal, gyro_yaw_cal;
long loop_timer;
float angle_pitch, angle_roll;
int angle_pitch_buffer, angle_roll_buffer;
boolean set_gyro_angles;
float angle_roll_acc, angle_pitch_acc;
float angle_pitch_output, angle_roll_output;

float pid_p_gain_roll = 0.15;               
float pid_i_gain_roll = 0.009;            
float pid_d_gain_roll = 0.07;                
int pid_max_roll = 200;                   

float pid_p_gain_pitch = pid_p_gain_roll;  
float pid_i_gain_pitch = pid_i_gain_roll;  
float pid_d_gain_pitch = pid_d_gain_roll; 
int pid_max_pitch = pid_max_roll;         

float pid_p_gain_yaw = 4.0;             
float pid_i_gain_yaw = 0.02;              
float pid_d_gain_yaw = 0.0;              
int pid_max_yaw = 400;    
float pid_error_temp;
float pid_i_mem_roll, pid_roll_setpoint, gyro_roll_input, pid_output_roll, pid_last_roll_d_error;
float pid_i_mem_pitch, pid_pitch_setpoint, gyro_pitch_input, pid_output_pitch, pid_last_pitch_d_error;
float pid_i_mem_yaw, pid_yaw_setpoint, gyro_yaw_input, pid_output_yaw, pid_last_yaw_d_error;

int sensor_count,sensor_setpoint,sensor1_output,sensor2_output,sensor3_output,sensor4_output;
int any2;
int any1;
int cal_int;



float sensor1, sensor2, sensor3, sensor4;

byte highByte,lowByte;

void setup() {
    Serial.begin(115200);   
   Wire.begin();

   Serial.println("Wait for 5 second...");
   

gyro_setup();
delay(250);
wifi_setup();
servo_setup();
esc_setup(); 

}





void esc_setup(){
 firstESC.attach(9);
 secondESC.attach(10);
 thirdESC.attach(11);
 fourthESC.attach(12);

   firstESC.writeMicroseconds(0);
   secondESC.writeMicroseconds(0);
   thirdESC.writeMicroseconds(0);
   fourthESC.writeMicroseconds(0); 
}

void servo_setup(){
 servo_x.attach(servo_x_pin);
 servo_y.attach(servo_y_pin);
pinMode(laser_pin,OUTPUT);
analogWrite(laser_pin,0);
}


void wifi_setup(){
  pinMode(CEq,  OUTPUT);
  pinMode(SCKq, OUTPUT);
  pinMode(CSNq, OUTPUT);
  pinMode(MOSIq,  OUTPUT);
  pinMode(MISOq, INPUT);
  pinMode(IRQq, INPUT);


  init_io();                        
  unsigned char status=SPI_Read(STATUS);


 digitalWrite(CEq, 0);
  delay(1);
se8r01_powerup();
se8r01_calibration();
se8r01_setup();
wifi_settings();
 if (mode=='r')
 {
 SPI_RW_Reg(WRITE_REG|iRF_BANK0_CONFIG, 0x3f); 
 }
 else
 {
 SPI_RW_Reg(WRITE_REG|iRF_BANK0_CONFIG, 0x3E); 
 }

digitalWrite(CEq, 1);

}

void gyro_setup() {
Wire.beginTransmission(0x68);                                      
  Wire.write(0x6B);                                                
  Wire.write(0x00);                                           
  Wire.endTransmission();                                        
  //Configure the accelerometer (+/-8g)
  Wire.beginTransmission(0x68);                                       
  Wire.write(0x1C);                                           
  Wire.write(0x10);                                              
  Wire.endTransmission();                                           
  //Configure the gyro (500dps full scale)
  Wire.beginTransmission(0x68);                                        
  Wire.write(0x1B);                                                
  Wire.write(0x08);                                                 
  Wire.endTransmission();   
  for (int cal_int = 0; cal_int < 2000 ; cal_int ++){                
    if(cal_int % 125 == 0)                            
    read_mpu_6050_data();                                             
    gyro_roll_cal += gyro_roll;                                              
    gyro_pitch_cal += gyro_pitch;                                         
    gyro_yaw_cal += gyro_yaw;                                          
    delay(3); 
   if(cal_int % 200==0) Serial.print(" . ");//Delay 3us to simulate the 250Hz program loop
  }
  Serial.println("");
  gyro_roll_cal /= 2000;                                                
  gyro_pitch_cal /= 2000;                                                  
  gyro_yaw_cal /= 2000;                                                


  loop_timer = micros(); 
}


void loop() {
counter += 1; 
if (counter>1) readystart2 = 1;

   
gyro_loop();
 gyro_roll_input = gyro_roll;           
  gyro_pitch_input = gyro_pitch;         
  gyro_yaw_input = gyro_yaw;             

  if (counter=2) {
pid_i_mem_roll = 0;
    pid_last_roll_d_error = 0;
    pid_i_mem_pitch = 0;
    pid_last_pitch_d_error = 0;
    pid_i_mem_yaw = 0;
    pid_last_yaw_d_error = 0;
  }
wifi_loop();
if(A_val==245) readystart=1;
esc_loop();
if (counter%3000 == 0)
readystart=0;
if(A_val>15){
//sensor_loop();
//sensor_process();
}
servo_loop();


print_output();
}


void read_mpu_6050_data(){                                            
  Wire.beginTransmission(0x68);                                     
  Wire.write(0x3B);                                                
  Wire.endTransmission();                                         
  Wire.requestFrom(0x68,14);                                         
  while(Wire.available() < 14);                                       
  acc_x = Wire.read()<<8|Wire.read();                                 
  acc_y = Wire.read()<<8|Wire.read();                              
  acc_z = Wire.read()<<8|Wire.read();                              
  temperature = Wire.read()<<8|Wire.read();                            
  gyro_roll = Wire.read()<<8|Wire.read();                               
  gyro_pitch = Wire.read()<<8|Wire.read();                                
  gyro_yaw = Wire.read()<<8|Wire.read();                                

}


void sensor_loop(){
 

  for (int i=0; i<4;i++){
  sensor_time_taken[i] = ultra_sensor[i].ping();
  Serial.print("Ping: ");
  Serial.print(sensor_time_taken[i] / 57);
  Serial.print("cm ");
  if ((sensor_time_taken[i]/57)==0) {
   pinMode(22+(2*i), OUTPUT);
 delayMicroseconds(1);
 digitalWrite(22+(2*i), LOW);
 delayMicroseconds(1);
  pinMode(22+(2*i), INPUT);
 delayMicroseconds(1); 
  }
  }
  Serial.println("");

  sensor1=sensor_time_taken[0] / 57;
  sensor2=sensor_time_taken[1] / 57;
  sensor3=sensor_time_taken[2] / 57;
  sensor4=sensor_time_taken[3] / 57;


}
float sensor_setpoint2,sensor_setpoint1,sensor_setpoint3,sensor_setpoint4;

void sensor_process(){

  int distance_limit=30;
  sensor_setpoint1=2000/sensor1;
  sensor_setpoint2=2000/sensor2;
  sensor_setpoint3=2000/sensor3;
  sensor_setpoint4=2000/sensor4;
  sensor_count++;
 if(sensor1<=distance_limit && sensor1!=0) sensor1_output=sensor_setpoint1;
 if(sensor2<=distance_limit && sensor2!=0) sensor2_output=sensor_setpoint2;
 if(sensor3<=distance_limit && sensor3!=0) sensor3_output=sensor_setpoint3;
 if(sensor4<=distance_limit && sensor4!=0) sensor4_output=sensor_setpoint4;
 if((sensor1<=distance_limit) && (sensor1!=0) &&(sensor3<=distance_limit) && (sensor3!=0)){
  if(sensor_count % 2==0){
    sensor2_output=100; 
    sensor1_output=100;
    sensor3_output=100;
    any1=1;
    
  }
  else {
    sensor4_output=100; 
     sensor1_output=100;
    sensor3_output=100;
    any1=2;
  }
 }
  if((sensor2<=distance_limit)&&(sensor2!=0) &&(sensor4<=distance_limit)&&(sensor4!=0)){
 if(sensor_count % 2==0){
    sensor1_output=100; 
     sensor2_output=100;
    sensor4_output=100;
    any2=1;
  }
  else {
    sensor3_output=100;
       sensor2_output=100;
    sensor4_output=100; 
    any2=2;
  }
  }
 if((sensor_count % 5==0) && (sensor1>=distance_limit)) sensor1_output=0;
 if((sensor_count % 5==0) && (sensor2>=distance_limit)) sensor2_output=0;
 if((sensor_count % 5==0) && (sensor3>=distance_limit)) sensor3_output=0;
 if((sensor_count % 5==0) && (sensor4>=distance_limit)) sensor4_output=0;
 if((sensor_count % 5==0) && (sensor1>=distance_limit) &&(sensor3>distance_limit)){
 if(any1==1){
    sensor2_output=0; 
     any1=0;
    
  }
  else if(any1==2) {
    sensor4_output=0; 
     any1=0;
   
  }
 }
  if((sensor_count % 5==0) && (sensor2>=distance_limit) &&(sensor4>=distance_limit)){
 if(any2==1){
    sensor1_output=0; 
     any2=0;

    
  }
  else if(any2==2){
    sensor3_output=0; 
     any2=0;

  }
  }
 


}



void gyro_loop() {
  read_mpu_6050_data();

  gyro_roll -= gyro_roll_cal;                                             
  gyro_pitch -= gyro_pitch_cal;                                              
  gyro_yaw -= gyro_yaw_cal;                                               
  
 
  angle_pitch += gyro_roll * 0.0000611;                                 
  angle_roll += gyro_pitch * 0.0000611;                                
  
 
  angle_pitch += angle_roll * sin(gyro_yaw * 0.000001066);              
  angle_roll -= angle_pitch * sin(gyro_yaw * 0.000001066);              
  
  //Accelerometer angle calculations
  acc_total_vector = sqrt((acc_x*acc_x)+(acc_y*acc_y)+(acc_z*acc_z));  
  //57.296 = 1 / (3.142 / 180) The Arduino asin function is in radians
  angle_pitch_acc = asin((float)acc_y/acc_total_vector)* 57.296;       
  angle_roll_acc = asin((float)acc_x/acc_total_vector)* -57.296;      
  
  //Place the MPU-6050 spirit level and note the values in the following two lines for calibration
  angle_pitch_acc -= 0.0;                                            
  angle_roll_acc -= 0.0;                                            

  if(set_gyro_angles){                                                
    angle_pitch = angle_pitch * 0.9996 + angle_pitch_acc * 0.0004;    
    angle_roll = angle_roll * 0.9996 + angle_roll_acc * 0.0004;       
  }
  else{                                                              
    angle_pitch = angle_pitch_acc;                                   
    angle_roll = angle_roll_acc;                                      
    set_gyro_angles = true;                                            
  }
  
  //To dampen the pitch and roll angles a complementary filter is used
  angle_pitch_output = angle_pitch_output * 0.9 + angle_pitch * 0.1;  
  angle_roll_output = angle_roll_output * 0.9 + angle_roll * 0.1;      
                                                   

  while(micros() - loop_timer < 4000);                                 
  loop_timer = micros();  

}


void wifi_loop(){
    if (mode=='r')
  {
    if(digitalRead(IRQq)==LOW)
    {
      delay(1);      
    unsigned char status = SPI_Read(STATUS);  
  
    if(status&STA_MARK_RX)                                              
    {
      SPI_Read_Buf(RD_RX_PLOAD, rx_buf, TX_PLOAD_WIDTH);           
      SPI_RW_Reg(FLUSH_RX,0); 
       
      for(byte i=0; i<TX_PLOAD_WIDTH; i++)
      {
        
      }
     
A_val = rx_buf[0];
T_val = rx_buf[1];
servo_x_val = rx_buf[2];
servo_y_val = rx_buf[3];
roll_val = rx_buf[4] ;
pitch_val = rx_buf[5];
Tr_val = rx_buf[6];

      SPI_RW_Reg(WRITE_REG+STATUS,0xff);
    }
     else{

    SPI_RW_Reg(WRITE_REG+STATUS,0xff);
     }   
    }
  delay(5);
    }
  else
  {
    
  }
}
int k;
float gyro_pitchaw_setpoint;
float output_channel_pitch,output_channel_roll;
float angle_pitch_setpoint,angle_roll_setpoint;
float pitch_angle,roll_angle;

void esc_loop(){
 if (readystart2==1) 
  {
   k++;
    if(k==100){
  angle_pitch_setpoint=angle_pitch_output;
  angle_roll_setpoint=angle_roll_output;
     
    }
 pitch_angle =( angle_pitch_output -  angle_pitch_setpoint)*10;
 roll_angle = (angle_roll_output -  angle_roll_setpoint)*10;
  
 int fin= T_val*6.12;
 if ((readystart==0) || (T_val==255)) fin=200;

 if ((roll_val<=2)&&(roll_val>=-2)) roll_val=0;
 if ((pitch_val<=2)&&(pitch_val>=-2)) pitch_val=0;

output_channel_pitch=pitch_val-119-6;
output_channel_roll=roll_val-125+6;
output_channel_pitch /=4;
output_channel_roll /=4;
calculate_pid();
int throttle=(T_val*(1000/245))+800;

 if(throttle>1801) throttle=1800;
 int throttle2=(T_val*(840/245))+960;

 if(throttle2>1801) throttle=1800;

 int yaw=Tr_val-122;
 esc1 = throttle - pid_output_roll+pid_output_pitch - pitch_angle+ roll_angle + roll_angle+sensor1_output+output_channel_pitch-output_channel_roll+yaw; 
    esc2 = throttle  + pid_output_roll+pid_output_pitch+pitch_angle+roll_angle+sensor2_output-output_channel_pitch-output_channel_roll-yaw ;
    esc3 = throttle2 + pid_output_roll-pid_output_pitch+pitch_angle-roll_angle+sensor3_output-output_channel_pitch+output_channel_roll+yaw; 
    esc4 = throttle-pid_output_roll - pid_output_pitch-pitch_angle-roll_angle +sensor4_output+output_channel_pitch+output_channel_roll-yaw; 




   firstESC.writeMicroseconds(esc1);
   secondESC.writeMicroseconds(esc2);
   thirdESC.writeMicroseconds(esc3);
   fourthESC.writeMicroseconds(esc4); 
  
  }
}

void servo_loop(){
  
  float x=(servo_x_val-122);
  float y=(servo_y_val-123);
 servo_x.writeMicroseconds(ccx*15.8);
 servo_y.writeMicroseconds(ccy*15.8);
  ccx= ccx + (x/50);
  ccy=ccy + (y/50);
 
  if (ccx>181) {
    ccx=180;
  }
   if (ccy>181) {
    ccy=180;
  }
  if (ccx<0){
    ccx=0;
  }
  if (ccy<0){
    ccy=0;
  }
analogWrite(laser_pin,A_val);
}

void wifi_settings()
{
        
        SPI_RW_Reg(WRITE_REG|iRF_BANK0_EN_AA, 0x01);        
        SPI_RW_Reg(WRITE_REG|iRF_BANK0_EN_RXADDR, 0x01);     
        SPI_RW_Reg(WRITE_REG|iRF_BANK0_SETUP_AW, 0x02);      
        
        SPI_RW_Reg(WRITE_REG|iRF_BANK0_SETUP_RETR, B00001010);      
        SPI_RW_Reg(WRITE_REG|iRF_BANK0_RF_CH, 40);
        SPI_RW_Reg(WRITE_REG|iRF_BANK0_RF_SETUP, 0x4f);       
        //SPI_RW_Reg(WRITE_REG|iRF_BANK0_DYNPD, 0x01);         
        //SPI_RW_Reg(WRITE_REG|iRF_BANK0_FEATURE, 0x07);       
        
 SPI_Write_Buf(WRITE_REG + TX_ADDR, TX_ADDRESS, TX_ADR_WIDTH);  
 SPI_Write_Buf(WRITE_REG + RX_ADDR_P0, TX_ADDRESS, TX_ADR_WIDTH);
 SPI_RW_Reg(WRITE_REG + RX_PW_P0, TX_PLOAD_WIDTH);
        
}

void calculate_pid(){
  //Roll calculations
  pid_error_temp = (gyro_roll_input/10) - pid_roll_setpoint;
  pid_i_mem_roll += pid_i_gain_roll * pid_error_temp;
  if(pid_i_mem_roll > pid_max_roll)pid_i_mem_roll = pid_max_roll;
  else if(pid_i_mem_roll < pid_max_roll * -1)pid_i_mem_roll = pid_max_roll * -1;
  
  pid_output_roll = pid_p_gain_roll * pid_error_temp + pid_i_mem_roll + pid_d_gain_roll * (pid_error_temp - pid_last_roll_d_error);
  if(pid_output_roll > pid_max_roll)pid_output_roll = pid_max_roll;
  else if(pid_output_roll < pid_max_roll * -1)pid_output_roll = pid_max_roll * -1;
  
  pid_last_roll_d_error = pid_error_temp;
 
  //Pitch calculations
  pid_error_temp = (gyro_pitch_input/10) - pid_pitch_setpoint;
  pid_i_mem_pitch += pid_i_gain_pitch * pid_error_temp;
  if(pid_i_mem_pitch > pid_max_pitch)pid_i_mem_pitch = pid_max_pitch;
  else if(pid_i_mem_pitch < pid_max_pitch * -1)pid_i_mem_pitch = pid_max_pitch * -1;
  
  pid_output_pitch = pid_p_gain_pitch * pid_error_temp + pid_i_mem_pitch + pid_d_gain_pitch * (pid_error_temp - pid_last_pitch_d_error);
  if(pid_output_pitch > pid_max_pitch)pid_output_pitch = pid_max_pitch;
  else if(pid_output_pitch < pid_max_pitch * -1)pid_output_pitch = pid_max_pitch * -1;
    
  pid_last_pitch_d_error = pid_error_temp;
    
  //Yaw calculations
  pid_error_temp = gyro_yaw_input - pid_yaw_setpoint;
  pid_i_mem_yaw += pid_i_gain_yaw * pid_error_temp;
  if(pid_i_mem_yaw > pid_max_yaw)pid_i_mem_yaw = pid_max_yaw;
  else if(pid_i_mem_yaw < pid_max_yaw * -1)pid_i_mem_yaw = pid_max_yaw * -1;
  
  pid_output_yaw = pid_p_gain_yaw * pid_error_temp + pid_i_mem_yaw + pid_d_gain_yaw * (pid_error_temp - pid_last_yaw_d_error);
  if(pid_output_yaw > pid_max_yaw)pid_output_yaw = pid_max_yaw;
  else if(pid_output_yaw < pid_max_yaw * -1)pid_output_yaw = pid_max_yaw * -1;
    
  pid_last_yaw_d_error = pid_error_temp;
}



void print_output(){
  Serial.print(" throttle ");
   Serial.print(T_val);
  Serial.print(" laser ");
   Serial.print(A_val);
  Serial.print("  ");
  Serial.print("m1");
  Serial.print(" "); 
  Serial.print(esc1);
  Serial.print("  ");
  Serial.print("m2");
  Serial.print(" "); 
  Serial.print(esc2);
  Serial.print("  ");
  Serial.print("m3");
  Serial.print(" "); 
  Serial.print(esc3);
  Serial.print("  ");
  Serial.print("m4");
  Serial.print(" "); 
  Serial.print(esc4);
  Serial.print("  ");
  Serial.print("Pitch_angle");
  Serial.print("  "); 
  Serial.print(roll_angle);

  Serial.print(" Roll_angle "); 
  Serial.print(pitch_angle);
   Serial.print(" pid_roll "); 
  Serial.print(pid_output_roll);
  Serial.print(" pid_pitch "); 
  Serial.print(pid_output_pitch);
    Serial.print(" counter "); 
  Serial.println(k);
}

