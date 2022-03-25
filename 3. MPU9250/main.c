// made by David Park, Wayne Kim
// 20200214
// POWER_ON
// mpu9250 control code

#define F_CPU 8000000UL
#include <avr/io.h>
#include <util/delay.h>
#include "pata_UART.h"
#include <avr/interrupt.h>
#include <math.h>

#define SS      PB2
#define MOSI   PB3
#define MISO   PB4
#define SCK      PB5

#define WHO_AM_I 0x75
#define READ_FLAG 0x80

#define ACCEL_XOUT_H 0x3B
#define ACCEL_XOUT_L 0x3C
#define ACCEL_YOUT_H 0x3D
#define ACCEL_YOUT_L 0x3E
#define ACCEL_ZOUT_H 0x3F
#define ACCEL_ZOUT_L 0x40

#define GYRO_XOUT_H 0x43
#define GYRO_XOUT_L 0x44
#define GYRO_YOUT_H 0x45
#define GYRO_YOUT_L 0x46
#define GYRO_ZOUT_H 0x47
#define GYRO_ZOUT_L 0x48

#define MAG_XOUT_L 0x03
#define MAG_XOUT_H 0x04
#define MAG_YOUT_L 0x05
#define MAG_YOUT_H 0x06
#define MAG_ZOUT_L 0x07
#define MAG_ZOUT_H 0x08

void calibrate_sensors();

float aRes = 2.0f / 32768.0f; // scaling of acc sensor
uint16_t ax, ay, az; //final acc data
float gRes;
float gx, gy, gz;
float mRes; 
float mx, my, mz;

uint8_t ArawData[6]; //raw data from mpu9250
int16_t Ainto16[3]; //change into 16bit
uint8_t GrawData[6];
int16_t Ginto16[3];
uint8_t MrawData[6];
uint16_t Minto16[3];

uint16_t RADATA[3];//Radial Data

uint8_t test;
volatile int counter = 0;
volatile int timer = 1;
float alpha = 1.00; // rate between gyro comp and accel comp
float radtodeg = 57.3f;
float pitch = 0, roll = 0, yaw = 0;
float filtered_x = 0, filtered_y = 0, filtered_z = 0; //final angle values : in degrees

float base_x_accel; //mean value of acceleration
float base_y_accel; 
float base_z_accel;
float base_x_gyro;   
float base_y_gyro;
float base_z_gyro;
float gyro_x, gyro_y, gyro_z; //
float accel_x, accel_y, accel_z; //
float gx = 0, gy = 0, gz = 0; //current gravity components

void GravityCalib(float prevgx, float prevgy, float prevgz){ //euler matrix transformation
   
   gx =   prevgx               -      gyro_x * prevgy      +   gyro_y   *   prevgz;
   gy =   gyro_x * prevgx         +      prevgy            -   gyro_z   *   prevgz;
   gz =   -gyro_y * prevgx      +      prevgy * gyro_z      +            prevgz;
}
ISR(TIMER0_OVF_vect){ //time interval 80ms 
   TCNT0 = 5;
   
   counter++;
   if(counter >= 10){
      counter = 0;
      timer = 1;
   }
}

void init(){ 
   DDRB = (1 << SCK) | (1 << SS) | (1 << MOSI);
   
   PORTB = (1 << SS);
   SPCR = (1 << SPE) | (1 << MSTR) | (1 << SPR0) | (1 << SPI2X); //mode0, sck freq / 16

   TIMSK = 0x01;
   TCCR0 = 0x04; //prescaler 256
   sei();
}

uint8_t SPI_write(uint8_t trans_data){ // send data via spi 
   PORTB &= ~(1 << SS); //pull ss down
   
   SPDR = trans_data;
   while(!(SPSR & (1 << SPIF)));
   
   SPDR = 0x00; //full duplex 
   while(!(SPSR & (1 << SPIF)));
   
   PORTB |= (1 << SS); //pull ss up
   return SPDR;
}

uint8_t ReadfronMPU(uint8_t regAddress){
   uint8_t data;
   
   data = SPI_write(regAddress | READ_FLAG); //full duplex
   
   return data;
}

int main(void)
{
   UART_init(BAUD_14k, TXRX);
   init();
   
   test = ReadfronMPU(WHO_AM_I);
   UART_tx(test); //0x71 should be returned(unique number)
   
   
   calibrate_sensors();
   
   while (1) 
   {
      
   ArawData[0] = ReadfronMPU(ACCEL_XOUT_H);
   ArawData[1] = ReadfronMPU(ACCEL_XOUT_L);
   ArawData[2] = ReadfronMPU(ACCEL_YOUT_H);
   ArawData[3] = ReadfronMPU(ACCEL_YOUT_L);
   ArawData[4] = ReadfronMPU(ACCEL_ZOUT_H);
   ArawData[5] = ReadfronMPU(ACCEL_ZOUT_L);
     
   GrawData[0] = ReadfronMPU(GYRO_XOUT_H);
   GrawData[1] = ReadfronMPU(GYRO_XOUT_L);
   GrawData[2] = ReadfronMPU(GYRO_YOUT_H);
   GrawData[3] = ReadfronMPU(GYRO_YOUT_L);
   GrawData[4] = ReadfronMPU(GYRO_ZOUT_H);
   GrawData[5] = ReadfronMPU(GYRO_ZOUT_L);
   
   Ainto16[0] = ((int16_t)ArawData[0]<<8)|ArawData[1]; //high + low
   Ainto16[1] = ((int16_t)ArawData[2]<<8)|ArawData[3];
   Ainto16[2] = ((int16_t)ArawData[4]<<8)|ArawData[5];
    
   Ginto16[0] = ((int16_t)GrawData[0]<<8)|GrawData[1]; //high + low
   Ginto16[1] = ((int16_t)GrawData[2]<<8)|GrawData[3];
   Ginto16[2] = ((int16_t)GrawData[4]<<8)|GrawData[5];
   
      int FS_SEL = 131;
   gyro_x = (float)(Ginto16[0] - base_x_gyro)*0.00061f; // radtodeg * 0.08ms * 250 / 32768 
   gyro_y = (float)(Ginto16[1] - base_y_gyro)*0.00061f; 
   gyro_z = (float)(Ginto16[2] - base_z_gyro)*0.00061f; 
   
   accel_x = (float)(Ainto16[0])*0.0006; // 2 * 9.8 / 32768
   accel_y = (float)(Ainto16[1])*0.0006; 
   accel_z = (float)(Ainto16[2])*0.0006; 
   
   accel_x-=gx; 
   accel_y-=gy;
   accel_z-=gz;
   
   GravityCalib(gx, gy, gz);
   
   
    //Ginto16[0] /= FS_SEL;
    //Ginto16[1] /= FS_SEL;
    //Ginto16[2] /= FS_SEL;
      
    pitch = atan2(accel_y, (sqrt((accel_x * accel_x) + (accel_y * accel_y)))) * radtodeg;
    roll =  atan2(-accel_x, (sqrt((accel_y * accel_y) + (accel_z * accel_z)))) * radtodeg;
    //yaw = atan2(sqrt((Ainto16[0] * Ainto16[0]) + (Ainto16[1] * Ainto16[1])), Ainto16[2]); 
    //default in radians. multiply radtodeg   
    
    filtered_x = (filtered_x + (gyro_x));// - (1.00f - alpha) * pitch;
    filtered_y = (filtered_y + (gyro_y));// - (1.00f - alpha) * roll;
    filtered_z = (filtered_z + (gyro_z));
    //UART_tx((int8_t)(base_x_accel*0.0006));
    //UART_tx((int8_t)(base_y_accel*0.0006));
    //UART_tx((int8_t)(base_z_accel*0.0006));
  

    UART_tx((int8_t)filtered_x);
    UART_tx((int8_t)filtered_y);
    UART_tx((int8_t)filtered_z);
    UART_NWL();
    //in radians
    
    
    while(!timer);
    timer = 0;
    }
}


void calibrate_sensors() { 

   int       num_readings = 1000;
   float        x_accel = 0;
   float        y_accel = 0;
   float        z_accel = 0;
   float        x_gyro = 0;
   float        y_gyro = 0;
   float        z_gyro = 0;
   
   //Serial.println("Starting Calibration");

   // Discard the first set of values read from the IMU
   // Read and average the raw values from the IMU
   
   
   
   for (int i = 0; i < num_readings; i++) { 
    ArawData[0] = ReadfronMPU(ACCEL_XOUT_H);
    ArawData[1] = ReadfronMPU(ACCEL_XOUT_L);
    ArawData[2] = ReadfronMPU(ACCEL_YOUT_H);
    ArawData[3] = ReadfronMPU(ACCEL_YOUT_L);
    ArawData[4] = ReadfronMPU(ACCEL_ZOUT_H);
    ArawData[5] = ReadfronMPU(ACCEL_ZOUT_L);
    
    GrawData[0] = ReadfronMPU(GYRO_XOUT_H);
    GrawData[1] = ReadfronMPU(GYRO_XOUT_L);
    GrawData[2] = ReadfronMPU(GYRO_YOUT_H);
    GrawData[3] = ReadfronMPU(GYRO_YOUT_L);
    GrawData[4] = ReadfronMPU(GYRO_ZOUT_H);
    GrawData[5] = ReadfronMPU(GYRO_ZOUT_L);
    
    Ainto16[0] = ((int16_t)ArawData[0] << 8) | ArawData[1]; //high + low
    Ainto16[1] = ((int16_t)ArawData[2] << 8) | ArawData[3];
    Ainto16[2] = ((int16_t)ArawData[4] << 8) | ArawData[5];
    
    Ginto16[0] = ((int16_t)GrawData[0] << 8) | GrawData[1]; //high + low
    Ginto16[1] = ((int16_t)GrawData[2] << 8) | GrawData[3];
    Ginto16[2] = ((int16_t)GrawData[4] << 8) | GrawData[5];
    
    
      x_accel += Ainto16[0];
      y_accel += Ainto16[1];
      z_accel += Ainto16[2];
      x_gyro += Ginto16[0];
      y_gyro += Ginto16[1];
      z_gyro += Ginto16[2];
   }

   
   x_gyro /= num_readings;
   y_gyro /= num_readings;
   z_gyro /= num_readings;

   gx = (float)x_accel/num_readings*0.0006f;
   gy = (float)y_accel/num_readings*0.0006f;
   gz = (float)z_accel/num_readings*0.0006f;
   base_x_gyro = x_gyro;
   base_y_gyro = y_gyro;
   base_z_gyro = z_gyro;
   base_x_accel = gx;
   base_y_accel = gy;
   base_z_accel = gz;
}
