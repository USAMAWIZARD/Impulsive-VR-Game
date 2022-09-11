#include <Wire.h>
#include "wifi_connect.h"
double accx, accy,accz, gyrox,gyroy,gyroz,temp,magx,magy,magz,magx_ses,magy_ses,magz_ses;
const int Mpu=0x68;
const int MAG_ADDR = 0x0C; // 0x say that we are working with hex numbers and 0C is the magnetometer sensor adress in hex (from data sheet)
float asax, asay, asaz;
int8_t Msens_x,Msens_y, Msens_z;
int16_t mag_xL, mag_xH, mag_yL, mag_yH, mag_zL, mag_zH;
int8_t control_1;
int8_t status_1;
int8_t status_2;
float Yaw;
int16_t mag_x, mag_y, mag_z;

void setup() 
{
Wire.begin();
Wire.setClock(400000); // I2C clock rate ,You can delete it but it helps the speed of I2C (default rate is 100000 Hz)
delay(100);
// turn Mpu on
Wire.beginTransmission(Mpu);
Wire.write(0x6B);  // Power register
Wire.write(0);     // wakes up the MPU-6050
Wire.endTransmission();
//

delay(100);
// Configure Gyro
Wire.beginTransmission(Mpu);
Wire.write(0x1B); // Gyro Config
Wire.write(0x0); // (0: 250 dps (131 LSB/dps); 1: 500 dps (65.5 LSB/dps) ; 2: 1000 dps ( 32.8 LSB/dps) ; 3: 2000 dps (16.4 LSB/dps)
Wire.endTransmission();
//

delay(100);
// Configure Accelerometer
Wire.beginTransmission(Mpu);
Wire.write(0x1C); // Accelerometer  Config
Wire.write(0x0); // (0: 2g (16384 LSB/g); 1: 4g (8192 LSB/g) ; 2: 8g ( 4096 LSB/g) ; 3: 16g (2048 LSB/g)
Wire.endTransmission();

Serial.begin(9600);  //Setting the baudrate
delay(100);
wifi_connect();

}
void loop()
{
// Get data from MPU
Wire.beginTransmission(Mpu);
Wire.write(0x3B); 
Wire.endTransmission();
Wire.requestFrom(Mpu,14,true);
char sensor_data[40];
accx=(int16_t)(Wire.read()<<8|Wire.read())/16384.00; // g
accy=(int16_t)(Wire.read()<<8|Wire.read())/16384.00; // g
accz=(int16_t)(Wire.read()<<8|Wire.read())/16384.00; // g
temp=(int16_t)(Wire.read()<<8|Wire.read())/340.00 + 36.53;  // Celsus
gyrox=(int16_t)(Wire.read()<<8|Wire.read())/250.00; // Dps 
gyroy=(int16_t)(Wire.read()<<8|Wire.read())/250.00; // Dps 
gyroz=(int16_t)(Wire.read()<<8|Wire.read())/250.00; // Dp
sprintf(sensor_data,"%lf,%lf,%lf,%lf,%lf,%lf \n",accx,accy,accz,gyrox,gyroy,gyroz);

send_udp_packet(sensor_data);
Serial.print(sensor_data);


}
