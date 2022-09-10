#include <Wire.h>

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
//mag
  Wire.beginTransmission(MAG_ADDR); 
  Wire.write(0x0B);   //  CONTROL 2
  Wire.write(0b01);   //  0 NORMAL OR 1 RESET
  Wire.endTransmission(true);
  delay(200);

  Wire.beginTransmission(MAG_ADDR);   //SLEEP MODE
  Wire.write(0x0A);   //  CONTROL 1
  Wire.write(0b00010000);   // 1 for 16 bit or 0 for 14 bit output, 0000 SLEEP MODE
  Wire.endTransmission(true);
  delay(200);

  
  Wire.beginTransmission(MAG_ADDR);   //ROM WRITE MODE
  Wire.write(0x0A);   //  CONTROL 1
  Wire.write(0b00011111); // 1 for 16 bit or 0 for 14 bit output, 1111 FUSE ROM ACCESS MODE
  Wire.endTransmission(true);
  delay(200);

  Wire.beginTransmission(MAG_ADDR);   //GET MAGNETIC SENSITIVITY DATA FOR CONVERTING RAW DATA
  Wire.write(0x10);     //  ASAX  
  Wire.endTransmission(false);
  Wire.requestFrom(MAG_ADDR, 3 , true);  //GET SENSITIVITY ADJUSMENT VALUES STARTS AT ASAX
  Msens_x = Wire.read();    //GET X SENSITIVITY ADJUSMENT VALUE
  Msens_y = Wire.read();    //GET Y SENSITIVITY ADJUSMENT VALUE
  Msens_z = Wire.read();    //GET Z SENSITIVITY ADJUSMENT VALUE
  Serial.println(Msens_x);
  Serial.println(Msens_y);
  Serial.println(Msens_z);
  Wire.endTransmission(true);
  asax = (((Msens_x-128))/256.0f)+1.0f;
  asay = (((Msens_y-128))/256.0f)+1.0f;
  asaz = (((Msens_z-128))/256.0f)+1.0f;
  Serial.print("Mx Sensitivity: ");  Serial.println(asax);
  Serial.print("My Sensitivity: ");  Serial.println(asay);
  Serial.print("Mz Sensitivity: ");  Serial.println(asaz); 
  delay(200);
  
  Wire.beginTransmission(MAG_ADDR);   //SLEEP MODE
  Wire.write(0x0A);   //  CONTROL 1
  Wire.write(0b00010000);  // 1 for 16 bit or 0 for 14 bit output, 0000 SLEEP MODE
  Wire.endTransmission(true);
  delay(200);
    
  Wire.beginTransmission(MAG_ADDR);   //CONT MODE 2
  Wire.write(0x0A);
  Wire.write(0b00010110); // 1 for 16 bit or 0 for 14 bit output, 0110 FOR CONT MODE 2 (X Hz?) 
  Wire.endTransmission(true);
  delay(200);


Serial.begin(9600);  //Setting the baudrate
delay(100);

}
void loop()
{
// Get data from MPU
Wire.beginTransmission(Mpu);
Wire.write(0x3B); 
Wire.endTransmission();
Wire.requestFrom(Mpu,14,true);
accx=(int16_t)(Wire.read()<<8|Wire.read())/16384.00; // g
accy=(int16_t)(Wire.read()<<8|Wire.read())/16384.00; // g
accz=(int16_t)(Wire.read()<<8|Wire.read())/16384.00; // g
temp=(int16_t)(Wire.read()<<8|Wire.read())/340.00 + 36.53;  // Celsus
gyrox=(int16_t)(Wire.read()<<8|Wire.read())/250.00; // Dps 
gyroy=(int16_t)(Wire.read()<<8|Wire.read())/250.00; // Dps 
gyroz=(int16_t)(Wire.read()<<8|Wire.read())/250.00; // Dps

  Wire.beginTransmission(MAG_ADDR);
  Wire.write(0x0A);
  Wire.endTransmission(false);
  Wire.requestFrom(MAG_ADDR, 1 , true);  
  control_1 = Wire.read();  // check DRDY bit if ready to read
  Serial.print("control_1: "); Serial.println(control_1,BIN);  
  Wire.endTransmission(true);

  Wire.beginTransmission(MAG_ADDR);
  Wire.write(0x02);
  Wire.endTransmission(false);
  Wire.requestFrom(MAG_ADDR, 1 , true);   
  status_1 = Wire.read();
  Serial.print("Status 1: "); Serial.println(status_1,BIN);  
  Wire.endTransmission(true);
    if(status_1 == 0b11) {

Wire.beginTransmission(MAG_ADDR);
Wire.write(0x03);
Wire.endTransmission(false);
Wire.requestFrom(MAG_ADDR, 7 , true);

mag_xL = Wire.read();
mag_xH = Wire.read();
mag_x = (mag_xH << 8) | mag_xL;

Serial.print("Available bytes left after reading mag x values: ");
Serial.println(Wire.available(),DEC);    

mag_yL = Wire.read();
mag_yH = Wire.read();
mag_y = (mag_yH << 8) | mag_yL;    

Serial.print("LOW BITS Mag_Y: "); Serial.println(mag_yL,BIN);  
Serial.print("HIGH BITS Mag_Y: "); Serial.println(mag_yH,BIN);
Serial.print("FULL BITS Mag_Y: "); Serial.println(mag_y,BIN);    

mag_zL = Wire.read();
mag_zH = Wire.read();
mag_z = (mag_zH << 8) | mag_zL;


status_2 = Wire.read();   // check if there is a magnetic overflow 
Serial.print("Status 2: "); Serial.println(status_2,BIN);  

//if(status_2 != 0x08)
Serial.print("  | mX = "); Serial.print(mag_x*asax*0.15); Serial.print(" [uT]");
Serial.print("  | mY = "); Serial.print(mag_y*asay*0.15); Serial.print(" [uT]");
Serial.print("  | mZ = "); Serial.print(mag_z*asaz*0.15); Serial.println(" [uT]");

  }

Wire.endTransmission(true);


Serial.print("accx= ");
Serial.print(accx);
Serial.print(" accy= ");
Serial.print(accy);
Serial.print(" accz= ");
Serial.println(accz);


}
