#include <Wire.h>
#include <Servo.h>

int MPU_ADDR = 0x68; // MPU6050 I2C address
Servo servo;                
int servoPin = 9;     

float AccX, AccY, AccZ;      
float GyroX, GyroY, GyroZ;   
float accAngleX;             
float gyroAngleX = 0;        
float roll = 0;              
float elapsedTime, currentTime, previousTime;

// Gyroscope calibration offsets
float gyroXoffset = 0;
float gyroYoffset = 0;

void setup() {

  Serial.begin(9600);
  Wire.begin(); // Start I2C communication
  Wire.beginTransmission(MPU_ADDR);
  Wire.write(0x6B);
  Wire.write(0x00);
  Wire.endTransmission(true);
  
  servo.attach(servoPin); // Attach the servo to pin 9
  servo.write(90);         // Initialize servo at midpoint
  delay(100);
  
  // Calibration phase
  for (int i = 0; i < 1000; i++) {
    Wire.beginTransmission(MPU_ADDR);
    Wire.write(0x43); // Start reading gyro data
    Wire.endTransmission(false);
    Wire.requestFrom(MPU_ADDR, 6, true);
    
    GyroX = (Wire.read() << 8 | Wire.read()) / 131.0; // X-axis rotation
    GyroY = (Wire.read() << 8 | Wire.read()) / 131.0; // Y-axis rotation

    gyroXoffset += GyroX; // Accumulate offsets
    gyroYoffset += GyroY; 
    delay(3);
  }
  
  gyroXoffset /= 1000; // Average offset
  gyroYoffset /= 1000; 
}

void loop() {
  //Read accelerometer data
  Wire.beginTransmission(MPU_ADDR);
  Wire.write(0x3B); 
  Wire.endTransmission(false);
  Wire.requestFrom(MPU_ADDR, 6, true); // Read 6 registers
  
  AccX = (Wire.read() << 8 | Wire.read()) / 16384.0; // X-axis
  AccY = (Wire.read() << 8 | Wire.read()) / 16384.0; // Y-axis
  AccZ = (Wire.read() << 8 | Wire.read()) / 16384.0; // Z-axis
  
  // Calculating Roll angle from accelerometer data
  accAngleX = (atan2(AccY, sqrt(pow(AccX, 2) + pow(AccZ, 2))) * 180 / PI);
  
  // === Read gyroscope data === //
  previousTime = currentTime;
  currentTime = millis();
  elapsedTime = (currentTime - previousTime) / 1000.0; // Elapsed time in seconds
  
  Wire.beginTransmission(MPU_ADDR);
  Wire.write(0x43); // Start reading gyro data
  Wire.endTransmission(false);
  Wire.requestFrom(MPU_ADDR, 6, true); // Read 6 registers
  
  GyroX = (Wire.read() << 8 | Wire.read()) / 131.0; // X-axis rotation
  GyroY = (Wire.read() << 8 | Wire.read()) / 131.0; // Y-axis rotation

  // Subtract calibration offsets
  GyroX -= gyroXoffset; 
  GyroY -= gyroYoffset;

  // Integrate gyroscope data to get angles
  gyroAngleX += GyroX * elapsedTime;

  // === Complementary Filter ===
  roll = 0.9 * gyroAngleX + 0.1 * accAngleX; // Adjust weights for better stability
  
  // Map roll angle to servo movement (0° to 180°), reverse the angle
  int mappedServoAngle = map(roll, -90, 90, 180, 0); // Reverse mapping

  mappedServoAngle = constrain(mappedServoAngle, 0, 180); 
  int mappedServoAnglec = mappedServoAngle-1;
  servo.write(mappedServoAnglec); // Move servo
  
  // Print data
  Serial.print("Roll: "); Serial.print(roll);
  Serial.print(" | Servo Angle: "); Serial.println(mappedServoAngle);
  
  delay(100); 
}
