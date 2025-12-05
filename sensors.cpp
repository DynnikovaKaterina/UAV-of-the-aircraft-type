// код выводит на экран показания датчиков BMP-280 и MPU-6500 
// (температура, давление, линейное ускорение и угловая скорость по 3 осям)
#include <Wire.h>
#include <Adafruit_BMP280.h>
#include <Adafruit_MPU6050.h>
#include <Adafruit_Sensor.h>

Adafruit_BMP280 bmp;
Adafruit_MPU6050 mpu;

void setup() {
  Serial.begin(9600);
  
  Serial.println("Arduino Nano + BMP280 + MPU-6500");
  Serial.println("================================");
  
  // Инициализация BMP280
  if (bmp.begin(0x76) || bmp.begin(0x77)) {
    Serial.println("BMP280: OK");
  } else {
    Serial.println("BMP280: FAILED");
  }
  
  // Инициализация MPU-6500
  if (mpu.begin()) {
    Serial.println("MPU-6500: OK");
    mpu.setAccelerometerRange(MPU6050_RANGE_2_G);
    mpu.setGyroRange(MPU6050_RANGE_250_DEG);
    mpu.setFilterBandwidth(MPU6050_BAND_5_HZ);
  } else {
    Serial.println("MPU-6500: FAILED");
  }
  
  Serial.println("TIME(s),TEMP(C),PRESS(hPa),ACC_X,ACC_Y,ACC_Z,GYRO_X,GYRO_Y,GYRO_Z");
}

void loop() {
  sensors_event_t a, g, temp;
  
  // Чтение BMP280
  float bmpTemp = bmp.readTemperature();
  float pressure = bmp.readPressure() / 100.0F;
  
  // Чтение MPU-6500
  mpu.getEvent(&a, &g, &temp);
  
  // Вывод в CSV формате
  Serial.print(millis() / 1000); Serial.print(",");
  Serial.print(bmpTemp, 1); Serial.print(",");
  Serial.print(pressure, 1); Serial.print(",");
  Serial.print(a.acceleration.x, 2); Serial.print(",");
  Serial.print(a.acceleration.y, 2); Serial.print(",");
  Serial.print(a.acceleration.z, 2); Serial.print(",");
  Serial.print(g.gyro.x, 2); Serial.print(",");
  Serial.print(g.gyro.y, 2); Serial.print(",");
  Serial.println(g.gyro.z, 2);
  
  delay(200);
}