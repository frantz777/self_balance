#include <Arduino.h>
#include <Adafruit_MPU6050.h>
//#include <Adafruit_Sensor.h>
//#include <Wire.h>

Adafruit_MPU6050 mpu;
//int counter = 0;
int LEDPIN = 19;
int currentState;

int ENA = {0};
int ENB = {2};
int int1 = {15};
int int2 = {13};
int int3 = {12};
int int4 = {14};

//int int1 = {8};
//int int2 = {7};
//int int3 = {6};
//int int4 = {5};



void setup(void) {
  Serial.begin(9600);
  Serial.println("Adafruit MPU6050 test!");
  pinMode(int1,OUTPUT);
  pinMode(int2,OUTPUT);
  pinMode(int3,OUTPUT);
  pinMode(int4,OUTPUT);
  Serial.println("Through set up");

  /*  // Try to initialize!
  if (!mpu.begin()) {
    Serial.println("Failed to find MPU6050 chip");
    while (1) {
      delay(10);
    }
  }
  Serial.println("MPU6050 Found!");

  mpu.setAccelerometerRange(MPU6050_RANGE_8_G);
  Serial.print("Accelerometer range set to: ");

  switch (mpu.getAccelerometerRange()) {
  case MPU6050_RANGE_2_G:
    Serial.println("+-2G");
    break;
  case MPU6050_RANGE_4_G:
    Serial.println("+-4G");
    break;
  case MPU6050_RANGE_8_G:
    Serial.println("+-8G");
    break;
  case MPU6050_RANGE_16_G:
    Serial.println("+-16G");
    break;
  }
  mpu.setGyroRange(MPU6050_RANGE_1000_DEG);
  Serial.print("Gyro range set to: ");
  switch (mpu.getGyroRange()) {
  case MPU6050_RANGE_250_DEG:
    Serial.println("+- 250 deg/s");
    break;
  case MPU6050_RANGE_500_DEG:
    Serial.println("+- 500 deg/s");
    break;
  case MPU6050_RANGE_1000_DEG:
    Serial.println("+- 1000 deg/s");
    break;
  case MPU6050_RANGE_2000_DEG:
    Serial.println("+- 2000 deg/s");
    break;
  }

  mpu.setFilterBandwidth(MPU6050_BAND_260_HZ);
  Serial.print("Filter bandwidth set to: ");
  switch (mpu.getFilterBandwidth()) {
  case MPU6050_BAND_260_HZ:
    Serial.println("260 Hz");
    break;
  case MPU6050_BAND_184_HZ:
    Serial.println("184 Hz");
    break;
  case MPU6050_BAND_94_HZ:
    Serial.println("94 Hz");
    break;
  case MPU6050_BAND_44_HZ:
    Serial.println("44 Hz");
    break;
  case MPU6050_BAND_21_HZ:
    Serial.println("21 Hz");
    break;
  case MPU6050_BAND_10_HZ:
    Serial.println("10 Hz");
    break;
  case MPU6050_BAND_5_HZ:
    Serial.println("5 Hz");
    break;
  }

  Serial.println("");
  delay(100);
  */
}


void loop() {
  //currentState = digitalRead(LEDPIN);
  currentState = HIGH;
  forward(150);
  delay(1000);
  backward(150);
  //digitalWrite(int1, HIGH);
  //digitalWrite(int2, LOW);
  //digitalWrite(int3, LOW);
  //digitalWrite(int4, HIGH);
  //Serial.println("Set high");
  //delay(3000);

  /*
  sensors_event_t a, g, temp;
  mpu.getEvent(&a, &g, &temp);
    if(currentState == LOW){

        double accX = 0;
        double accY = 0;
        double accZ = 0;
        double gyrX = 0;
        double gyrY = 0;
        double gyrZ = 0;
        accX = a.acceleration.x;
        accY = a.acceleration.y;
        accZ = a.acceleration.z;
        gyrX = g.gyro.x;
        gyrY = g.gyro.y;
        gyrZ = g.gyro.z;

        Serial.print(accX);
        Serial.print(", ");
        Serial.print(accY);
        Serial.print(", ");
        Serial.print(accZ);
        Serial.print(", ");
        Serial.print(gyrX);
        Serial.print(", ");
        Serial.print(gyrY);
        Serial.print(", ");
        Serial.print(gyrZ);
        Serial.print(", ");
        Serial.println("");
        
    }
  else{
    digitalWrite(int1, HIGH);
    digitalWrite(int2, LOW);
    digitalWrite(int3, HIGH);
    digitalWrite(int4, LOW);
    Serial.print(", ");

    //delay(500);
  }
  */
}

void forward(int PWM){
  Serial.println("Forward");
  digitalWrite(int1, HIGH);
  digitalWrite(int2, LOW);
  digitalWrite(int3, LOW);
  digitalWrite(int4, HIGH);
}
void backward(int PWM){
  Serial.println("Backward");
  digitalWrite(ENA, PWM);
  digitalWrite(ENB, PWM);
  digitalWrite(int1, LOW);
  digitalWrite(int2, HIGH);
  digitalWrite(int3, HIGH);
  digitalWrite(int4, LOW);
}