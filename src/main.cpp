#include <Arduino.h>
#include <Adafruit_MPU6050.h>
#include <math.h>
//#include <Adafruit_Sensor.h>
#include <Wire.h>

unsigned long myTime;

Adafruit_MPU6050 mpu;
int LEDPIN = 19;
int currentState;
int counter = {0};
int ENA = {0};
int ENB = {2};
int int1 = {15};
int int2 = {13};
int int3 = {12};
int int4 = {14};

double accX_tot = {0};
double accY_tot = {0};
double accZ_tot = {0};
double gyrX_tot = {0};
double gyrY_tot = {0};
double gyrZ_tot = {0};

double accX_offset = {-.51};
double accY_offset = {.02};
double accZ_offset = {-1.64};
double gyrX_offset = {-.01};
double gyrY_offset = {.01};
double gyrZ_offset = {-.01};

double Kp = {.3};
double Ki = {.2};
double Kd = {0};

double U_tot = {0};
double x_rot = {0};


void setup(void) {
  Serial.begin(9600);
  Serial.println("Adafruit MPU6050 test!");
  pinMode(int1,OUTPUT);
  pinMode(int2,OUTPUT);
  pinMode(int3,OUTPUT);
  pinMode(int4,OUTPUT);
  pinMode(ENA,OUTPUT);
  pinMode(ENB,OUTPUT);
  Serial.println("Through set up");
  

  // Try to initialize!
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

  mpu.setFilterBandwidth(MPU6050_BAND_94_HZ);
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
  
}

void moveForward(int PWM){
  Serial.println("Forward");
  digitalWrite(int1, HIGH);
  digitalWrite(int2, LOW);
  digitalWrite(int3, LOW);
  digitalWrite(int4, HIGH);
  digitalWrite(ENA, PWM);
  digitalWrite(ENB, PWM);
}
void moveBackward(int PWM){
  Serial.println("Backward");
  digitalWrite(ENA, PWM);
  digitalWrite(ENB, PWM);
  digitalWrite(int1, LOW);
  digitalWrite(int2, HIGH);
  digitalWrite(int3, HIGH);
  digitalWrite(int4, LOW);
}

void loop() {
  //currentState = digitalRead(LEDPIN);
  currentState = HIGH;
  myTime = millis();
  sensors_event_t a, g, temp;
  mpu.getEvent(&a, &g, &temp);
  double curr_angle_acc = {0};
  double accX = {0};
  double accY = {0};
  double accZ = {0};
  double gyrX = {0};
  double gyrY = {0};
  double gyrZ = {0};

  accX = a.acceleration.x + accX_offset;
  accY = a.acceleration.y + accY_offset;
  accZ = a.acceleration.z + accZ_offset;
  gyrX = g.gyro.x + gyrX_offset;
  gyrY = g.gyro.y + gyrY_offset;
  gyrZ = g.gyro.z + gyrZ_offset;
  Serial.print("Y: ");
  Serial.print(accY);
  Serial.print(", Z: ");
  Serial.print(accZ);
  Serial.print(", Angle Deg: ");
  curr_angle_acc = atan(accZ / accY) * RAD_TO_DEG;

  Serial.println(curr_angle_acc);

  //U_tot = Kp * ;

  // Serial.print(accX);
  // Serial.print(", ");
  // Serial.print(accY);
  // Serial.print(", ");
  // Serial.print(accZ);
  // Serial.print(", ");
  // Serial.print(gyrX);
  // Serial.print(", ");
  // Serial.print(gyrY);
  // Serial.print(", ");
  // Serial.print(gyrZ);
  // Serial.print(", ");
  // Serial.print(counter);
  // Serial.println("");

}

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
      delay(500);
    }


    if (counter == 1000){
    accX_tot = accX_tot/counter;
    accY_tot = accY_tot/counter;
    accZ_tot = accZ_tot/counter;
    gyrX_tot = gyrX_tot/counter;
    gyrY_tot = gyrY_tot/counter;
    gyrZ_tot = gyrZ_tot/counter;

    Serial.println("Average Values:");
    Serial.println("");
    Serial.print(accX_tot);
    Serial.print(", ");
    Serial.print(accY_tot);
    Serial.print(", ");
    Serial.print(accZ_tot);
    Serial.print(", ");
    Serial.print(gyrX_tot);
    Serial.print(", ");
    Serial.print(gyrY_tot);
    Serial.print(", ");
    Serial.print(gyrZ_tot);
    Serial.print(", ");
    delay(100000);
  }

    accX_tot = accX_tot + accX;
    accY_tot = accY_tot + accY;
    accZ_tot = accZ_tot + accZ;
    gyrX_tot = gyrX_tot + gyrX;
    gyrY_tot = gyrY_tot + gyrY;
    gyrZ_tot = gyrZ_tot + gyrZ;
    counter++;

    */