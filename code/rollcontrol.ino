const int hreq = 6;
double rollreq= 20;

#include <ESP32Servo.h>
#include <I2Cdev.h>
#include <MPU6050.h>
#include "KalmanFilter.h"
#include <WiFi.h>
/////////////////////////////////////////// Wi-Fi credentials
const char* ssid = "abcde";
const char* password = "12345678";
////////////////////////////////////////// IP address and port of the PC
const char* serverIP = "192.168.89.239";
const int serverPort = 1234;
WiFiClient client;
#if I2CDEV_IMPLEMENTATION == I2CDEV_ARDUINO_WIRE
    #include <Wire.h>
#endif
MPU6050 accelgyro;
int16_t ax, ay, az;
int16_t gx, gy, gz;
Kalman kalmanX; // Create the Kalman instances
Kalman kalmanY;
uint32_t timer;
double gyroXangle, gyroYangle; // Angle calculate using the gyro only
double compAngleX, compAngleY; // Calculated angle using a complementary filter
double kalAngleX, kalAngleY; // Calculated angle using a Kalman filter
double gyr[2]={0,0};
/////////////////////////////////////////////////////////////////////////////////////////
const int trigPin = 14;
const int echoPin = 12;
const int MAX_HEIGHT = 10;
const double SOUND_VELOCITY=0.034826;
const float tol=0.5;
double distance=0.0;
double duration=0.0;
//////////////////////////////////////////////////////////////////////////////////////////
unsigned long currentTime, previousTime, currentTime2, previousTime2;
double elapsedTime, elapsedTime2;
double error, error2;
double lastError, lastError2;
double input, output, setPoint;
double input2, output2, setPoint2;
double cumError, rateError;
double cumError2, rateError2;

////////////////////////////////PID constants/////////////////////////////////////////////
double kp = 5;
double ki = 0.07;
double kd = 3;
double kp2 = 0.05;
double ki2 = 0.07;
double kd2 = 3;
//////////////////////////////////////////////////////////////////////////////////////////
const int  c1 = 35;
const int  c2 = 34;
const int  c3 = 39;
const int  c4 = 36;
///////////////////////////////////////////////////////////////////////////////////////////


Servo bli;
Servo blo;
Servo bri;
Servo bro;
int pos=0;
int baseangle= 40;

void bli_(int pos){
     bli.write(180-pos);
}
void blo_(int pos){
     blo.write(pos);
}
void bri_(int pos){
     bri.write(pos);
}
void bro_(int pos){
     bro.write(180-pos);
}


double dist(){
  float distance, duration;
  digitalWrite(trigPin, LOW);
  delayMicroseconds(2);
  
  digitalWrite(trigPin, HIGH);
  delayMicroseconds(10);
  digitalWrite(trigPin, LOW);
  
  duration = pulseIn(echoPin, HIGH);
  
  distance = duration * SOUND_VELOCITY/2;
  return distance;
}



void mpu_setup() {
    // join I2C bus (I2Cdev library doesn't do this automatically)
    #if I2CDEV_IMPLEMENTATION == I2CDEV_ARDUINO_WIRE
        Wire.begin();
    #elif I2CDEV_IMPLEMENTATION == I2CDEV_BUILTIN_FASTWIRE
        Fastwire::setup(400, true);
    #endif

    // initialize device
    Serial.println("Initializing I2C devices...");
    accelgyro.initialize();

    // verify connection
    Serial.println("Testing device connections...");
    Serial.println(accelgyro.testConnection() ? "MPU6050 connection successful" : "MPU6050 connection failed");

    accelgyro.getMotion6(&ax, &ay, &az, &gx, &gy, &gz);
    double roll=0.0;
    double pitch=0.0;
    if(az!=0)
      roll  = atan2(ay, az) * RAD_TO_DEG;
    else
      roll  = atan2(ay, az+0.0001) * RAD_TO_DEG;
    if(sqrt(ay * ay + az * az)!=0.0)  
      pitch = atan(-ax / sqrt(ay * ay + az * az)) * RAD_TO_DEG;
    else
      pitch = atan(-ax /0.00001 + sqrt(ay * ay + az * az)) * RAD_TO_DEG;
    kalmanX.setAngle(roll); // Set starting angle
    kalmanY.setAngle(pitch);
    gyroXangle = roll;
    gyroYangle = pitch;
    compAngleX = roll;
    compAngleY = pitch;

    timer = micros();
}

double computePID(double inp){     
    currentTime = millis();                //get current time
    elapsedTime = (double)(currentTime - previousTime);        //compute time elapsed from previous computation
    double Setpoint=hreq;
    error = Setpoint - inp;                                // determine error
    cumError += error * elapsedTime;                // compute integral
    rateError = (error - lastError)/elapsedTime;   // compute derivative

    double out = kp*error + ki*cumError + kd*rateError;                //PID output               
    
    // Serial.print("Cum: ");
    // Serial.println(cumError);
    lastError = error;                                //remember current error
    previousTime = currentTime;                        //remember current time

    if(out < 0){
      out = 0;
    }else if(out > 180){
      out = 180;
    }
    return out;                                        //have function return the PID output
}

double computePID2(double inp){     
    currentTime2 = millis();                //get current time
    elapsedTime2 = (double)(currentTime2 - previousTime2);        //compute time elapsed from previous computation
    double Setpoint2=rollreq;
    error2 = Setpoint2 - inp;                                // determine error
    cumError2 += error2 * elapsedTime2;                // compute integral
    rateError2 = (error2 - lastError2)/elapsedTime2;   // compute derivative

    double out2 = kp2*error2 + ki2*cumError2 + kd2*rateError2;                //PID output               
    
    // Serial.print("Cum: ");
    // Serial.println(cumError);
    lastError2 = error2;                                //remember current error
    previousTime2 = currentTime2;                        //remember current time

    if(out2 < 0){
      out2 = 0;
    }else if(out2 > 180){
      out2 = 180;
    }
    return out2;                                        //have function return the PID output
}

void setup() {
  Serial.begin(9600);

  bli.attach(27);
  blo.attach(26);
  bri.attach(15);
  bro.attach(2);

  WiFi.begin(ssid, password);
  while (WiFi.status() != WL_CONNECTED)
    delay(1000);
  if (client.connect(serverIP, serverPort)) {
    // client.println("Connected to server");
  }
  delay(2000);
  
  mpu_setup();

  pinMode(trigPin, OUTPUT);
  pinMode(echoPin, INPUT);

  bri_(0);
  bro_(0);
  bli_(0);
  blo_(0);

  pinMode(c1, INPUT);
  pinMode(c2, INPUT);
  pinMode(c3, INPUT);
  pinMode(c4, INPUT);

  delay(500);
}

double* get_rp() {
    accelgyro.getMotion6(&ax, &ay, &az, &gx, &gy, &gz);

    double dt = (double)(micros() - timer) / 1000000; // Calculate delta time
    timer = micros();
    double roll=0.0;
    double pitch=0.0;
    if(az!=0)
      roll  = atan2(ay, az) * RAD_TO_DEG;
    else
      roll  = atan2(ay, az+0.0001) * RAD_TO_DEG;
    if(sqrt(ay * ay + az * az)!=0.0)  
      pitch = atan(-ax / sqrt(ay * ay + az * az)) * RAD_TO_DEG;
    else
      pitch = atan(-ax /0.00001 + sqrt(ay * ay + az * az)) * RAD_TO_DEG;

    double gyroXrate = gx / 131.0; // Convert to deg/s
    double gyroYrate = gy / 131.0; // Convert to deg/s

    if ((roll < -90 && kalAngleX > 90) || (roll > 90 && kalAngleX < -90)) {
      kalmanX.setAngle(roll);
      compAngleX = roll;
      kalAngleX = roll;
      gyroXangle = roll;
    } 
    else
      kalAngleX = kalmanX.getAngle(roll, gyroXrate, dt);
    if (abs(kalAngleX) > 90)
      gyroYrate = -gyroYrate; // Invert rate, so it fits the restriced accelerometer reading
    kalAngleY = kalmanY.getAngle(pitch, gyroYrate, dt);

    gyroXangle += gyroXrate * dt; // Calculate gyro angle without any filter
    gyroYangle += gyroYrate * dt;
    //gyroXangle += kalmanX.getRate() * dt; // Calculate gyro angle using the unbiased rate
    //gyroYangle += kalmanY.getRate() * dt;

    compAngleX = 0.93 * (compAngleX + gyroXrate * dt) + 0.07 * roll; // Calculate the angle using a Complimentary filter
    compAngleY = 0.93 * (compAngleY + gyroYrate * dt) + 0.07 * pitch;

    // Reset the gyro angle when it has drifted too much
    if (gyroXangle < -180 || gyroXangle > 180)
      gyroXangle = kalAngleX;
    if (gyroYangle < -180 || gyroYangle > 180)
      gyroYangle = kalAngleY;

    gyr[0]= kalAngleX;
    gyr[1]= kalAngleY;

    Serial.println(kalAngleY);

    delay(2);
    return gyr;
}

void loop() {
  if(millis()<30000){
    double h = dist();
    if(h < (hreq - tol) || h > (hreq + tol)){
        pos =computePID(h);
    }

    bri_(pos);
    bro_(pos);
    bli_(pos);
    blo_(pos);
    delay(500);

    double *g= get_rp();
    double roll=g[1];
    double pitch=g[0];

    String data = String(hreq)+" "+String(roll)+" "+String(pitch)+" "+String(analogRead(c1))+" "+String(analogRead(c2))+" "+String(analogRead(c3))+" "+String(analogRead(c4))+" "+String(pos)+" "+String(dist());
    client.println(data);
    Serial.println(data);
  }
  else{
    double *g= get_rp();
    double roll=g[1];
    double pitch=g[0];

      if(roll < (rollreq - tol) || roll > (rollreq + tol)){
          if(pos==180 && abs(roll)<abs(rollreq)){
            baseangle=180-computePID2(roll);
            Serial.println("max reach");
            Serial.println(baseangle);
          }
          else{
            pos =computePID2(roll);
            Serial.println("not max");
            Serial.println(pos);
          }
      }

    if(rollreq>=0){
      bli_(baseangle);
      blo_(pos);
      bri_(baseangle);
      bro_(pos);
    }
    else{
      bri_(pos);
      bro_(baseangle);
      bli_(pos);
      blo_(baseangle);
    }
    String data = String(hreq)+" "+String(roll)+" "+String(pitch)+" "+String(analogRead(c1))+" "+String(analogRead(c2))+" "+String(analogRead(c3))+" "+String(analogRead(c4))+" "+String(pos)+" "+String(dist());
    client.println(data);
  }

  delay(50);
    
}