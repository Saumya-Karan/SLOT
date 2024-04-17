#include <MPU6050.h>
#include <ESP32Servo.h>
//////////////////////////////////////////////////////////////////////////////////////////
Servo fl;  //16
Servo fr;  //25
Servo bl;  //18
Servo br;  //19

#if I2CDEV_IMPLEMENTATION == I2CDEV_ARDUINO_WIRE
#include <Wire.h>
#endif

MPU6050 accelgyro;
int16_t ax, ay, az;
int16_t gx, gy, gz;
Kalman kalmanX;  // Create the Kalman instances
Kalman kalmanY;
uint32_t timer;

double gyroXangle, gyroYangle;  // Angle calculate using the gyro only
double compAngleX, compAngleY;  // Calculated angle using a complementary filter
double kalAngleX, kalAngleY;    // Calculated angle using a Kalman filter
double gyr[2] = { 0, 0 };

unsigned long currentTime, previousTime;
double elapsedTime;
double errorp, errorr, errorh, errorc;
double lastErrorp, lastErrorr, lastErrorh, lastErrorc;
double input, output, setPoint;
double cumErrorp, rateErrorp, cumErrorr, rateErrorr, cumErrorh, rateErrorh, cumErrorc, rateErrorc;

////////////////////////////////PID constants/////////////////////////////////////////////
double kp = 0.05;
double ki = 0.07;
double kd = 3;

// double kph = 5;  //5
// double kih = 0.07;   //0.07
// double kdh = 3; //3

double kph = 10;  //10
double kih = 0.07;   //0.07
double kdh = 3; //3

double kpc = 100;  //10;
double kic = 0;     //0.07;
double kdc = 0;     //3;
int ul = 160, def = 90, rel = 70, ulc = 120, defc = 60, relc = 40;

void fl_(int pos) {
  fl.write(pos-20);  //215
  // Serial.print("fl:");
  // Serial.print(pos);
}

void fr_(int pos) {
  fr.write(pos);
  // Serial.print("fr:");
  // Serial.print(pos);
}

void bl_(int pos) {
  bl.write(pos);  //225
  // Serial.print("bl:");
  // Serial.print(pos);
}

void br_(int pos) {
  br.write(pos-23);
  // Serial.print("br:");
  // Serial.print(pos);
}

////////////////////////crawl function////////////////////
void backward(int pos) {
  fl_(ul);
  fr_(ul);
  bl_(def-rel);
  br_(ul);
  delay(1000);

  fl_(ul);
  fr_(ul);
  bl_(ul);
  br_(def-rel);
  delay(1000);
}
////////////////////////////////////////////////////
void forward(int pos) {
  bl_(ul);
  br_(ul);
  fl_(def-rel);
  fr_(ul);
  delay(1000);

  bl_(ul);
  br_(ul);
  fl_(ul);
  fr_(def-rel);
  delay(1000);
  
}
////////////////////////////////////////////////////
void right(int pos) {

  bl_(ul);
  fl_(ul);
  br_(def-rel);
  fr_(ul);
  delay(1000);

  bl_(ul);
  fl_(ul);
  br_(ul);
  fr_(def-rel);
  delay(1000);
}
////////////////////////////////////////////////////
void left(int pos) {
  br_(ul);
  fr_(ul);
  bl_(def-rel);
  fl_(ul);
  delay(1000);

  br_(ul);
  fr_(ul);
  bl_(ul);
  fl_(def-rel);
  delay(1000);
}

void right_turn(int pos) {
  fl_(ul);
  fr_(ul);
  br_(def-rel);
  bl_(def);
  delay(1000);

  fl_(ul);
  fr_(def);
  br_(ul-20);
  bl_(def-rel);
  delay(1000);

  fl_(ul);
  fr_(def-rel);
  br_(def);
  bl_(ul);
  delay(1000);
  
}

void left_turn(int pos) {
  // Serial.println("forward");
  fr_(ul);
  fl_(ul);
  bl_(def-rel);
  br_(def);
  // Serial.print("back right");
  delay(1000);

  fr_(ul);
  fl_(def);
  bl_(ul-20);
  br_(def-rel);
  //  Serial.print("front left");
  delay(1000);

  fr_(ul);
  fl_(def-rel);
  bl_(def);
  br_(ul);
  //  Serial.print("back left");
  delay(1000);

  
}
//17.5, 13.5
void left_45(int pos) {
  bl_(ul);
  br_(ul);
  fl_(def-rel);
  fr_(ul);
  delay(1000); //left leg

  bl_(ul);
  br_(ul);
  fl_(125);
  fr_(110); 
  delay(1000); 

  



  // bl_(ul);
  // br_(ul);
  // fl_(ul);
  // fr_(def-rel); 
  // delay(1000);  //right leg

  // bl_(ul);
  // br_(ul);
  // fl_(def-80);
  // fr_(ul);
  // delay(1000);
  
}

void crawl_forward(int pos) {
  // Serial.println("forward");
  fr_(defc);
  bl_(defc);
  fl_(ulc);
  br_(defc-relc);
  //  Serial.print("front left");
  delay(1000);

  br_(ulc);
  fl_(defc-relc);
  bl_(defc);
  fr_(defc);
  // Serial.print("back right");
  delay(1000);

  br_(defc);
  fl_(defc);
  bl_(defc-relc);
  fr_(ulc);  //offset for leg balance, accounting for asymmetry
  //  Serial.print("front right");
  delay(1000);

  fr_(defc-relc);
  bl_(ulc);
  fl_(defc);
  br_(defc);
  //  Serial.print("back left");
  delay(1000);
}

void crawl_backward(int pos) {
  // Serial.println("backward");
  br_(ulc);
  fl_(defc-relc);
  bl_(defc);
  fr_(defc);
  // Serial.print("back right");
  delay(1000);

  fr_(defc);
  bl_(defc);
  fl_(ulc);
  br_(defc-relc);
  //  Serial.print("front left");
  delay(1000);

  fr_(defc-relc);
  bl_(ulc);
  fl_(defc);
  br_(defc);
  //  Serial.print("back left");
  delay(1000);

  br_(defc);
  fl_(defc);
  bl_(defc-relc);
  fr_(ulc);  //offset for leg balance, accounting for asymmetry
  //  Serial.print("front right");
  delay(1000);

}


double pitchPID(double inp, int pitchreq) {
  currentTime = millis();                              //get current time
  elapsedTime = (double)(currentTime - previousTime);  //compute time elapsed from previous computation
  double Setpoint = abs(pitchreq);

  errorp = Setpoint - inp;                         // determine error
  cumErrorp += errorp * elapsedTime;                // compute integral
  rateErrorp = (errorp - lastErrorp) / elapsedTime;  // compute derivative

  double out = kp * errorp + ki * cumErrorp + kd * rateErrorp;  //PID output

  // Serial.print("  errorp: ");
  // Serial.print(errorp);
  lastErrorp = errorp;           //remember current error
  previousTime = currentTime;  //remember current time
  // Serial.print("  out:");
  // Serial.print(out);
  if (out < 0) {
    out = 0;
  } else if (out > 180) {
    out = 180;
  }
  return out;  //have function return the PID output
}

double rollPID(double inp, int rollreq) {
  currentTime = millis();                              //get current time
  elapsedTime = (double)(currentTime - previousTime);  //compute time elapsed from previous computation
  double Setpoint = abs(rollreq);

  errorr = Setpoint - inp;                         // determine error
  cumErrorr += errorr * elapsedTime;                // compute integral
  rateErrorr = (errorr - lastErrorr) / elapsedTime;  // compute derivative

  double out = kp * errorr + ki * cumErrorr + kd * rateErrorr;  //PID output

  // Serial.print("  errorr: ");
  // Serial.print(errorr);
  lastErrorr = errorr;           //remember current error
  previousTime = currentTime;  //remember current time
  // Serial.print("  out:");
  // Serial.print(out);
  if (out < 0) {
    out = 0;
  } else if (out > 180) {
    out = 180;
  }
  return out;  //have function return the PID output
}

double heightPID(double inp, int hreq) {
  currentTime = millis();                              //get current time
  elapsedTime = (double)(currentTime - previousTime);  //compute time elapsed from previous computation
  double Setpoint = abs(hreq);

  errorh = Setpoint - inp;                         // determine error
  cumErrorh += errorh * elapsedTime;                // compute integral
  rateErrorh = (errorh - lastErrorh) / elapsedTime;  // compute derivative

  double out = kph * errorh + kih * cumErrorh + kdh * rateErrorh;  //PID output

  // Serial.print("  cumErrorh: ");
  // Serial.print(cumErrorh);
  lastErrorh = errorh;           //remember current error
  previousTime = currentTime;  //remember current time
  // Serial.print("  out:");
  // Serial.print(out);
  if (out < 0) {
    out = 0;
  } else if (out > 180) {
    out = 180;
  }
  if (cumErrorh < 0) {
    cumErrorh = 0;
  }
 else if (cumErrorh > 3000) {
    cumErrorh = 3000;
  }
  return out;  //have function return the PID output
}

double crawlPID(double inp, int dreq) {
  currentTime = millis();                              //get current time
  elapsedTime = (double)(currentTime - previousTime);  //compute time elapsed from previous computation
  double Setpoint = abs(dreq);

  errorh = Setpoint - inp;                         // determine error
  cumErrorc += errorc * elapsedTime;                // compute integral
  rateErrorc = (errorc - lastErrorc) / elapsedTime;  // compute derivative

  double out = kpc * errorc + kic * cumErrorc + kdc * rateErrorc;  //PID output

  // Serial.print("  errorc: ");
  // Serial.print(errorc);
  lastErrorc = errorc;           //remember current error
  previousTime = currentTime;  //remember current time
  // Serial.print("  out:");
  // Serial.print(out);
  if (out < 0) {
    out = 0;
  } else if (out > 180) {
    out = 180;
  }
  return out;  //have function return the PID output
}

double* get_rp() {
  accelgyro.getMotion6(&ax, &ay, &az, &gx, &gy, &gz);

  double dt = (double)(micros() - timer) / 1000000;  // Calculate delta time
  timer = micros();
  double roll = 0.0;
  double pitch = 0.0;
  if (az != 0)
    roll = atan2(ay, az) * RAD_TO_DEG;
  else
    roll = atan2(ay, az + 0.0001) * RAD_TO_DEG;
  if (sqrt(ay * ay + az * az) != 0.0)
    pitch = atan(-ax / sqrt(ay * ay + az * az)) * RAD_TO_DEG;
  else
    pitch = atan(-ax / 0.00001 + sqrt(ay * ay + az * az)) * RAD_TO_DEG;

  double gyroXrate = gx / 131.0;  // Convert to deg/s
  double gyroYrate = gy / 131.0;  // Convert to deg/s

  if ((roll < -90 && kalAngleX > 90) || (roll > 90 && kalAngleX < -90)) {
    kalmanX.setAngle(roll);
    compAngleX = roll;
    kalAngleX = roll;
    gyroXangle = roll;
  } else
    kalAngleX = kalmanX.getAngle(roll, gyroXrate, dt);
  if (abs(kalAngleX) > 90)
    gyroYrate = -gyroYrate;  // Invert rate, so it fits the restriced accelerometer reading
  kalAngleY = kalmanY.getAngle(pitch, gyroYrate, dt);

  gyroXangle += gyroXrate * dt;  // Calculate gyro angle without any filter
  gyroYangle += gyroYrate * dt;
  //gyroXangle += kalmanX.getRate() * dt; // Calculate gyro angle using the unbiased rate
  //gyroYangle += kalmanY.getRate() * dt;

  compAngleX = 0.93 * (compAngleX + gyroXrate * dt) + 0.07 * roll;  // Calculate the angle using a Complimentary filter
  compAngleY = 0.93 * (compAngleY + gyroYrate * dt) + 0.07 * pitch;
  double yaw = atan2(gy, gx) * RAD_TO_DEG;
  // Reset the gyro angle when it has drifted too much
  if (gyroXangle < -180 || gyroXangle > 180)
    gyroXangle = kalAngleX;
  if (gyroYangle < -180 || gyroYangle > 180)
    gyroYangle = kalAngleY;

  gyr[0] = kalAngleX;
  gyr[1] = kalAngleY;
  // Serial.print("yaw:");
  // Serial.println(yaw);
  // Serial.print("kalAngleY: ");
  // Serial.print(kalAngleY);
  // Serial.print("kalAngleX: ");
  // Serial.print(kalAngleX);

  delay(2);
  return gyr;
}


void mpu_setup() {
// join I2C bus (I2Cdev library doesn't do this automatically)
#if I2CDEV_IMPLEMENTATION == I2CDEV_ARDUINO_WIRE
  Wire.begin();
#elif I2CDEV_IMPLEMENTATION == I2CDEV_BUILTIN_FASTWIRE
  Fastwire::setup(400, true);
#endif

  // initialize device
  // Serial.println("Initializing I2C devices...");
  accelgyro.initialize();

  // verify connection
  // Serial.println("Testing device connections...");
  Serial.println(accelgyro.testConnection() ? "MPU6050 connection successful" : "MPU6050 connection failed");

  accelgyro.getMotion6(&ax, &ay, &az, &gx, &gy, &gz);
  double roll = 0.0;
  double pitch = 0.0;
  if (az != 0)
    roll = atan2(ay, az) * RAD_TO_DEG;
  else
    roll = atan2(ay, az + 0.0001) * RAD_TO_DEG;
  if (sqrt(ay * ay + az * az) != 0.0)
    pitch = atan(-ax / sqrt(ay * ay + az * az)) * RAD_TO_DEG;
  else
    pitch = atan(-ax / 0.00001 + sqrt(ay * ay + az * az)) * RAD_TO_DEG;
  kalmanX.setAngle(roll);  // Set starting angle
  kalmanY.setAngle(pitch);
  gyroXangle = roll;
  gyroYangle = pitch;
  compAngleX = roll;
  compAngleY = pitch;

  timer = micros();
}