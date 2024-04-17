
const int  flc = 13; //bri
const int  frc = 4;  //bro
const int  brc = 27; //bli
const int  blc = 33;

///////////////////////Ultrasonic////////////Z//////////////////////////////////////////////////////
const int trigPin = 14;
const int echoPin = 12;
const uint8_t ftrigPin = 32;
const uint8_t fechoPin = 23;

void servo_setup(){
  ESP32PWM::allocateTimer(0);
  ESP32PWM::allocateTimer(1);
  ESP32PWM::allocateTimer(2);
  ESP32PWM::allocateTimer(3);
  fl.setPeriodHertz(50);// Standard 50hz servo
  bl.setPeriodHertz(50);
  fr.setPeriodHertz(50);
  br.setPeriodHertz(50);
  // myservo.attach(26, 544, 2400);   // attaches the servo on pin 18 to the servo object
                                         // using SG90 servo min/max of 500us and 2400us
                                         // for MG995 large servo, use 1000us and 2000us,
                                         // which are the defaults, so this line could be
                                         // "myservo.attach(servoPin);"

  br.attach(19, 544, 2400);
  fr.attach(25, 544, 2400);
  bl.attach(18, 544, 2400);
  fl.attach(26, 544, 2400);
}

void pin_setup(){
  pinMode(trigPin, OUTPUT);
  pinMode(echoPin, INPUT);
  pinMode(ftrigPin, OUTPUT);
  pinMode(fechoPin, INPUT);

  pinMode(frc, INPUT);
  pinMode(flc, INPUT);
  pinMode(brc, INPUT);
  pinMode(blc, INPUT);
}


