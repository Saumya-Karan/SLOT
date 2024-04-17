#include "KalmanFilter.h"
#include "BluetoothSerial.h"
#include "motion.h"
#include "setup.h"

void (*command[8])(int);

BluetoothSerial ESP_BT;
int incoming = 0;

const int MAX_HEIGHT = 10;
const double SOUND_VELOCITY = 0.034826;
const float tol = 0.5;
const float told = 0.2;
double distance = 0.0;
double duration = 0.0;
//////////////////////////////////////////////////////////////////////////////////////////Desired variable

double pitchreq = 0, rollreq = 0, hreq = 8, dreq = 15;

int posh = 7, posr = 0, posp = 0, posc= 20, value;
int baseangle = 0;
int button = 0;

double dist() {
  float distance, duration;
  digitalWrite(trigPin, LOW);
  delayMicroseconds(2);

  digitalWrite(trigPin, HIGH);
  delayMicroseconds(10);
  digitalWrite(trigPin, LOW);

  duration = pulseIn(echoPin, HIGH);

  distance = duration * SOUND_VELOCITY / 2;
  // if(distance<9.0 && distance >0.0){
  //   distance= distance - 10.0;
  // }
  return distance;
}

double fdist() {
  float distance, duration;
  digitalWrite(ftrigPin, LOW);
  delayMicroseconds(2);

  digitalWrite(ftrigPin, HIGH);
  delayMicroseconds(10);
  digitalWrite(ftrigPin, LOW);

  duration = pulseIn(fechoPin, HIGH);

  distance = duration * SOUND_VELOCITY / 2;
  return distance;
}

void setup() {
  Serial.begin(115200);
  ESP_BT.begin("FlexQuad");

  servo_setup();

  command[0] = forward;
  command[1] = left;
  command[2] = backward;
  command[3] = right;
  command[4] = left_turn;
  command[5] = right_turn;
  command[6] = crawl_forward;
  command[7] = crawl_backward;

  pin_setup();
  mpu_setup();

  baseangle = 160;
  fr_(baseangle);
  fl_(baseangle);
  bl_(baseangle);
  br_(baseangle);
  delay(2000);
}

void loop() {

  double* g = get_rp();
  double roll = g[0];
  double pitch = g[1];
  double height = dist();
  distance = fdist();
  if (height > 200) {
      height = 0;
    }
  if (ESP_BT.available()) {
    incoming = ESP_BT.read();  //Read what we receive
  }

  if ((incoming > 89) && (incoming < 121)) {  //////////////////////////pitch
    pitchreq = incoming - 90 - 15;
    // Serial.print("desired pitch: ");
    // Serial.print(pitchreq);

    if ((abs(pitch) < (abs(pitchreq) - tol)) || (abs(pitch) > (abs(pitchreq) + tol))) {
      if (posp == 180 && abs(pitch) < abs(pitchreq)) {
        baseangle = 180 - pitchPID(abs(pitch), pitchreq);
        // Serial.print("max reach");
        // Serial.print(baseangle);
      } else {
        posp = pitchPID(abs(pitch), pitchreq);
        // Serial.print("  not max: ");
        // Serial.print(posp);
      }
    }
    if (pitchreq >= 0) {
      bl_(baseangle);
      fl_(posp);
      br_(baseangle);
      fr_(posp);
    } else {
      bl_(posp);
      fl_(baseangle);
      br_(posp);
      fr_(baseangle);
    }
  }


  else if ((incoming > 120) && (incoming < 152)) {  //////////////////////roll
    rollreq = incoming - 121-15;
    // Serial.print("desired roll: ");
    // Serial.println(rollreq);

    if ((abs(roll) < (abs(rollreq) - tol)) || (abs(roll) > (abs(rollreq) + tol))) {
      if (posr == 180 && abs(roll) < abs(rollreq)) {
        baseangle = 180 - rollPID(abs(roll), rollreq);
        // Serial.print("max reach");
        // Serial.print(baseangle);
      } else {
        posr = rollPID(abs(roll), rollreq);
        // Serial.print("  not max: ");
        // Serial.print(posr);
      }
    }
    if (rollreq >= 0) {
      bl_(baseangle);
      br_(posr);
      fl_(baseangle);
      fr_(posr);
    } else {
      bl_(posr);
      br_(baseangle);
      fl_(posr);
      fr_(baseangle);
    }
  }


  else if ((incoming > 151) && (incoming < 163)) {  //////////////////////////height
    hreq = incoming - 152;  //112
    // Serial.print("desired height: ");
    // Serial.println(hreq);
    
    if (height < (hreq - tol) || height > (hreq + tol)) {
      posh = heightPID(height, hreq);
    }
    fl_(posh);
    fr_(posh);
    bl_(posh);
    br_(posh);
  }

  else if(incoming<89) {  /////////////////////////////////////////////////////crawl
    // if(distance < (dreq - told) || h > (dreq + told)){
    //   posc =computePID(distance);
    // }
    if (value = incoming % 10) {
      button = floor(incoming / 10);
      command[button - 1](posc);
    }
  }

  else{
     if(distance <25.00){
      fl_(160);
      fr_(160);
      bl_(160);
      br_(160);

     }
  }
  // String data = String(hreq)+" "+String(roll)+" "+String(pitch)+" "+String(analogRead(c1))+" "+String(analogRead(c2))+" "+String(analogRead(c3))+" "+String(analogRead(c4))+" "+String(pos)+" "+String(dist());
  // Serial.print("desired height: ");
  // Serial.println(hreq);
  // Serial.print("  frc: ");
  // Serial.print(analogRead(frc));  
  // Serial.print("  flc: ");
  // Serial.print(analogRead(flc));
  // Serial.print("  brc ");
  // Serial.print(analogRead(brc));
  // Serial.print("  blc ");
  // Serial.print(analogRead(blc));
  // Serial.print("  posh: ");
  // Serial.print(posh);
  // Serial.print("  pitch: ");
  // Serial.print(pitch);
  // Serial.print("  roll: ");
  // Serial.print(roll);
  // Serial.print("  height: ");
  // Serial.println(height);
  // Serial.print("  distance: ");
  // Serial.print(distance);
  // Serial.println("");
}
