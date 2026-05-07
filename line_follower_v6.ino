//--------IR pin-----------
#define ir1 PA0 // left most 
#define ir2 PA6
#define ir3 PA1
#define ir4 PA7
#define ir5 PA2 // center 
#define ir6 PB0
#define ir7 PA3
#define ir8 PB1
#define ir9 PA4 // right most

//---------MOTOR PIN--------------
#define ml1 PB7
#define ml2 PB6
#define mr1 PB9
#define mr2 PB8

// ---- HC-05 on Serial1 (PA9=TX, PA10=RX) ----
HardwareSerial BTSerial(PA10, PA9);

// Variables
float I = 0;
float previousError = 0;
float P, D, error;
float Pvalue, Ivalue, Dvalue;
float Kp = 14;
float Ki = 0;
float Kd = 18;

uint8_t multiP = 1;
uint8_t multiI = 1;
uint8_t multiD = 1;

int lsp, rsp;
int base_speed = 88;
int min_speed = 50;
int max_speed = 255;
int base_speed2 = base_speed * 1.5;

bool sharpTurn = false;

// ---- Analog sensor config ----
const float sensorPos[9] = {-60.0, -45.0, -30.0, -15.0, 0.0, 15.0, 30.0, 45.0, 60.0};
const int whiteValue = 4095;
const int noiseFloor = 20;

// ---- BT command buffer ----
String btBuffer = "";

void printParams(Stream &s) {
  s.print("Kp="); s.print(Kp);
  s.print(" Ki="); s.print(Ki);
  s.print(" Kd="); s.print(Kd);
  s.print(" base_speed="); s.print(base_speed);
  s.print(" min_speed="); s.print(min_speed);
  s.print(" max_speed="); s.println(max_speed);
}

// ---- Process one complete BT command string ----
void processCommand(String cmd) {
  cmd.trim();
  if (cmd.length() == 0) return;

  char type = cmd.charAt(0);
  float val  = cmd.substring(1).toFloat();

  switch (type) {
    case 'p': case 'P':
      Kp = val;
      BTSerial.print("Kp set to "); BTSerial.println(Kp);
      break;
    case 'i': case 'I':
      Ki = val;
      BTSerial.print("Ki set to "); BTSerial.println(Ki);
      break;
    case 'd': case 'D':
      Kd = val;
      BTSerial.print("Kd set to "); BTSerial.println(Kd);
      break;
    case 's': case 'S':
      base_speed  = (int)val;
      base_speed2 = base_speed * 1.5;
      BTSerial.print("base_speed set to "); BTSerial.println(base_speed);
      break;
    case 'n': case 'N':                          // min speed
      min_speed = (int)val;
      BTSerial.print("min_speed set to "); BTSerial.println(min_speed);
      break;
    case 'x': case 'X':                          // max speed
      max_speed = (int)val;
      BTSerial.print("max_speed set to "); BTSerial.println(max_speed);
      break;
    case '?':
      printParams(BTSerial);
      break;
    default:
      BTSerial.println("Unknown cmd. Use p/i/d/s/n/x<value> or ?");
      break;
  }
}

// ---- Non-blocking BT read, fires when newline received ----
void handleBluetooth() {
  while (BTSerial.available()) {
    char c = (char)BTSerial.read();
    if (c == '\n' || c == '\r') {
      if (btBuffer.length() > 0) {
        processCommand(btBuffer);
        btBuffer = "";
      }
    } else {
      btBuffer += c;
    }
  }
}

void setup() {
  pinMode(ml1, OUTPUT);
  pinMode(ml2, OUTPUT);
  pinMode(mr1, OUTPUT);
  pinMode(mr2, OUTPUT);

  analogReadResolution(12);
  analogWriteFrequency(20000);

  pinMode(ir1, INPUT);
  pinMode(ir2, INPUT);
  pinMode(ir3, INPUT);
  pinMode(ir4, INPUT);
  pinMode(ir5, INPUT);
  pinMode(ir6, INPUT);
  pinMode(ir7, INPUT);
  pinMode(ir8, INPUT);
  pinMode(ir9, INPUT);

  Serial.begin(115200);

  BTSerial.begin(9600);          // HC-05 default baud
  BTSerial.println("BT Ready. Commands: p/i/d/s/n/x<value> or ?");
  printParams(BTSerial);
}

void loop() {

  handleBluetooth();             // <-- check BT every loop, non-blocking

  // ---- Read raw analog values ----
  int raw[9];
  raw[0] = analogRead(ir1);
  raw[1] = analogRead(ir2);
  raw[2] = analogRead(ir3);
  raw[3] = analogRead(ir4);
  raw[4] = analogRead(ir5);
  raw[5] = analogRead(ir6);
  raw[6] = analogRead(ir7);
  raw[7] = analogRead(ir8);
  raw[8] = analogRead(ir9);

  // ---- Weighted centroid position ----
  float weightedSum = 0;
  float totalWeight  = 0;

  for (int i = 0; i < 9; i++) {
    float weight = whiteValue - raw[i];
    if (weight < noiseFloor) weight = 0;
    weightedSum += weight * sensorPos[i];
    totalWeight  += weight;
  }

  // ---- Read potentiometer for Kp ----
  // float kp_adc = analogRead(PA5);
  // Kp = map(kp_adc, 0, 4095, 0, 25);

  if (Serial.available() > 0) {
    char inputvalue = char(Serial.read());
    if (inputvalue == 'd') {
      Serial.print("weights: ");
      for (int i = 0; i < 9; i++) {
        Serial.print(whiteValue - raw[i]);
        Serial.print("  ");
      }
      Serial.print("| pos: ");
      Serial.print(totalWeight > 100 ? weightedSum / totalWeight : 999);
      Serial.print("  error: ");
      Serial.print(error);
      Serial.print("  totalW: ");
      Serial.println(totalWeight);
    }
  }

  // ---- All sensors on white = line lost ----
  if (totalWeight < 200) {
    delay(75);
    if (previousError > 0) {
      motor_drive(base_speed2/2, -base_speed2);
    } else {
      motor_drive(-base_speed2, base_speed2/2);
    }
    return;
  }

  // ---- All sensors on black = stop (junction) ----
  if (totalWeight > 3000) {
    delay(75);
    motor_drive(0, 0);
    return;
  }

  if (totalWeight > 1000 && abs(error) > 1000) {
    sharpTurn = true;
  }

  while (sharpTurn) {
    if (abs(error) > 1000) {
      if (error < 0) motor_drive(base_speed2, -base_speed2);
      else motor_drive(-base_speed2, base_speed2);
      return;
    } else if (abs(error) < 100){
      delay(1300);
      sharpTurn = false;
    }
  }

  // ---- PID ----
  error = -(weightedSum / totalWeight);

  P = error;
  I = I + error;
  D = error - previousError;
  Pvalue = (Kp / pow(10, multiP)) * P;
  Ivalue = (Ki / pow(10, multiI)) * I;
  Dvalue = (Kd / pow(10, multiD)) * D;

  float PIDvalue = Pvalue + Ivalue + Dvalue;
  previousError = error;

  lsp = base_speed + PIDvalue;
  rsp = base_speed - PIDvalue;

  if (lsp > max_speed)  lsp = max_speed;
  if (lsp < -max_speed) lsp = -max_speed;
  if (rsp > max_speed)  rsp = max_speed;
  if (rsp < -max_speed) rsp = -max_speed;

  if (lsp > 0 && lsp < min_speed) lsp = min_speed;
  if (rsp > 0 && rsp < min_speed) rsp = min_speed;

  motor_drive(lsp, rsp);
}

void motor_drive(float left, float right) {
  if (left > 0) {
    analogWrite(ml2, left);
    analogWrite(ml1, 0);
  } else {
    analogWrite(ml2, 0);
    analogWrite(ml1, -left);
  }
  if (right > 0) {
    analogWrite(mr2, right);
    analogWrite(mr1, 0);
  } else {
    analogWrite(mr2, 0);
    analogWrite(mr1, -right);
  }
}