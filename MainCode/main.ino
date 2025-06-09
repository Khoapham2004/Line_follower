// Khai báo chân cảm biến IR
const int sensorPins[5] = {A6, A3, A2, A1, A0};
int sensorValues[5] = {0};
int threshold = 550; // Ngưỡng phát hiện line

// Khai báo chân điều khiển motor (TB6612FNG)
const int in1 = 7;
const int in2 = 8;
const int ena = 5; // PWM trái

const int in3 = 4;
const int in4 = A7; // không dùng được, nên nối 5V trực tiếp
const int enb = 6; // PWM phải

// Biến PID
float error = 0, prev_error = 0;
float p = 0, i_term = 0, d = 0, pid = 0;
float KP = 15.0;
float KI = 0.0;
float KD = 60.0;
int pid_phai;
int pid_trai;
// Tốc độ cơ bản
int giatribandau = 180;
unsigned long startTime = 0;
bool conditionMet = false;
unsigned long conditionStart = 0;

void setup() {
  Serial.begin(9600);

  // Motor pins
  pinMode(in1, OUTPUT);
  pinMode(in2, OUTPUT);
  pinMode(ena, OUTPUT);

  pinMode(in3, OUTPUT);
  pinMode(in4, OUTPUT);
  pinMode(enb, OUTPUT);
}

void loop() {
  readSensors();
  calculatePID();
  controlMotors();
}

void readSensors() {
  for (int i = 0; i < 5; i++) {
    sensorValues[i] = analogRead(sensorPins[i]);
  }

  if (sensorValues[0] > threshold && sensorValues[1] > threshold && sensorValues[2] > threshold && sensorValues[3] > threshold && sensorValues[4] < threshold)
    error = 10;
  else if (sensorValues[0] > threshold && sensorValues[1] > threshold && sensorValues[2] > threshold && sensorValues[3] < threshold && sensorValues[4] < threshold)
    error = 8;
  else if (sensorValues[0] > threshold && sensorValues[1] > threshold && sensorValues[2] < threshold && sensorValues[3] < threshold && sensorValues[4] < threshold)
    error = 6;
  else if (sensorValues[0] > threshold && sensorValues[1] > threshold && sensorValues[2] < threshold && sensorValues[3] < threshold && sensorValues[4] > threshold)
    error = 4;
  else if (sensorValues[0] > threshold && sensorValues[1] > threshold && sensorValues[2] > threshold && sensorValues[3] < threshold && sensorValues[4] > threshold)
    error = 2;
  else if (sensorValues[0] > threshold && sensorValues[1] > threshold && sensorValues[2] < threshold && sensorValues[3] > threshold && sensorValues[4] > threshold)
    error = 0;
  else if (sensorValues[0] > threshold && sensorValues[1] < threshold && sensorValues[2] > threshold && sensorValues[3] > threshold && sensorValues[4] > threshold)
    error = -2;
  else if (sensorValues[0] > threshold && sensorValues[1] < threshold && sensorValues[2] < threshold && sensorValues[3] > threshold && sensorValues[4] > threshold)
    error = -4;
  else if (sensorValues[0] < threshold && sensorValues[1] < threshold && sensorValues[2] < threshold && sensorValues[3] > threshold && sensorValues[4] > threshold)
    error = -6;
  else if (sensorValues[0] < threshold && sensorValues[1] < threshold && sensorValues[2] > threshold && sensorValues[3] > threshold && sensorValues[4] > threshold)
    error = -8;
  else if (sensorValues[0] < threshold && sensorValues[1] > threshold && sensorValues[2] > threshold && sensorValues[3] > threshold && sensorValues[4] > threshold)
    error = -10;
}

void calculatePID() {
  p = error;
  i_term += error;
  d = error - prev_error;
  pid = KP * p + KI * i_term + KD * d;
  prev_error = error;

  pid_phai = giatribandau - pid;
  pid_trai = giatribandau + pid;
  // Giới hạn trong khoảng PWM an toàn
  pid_phai = constrain(pid_phai, 0, 255);
  pid_trai = constrain(pid_trai, 0, 255);
}

void controlMotors() {
  // Điều khiển chiều quay: tiến thẳng
  digitalWrite(in1, HIGH);
  digitalWrite(in2, LOW);  // bánh trái tiến
  digitalWrite(in3, LOW);
  digitalWrite(in4, HIGH); // bánh phải tiến

  // Gửi tín hiệu PWM đến motor
  analogWrite(ena, pid_trai/1.1 );
  analogWrite(enb, pid_phai/1.1 );

  // In ra tốc độ bánh xe
  Serial.print("Toc do banh trai: ");
  Serial.print(pid_trai );
  Serial.print(" | Toc do banh phai: ");
  Serial.println(pid_phai );
}
