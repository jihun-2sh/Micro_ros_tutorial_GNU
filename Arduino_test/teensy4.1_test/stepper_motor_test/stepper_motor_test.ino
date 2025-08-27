// Arduino UNO + L298N + OT-42HS40-005B Stepper
// ENA/ENB 핀도 사용, 3초 정방향 / 3초 역방향 반복

int ENA = 9;   // L298N ENA
int ENB = 10;   // L298N ENB
int IN1 = 2;
int IN2 = 3;
int IN3 = 4;
int IN4 = 5;

bool forwardDir = true;
unsigned long lastToggleTime = 0;

// 정방향 스텝
void stepForward() {
  digitalWrite(IN1, HIGH); digitalWrite(IN2, LOW);
  digitalWrite(IN3, HIGH); digitalWrite(IN4, LOW);
  delay(10);

  digitalWrite(IN1, LOW); digitalWrite(IN2, HIGH);
  digitalWrite(IN3, HIGH); digitalWrite(IN4, LOW);
  delay(10);

  digitalWrite(IN1, LOW); digitalWrite(IN2, HIGH);
  digitalWrite(IN3, LOW);  digitalWrite(IN4, HIGH);
  delay(10);

  digitalWrite(IN1, HIGH); digitalWrite(IN2, LOW);
  digitalWrite(IN3, LOW);  digitalWrite(IN4, HIGH);
  delay(10);
}

// 역방향 스텝
void stepBackward() {
  digitalWrite(IN1, HIGH); digitalWrite(IN2, LOW);
  digitalWrite(IN3, LOW);  digitalWrite(IN4, HIGH);
  delay(10);

  digitalWrite(IN1, LOW); digitalWrite(IN2, HIGH);
  digitalWrite(IN3, LOW);  digitalWrite(IN4, HIGH);
  delay(10);

  digitalWrite(IN1, LOW); digitalWrite(IN2, HIGH);
  digitalWrite(IN3, HIGH); digitalWrite(IN4, LOW);
  delay(10);

  digitalWrite(IN1, HIGH); digitalWrite(IN2, LOW);
  digitalWrite(IN3, HIGH); digitalWrite(IN4, LOW);
  delay(10);
}

void setup() {
  pinMode(ENA, OUTPUT);
  pinMode(ENB, OUTPUT);
  pinMode(IN1, OUTPUT);
  pinMode(IN2, OUTPUT);
  pinMode(IN3, OUTPUT);
  pinMode(IN4, OUTPUT);

  // 모터 활성화
  digitalWrite(ENA, HIGH);
  digitalWrite(ENB, HIGH);

  lastToggleTime = millis();
}

void loop() {
  // 3초마다 방향 전환
  if (millis() - lastToggleTime >= 3000) {
    forwardDir = !forwardDir;
    lastToggleTime = millis();
  }

  // 방향에 따라 스텝 실행
  if (forwardDir) {
    stepForward();
  } else {
    stepBackward();
  }
}
