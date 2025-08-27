// Teensy 4.1 LED Blink 예제
// 내장 LED는 13번 핀에 연결되어 있음

void setup() {
  pinMode(13, OUTPUT);   // LED 핀을 출력 모드로 설정
}

void loop() {
  digitalWrite(13, HIGH);  // LED 켜기
  delay(1000);             // 1초 대기 (1000ms)
  
  digitalWrite(13, LOW);   // LED 끄기
  delay(1000);             // 1초 대기
}
