#include "../../platformio/src/setup.cpp"

#include <Arduino.h>
#include <rcl/rcl.h>
#include <rclc/rclc.h>
#include <rclc/executor.h>
#include <rcl/error_handling.h>
#include <rmw_microros/rmw_microros.h>
#include <micro_ros_platformio.h>

#include <std_msgs/msg/float32.h>              // /stepper_cmd (deg)
#include <std_msgs/msg/float32_multi_array.h>  // /encoder_data [deg, deg/s, tx_time_sec]

// ===================== 매크로/유틸 =====================
#define RCCHECK(fn) { rcl_ret_t rc = fn; if (rc != RCL_RET_OK) { return false; } }

#define EXECUTE_EVERY_N_MS(MS, X) do { \
  static int64_t _t = -1; \
  if (_t == -1) _t = uxr_millis(); \
  if ((int32_t)(uxr_millis() - _t) > (MS)) { X; _t = uxr_millis(); } \
} while (0)

// ===================== 핀 설정 =====================
// L298N (Teensy 4.1)
const int ENA = 9;    // PWM 가능
const int ENB = 10;   // PWM 가능
const int IN1 = 2;
const int IN2 = 3;
const int IN3 = 4;
const int IN4 = 5;

// Encoder (Teensy 4.1의 인터럽트 가능한 핀 사용; 모터 핀과 충돌 금지)
const int ENCODER_A = 6;
const int ENCODER_B = 7;

// ===================== 모터 파라미터 =====================
const int   STEPS_PER_REV = 200;     // OT-42HS40-005B : 1.8° → 200 steps/rev
const bool  HOLD_TORQUE   = true;    // 회전 종료 후 코일 전류 유지(고정)
const int   EN_DUTY       = 170;     // 0~255 PWM 듀티 (전류/발열 튜닝)
const uint32_t STEP_DELAY_US = 3000; // 스텝 간격(마이크로초). 너무 작으면 스텝미스

// ===================== Encoder 설정 =====================
#define CPR 1024
#define QUAD_FACTOR 4
#define DEG_PER_COUNT (360.0f / (CPR * QUAD_FACTOR))

volatile long encoderCount = 0;
unsigned long prevTime_us = 0;
float prevDeg = 0.0f;

// ===================== micro-ROS 엔티티 =====================
rclc_support_t support;
rcl_node_t node;
rcl_timer_t timer;                  // encoder publish timer
rclc_executor_t executor;
rcl_allocator_t allocator;

rcl_publisher_t pub_encoder;
std_msgs__msg__Float32MultiArray msg_encoder;
float encoder_payload[3];           // [0]:deg, [1]:deg/s, [2]:tx_time_sec

rcl_subscription_t sub_stepper;     // /stepper_cmd (deg)
std_msgs__msg__Float32 msg_cmd;

// ===================== Stepper 제어(논블로킹) =====================
volatile long steps_remaining = 0;   // 남은 스텝 수(>0이면 동작)
volatile bool step_dir_cw     = true;
volatile int  seq_index       = 0;   // 0~3 (full-step 2-coil on)
uint32_t last_step_time_us    = 0;

inline void coils_off() {
  digitalWrite(IN1, LOW); digitalWrite(IN2, LOW);
  digitalWrite(IN3, LOW); digitalWrite(IN4, LOW);
}

inline void step_sequence_write(int idx) {
  // CW 기준 시퀀스. CCW는 idx를 역방향으로 진행.
  switch (idx & 3) {
    case 0: // A+, B+
      digitalWrite(IN1, HIGH); digitalWrite(IN2, LOW);
      digitalWrite(IN3, HIGH); digitalWrite(IN4, LOW);
      break;
    case 1: // A-, B+
      digitalWrite(IN1, LOW);  digitalWrite(IN2, HIGH);
      digitalWrite(IN3, HIGH); digitalWrite(IN4, LOW);
      break;
    case 2: // A-, B-
      digitalWrite(IN1, LOW);  digitalWrite(IN2, HIGH);
      digitalWrite(IN3, LOW);  digitalWrite(IN4, HIGH);
      break;
    case 3: // A+, B-
      digitalWrite(IN1, HIGH); digitalWrite(IN2, LOW);
      digitalWrite(IN3, LOW);  digitalWrite(IN4, HIGH);
      break;
  }
}

// 논블로킹 한 스텝 처리: loop()에서 주기적으로 호출
void stepper_tick() {
  if (steps_remaining <= 0) {
    if (!HOLD_TORQUE) coils_off();
    return;
  }
  uint32_t now = micros();
  if (now - last_step_time_us < STEP_DELAY_US) return;

  // 다음 시퀀스 인덱스
  if (step_dir_cw) {
    seq_index = (seq_index + 1) & 3;
  } else {
    seq_index = (seq_index - 1) & 3;  // 음수 방지용 &3
  }
  step_sequence_write(seq_index);

  steps_remaining--;
  last_step_time_us = now;

  // 완료 시 처리
  if (steps_remaining <= 0 && !HOLD_TORQUE) {
    coils_off();
  }
}

inline long degrees_to_steps(float deg_abs) {
  return (long)((STEPS_PER_REV * deg_abs / 360.0f) + 0.5f);
}

// ===================== Encoder ISR =====================
void ISR_A() {
  bool A = digitalRead(ENCODER_A);
  bool B = digitalRead(ENCODER_B);
  encoderCount += (A == B) ? 1 : -1;
}
void ISR_B() {
  bool A = digitalRead(ENCODER_A);
  bool B = digitalRead(ENCODER_B);
  encoderCount += (A != B) ? 1 : -1;
}

// ===================== Timer 콜백(Encoder publish @100Hz) =====================
void timer_callback(rcl_timer_t* /*timer*/, int64_t /*last_call_time*/) {
  unsigned long now_us = micros();
  float dt = (now_us - prevTime_us) / 1.0e6f;
  if (dt <= 0.0f) { prevTime_us = now_us; return; }

  noInterrupts();
  long cnt = encoderCount;
  interrupts();

  float deg = cnt * DEG_PER_COUNT;
  float dps = (deg - prevDeg) / dt;

  prevDeg = deg;
  prevTime_us = now_us;

  encoder_payload[0] = deg;
  encoder_payload[1] = dps;
  encoder_payload[2] = now_us / 1.0e6f;

  msg_encoder.data.data = encoder_payload;
  msg_encoder.data.size = 3;
  msg_encoder.data.capacity = 3;

  rcl_publish(&pub_encoder, &msg_encoder, NULL);
}

// ===================== Subscriber 콜백(/stepper_cmd: deg) =====================
void sub_callback(const void* msgin) {
  const std_msgs__msg__Float32* m = (const std_msgs__msg__Float32*)msgin;
  float target_deg = m->data;

  bool cw = (target_deg >= 0.0f);
  long need = degrees_to_steps(fabsf(target_deg));
  if (need <= 0) return;

  // 새 명령 세팅 (누적이 아니라 명령 당 “정해진 각도만큼”)
  noInterrupts();
  step_dir_cw = cw;
  steps_remaining = need;
  interrupts();
}

// ===================== micro-ROS 엔티티 생성/해제 =====================
bool create_entities() {
  allocator = rcl_get_default_allocator();
  RCCHECK(rclc_support_init(&support, 0, NULL, &allocator));
  RCCHECK(rclc_node_init_default(&node, "robot_node", "", &support));

  // Publisher: /encoder_data
  RCCHECK(rclc_publisher_init_default(
    &pub_encoder, &node,
    ROSIDL_GET_MSG_TYPE_SUPPORT(std_msgs, msg, Float32MultiArray),
    "encoder_data"
  ));

  // Subscriber: /stepper_cmd
  RCCHECK(rclc_subscription_init_default(
    &sub_stepper, &node,
    ROSIDL_GET_MSG_TYPE_SUPPORT(std_msgs, msg, Float32),
    "stepper_cmd"
  ));

  // Timer: 100Hz
  RCCHECK(rclc_timer_init_default(
    &timer, &support, RCL_MS_TO_NS(10), timer_callback));

  // Executor: 타이머 1 + 서브스크립션 1
  executor = rclc_executor_get_zero_initialized_executor();
  RCCHECK(rclc_executor_init(&executor, &support.context, 2, &allocator));
  RCCHECK(rclc_executor_add_timer(&executor, &timer));
  RCCHECK(rclc_executor_add_subscription(
    &executor, &sub_stepper, &msg_cmd, &sub_callback, ON_NEW_DATA));

  // encoder msg 초기화
  msg_encoder.data.data = nullptr;
  msg_encoder.data.size = 0;
  msg_encoder.data.capacity = 0;

  return true;
}

void destroy_entities() {
  rcl_publisher_fini(&pub_encoder, &node);
  rcl_subscription_fini(&sub_stepper, &node);
  rcl_timer_fini(&timer);
  rclc_executor_fini(&executor);
  rcl_node_fini(&node);
  rclc_support_fini(&support);
}

// ===================== 상태 머신 =====================
enum State { WAITING_AGENT, AGENT_AVAILABLE, AGENT_CONNECTED, AGENT_DISCONNECTED };
State state;

void setup() {
  // micro-ROS 트랜스포트(시리얼 등)
  set_microros_transports();

  // 핀모드
  pinMode(ENA, OUTPUT); pinMode(ENB, OUTPUT);
  pinMode(IN1, OUTPUT); pinMode(IN2, OUTPUT);
  pinMode(IN3, OUTPUT); pinMode(IN4, OUTPUT);

  pinMode(ENCODER_A, INPUT_PULLUP);
  pinMode(ENCODER_B, INPUT_PULLUP);
  attachInterrupt(digitalPinToInterrupt(ENCODER_A), ISR_A, CHANGE);
  attachInterrupt(digitalPinToInterrupt(ENCODER_B), ISR_B, CHANGE);

  // PWM 해상도(UNO와 동일 8비트)
  analogWriteResolution(8);
  analogWrite(ENA, EN_DUTY);
  analogWrite(ENB, EN_DUTY);

  // 상태
  prevTime_us = micros();
  state = WAITING_AGENT;

  // 초기에 코일 풀고 시작하고 싶으면 주석 해제
  // coils_off();
}

void loop() {
  // 스텝모터 논블로킹 구동
  stepper_tick();

  // micro-ROS 에이전트 연결 상태머신
  switch (state) {
    case WAITING_AGENT:
      EXECUTE_EVERY_N_MS(500,
        state = (RMW_RET_OK == rmw_uros_ping_agent(100, 1)) ? AGENT_AVAILABLE : WAITING_AGENT;
      );
      break;

    case AGENT_AVAILABLE:
      state = create_entities() ? AGENT_CONNECTED : WAITING_AGENT;
      if (state == WAITING_AGENT) destroy_entities();
      break;

    case AGENT_CONNECTED:
      EXECUTE_EVERY_N_MS(200,
        state = (RMW_RET_OK == rmw_uros_ping_agent(100, 1)) ? AGENT_CONNECTED : AGENT_DISCONNECTED;
      );
      if (state == AGENT_CONNECTED) {
        // 타이머+서브스크립션 콜백 처리
        rclc_executor_spin_some(&executor, RCL_MS_TO_NS(2));
      }
      break;

    case AGENT_DISCONNECTED:
      destroy_entities();
      state = WAITING_AGENT;
      break;
  }
}
