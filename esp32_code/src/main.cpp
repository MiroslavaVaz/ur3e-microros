#include <Arduino.h>
#include <micro_ros_platformio.h>
#include <rcl/rcl.h>
#include <rclc/rclc.h>
#include <rclc/executor.h>
#include <std_msgs/msg/int32.h>

// GPIO pins for PBs
#define PB1 7    // Joint 1 - shoulder_pan
#define PB2 6    // Joint 2 - shoulder_lift
#define PB3 10   // Joint 3 - elbow
#define PB4 5    // Joint 4 - wrist_1
#define PB5 4    // Joint 5 - wrist_2
#define PB6 8    // Joint 6 - wrist_3
#define PB7 3    // Short press = direction toggle | Long press = EXECUTE

// Long press threshold (milliseconds)
#define LONG_PRESS_MS 3000

rcl_publisher_t publisher;
rclc_support_t support;
rcl_allocator_t allocator;
rcl_node_t node;
rclc_executor_t executor;
std_msgs__msg__Int32 msg;

bool last_pb1 = HIGH;
bool last_pb2 = HIGH;
bool last_pb3 = HIGH;
bool last_pb4 = HIGH;
bool last_pb5 = HIGH;
bool last_pb6 = HIGH;
bool last_pb7 = HIGH;

// Debounce
unsigned long last_press_time = 0;
const unsigned long debounce_ms = 150;

// PB7 long press tracking
unsigned long pb7_press_start = 0;   // when PB7 was first pressed down
bool pb7_long_fired = false;         // prevents repeated firing while held

void publish_value(int value)
{
  msg.data = value;
  rcl_publish(&publisher, &msg, NULL);
}

void setup()
{
  Serial.begin(115200);
  set_microros_serial_transports(Serial);
  delay(2000);

  pinMode(PB1, INPUT_PULLUP);
  pinMode(PB2, INPUT_PULLUP);
  pinMode(PB3, INPUT_PULLUP);
  pinMode(PB4, INPUT_PULLUP);
  pinMode(PB5, INPUT_PULLUP);
  pinMode(PB6, INPUT_PULLUP);
  pinMode(PB7, INPUT_PULLUP);

  allocator = rcl_get_default_allocator();
  rclc_support_init(&support, 0, NULL, &allocator);
  rclc_node_init_default(&node, "esp32_joint_buttons", "", &support);
  rclc_publisher_init_default(
    &publisher,
    &node,
    ROSIDL_GET_MSG_TYPE_SUPPORT(std_msgs, msg, Int32),
    "/joint_button"
  );
  rclc_executor_init(&executor, &support.context, 1, &allocator);
}

void loop()
{
  bool current_pb1 = digitalRead(PB1);
  bool current_pb2 = digitalRead(PB2);
  bool current_pb3 = digitalRead(PB3);
  bool current_pb4 = digitalRead(PB4);
  bool current_pb5 = digitalRead(PB5);
  bool current_pb6 = digitalRead(PB6);
  bool current_pb7 = digitalRead(PB7);

  unsigned long now = millis();

  // ── PB1–PB6: normal debounced press ───────────────────────────────────────
  if ((now - last_press_time) > debounce_ms)
  {
    if      (last_pb1 == HIGH && current_pb1 == LOW) { publish_value(1); last_press_time = now; }
    else if (last_pb2 == HIGH && current_pb2 == LOW) { publish_value(2); last_press_time = now; }
    else if (last_pb3 == HIGH && current_pb3 == LOW) { publish_value(3); last_press_time = now; }
    else if (last_pb4 == HIGH && current_pb4 == LOW) { publish_value(4); last_press_time = now; }
    else if (last_pb5 == HIGH && current_pb5 == LOW) { publish_value(5); last_press_time = now; }
    else if (last_pb6 == HIGH && current_pb6 == LOW) { publish_value(6); last_press_time = now; }
  }

  // ── PB7: long press vs short press ────────────────────────────────────────

  // PB7 just pressed down — start timer
  if (last_pb7 == HIGH && current_pb7 == LOW)
  {
    pb7_press_start = now;
    pb7_long_fired = false;
  }

  // PB7 held down — check if long press threshold reached
  if (current_pb7 == LOW && !pb7_long_fired)
  {
    if ((now - pb7_press_start) >= LONG_PRESS_MS)
    {
      publish_value(9);        // 9 = EXECUTE all joints
      pb7_long_fired = true;   // don't fire again until released and re-pressed
    }
  }

  // PB7 released — decide if it was a short press
  if (last_pb7 == LOW && current_pb7 == HIGH)
  {
    if (!pb7_long_fired)
    {
      // Was a short press — direction toggle
      publish_value(0);
    }
    // Reset for next press
    pb7_long_fired = false;
    pb7_press_start = 0;
  }

  last_pb1 = current_pb1;
  last_pb2 = current_pb2;
  last_pb3 = current_pb3;
  last_pb4 = current_pb4;
  last_pb5 = current_pb5;
  last_pb6 = current_pb6;
  last_pb7 = current_pb7;

  rclc_executor_spin_some(&executor, RCL_MS_TO_NS(50));
  delay(20);
}