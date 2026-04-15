#include <Arduino.h>
#include <micro_ros_platformio.h>

#include <rcl/rcl.h>
#include <rclc/rclc.h>
#include <rclc/executor.h>

#include <std_msgs/msg/int32.h>

// map PBs to GPIO pins
#define PB1 1   // Joint 1 - Base
#define PB2 2   // Joint 2 - Shoulder
#define PB3 3   // Joint 3 - Elbow
#define PB4 4   // Joint 4 - Wrist 1
#define PB5 5   // Joint 5 - Wrist 2
#define PB6 6   // Joint 6 - Wrist 3

// micro-ROS objects
rcl_publisher_t publisher;
rclc_support_t support;
rcl_allocator_t allocator;
rcl_node_t node;
rclc_executor_t executor;
std_msgs__msg__Int32 msg;


// Button states
// HIGH = not pressed
// LOW  = pressed
bool last_pb1 = HIGH;
bool last_pb2 = HIGH;
bool last_pb3 = HIGH;
bool last_pb4 = HIGH;
bool last_pb5 = HIGH;
bool last_pb6 = HIGH;

// Simple debounce timing
unsigned long last_press_time = 0;
const unsigned long debounce_ms = 150;

void publish_joint(int joint_number)
{
  msg.data = joint_number;
  rcl_publish(&publisher, &msg, NULL);

  Serial.print("Published joint: ");
  Serial.println(joint_number);
}

void setup()
{
  Serial.begin(115200);
  delay(2000);

  // micro-ROS over serial
  set_microros_serial_transports(Serial);
  delay(2000);

  // Pushbuttons use internal pull-ups
  pinMode(PB1, INPUT_PULLUP);
  pinMode(PB2, INPUT_PULLUP);
  pinMode(PB3, INPUT_PULLUP);
  pinMode(PB4, INPUT_PULLUP);
  pinMode(PB5, INPUT_PULLUP);
  pinMode(PB6, INPUT_PULLUP);

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

  Serial.println("ESP32 micro-ROS joint button publisher started");
}

void loop()
{
  bool current_pb1 = digitalRead(PB1);
  bool current_pb2 = digitalRead(PB2);
  bool current_pb3 = digitalRead(PB3);
  bool current_pb4 = digitalRead(PB4);
  bool current_pb5 = digitalRead(PB5);
  bool current_pb6 = digitalRead(PB6);

  unsigned long now = millis();

  if ((now - last_press_time) > debounce_ms)
  {
    if (last_pb1 == HIGH && current_pb1 == LOW) {
      publish_joint(1);
      last_press_time = now;
    }
    else if (last_pb2 == HIGH && current_pb2 == LOW) {
      publish_joint(2);
      last_press_time = now;
    }
    else if (last_pb3 == HIGH && current_pb3 == LOW) {
      publish_joint(3);
      last_press_time = now;
    }
    else if (last_pb4 == HIGH && current_pb4 == LOW) {
      publish_joint(4);
      last_press_time = now;
    }
    else if (last_pb5 == HIGH && current_pb5 == LOW) {
      publish_joint(5);
      last_press_time = now;
    }
    else if (last_pb6 == HIGH && current_pb6 == LOW) {
      publish_joint(6);
      last_press_time = now;
    }
  }

  last_pb1 = current_pb1;
  last_pb2 = current_pb2;
  last_pb3 = current_pb3;
  last_pb4 = current_pb4;
  last_pb5 = current_pb5;
  last_pb6 = current_pb6;

  rclc_executor_spin_some(&executor, RCL_MS_TO_NS(50));
  delay(20);
}