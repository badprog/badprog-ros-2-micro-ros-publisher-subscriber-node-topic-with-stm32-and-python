// https://github.com/badprog

// Misc library
#include <Arduino.h>
#include <IWatchdog.h>
#include <micro_ros_platformio.h>

// ROS 2 library
#include <rcl/rcl.h>
#include <rclc/executor.h>
#include <rclc/rclc.h>
#include <rmw_microros/rmw_microros.h>
#include <std_msgs/msg/bool.h>
#include <std_msgs/msg/int32.h>

// Native STM32 library
#include <stm32f3xx_hal.h>

// LED configuration (Port E for F3 Discovery)
#define LED_PORT GPIOE
#define LED_RED_NORTH GPIO_PIN_9    // Error / Waiting for the Agent
#define LED_ORANGE_NE GPIO_PIN_10   // Data received (Subscriber)
#define LED_GREEN_EAST GPIO_PIN_11  // Data sent (Publisher)

// Node and topic names
#define NODE_STM32 "badprog_node_stm32f3discovery"
#define TOPIC_COUNTER "badprog_topic_counter"
#define TOPIC_LED_STATE "badprog_topic_led_state"

// ROS 2 variables
rcl_publisher_t publisher;
std_msgs__msg__Int32 msg_pub;
rcl_subscription_t subscriber;
std_msgs__msg__Bool msg_sub;
rclc_executor_t executor;
rclc_support_t support;
rcl_allocator_t allocator;
rcl_node_t node;
rcl_timer_t timer;

// Global misc var
unsigned long last_time_blink = 0;
unsigned long last_time_ping = 0;

// Bpard state
enum states { BOARD_STATE_WAITING_AGENT, BOARD_STATE_AGENT_CONNECTED };
states BOARD_STATE = BOARD_STATE_WAITING_AGENT;

// ========================================================
// Handle error by blinking quickly the red LED (North).
// ========================================================
void error_loop() {
  while (1) {
    HAL_GPIO_TogglePin(LED_PORT, LED_RED_NORTH);
    HAL_Delay(100);
  }
}

// Macro executed only once to check the code of a return
// function (error or success).
#define RCCHECK(fn)                \
  do {                             \
    rcl_ret_t temp_rc = fn;        \
    if ((temp_rc != RCL_RET_OK)) { \
      error_loop();                \
    }                              \
  } while (0)

// ========================================================
// timer_callback
// Called every second by the timer.
// Blink the green LED (East) after each message sent.
// Increments the next data value if the message is published.
// ========================================================
void timer_callback(rcl_timer_t* timer, int64_t last_call_time) {
  (void)last_call_time;
  if (timer != NULL) {
    if (rcl_publish(&publisher, &msg_pub, NULL) == RCL_RET_OK) {
      msg_pub.data++;
      HAL_GPIO_TogglePin(LED_PORT, LED_GREEN_EAST);
    }
  }
}

// ========================================================
// subscription_callback
// Called when a message is coming from the PC.
// Change the state of the orange LED (North East) according
// to the boolean received on the topic.
// ========================================================
void subscription_callback(const void* msgin) {
  const std_msgs__msg__Bool* m = (const std_msgs__msg__Bool*)msgin;
  HAL_GPIO_WritePin(LED_PORT, LED_ORANGE_NE,
                    m->data ? GPIO_PIN_SET : GPIO_PIN_RESET);
}

// ===================================================
// init_stm32_gpio
// Init GPIO E in order to set the GPIO_InitTypeDef
// strcuture members.
// ===================================================
void init_stm32_gpio(void) {
  __HAL_RCC_GPIOE_CLK_ENABLE();  // Activate GPIO E
  GPIO_InitTypeDef GPIO_InitStruct = {0};
  GPIO_InitStruct.Pin = LED_RED_NORTH | LED_ORANGE_NE | LED_GREEN_EAST;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(LED_PORT, &GPIO_InitStruct);
}

// ===================================================
// init_rclc_base
// Create the allocator (memory manager).
// Init the support (context, clock, allocator).
// Init the node (context, pointer) for the STM32 board.
// ===================================================
bool init_rclc_base() {
  allocator = rcl_get_default_allocator();
  RCCHECK(rclc_support_init(&support, 0, NULL, &allocator));
  RCCHECK(rclc_node_init_default(&node, NODE_STM32, "", &support));
  return true;
}

// ===================================================
// init_rclc_topics
// Init a topic to send data (publisher).
// Init a topic to receive data (subscription).
// The message is typed for both directions.
// ===================================================
bool init_rclc_topics() {
  RCCHECK(rclc_publisher_init_default(
      &publisher, &node, ROSIDL_GET_MSG_TYPE_SUPPORT(std_msgs, msg, Int32),
      TOPIC_COUNTER));

  RCCHECK(rclc_subscription_init_default(
      &subscriber, &node, ROSIDL_GET_MSG_TYPE_SUPPORT(std_msgs, msg, Bool),
      TOPIC_LED_STATE));
  return true;
}

// ===================================================
// init_rclc_timers
// Init a timer with a callback.
// ===================================================
bool init_rclc_timers() {
  RCCHECK(rclc_timer_init_default(&timer, &support, RCL_MS_TO_NS(1000),
                                  timer_callback));
  return true;
}

// ===================================================
// init_rclc_executor
// The executor orchestrates the tasks and manages
// every event as a scheduler.
// The number of handles has to be specified in order
// to know how many "handles" it has to manage.
// ===================================================
bool init_rclc_executor() {
  const size_t number_of_handles = 2;  // Timer + Subscriber
  RCCHECK(rclc_executor_init(&executor, &support.context, number_of_handles,
                             &allocator));
  RCCHECK(rclc_executor_add_timer(&executor, &timer));  // Handle 1
  RCCHECK(rclc_executor_add_subscription(&executor, &subscriber, &msg_sub,
                                         &subscription_callback,
                                         ON_NEW_DATA));  // Handle 2
  return true;
}

// ===================================================
// init_rclc
// Init every entity from ROS 2 (Node, Topic (Sub and
// Pub), Timer.
// Give these entities to be executed by the executor.
// ===================================================
bool init_rclc_entities() {
  if (!init_rclc_base()) return false;
  if (!init_rclc_topics()) return false;
  if (!init_rclc_timers()) return false;
  if (!init_rclc_executor()) return false;
  return true;
}

// ===================================================
// setup
// Set the microcontroller hardware for our program.
// ===================================================
void setup() {
  init_stm32_gpio();
  IWatchdog.begin(4000000);  // Trigger the Watchdog after 4 s.
  SerialUSB.begin();
  set_microros_serial_transports(SerialUSB);
  msg_pub.data = 0;
}

// ===================================================
// loop
// BOARD_STATE_WAITING_AGENT
// BOARD_STATE_AGENT_CONNECTED
// ===================================================
void loop() {
  IWatchdog.reload();  // Reset the Watchdog

  switch (BOARD_STATE) {
    case BOARD_STATE_WAITING_AGENT:
      // Blink the red North LED slowly waiting for the Agent connection
      if ((millis() - last_time_blink) > 500) {  // Blink every 500 ms
        HAL_GPIO_TogglePin(LED_PORT, LED_RED_NORTH);
        last_time_blink = millis();
      }

      // The Agent is connected
      if (rmw_uros_ping_agent(100, 1) == RCL_RET_OK) {
        // Init OK
        if (init_rclc_entities()) {
          BOARD_STATE = BOARD_STATE_AGENT_CONNECTED;
          HAL_GPIO_WritePin(LED_PORT, LED_RED_NORTH,
                            GPIO_PIN_RESET);  // Turn off the red North LED
        }
      }
      break;

    case BOARD_STATE_AGENT_CONNECTED:
      // If the Agent is not connected anymore
      if ((millis() - last_time_ping) > 2000) {
        last_time_ping = millis();

        if (rmw_uros_ping_agent(100, 1) != RCL_RET_OK) {
          NVIC_SystemReset();  // Reset the board.
        }
      }

      // Execute tasks from the handles
      rclc_executor_spin_some(&executor, RCL_MS_TO_NS(100));
      break;
  }
}
