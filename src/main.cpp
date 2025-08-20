#include <Arduino.h>
#include <WiFi.h>

// Sensor pins for IR vehicle detection
#define IR1_PIN 34
#define IR2_PIN 35
#define IR3_PIN 22
#define IR4_PIN 23

// Traffic light control pins for each lane
#define RED_PIN1 19
#define YELLOW_PIN1 18
#define GREEN_PIN1 5

#define RED_PIN2 17
#define YELLOW_PIN2 16
#define GREEN_PIN2 4

#define RED_PIN3 27
#define YELLOW_PIN3 26
#define GREEN_PIN3 25

#define RED_PIN4 13
#define YELLOW_PIN4 15
#define GREEN_PIN4 14

#define SERIAL_BAUD 115200 // Serial communication baud rate

// Enum for traffic light states
enum TrafficState
{
  RED,         // Red light
  RED_YELLOW,  // Red and yellow together (prepare to go)
  GREEN,       // Green light
  YELLOW       // Yellow light (prepare to stop)
};

// Structure to hold the duration for each state (in milliseconds)
struct StateDuration
{
  int redDuration;
  int redYellowDuration;
  int yellowDuration;
  int greenDuration;
};

// Structure representing a traffic light for a lane
struct TrafficLight
{
  int redPin;
  int yellowPin;
  int greenPin;
  int laneId;
  TrafficState state;
  StateDuration duration;
};

// Array of traffic lights for each lane, with initial durations and state
TrafficLight trafficLights[4] = {
    {RED_PIN1, YELLOW_PIN1, GREEN_PIN1, 1, RED, {7000, 2000, 4000, 5000}},
    {RED_PIN2, YELLOW_PIN2, GREEN_PIN2, 2, RED, {7000, 2000, 4000, 5000}},
    {RED_PIN3, YELLOW_PIN3, GREEN_PIN3, 3, RED, {7000, 2000, 4000, 5000}},
    {RED_PIN4, YELLOW_PIN4, GREEN_PIN4, 4, RED, {7000, 2000, 4000, 5000}}};

int activeLane = 1; // Currently active lane (for logging)
int numLanes = 4;   // Total number of lanes
int laneIndex = 0;  // Index of the current lane being controlled

// Function declarations for FreeRTOS tasks and light control
void trafficControlTask(void *pvParameters);
void cloudLoggingTask(void *pvParameters);
void setTrafficLights(int lane_id);

void setup()
{
  // Set IR sensor pins as input
  pinMode(IR1_PIN, INPUT);
  pinMode(IR2_PIN, INPUT);
  pinMode(IR3_PIN, INPUT);
  pinMode(IR4_PIN, INPUT);

  // Array of all traffic light pins for initialization
  int trafficPins[12] = {RED_PIN1, RED_PIN2, RED_PIN3, RED_PIN4,
                         YELLOW_PIN1, YELLOW_PIN2, YELLOW_PIN3, YELLOW_PIN4,
                         GREEN_PIN1, GREEN_PIN2, GREEN_PIN3, GREEN_PIN4};

  // Set all traffic light pins as output and turn them off initially
  for (int i = 0; i < 12; i++)
  {
    pinMode(trafficPins[i], OUTPUT);
    digitalWrite(trafficPins[i], LOW);
  }

  // Initialize serial communication for debugging/logging
  Serial.begin(SERIAL_BAUD);

  // Create the traffic control task on core 1
  xTaskCreatePinnedToCore(
      trafficControlTask, "Traffic Control", 4096, NULL, 1, NULL, 1); // Core 1

  // Create the cloud logging task on core 0
  xTaskCreatePinnedToCore(
      cloudLoggingTask, "Cloud Logging", 4096, NULL, 1, NULL, 0); // Core 0
}

void loop()
{
  // Main loop is empty because all logic runs in FreeRTOS tasks
}

// Task to control the traffic lights in a round-robin fashion
void trafficControlTask(void *pvParameters)
{
  TrafficState currentState = RED;           // Start with RED state
  unsigned long stateStartTime = millis();   // Track when the state started

  while (1)
  {
    TrafficLight &activeLight = trafficLights[laneIndex]; // Current lane's light

    unsigned long now = millis();
    unsigned long elapsed = now - stateStartTime; // Time spent in current state

    // State machine for traffic light sequence
    switch (currentState)
    {
    case RED:
      // Immediately transition to RED_YELLOW (no delay for RED)
      currentState = RED_YELLOW;
      stateStartTime = now;
      break;

    case RED_YELLOW:
      // Wait for red-yellow duration, then go to GREEN
      if (elapsed >= activeLight.duration.redYellowDuration)
      {
        currentState = GREEN;
        stateStartTime = now;
      }
      break;

    case GREEN:
      // Wait for green duration, then go to YELLOW
      if (elapsed >= activeLight.duration.greenDuration)
      {
        currentState = YELLOW;
        stateStartTime = now;
      }
      break;

    case YELLOW:
      // Wait for yellow duration, then move to next lane and start RED_YELLOW
      if (elapsed >= activeLight.duration.yellowDuration)
      {
        laneIndex = (laneIndex + 1) % numLanes; // Next lane (wrap around)
        currentState = RED_YELLOW;
        stateStartTime = now;
      }
      break;
    }

    // Update the state of each traffic light
    for (int i = 0; i < numLanes; i++)
    {
      trafficLights[i].state = (i == laneIndex) ? currentState : RED; // Only active lane changes state
      setTrafficLights(trafficLights[i].laneId); // Update hardware pins
    }

    activeLane = trafficLights[laneIndex].laneId; // Update global active lane for logging

    vTaskDelay(10 / portTICK_PERIOD_MS); // Small delay to yield to other tasks
  }
}

// Set the hardware pins for a given lane based on its state
void setTrafficLights(int lane_id)
{
  TrafficLight &light = trafficLights[lane_id - 1];
  TrafficState state = light.state;

  // Set pin outputs based on the current state
  switch (state)
  {
  case RED:
    digitalWrite(light.redPin, LOW);      // Red ON
    digitalWrite(light.yellowPin, HIGH);  // Yellow OFF
    digitalWrite(light.greenPin, HIGH);   // Green OFF
    break;
  case RED_YELLOW:
    digitalWrite(light.redPin, LOW);      // Red ON
    digitalWrite(light.yellowPin, LOW);   // Yellow ON
    digitalWrite(light.greenPin, HIGH);   // Green OFF
    break;
  case GREEN:
    digitalWrite(light.redPin, HIGH);     // Red OFF
    digitalWrite(light.yellowPin, HIGH);  // Yellow OFF
    digitalWrite(light.greenPin, LOW);    // Green ON
    break;
  case YELLOW:
    digitalWrite(light.redPin, HIGH);     // Red OFF
    digitalWrite(light.yellowPin, LOW);   // Yellow ON
    digitalWrite(light.greenPin, HIGH);   // Green OFF
    break;
  }
}

void log_if_violation(int lane_idx, String message){
  if (trafficLights[lane_idx].state == RED || trafficLights[lane_idx].state == RED_YELLOW) {
    Serial.printf("%s\n", message.c_str());
  }

  vTaskDelay(10 / portTICK_PERIOD_MS); // Small delay to yield to other tasks
}

// Task to log IR sensor and lane status to Serial (could be extended for cloud)
void cloudLoggingTask(void *pvParameters)
{
  int irPins[4] = {IR1_PIN, IR2_PIN, IR3_PIN, IR4_PIN};
  while (1)
  {
    // Read all IR sensors (vehicle detection)
    int ir1 = digitalRead(IR1_PIN);
    for (int i = 0; i < 2; i++) {
      int irValue = digitalRead(irPins[i]);
      String signal = irValue ? "HIGH" : "LOW";
      Serial.printf("IR %d: %s\n", (i+1), signal.c_str());
      if (irValue == LOW) { // Vehicle detected
        log_if_violation(i, "BEAM_BROKEN_CAM" + String(i + 1));
      }
    }

    vTaskDelay(1000 / portTICK_PERIOD_MS); // Log every 1s
  }
}