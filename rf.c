#include <Arduino.h>

// Define GPIO pins for HT12D output
#define LANE1_PIN 14  // D8 on HT12D (Highest Priority)
#define LANE2_PIN 27  // D9
#define LANE3_PIN 26  // D10
#define LANE4_PIN 25  // D11 (Lowest Priority)
#define VT_PIN 33     // Valid Transmission Pin (VT)

// Define output indicators
#define LED_PIN 5      // Example: LED to signal priority lane
#define BUZZER_PIN 18  // Example: Buzzer for emergency

// Function to handle priority-based actions
void handlePriority(int lane) {
    Serial.print("Priority Signal Received: Lane ");
    Serial.println(lane);

    // Activate signals based on priority
    if (lane == 1) {  // Emergency Vehicle (Highest Priority)
        Serial.println("Lane 1: Emergency Vehicle - Clearing Traffic!");
        digitalWrite(BUZZER_PIN, HIGH);
        delay(2000);
        digitalWrite(BUZZER_PIN, LOW);
    } 
    else if (lane == 2) {
        Serial.println("Lane 2: High-Priority Vehicle - Adjusting Signals");
        digitalWrite(LED_PIN, HIGH);
        delay(3000);
        digitalWrite(LED_PIN, LOW);
    } 
    else if (lane == 3) {
        Serial.println("Lane 3: Moderate Traffic Priority");
    } 
    else if (lane == 4) {
        Serial.println("Lane 4: General Traffic Control");
    } 
    else {
        Serial.println("Unknown Signal Received!");
    }
}

// Function to check for incoming priority signals
void checkSignals() {
    if (digitalRead(VT_PIN) == HIGH) {  // Valid transmission detected
        if (digitalRead(LANE1_PIN) == HIGH) {
            handlePriority(1);
        } 
        else if (digitalRead(LANE2_PIN) == HIGH) {
            handlePriority(2);
        } 
        else if (digitalRead(LANE3_PIN) == HIGH) {
            handlePriority(3);
        } 
        else if (digitalRead(LANE4_PIN) == HIGH) {
            handlePriority(4);
        }
    }
}

void setup() {
    Serial.begin(115200);

    // Configure input pins (HT12D outputs)
    pinMode(LANE1_PIN, INPUT);
    pinMode(LANE2_PIN, INPUT);
    pinMode(LANE3_PIN, INPUT);
    pinMode(LANE4_PIN, INPUT);
    pinMode(VT_PIN, INPUT);

    // Configure output indicators
    pinMode(LED_PIN, OUTPUT);
    pinMode(BUZZER_PIN, OUTPUT);
}

void loop() {
    checkSignals();
    delay(100);  // Small delay to prevent excessive polling
}