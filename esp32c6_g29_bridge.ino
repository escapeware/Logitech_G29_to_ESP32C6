/*
  Minimal ESP32-C6 sketch for reading
  3×74HC165 on a Logitech G29 wheel PCB

  Only shows:
   • Pin definitions for CLK / PL / Q
   • setup() to configure GPIOs
   • loop() to latch & clock out 24 bits
   • Serial debug of raw bitstream
*/

#include <Arduino.h>

// ── Pin assignments ────────────────────────────────────────────────
// Data pins are listed in descending order from top to bottom. 

//Power pins: Pin 1 is GND, pin 7 is power (5v) 

// Connect G29 74HC165 pins to these ESP32-C6 GPIOs:
#define PIN_165_MISO    10  // Q / Serial data out Pin 2
#define PIN_595_RCLK    20  // RCLK (storage register clock; not strictly needed for read), Pin 3
#define PIN_SHARED_CLK  11  // SRCLK (shift register clock), Pin 4
#define PIN_595_SER     23  // SER (for chaining, kept LOW if unused), Pin 5
#define PIN_165_PL       1  // PL (parallel load / latch) Pin 6 

// Total bits = 3 × 8
static const int NUM_BITS = 24;

void setup() {
  Serial.begin(115200);
  delay(100);
  Serial.println("=== G29 74HC165 Reader ===");

  pinMode(PIN_165_MISO,    INPUT);
  pinMode(PIN_SHARED_CLK,  OUTPUT);
  pinMode(PIN_165_PL,      OUTPUT);
  pinMode(PIN_595_SER,     OUTPUT);
  pinMode(PIN_595_RCLK,    OUTPUT);

  // Idle states
  digitalWrite(PIN_SHARED_CLK, LOW);
  digitalWrite(PIN_165_PL,     HIGH);
  digitalWrite(PIN_595_SER,    LOW);
  digitalWrite(PIN_595_RCLK,   LOW);
}

void loop() {
  uint8_t bits[NUM_BITS];

  // 1) Latch parallel inputs into the shift registers
  digitalWrite(PIN_165_PL, LOW);
  delayMicroseconds(1);
  digitalWrite(PIN_165_PL, HIGH);
  delayMicroseconds(1);

  // 2) Clock out each bit
  for (int i = 0; i < NUM_BITS; i++) {
    digitalWrite(PIN_SHARED_CLK, LOW);
    delayMicroseconds(1);
    digitalWrite(PIN_SHARED_CLK, HIGH);
    delayMicroseconds(1);

    bits[i] = digitalRead(PIN_165_MISO);
  }

  // 3) (Optional) pulse RCLK if you’re also chaining 74HC595s
  digitalWrite(PIN_595_RCLK, HIGH);
  delayMicroseconds(1);
  digitalWrite(PIN_595_RCLK, LOW);

  // 4) Print raw bitstream for debugging
  Serial.print("G29 buttons: ");
  for (int i = 0; i < NUM_BITS; i++) {
    Serial.print(bits[i]);
    if (i % 8 == 7 && i < NUM_BITS - 1) Serial.print(' ');
  }
  Serial.println();

  delay(10);
}
