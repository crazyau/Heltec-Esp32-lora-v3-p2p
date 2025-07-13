/**
 * LoRa Packet Send/Receive with 3 Digital Inputs, OLED Display, and Diagnostics
 * Displays local and remote digital input states on screen test
 * Compatible with Heltec WiFi LoRa 32 V3
 * working version with inputs and outputs
 * Lipo range between 4.2 and minimum of3.2v
 *to do: add deep sleep or screen of when battery voltage is low,
 */

#define HELTEC_POWER_BUTTON
#include <heltec_unofficial.h>
#include "frame1.h"


// === Configuration ===

const float FREQUENCY = 915.9;
const float BANDWIDTH = 500.0;
const int SPREADING_FACTOR = 7;
const int TRANSMIT_POWER = 22;
const int NO_RX_TIMEOUT_PIN = 45;
const unsigned long RX_TIMEOUT_MS = 20000;             // 20 seconds
const int GROUP_ID = 2;                                // <-- Change per device to isolate groups
const unsigned long BATTERY_READ_INTERVAL_MS = 60000;  // 1 minute
uint64_t lastBatteryReadTime = 0;


const int DIGITAL_INPUT_PIN1 = 5;
const int DIGITAL_INPUT_PIN2 = 6;
const int DIGITAL_INPUT_PIN3 = 7;
const int remoteOutputPins[] = { 40, 41, 42 };  // New outputs controlled by remote inputs


const int MIN_PAUSE_FACTOR = 50;                                  //was 100
const unsigned long PERIODIC_TRANSMIT_INTERVAL_MS = 10 * 1000UL;  // 10 seconds

// === Data Structure for LoRa Packet (Optimized) ===
// This struct packs data efficiently into raw bytes
struct LoRaData {
  uint16_t counter;              // 2 bytes (0 to 65535)
  uint8_t inputStates;           // 1 byte: Bit 0 for Input1, Bit 1 for Input2, Bit 2 for Input3
  uint8_t batteryVoltageScaled;  // 1 byte: Battery voltage scaled (e.g., actual voltage * 10)
  uint8_t groupId;               // 1 byte
};
// Total packet size for LoRaData will be 2 + 1 + 1 + 1 = 5 bytes.
// This is significantly smaller than a string representation.

// === Globals ===
// We no longer need rxdata as a String, but a buffer for the struct
LoRaData receivedLoRaData;  // Global to store the last received struct data

volatile bool rxFlag = false;
uint64_t last_rx_time = 0;  // Time of last valid receive
float lastRSSI = 0.0;
long counter = 0;                    // Keep counter as long, but only transmit uint16_t portion
uint64_t last_tx = 0;                // This still tracks the last *actual* transmission for airtime limits
uint64_t last_periodic_tx_time = 0;  // New variable for the 10-second interval
uint64_t tx_duration_ms;
uint64_t minimum_pause_ms = 0;
uint64_t ledOffTime = 0;
const uint32_t LED_FLASH_DURATION_MS = 100;  // LED ON time in ms

int currentDigitalInputState1 = -1;
int previousDigitalInputState1 = -1;
int currentDigitalInputState2 = -1;
int previousDigitalInputState2 = -1;
int currentDigitalInputState3 = -1;
int previousDigitalInputState3 = -1;

// These will be updated from the receivedLoRaData struct
int receivedDigitalInputState1 = -1;
int receivedDigitalInputState2 = -1;
int receivedDigitalInputState3 = -1;

float batteryVoltage = 0.0;
float receivedBatteryVoltage = 0.0;  // To store the decoded remote battery voltage
int txFailures = 0;
int rxFailures = 0;
int malformedPackets = 0;  // This will mostly apply if packet size is wrong now

float readBatteryVoltage() {
  return heltec_vbat();
}

// --- New function for scrolling an image ---
void scrollImage(const unsigned char* image_bits, int image_width, int image_height, int start_x, int end_x, int y_pos, unsigned long scroll_speed_ms) {
  // scroll_speed_ms: controls how fast the image moves (delay per pixel movement)
  // start_x: initial X position
  // end_x: final X position (e.g., negative width to scroll completely off screen)
  // y_pos: fixed Y position for the image

  // Determine the direction of scrolling
  int step = (start_x < end_x) ? 1 : -1;

  for (int x = start_x; (step == 1 && x <= end_x) || (step == -1 && x >= end_x); x += step) {
    display.clear();  // Clear the entire display
    display.drawXbm(x, y_pos, image_width, image_height, image_bits);
    display.display();
    delay(scroll_speed_ms);
  }
}

void logo() {
  display.clear();

  // Example 1: Static image for a moment
  // display.drawXbm(0, 0, FRAME1_WIDTH, FRAME1_HEIGHT, frame1_bits);
  // display.display();
  // delay(2000); // Display static for 2 seconds

  // Example 2: Scroll the image from right to left
  // Start X at display.width() (off-screen right)
  // End X at -AUSTRALIA_WIDTH (off-screen left)
  // Y position fixed at 0
  // Scroll speed: 10ms delay per pixel movement (adjust for desired speed)
  scrollImage(frame1_bits, FRAME1_WIDTH, FRAME1_HEIGHT,
              display.width(), -FRAME1_WIDTH, 0, 1);  // Adjust Y-pos if needed last digit is scroll delay

  display.clear();  // Clear after scrolling
  display.display();
}

String stateToString(int val) {
  return val == LOW ? "ON" : "OFF";
}

void displayInputStates() {
  display.clear();
  display.setTextAlignment(TEXT_ALIGN_LEFT);
  display.setFont(ArialMT_Plain_10);

  display.drawString(0, 0, "Local:");
  display.drawString(60, 0, "Group: " + String(GROUP_ID));
  display.drawString(0, 12, "1: " + stateToString(currentDigitalInputState1));
  display.drawString(42, 12, "2: " + stateToString(currentDigitalInputState2));
  display.drawString(84, 12, "3: " + stateToString(currentDigitalInputState3));

  display.drawString(0, 24, "Remote:");
  display.drawString(0, 36, "1: " + stateToString(receivedDigitalInputState1));
  display.drawString(42, 36, "2: " + stateToString(receivedDigitalInputState2));
  display.drawString(84, 36, "3: " + stateToString(receivedDigitalInputState3));

  // Display remote battery voltage
  char remoteBatStr[20];
  snprintf(remoteBatStr, sizeof(remoteBatStr), "R Bat: %.2f V", receivedBatteryVoltage);
  display.drawString(60, 24, remoteBatStr);


  // FLASHING RX TIMEOUT MESSAGE (moved slightly down to accommodate remote battery)
  if (millis() - last_rx_time > RX_TIMEOUT_MS) {
    static bool showTimeout = true;
    static unsigned long lastFlashTime = 0;
    if (millis() - lastFlashTime > 800) {
      showTimeout = !showTimeout;
      lastFlashTime = millis();
    }

    if (showTimeout) {
      float timeout = (millis() - last_rx_time) / 1000.0;
      display.drawString(0, 48, "RX: TIMEOUT  " + String(timeout, 1) + " s");  // Adjusted Y position
    }
  } else {
    char rssiStr[24];
    snprintf(rssiStr, sizeof(rssiStr), "RX: OK RSSI: %.1f dBm", lastRSSI);
    display.drawString(0, 48, rssiStr);  // Adjusted Y position
  }

  display.display();
}
void rx() {
  rxFlag = true;
}

void setup() {
  heltec_setup();
  heltec_ve(true);
  logo();
  both.println("Radio init");

  int status = radio.begin();
  if (status != RADIOLIB_ERR_NONE) {
    both.printf("Radio.begin() failed, code %d\n", status);
    while (true)
      ;
  }
  radio.setDio1Action(rx);

  radio.setFrequency(FREQUENCY);
  radio.setBandwidth(BANDWIDTH);
  radio.setSpreadingFactor(SPREADING_FACTOR);
  radio.setOutputPower(TRANSMIT_POWER);
  radio.startReceive(RADIOLIB_SX126X_RX_TIMEOUT_INF);

  pinMode(DIGITAL_INPUT_PIN1, INPUT_PULLUP);
  pinMode(DIGITAL_INPUT_PIN2, INPUT_PULLUP);
  pinMode(DIGITAL_INPUT_PIN3, INPUT_PULLUP);
  pinMode(40, OUTPUT);
  pinMode(41, OUTPUT);
  pinMode(42, OUTPUT);
  pinMode(NO_RX_TIMEOUT_PIN, OUTPUT);
  digitalWrite(NO_RX_TIMEOUT_PIN, LOW);  // Default LOW

  // Initialize last_periodic_tx_time to prevent immediate transmission on boot
  last_periodic_tx_time = millis();
}

void loop() {
  heltec_loop();

  if (ledOffTime != 0 && millis() > ledOffTime) {
    heltec_led(0);  // Turn LED OFF
    ledOffTime = 0;
  }
  currentDigitalInputState1 = digitalRead(DIGITAL_INPUT_PIN1);
  currentDigitalInputState2 = digitalRead(DIGITAL_INPUT_PIN2);
  currentDigitalInputState3 = digitalRead(DIGITAL_INPUT_PIN3);

  if (millis() - lastBatteryReadTime > BATTERY_READ_INTERVAL_MS) {
    batteryVoltage = readBatteryVoltage();
    lastBatteryReadTime = millis();
  }

  bool digitalChanged =
    currentDigitalInputState1 != previousDigitalInputState1 || currentDigitalInputState2 != previousDigitalInputState2 || currentDigitalInputState3 != previousDigitalInputState3;

  previousDigitalInputState1 = currentDigitalInputState1;
  previousDigitalInputState2 = currentDigitalInputState2;
  previousDigitalInputState3 = currentDigitalInputState3;

  displayInputStates();  // Update OLED always

  // Check for transmit conditions:
  // 1. Enough time has passed since the last transmission (airtime legal limit)
  // 2. Either 10 seconds have passed, OR a digital input has changed, OR the button was clicked
  bool tx_legal = millis() - last_tx > max(minimum_pause_ms, (uint64_t)(100UL));  // Using a small base minimum pause
  bool periodic_time_elapsed = millis() - last_periodic_tx_time >= PERIODIC_TRANSMIT_INTERVAL_MS;

  if (tx_legal && (periodic_time_elapsed || digitalChanged || button.isSingleClick())) {
    // If the transmission is triggered by a periodic interval, update its timer
    if (periodic_time_elapsed) {
      last_periodic_tx_time = millis();
    }

    // Prepare the data packet using the struct
    LoRaData dataToSend;
    dataToSend.counter = counter++;  // counter will wrap around at 65535 automatically for uint16_t

    // Pack digital input states into a single byte
    dataToSend.inputStates = 0;
    if (currentDigitalInputState1 == LOW) dataToSend.inputStates |= (1 << 0);  // Bit 0
    if (currentDigitalInputState2 == LOW) dataToSend.inputStates |= (1 << 1);  // Bit 1
    if (currentDigitalInputState3 == LOW) dataToSend.inputStates |= (1 << 2);  // Bit 2

    // Scale battery voltage (e.g., 3.75V becomes 37, assuming 0.01V resolution is not strictly needed)
    // For 0.1V resolution, multiply by 10. Max 25.5V if uint8_t
    // For 0.01V resolution, multiply by 100. Max 2.55V if uint8_t (not enough for 4.2V)
    // Since battery voltage is typically 3.2V - 4.2V, multiplying by 10 gives a range of 32-42.
    // This fits well within a uint8_t.
    dataToSend.batteryVoltageScaled = (uint8_t)(batteryVoltage * 10.0);
    dataToSend.groupId = (uint8_t)GROUP_ID;  // Ensure GROUP_ID fits in a byte

    printf("TX [Counter:%u, Inputs:0x%02X, Batt:%.2fV, Group:%u] ",
           dataToSend.counter,
           dataToSend.inputStates,
           (float)dataToSend.batteryVoltageScaled / 10.0,  // Print original value for diagnostics
           dataToSend.groupId);

    radio.clearDio1Action();
    heltec_led(50);
    uint64_t transmit_start_time = millis();
    // Transmit the raw bytes of the struct
    int tx_status = radio.transmit((uint8_t*)&dataToSend, sizeof(dataToSend));
    tx_duration_ms = millis() - transmit_start_time;
    heltec_led(0);

    if (tx_status == RADIOLIB_ERR_NONE) {
      printf("OK (%i ms)\n", (int)tx_duration_ms);
    } else {
      both.printf("fail (%i)\n", tx_status);
      txFailures++;
    }

    minimum_pause_ms = tx_duration_ms * MIN_PAUSE_FACTOR;
    last_tx = millis();
    radio.setDio1Action(rx);
    radio.startReceive(RADIOLIB_SX126X_RX_TIMEOUT_INF);
  }

  if (rxFlag) {
    rxFlag = false;
    // Read raw bytes directly into the receivedLoRaData struct
    int read_status = radio.readData((uint8_t*)&receivedLoRaData, sizeof(receivedLoRaData));

    if (read_status == RADIOLIB_ERR_NONE) {
      float rssi = radio.getRSSI();
      float snr = radio.getSNR();
      heltec_led(1);                                  // Turn LED ON
      ledOffTime = millis() + LED_FLASH_DURATION_MS;  // Schedule OFF
      lastRSSI = rssi;
      last_rx_time = millis();

      // Check if the received packet size matches our expected struct size
      // This is a basic form of malformed packet detection
      if (radio.getPacketLength() != sizeof(receivedLoRaData)) {
        malformedPackets++;
        both.printf("Malformed RX: Incorrect size (%d bytes, expected %d)\n", radio.getPacketLength(), sizeof(receivedLoRaData));
        radio.startReceive(RADIOLIB_SX126X_RX_TIMEOUT_INF);  // Continue receiving
        return;
      }

      // Check GROUP_ID first
      if (receivedLoRaData.groupId != GROUP_ID) {
        printf("Ignored packet from group %u\n", receivedLoRaData.groupId);
        radio.startReceive(RADIOLIB_SX126X_RX_TIMEOUT_INF);  // Continue receiving
        return;
      }

      // Group matched, decode received data
      // Unpack digital input states from the byte
      receivedDigitalInputState1 = (receivedLoRaData.inputStates & (1 << 0)) ? LOW : HIGH;
      receivedDigitalInputState2 = (receivedLoRaData.inputStates & (1 << 1)) ? LOW : HIGH;
      receivedDigitalInputState3 = (receivedLoRaData.inputStates & (1 << 2)) ? LOW : HIGH;

      // Unscale battery voltage
      receivedBatteryVoltage = (float)receivedLoRaData.batteryVoltageScaled / 10.0;

      // Update output pins
      digitalWrite(remoteOutputPins[0], receivedDigitalInputState1);
      digitalWrite(remoteOutputPins[1], receivedDigitalInputState2);
      digitalWrite(remoteOutputPins[2], receivedDigitalInputState3);

      printf("RX: [Counter:%u, Inputs:0x%02X, Batt:%.2fV, Group:%u]\n",
             receivedLoRaData.counter,
             receivedLoRaData.inputStates,
             receivedBatteryVoltage,
             receivedLoRaData.groupId);
      printf("RSSI: %.1f dBm, SNR: %.1f dB\n", rssi, snr);
      displayInputStates();  // Update display with new remote values
    } else {
      rxFailures++;
      both.printf("radio.readData() failed, code %d\n", read_status);
    }

    radio.startReceive(RADIOLIB_SX126X_RX_TIMEOUT_INF);
  }

  static uint32_t lastDiagPrint = 0;
  if (millis() - lastDiagPrint > 60000) {
    printf("=== Diagnostics ===\n");
    printf("     TX Failures: %d\n", txFailures);
    printf("     RX Failures: %d\n", rxFailures);
    printf("     Malformed Packets: %d\n", malformedPackets);
    printf("     Local Battery Voltage: %.2f V\n", batteryVoltage);
    printf("     Local Battery: %.2f V (%d%%)\n", batteryVoltage, heltec_battery_percent());
    printf("     Time since last RX: %.1f s\n", (millis() - last_rx_time) / 1000.0);

    lastDiagPrint = millis();
  }
  if (millis() - last_rx_time > RX_TIMEOUT_MS) {
    digitalWrite(NO_RX_TIMEOUT_PIN, HIGH);  // No packet received in 20 sec
  } else {
    digitalWrite(NO_RX_TIMEOUT_PIN, LOW);  // Packet received recently
  }
}