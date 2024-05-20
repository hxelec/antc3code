#include <TFT_eSPI.h>
#include <SPI.h>
#include <Adafruit_NeoPixel.h>
#include <esp_now.h>
#include <WiFi.h>
#include "logo.h"
#include <Wire.h>
#include <Adafruit_ADS1X15.h>
#include <ezButton.h>

TFT_eSPI tft = TFT_eSPI();

Adafruit_NeoPixel pixels(1, 36, NEO_GRB + NEO_KHZ800);

Adafruit_ADS1115 ads;

const int BAT_SENSE = 4;

unsigned long targetTime = 0;

uint8_t broadcastAddress[] = {0x30, 0x30, 0xF9, 0xFE, 0x62, 0x54};

const int SERVO_PIN = 37;

const int LEFT_BUTTON = 16;

const int jRight = 38;

const int jLeft = 39;

int currentSpeed = 1;

unsigned long currentMillis = millis();

String macro = String("C");

int motor_A;
int motor_B;
bool servo_status;

int espnowpercent;

typedef struct MotorData {
  int a;
  int b;
  int c;
} MotorData;

typedef struct BatteryCheck {
  int batt;
} BatteryCheck;

MotorData motors;
BatteryCheck battery;

ezButton button_l(jLeft);
ezButton button_r(jRight);

esp_now_peer_info_t peerInfo;

void OnDataSent(const uint8_t *mac_addr, esp_now_send_status_t status) {
  Serial.print("\r\nLast Packet Send Status:\t");
  Serial.println(status == ESP_NOW_SEND_SUCCESS ? "Delivery Success" : "Delivery Fail");
}

void OnDataRecv(const uint8_t * mac, const uint8_t *incomingData, int len) {
  memcpy(&battery, incomingData, sizeof(battery));
  Serial.print("Bytes received: ");
  Serial.println(len);
  espnowpercent = battery.batt;
}

void setup(void) {
  pinMode(jLeft, INPUT_PULLUP);
  pinMode(jRight, INPUT_PULLUP);
  pinMode(SERVO_PIN, INPUT_PULLUP);
  pinMode(LEFT_BUTTON, INPUT_PULLUP);
  tft.init();
  tft.setRotation(0);
  tft.fillScreen(TFT_BLACK);
  pixels.begin();
  pixels.setBrightness(20);
  button_l.setDebounceTime(50);
  button_r.setDebounceTime(50);
  WiFi.mode(WIFI_STA);
  Wire.begin(34, 33);
  if (!ads.begin()) {
    Serial.println("Failed to initialize ADS1115");
    while (1);
  }
  if (esp_now_init() != ESP_OK) {
    Serial.println("Error initializing ESP-NOW");
    return;
  }
  esp_now_register_send_cb(OnDataSent);

  memcpy(peerInfo.peer_addr, broadcastAddress, 6);
  peerInfo.channel = 0;  
  peerInfo.encrypt = false;
  
  // Add peer        
  if (esp_now_add_peer(&peerInfo) != ESP_OK){
    Serial.println("Failed to add peer");
    return;
  }
  esp_now_register_recv_cb(OnDataRecv);
}

void loop() {
  button_l.loop();
  button_r.loop();

  if(button_l.isReleased()) {
    if (currentSpeed > 0) {
      currentSpeed--;
    }
  }

  if(button_r.isReleased()) {
    if (currentSpeed < 3) {
      currentSpeed++;
    }
  }

  getReadings();

  updateDisplay();

  motors.a = motor_A;
  motors.b = motor_B;
  motors.c = digitalRead(SERVO_PIN);

  esp_err_t result = esp_now_send(broadcastAddress, (uint8_t *) &motors, sizeof(motors));
  
  if (result == ESP_OK) {
    Serial.println("Sent with success");
  }
  else {
    Serial.println("Error sending the data");
  }
}

void getReadings() {
  if (digitalRead(LEFT_BUTTON) == HIGH) {
    int total_speed;
    int total_steering_factor;
    if (ads.readADC_SingleEnded(2) >= 8000) {
      total_speed = map(ads.readADC_SingleEnded(2), 8000, 16000, 10000, 16000);
    } else {
      total_speed = map(ads.readADC_SingleEnded(2), 0, 8000, 0, 6000);
    }

    if (ads.readADC_SingleEnded(1) >= 8000) {
      total_steering_factor = map(ads.readADC_SingleEnded(1), 8000, 16000, -2000, -8000);
    } else {
      total_steering_factor = map(ads.readADC_SingleEnded(1), 0, 8000, 8000, 2000);  
    }
    int motor_A_phase_A = total_speed - total_steering_factor;
    int motor_B_phase_A = total_speed + total_steering_factor;
    /*
    if (motor_A_phase_A > 11000) {
      motor_A_phase_A = motor_A_phase_A + 500;
    }
    if (motor_A_phase_A < 8000) {
      motor_A_phase_A = motor_A_phase_A - 1000;
    }
    */
    if (motor_A_phase_A > 16000) {
      motor_A = 16000;
    } else if (motor_A_phase_A < 0) {
      motor_A = 0;
    } else {
      motor_A = motor_A_phase_A;
    }
    if (motor_B_phase_A > 16000) {
      motor_B = 16000;
    } else if (motor_B_phase_A < 0) {
      motor_B = 0;
    } else {
      motor_B = motor_B_phase_A;
    }
    macro = String("C");
  } else {
    motor_A = 16000;
    motor_B = 0;
    macro = String("T");
  }
  servo_status = digitalRead(SERVO_PIN);
  /*
  motor_A = (16000/2);
  motor_B = (16000/2);
  servo_status = 0;
  */
}

void updateDisplay() {
  pixels.fill(0xFF0000);
  tft.fillScreen(TFT_BLACK);
  tft.setRotation(0);

  tft.setTextSize(4);
  tft.setTextColor(TFT_WHITE);
  tft.setCursor(53, 10);
  tft.print(macro); //macro indicator

  tft.setTextSize(2);
  int remotePercentage = map(analogRead(BAT_SENSE), 2100, 2600, 0, 100);
  if (remotePercentage > 40) {
    tft.setTextColor(TFT_GREEN);
  } else {
    tft.setTextColor(TFT_RED);
  }
  tft.setCursor(0, 5);
  tft.print(remotePercentage);

  tft.setTextSize(2);
  String outbat = "  ";
  if (espnowpercent > 99) {
    outbat = "FC";
  } else if (espnowpercent > 40) {
    tft.setTextColor(TFT_GREEN);
    outbat = String(espnowpercent);
  } else {
    tft.setTextColor(TFT_RED);
    if (espnowpercent > 10) {
      outbat = String(espnowpercent);
    } else {
      outbat = "0" + String(espnowpercent);
    }
  }
  tft.setCursor(102, 5);
  tft.print(outbat);

  tft.drawXBitmap(43, 46, logo, logoWidth, logoHeight, TFT_WHITE);
  
  tft.setTextSize(4);
  tft.setTextColor(TFT_WHITE);
  tft.setCursor(53, 90);
  if (currentSpeed == 0) {
    tft.print("L");
  } else if (currentSpeed == 1) {
    tft.print("M");
  } else if (currentSpeed == 2) {
    tft.print("H");
  } else {
    tft.print("X");
  }
}