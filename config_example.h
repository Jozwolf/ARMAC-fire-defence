// config_example.h — template for ARMAC site-specific credentials
// Copy this file to config.h (in the same directory) and fill in your own values.
// config.h is listed in .gitignore and will not be committed to the repository.

#pragma once

// ---- AR1: WiFi + MQTT site profiles ----
// Add or remove profiles to match your network locations.
// Each profile: SSID, WiFi password, MQTT broker IP, descriptive label.
#define NET1_SSID   "YourNetworkSSID"
#define NET1_PASS   "YourWiFiPassword"
#define NET1_MQTT   "192.168.x.x"
#define NET1_LABEL  "Site 1 MQTT broker"

#define NET2_SSID   "YourSecondNetworkSSID"
#define NET2_PASS   "YourSecondWiFiPassword"
#define NET2_MQTT   "192.168.x.x"
#define NET2_LABEL  "Site 2 MQTT broker"

// ---- AR1: MQTT broker credentials ----
#define MQTT_USER     "your_mqtt_username"
#define MQTT_PASSWORD "your_mqtt_password"

// ---- AR2/AR3: inter-Arduino WiFi AP (AR2 hosts, AR3 connects) ----
// AR2 creates this access point; AR3 connects to it for sensor UDP traffic.
#define AR2_AP_SSID     "Arduino2_AP"
#define AR2_AP_PASSWORD "your_ap_password"
