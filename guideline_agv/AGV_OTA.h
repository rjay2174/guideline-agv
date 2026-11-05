#pragma once
#include <Arduino.h>
#include <WiFi.h>
#include <WiFiClientSecure.h>
#include <HTTPClient.h>
#include <Update.h>
#include "AGV_Config.h"

inline bool updateFromGitHub(String& logMsg) {
  if (WiFi.status() != WL_CONNECTED) {
    logMsg = "OTA Failed: Not connected to Wi-Fi";
    Serial.println("[OTA] Error: Device is not connected to router Wi-Fi.");
    return false;
  }

  Serial.println("\n[OTA] Connecting to GitHub Releases...");
  Serial.printf("[OTA] Target URL: %s\n", GITHUB_OTA_URL.c_str());

  WiFiClientSecure client;
  client.setInsecure(); // Skip certificate verification to conserve SRAM for the TLS handshake

  HTTPClient http;
  // Critical for GitHub: Follow 302 redirects to objects.githubusercontent.com
  http.setFollowRedirects(HTTPC_FORCE_FOLLOW_REDIRECTS);
  http.setConnectTimeout(10000);
  http.setTimeout(25000);

  if (!http.begin(client, GITHUB_OTA_URL)) {
    logMsg = "OTA Error: Host unreachable";
    Serial.println("[OTA] http.begin() failed.");
    return false;
  }

  int code = http.GET();
  Serial.printf("[OTA] GitHub HTTP Response Code: %d\n", code);

  if (code != HTTP_CODE_OK) {
    logMsg = "OTA Error: HTTP " + String(code);
    Serial.printf("[OTA] Failed to fetch binary. Verify release exists and asset name is %s\n", GITHUB_BIN);
    http.end();
    return false;
  }

  int length = http.getSize();
  Serial.printf("[OTA] Binary size: %d bytes\n", length);

  if (length <= 0) {
    logMsg = "OTA Error: Invalid binary size";
    http.end();
    return false;
  }

  if (!Update.begin(length)) {
    logMsg = "OTA Error: Partition allocation failed";
    Serial.printf("[OTA] Update.begin failed: %s\n", Update.errorString());
    http.end();
    return false;
  }

  WiFiClient* stream = http.getStreamPtr();
  size_t written = Update.writeStream(*stream);

  if (written == length) {
    Serial.println("[OTA] Download completed successfully. Finalizing flash...");
  } else {
    Serial.printf("[OTA] Incomplete flash: wrote %d of %d bytes.\n", written, length);
  }

  if (Update.end()) {
    if (Update.isFinished()) {
      Serial.println("[OTA] Firmware verification successful! Rebooting system...");
      logMsg = "OTA Complete! Restarting...";
      delay(1500);
      ESP.restart();
      return true;
    } else {
      logMsg = "OTA Error: Update not finished";
      Serial.println("[OTA] Verification incomplete.");
    }
  } else {
    logMsg = "OTA Error: " + String(Update.errorString());
    Serial.printf("[OTA] Flash commit error: %s\n", Update.errorString());
  }

  http.end();
  return false;
}