#pragma once
#include <Arduino.h>

struct Reading { float r, g, b, c; };
struct DynamicStation {
  bool active;
  bool isStart;
  char name[16];
  Reading cal;
  char keyUID[24];
};

constexpr const char* AGV_FW_BUILD = "2026-11-06.2-s3-center-priority";
constexpr const char* AGV_UI_BUILD = "2026-11-06-ap-stable-fast-pd";

enum RunState : uint8_t {
  IDLE,
  FOLLOW,
  STATION_WAIT,
  OBSTACLE_STOP,
  FAULT_STOP,
  FINISHED,
  MANUAL,
  RECOVERY_SEARCH,
  LOAD_TEST_RUN,
  SERVICE_TEST
};

// Hidden Backend Infrastructure
const char* GITHUB_OWNER = "rjay2174";
const char* GITHUB_REPO  = "guideline-agv";
const char* GITHUB_BIN   = "firmware.bin";
const String GITHUB_OTA_URL = String("https://github.com/") + GITHUB_OWNER + "/" + GITHUB_REPO + "/releases/latest/download/" + GITHUB_BIN;

// Router defaults (hidden)
const char* DEFAULT_STA_SSID = ""; // Public GitHub build: requires saved NVS router SSID
const char* DEFAULT_STA_PASS = ""; // Public GitHub build: requires saved NVS router password
const char* DEVICE_HOSTNAME  = "guideline-agv";

constexpr uint32_t PWM_FREQ_HZ   = 1000;
constexpr uint32_t RC_TIMEOUT_MS = 1200; // HTTP/BLE/manual fallback watchdog
// WebSocket frames arrive every 40ms. Missed control data for 450ms stops wheels
// AND disarms the connection; another fresh joystick press is required.
constexpr uint32_t WS_RC_TIMEOUT_MS = 450;
constexpr uint32_t LOOP_GUARD_MS = 350; // Retained for compatibility
// 2 seconds bridging straight, followed by up to 5 seconds active search.
constexpr uint32_t STRAIGHT_BRIDGE_MIN_MS = 2000;
constexpr uint32_t LINE_SEARCH_MAX_MS = 5000;
constexpr uint32_t STRAIGHT_BRIDGE_MAX_MS = STRAIGHT_BRIDGE_MIN_MS + LINE_SEARCH_MAX_MS;
// Five IR channels simultaneously reading BLACK for 3 continuous seconds
// while motors are driven indicates a possible lifted/stalled robot.
constexpr uint32_t ALL_IR_BLACK_STOP_MS = 3000;
constexpr uint8_t ALL_IR_BLACK_MASK = 0x1Fu; // S1..S5 = 11111
constexpr uint32_t RFID_REARM_MS = 900;
constexpr uint32_t STATION_DEPARTURE_MS = 900;

inline const char* getStateName(RunState s) {
  switch (s) {
    case IDLE: return "Ready on Track";
    case FOLLOW: return "Following Track";
    case STATION_WAIT: return "Stopped at Station";
    case OBSTACLE_STOP: return "Obstacle In Path";
    case FAULT_STOP: return "Safety Stop";
    case FINISHED: return "Trip Finished";
    case RECOVERY_SEARCH: return "Searching For Track";
    case LOAD_TEST_RUN: return "Load Test Running";
    case SERVICE_TEST: return "Motor Service Test";
    default: return "Manual Control";
  }
}