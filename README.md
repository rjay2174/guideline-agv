# GUIDELine AGV – ESP32-S3 modular firmware

**Source uploaded:** 2026-11-05, build `2026-11-05.2-ir-start-rfid-github`.

## Project files

- [Main Arduino sketch](guideline_agv/guideline_agv.ino)
- [Configuration](guideline_agv/AGV_Config.h)
- [RFID driver and authorization](guideline_agv/AGV_RFID.h)
- [Sensors and calibrations](guideline_agv/AGV_Sensors.h)
- [Communications, Wi-Fi and HTTP API](guideline_agv/AGV_Comm.h)
- [LittleFS web interface](guideline_agv/data/index.html)
- [Embedded web fallback](guideline_agv/AGV_WebPage.h)
- [GitHub Actions build and release workflow](.github/workflows/build-release.yml)

The project uses the ESP32-S3 and Arduino MFRC522 library. The main sketch is in the `guideline_agv` directory.

## Before a first GitHub OTA update

**This is a PUBLIC repository. Do not commit Wi-Fi passwords or actual Master RFID UIDs.**

The source intentionally uses **blank default router credentials** and **no hardcoded Master Card**. Existing router/Master settings remain available after OTA only when previously saved in ESP32 NVS.

1. On the **currently running AGV**, stop the motors and save router SSID/password through **Wi-Fi & Cloud OTA**. Confirm the router connection survives reboot.
2. **Save your currently working Master Card to NVS before updating.** In **Stations & RFID**, scan the current Master Card, enter that *same* UID in the Master replacement field, and click **Replace Master UID**. That action writes it to NVS without changing which card is authorized. Verify after reboot.
3. Keep the current application partition scheme, with both `ota_0` and `ota_1`, and confirm the compiled binary fits the OTA slot.
4. Power the AGV safely, with wheels raised. Recheck all-black IR stop, obstacle interlock, RFID start and stopping behavior after update.

### Important security restrictions

The published sketch currently contains a publicly known **bench-only AP password**. Anyone who can connect to the access point may be able to reach control endpoints; **do not use this build near bystanders or in an uncontrolled environment** until AP authentication and web authorization are hardened. Likewise, GitHub downloads currently use `WiFiClientSecure::setInsecure()`, which omits certificate verification. Do not treat this as a production-secure deployment or release.

## Build and publish firmware.bin

1. In GitHub, select **Actions → Build and publish GUIDELine AGV firmware → Run workflow**.
2. Inspect the build logs. Only a successful compile creates a release asset named `firmware.bin`.
3. Verify `https://github.com/rjay2174/guideline-agv/releases/latest/download/firmware.bin` is available.
4. With the AGV stopped, connected to the router, and safety checks complete, open **Wi-Fi & Cloud OTA → Check & Install GitHub Update**.
5. Verify installed version after reboot at `http://guideline-agv.local/version` or by router-assigned IP.

A GitHub source commit by itself does **not** update the ESP32. A successful release build and an explicit update action are still necessary.
