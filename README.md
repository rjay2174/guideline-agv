# GUIDELine AGV - Firmware OTA Repository

This public repository is intended to hold ESP32-S3 firmware source and GitHub Releases assets for GUIDELine AGV.

The firmware's built-in GitHub OTA uses:

https://github.com/rjay2174/guideline-agv/releases/latest/download/firmware.bin

**No firmware binary has been published yet.** A release must include an ESP32-S3 *application* binary named exactly `firmware.bin`. The source ZIP and a merged/bootloader flash image cannot be used as this OTA asset.

## Security and deployment

Do not commit local router SSID/password, private RFID data, or unique device credentials to this public repository. Do not publish an OTA binary that embeds private router passwords. The locally supplied sanitized GitHub source package has blank router defaults; save router credentials to the device NVS before OTA. Its public demonstration AP key is for controlled bench testing only.

The firmware updater currently disables TLS certificate verification. Treat GitHub updates as controlled tests until signed firmware validation and secure transport are implemented.

## Release process

1. Upload the sanitized `guideline_agv/` source directory to this repository.
2. Run the workflow **Build and publish GUIDELine AGV firmware** under Actions. It compiles using Arduino ESP32 core and MFRC522.
3. Verify that the action succeeds and publishes a latest release containing `firmware.bin` and a SHA-256 file.
4. Stop the AGV and verify that router Wi-Fi is connected.
5. In the AGV Web UI, choose **Wi-Fi & Cloud OTA → Check & Install GitHub Update**.
6. After reboot, open `http://guideline-agv.local/version` (or router-IP `/version`) to confirm the installed firmware build.

Do not permit wheel movement until hardware safety checks pass.