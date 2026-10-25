# Neuromorphic Robot Android App (source)

A native Android control app for the ESP32-hosted Smart Neuromorphic Robot firmware. It uses a clean five-tab layout:

- **Home:** connection, mode, ToF, vibration, PWM command, estimated location, safety and sensor health.
- **Map:** live topological map, road connections, junctions, destination marker naming.
- **Drive:** exploration/destination/manual modes, destination shortcuts, hold-to-move buttons, enable and stop.
- **Learning:** SNN concepts/synapses, prediction error, confidence, trip success/failure and locally stored reports.
- **Settings:** connection help, erase all robot session memory, delete phone reports separately.

## Important: APK build status

This source project includes a GitHub Actions workflow that builds a debug APK. The current execution environment has Java but no Android SDK/Gradle installation, so an APK binary cannot be compiled here. Do not treat this source ZIP as an APK.

## Build an APK using GitHub Actions

1. Create a GitHub repository from the files in this folder (upload the folder contents, not the enclosing folder).
2. Push/commit them to the `main` branch, or open **Actions → Build Android APK → Run workflow**.
3. Wait for the workflow to finish successfully.
4. Open that workflow run and download the `NeuromorphicRobot-debug-apk` artifact.
5. Extract it and install `app-debug.apk` on your Android phone. Android may ask you to allow installation from the browser/files app.

The APK is a debug build, intended for personal testing rather than Play Store publication.

## Connect to the ESP32

1. Flash `Phase2_Neuromorphic_Robot.ino` to the ESP32.
2. Connect the phone to Wi-Fi `Neuromorphic-Robot` (password `RobotMap8266`) or use the robot's configured AP.
3. Keep the phone connected even if Android says the Wi-Fi has no internet.
4. Open the app. It connects to `http://192.168.4.1` and polls `/api`; commands use `/cmd`.

This app is configured for the ESP32 AP address `192.168.4.1`. If firmware networking changes, update `BASE` in `MainActivity.java`.

## Memory deletion

- **Erase all robot memory** sends `/cmd?cmd=reset_memory`. It stops the motors and clears the learned map, destinations, concept prototypes, synaptic weights/network state, and session counters before initializing a fresh network.
- **Delete phone reports** clears snapshots stored by this app on the phone. It does not erase robot memory.

These actions are separate and each requires confirmation. Robot learning is RAM-only and also resets when the ESP32 reboots.

## Safety

Test with wheels raised first. The app is not an emergency-stop guarantee if Wi-Fi disconnects. The ESP32 firmware must independently enforce safety. Confirm sensor and motor wiring before enabling movement.
