# Source code repo for FTC#24751 GreenAms Robotics Team for the 2023-2024 CenterStage season

**WINNING ALLIANCE CAPTAIN - FIRST TECH CHALLENGE VIETNAM 2023-2024**

This repo contains the Android Studio projects with code for FTC#24751 GreenAms Robotics Team's FTC robot for the CenterStage season. It's based on the [original FTC SDK](https://github.com/FIRST-Tech-Challenge/FtcRobotController).

# How to build?

- Step 1: Clone this repo by either using `git clone https://github.com/gart6520/CenterStage` or by downloading zip on Github (although this is not recommended)
- Step 2: Open it in Android Studio 2021.2 (codename Chipmunk) or later.
- Step 3: Wait for Gradle to Sync
- Step 4: Build

The build should success. You can either deploy to Control Hub via `adb` and Android Studio or by generating `.apk` file and then install it manually.

# Additional libraries used
- [RoadRunner](https://github.com/acmerobotics/road-runner) version 0.5.6 by [acmerobotics](https://github.com/acmerobotics)
- [FtcDashboard](https://github.com/acmerobotics/ftc-dashboard) by [acmerobotics](https://github.com/acmerobotics). Should be disabled on official events.
- [MeepMeep](https://github.com/NoahBres/MeepMeep) by [NoahBres](https://github.com/NoahBres/MeepMeep). Used for testing trajectory.
- [homeostasis](https://github.com/Thermal-Equilibrium/homeostasis-FTC) by [Thermal-Equilibrium](https://github.com/Thermal-Equilibrium)

Made possible by @KhiemGOM, @gvl610 and @mynameiskhanhhh.
