# Getting Started

## Installation
To install & use this system, download the `Release-[version-name].zip` file and extract the contents into `Teamcode/src/main/java/org/ftc/teamcode`. Then open `Teamcode/build.gradle` and paste the following lines into the `dependencies` area (between the `{ }`).
```gradle
implementation "org.jetbrains.kotlin:kotlin-stdlib-jdk7:$kotlin_version"
implementation 'com.acmerobotics.roadrunner:core:0.5.3'
```
This will add all of the dependencies required for the system to function.

## Basic Usage
There are 3 main aspects to the system. These aspects are:
- [Mechanisms/Subsystems](https://github.com/AtomicRobotics3805/2022-Offseason/blob/docs/README.md#mechanisms--subsystems)
- [Commands/OpModes](https://github.com/AtomicRobotics3805/2022-Offseason/blob/docs/README.md#commands--opmodes)
- [Trajectories/Roadrunner](https://github.com/AtomicRobotics3805/2022-Offseason/blob/docs/README.md#trajectories--roadrunner)

