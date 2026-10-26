# GaitLib

C++ vendordep for running Quantum Wind's FRC-legal CVT swerve module. It targets
WPILib 2026 on every platform WPILib ships (roboRIO, Linux x86-64/ARM32/ARM64,
Windows x86-64/ARM64, macOS universal) and uses the standard WPILib swerve types
(`frc::SwerveModuleState`, `frc::SwerveModulePosition`), so modules drop straight
into `frc::SwerveDriveKinematics` and the pose estimators.

## Layout

```
src/main/native/include/gaitlib/  public headers (what robot code includes)
  CvtSwerveModule.h               the module: Periodic(), SetDesiredState(), GetState(), GetPosition()
  CvtSwerveModuleConfig.h         geometry, fixed reduction, CVT ratio range, ratio policy
  CvtKinematics.h                 motor speed <-> wheel speed at a CVT ratio, ratio selection
  ModuleIO.h                      hardware interface (implemented in the robot project)
  sim/ModuleIOSim.h               physics simulation of the module
src/main/native/cpp/              implementation
src/test/native/cpp/              GoogleTest unit tests (run on desktop)
consumer/GaitLib.json             vendordep descriptor template
```

GaitLib has no motor-controller vendor dependencies. Hardware is accessed through
`gaitlib::ModuleIO`, which the robot project implements with whatever controllers
the module uses (CTRE, REV, a servo for the CVT, ...).

Wheel distance has to come from a sensor on the CVT output: with a variable
ratio, motor rotations don't map to wheel distance. `ModuleIOInputs::wheelPosition`
is wheel-side for this reason.

## Building

Requires JDK 17 (the one bundled with the WPILib installer works; Gradle 8.14
does not run on newer JDKs) and a host C++ compiler for desktop builds.

```bash
./gradlew build                     # all installed cross toolchains + desktop, runs unit tests
./gradlew build -Ponlylinuxathena   # roboRIO only
./gradlew installRoboRioToolchain installArm32Toolchain installArm64Toolchain   # one-time
```

Cross targets whose toolchain isn't installed are skipped. Windows and macOS
binaries can only be built on those OSes, which CI handles. Warnings are errors.

## How the vendordep is distributed

GaitLib is a normal WPILib vendordep, but it isn't hosted anywhere. Its Maven
artifacts are committed into the robot project's git repo next to the JSON:

```
<robot project>/
  vendordeps/GaitLib.json          "mavenUrls": ["libs/maven"]   (relative to the robot project)
  libs/maven/com/quantumwind/gaitlib-cpp/<version>/...
```

No tokens or network access to our repo are needed, and builds work offline at
events.

**Releases:** CI builds every platform on each push. Pushing a tag like `v0.1.0`
creates a GitHub release with `GaitLib-0.1.0.zip`. Unzip it in the root of the
robot project and commit `vendordeps/GaitLib.json` and `libs/maven`. Untagged
builds produce the same zip as a workflow artifact.

**Local development:** publish what this machine can build straight into a
robot project checkout:

```bash
./gradlew publishVendordep -ProbotProject=../Robot -PgaitlibVersion=0.1.0-dev
```

Old versions under `libs/maven/com/quantumwind/gaitlib-cpp/` can be deleted once
the JSON no longer points at them.

## Using GaitLib in a robot project

1. Install the vendordep as above.
2. Add this to the robot project's `build.gradle`. Otherwise GradleRIO also asks
   WPI's public vendor mirror for `com.quantumwind` artifacts, which can stall the
   build for a long time:

   ```groovy
   wpi.maven.useFrcMavenVendorCache = false
   ```

3. Use it:

   ```cpp
   #include <gaitlib/CvtSwerveModule.h>
   #include <gaitlib/sim/ModuleIOSim.h>

   gaitlib::CvtSwerveModuleConfig config;  // fill in from CAD
   gaitlib::CvtSwerveModule module{config, frc::RobotBase::IsSimulation()
       ? std::unique_ptr<gaitlib::ModuleIO>{std::make_unique<gaitlib::sim::ModuleIOSim>(config)}
       : std::make_unique<MyModuleIOTalonFX>(/* ... */)};

   // every loop
   module.Periodic();
   module.SetDesiredState(state);
   ```

To bump the WPILib version, change `wpilibVersion` in `gradle.properties`, the
plugin versions in `build.gradle`, and `frcYear` in `consumer/GaitLib.json`.
Keep it in step with the robot project's GradleRIO.
