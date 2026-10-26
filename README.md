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
src/test/native/cpp/              GoogleTest unit tests (run on desktop), mirroring src/main/native/cpp
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

## Tests

`src/test/native/cpp` mirrors `src/main/native/cpp` one-to-one: `cpp/sim/ModuleIOSim.cpp`
is tested by `src/test/native/cpp/sim/ModuleIOSimTest.cpp`. `./gradlew check` (and so
`build`) fails on a missing or orphaned test file. `main.cpp` is the only extra test file.

CI requires 100% line, function, and branch coverage of `src/main/native`. To run the same gate
locally (Linux, needs [gcovr](https://gcovr.com)):

```bash
./gradlew coverage                  # report: build/reports/coverage/index.html
```

## How the vendordep is distributed

GaitLib is a normal WPILib vendordep whose Maven artifacts live in this repo's
private GitHub Packages registry (`https://maven.pkg.github.com/Quantum-Wind/GaitLib`).

- **Pull requests** build and test every platform. Nothing is published.
- **Every push to `main`** publishes `com.quantumwind:gaitlib-cpp:<gaitlibVersionBase>.<CI run number>`
  (e.g. `0.1.57`) and creates a GitHub release `v0.1.57` with the matching `GaitLib.json`.
  Bump `gaitlibVersionBase` in `gradle.properties` for breaking changes.
- **Local builds** are versioned `<gaitlibVersionBase>-local` and never published by CI.

## Using GaitLib in a robot project

1. Download `GaitLib.json` from the latest GitHub release into the robot project's
   `vendordeps/`. To upgrade later, grab a newer release's JSON (or edit both
   `version` fields).
2. GitHub Packages requires a login even to read, and GradleRIO doesn't pass
   credentials for vendordep repositories, so add this to the robot project's
   `build.gradle`:

   ```groovy
   // Log in to GaitLib's GitHub Packages registry (declared in vendordeps/GaitLib.json).
   repositories.withType(MavenArtifactRepository).configureEach { repo ->
       if (repo.url.toString().startsWith('https://maven.pkg.github.com/Quantum-Wind/')) {
           repo.credentials {
               username = project.findProperty('gpr.user') ?: System.getenv('GITHUB_ACTOR')
               password = project.findProperty('gpr.key') ?: System.getenv('GITHUB_TOKEN')
           }
       }
   }

   // Don't ask WPI's public vendor mirror for com.quantumwind artifacts; that lookup
   // can stall the build for a long time.
   wpi.maven.useFrcMavenVendorCache = false
   ```

3. Each developer adds a GitHub personal access token (classic) with the
   `read:packages` scope to `~/.gradle/gradle.properties`:

   ```properties
   gpr.user=<github username>
   gpr.key=<token>
   ```

   For the robot repo's own CI, its `GITHUB_TOKEN` works once this package grants
   that repo read access (package settings -> "Manage Actions access").

4. Use it:

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

### Trying unreleased changes

Run `./gradlew publishToMavenLocal` here, set both `version` fields in the robot
project's `GaitLib.json` to `<gaitlibVersionBase>-local` (e.g. `0.1-local`), and
add `repositories { mavenLocal() }` to the robot project's `build.gradle`.
Undo both before committing robot code.

To bump the WPILib version, change `wpilibVersion` in `gradle.properties`, the
plugin versions in `build.gradle`, and `frcYear` in `consumer/GaitLib.json`.
Keep it in step with the robot project's GradleRIO.
