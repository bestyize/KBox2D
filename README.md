# KBox2D

**English** | [简体中文](README.zh-CN.md)

KBox2D is an experimental 2D physics engine built with Kotlin Multiplatform. The project aims to bring core JBox2D / Box2D functionality to Kotlin Multiplatform, with runnable examples powered by Compose Multiplatform.

![KBox2D desktop demo](doc/pic/demo.png)

## Maven Central publication

The `kbox2d` engine module is configured for Maven Central under
`io.github.bestyize:kbox2d` (KMP root publication); platform variants are
published alongside it. **Version 0.1.0 is planned, not yet available on
Maven Central.** Once it is published, add the following dependency to a
Kotlin Multiplatform project:

```kotlin
commonMain.dependencies {
    implementation("io.github.bestyize:kbox2d:0.1.0")
}
```

Maintainers: see [Maven Central publishing guide](docs/MAVEN_CENTRAL.md) for
namespace verification, GPG keys, GitHub Secrets, local validation, and
version-tag releases.

## Repository structure

- [`kbox2d/`](kbox2d/) — cross-platform physics engine core
- [`composeApp/`](composeApp/) — Compose Multiplatform demo application
- [`iosApp/`](iosApp/) — iOS host project
- [`doc/`](doc/) — documentation
- [`PLAN.md`](PLAN.md) — migration plan and pending work
- [`EXEC.md`](EXEC.md) — implementation log and timeline

## Current capabilities

Implemented and exercised in the repository:

- **Rigid-body simulation:** `World`, `Body`, and `Fixture`
- **Collision pipeline:** contact management, island solving, and `ContactSolver`
- **Joints:** distance, revolute, prismatic, rope, wheel, gear, pulley, and other joint runtimes
- **Continuous collision detection:** including bullet-body scenarios
- **Particles:** particles and groups, queries, ray casting, destroy-in-shape, group depth, and related operations
- **Compose demo:** boundaries, obstacles, click-to-spawn interactions, pause, reset, and clearing dynamic bodies

## Who is this for?

KBox2D may be useful if you want to:

- Migrate a JBox2D-based application to Kotlin Multiplatform
- Integrate 2D physics into a Compose Multiplatform project
- Explore how Box2D / JBox2D concepts can be implemented in multiplatform Kotlin
- Run and study a working 2D physics example

## Quick start

Run the following commands from the repository root.

### Compile the JVM demo and run core tests

```bash
./gradlew :composeApp:compileKotlinJvm :kbox2d:jvmTest
```

### Run the desktop demo

```bash
./gradlew :composeApp:run
```

### Android / iOS

- **Android:** open the project in Android Studio and run the `composeApp` application.
- **iOS:** on macOS, open the `iosApp/` project in Xcode and run the iOS app.

## Documentation

The detailed guides below are currently written in Chinese:

- [Documentation index](doc/README.md)
- [User manual](doc/USER_MANUAL.md)
- [API overview](doc/API_OVERVIEW.md)
- [Migration guide](doc/MIGRATION_GUIDE.md)
- [Migration plan](PLAN.md)
- [Execution timeline](EXEC.md)

## Suggested reading order

1. [`doc/USER_MANUAL.md`](doc/USER_MANUAL.md)
2. [`doc/API_OVERVIEW.md`](doc/API_OVERVIEW.md)
3. [`doc/MIGRATION_GUIDE.md`](doc/MIGRATION_GUIDE.md)
4. [`SimpleDropDemoScreen.kt`](composeApp/src/commonMain/kotlin/xyz/thewind/kbox2d/SimpleDropDemoScreen.kt)
5. [`WorldStepTest.kt`](kbox2d/src/commonTest/kotlin/xyz/thewind/kbox2d/dynamics/WorldStepTest.kt)
6. [`JointRuntimeTest.kt`](kbox2d/src/commonTest/kotlin/xyz/thewind/kbox2d/dynamics/joints/JointRuntimeTest.kt)
7. [`ParticleSystemTest.kt`](kbox2d/src/commonTest/kotlin/xyz/thewind/kbox2d/particle/ParticleSystemTest.kt)
8. [`PLAN.md`](PLAN.md)
9. [`EXEC.md`](EXEC.md)

## Desktop demo

The current desktop demo shows objects falling inside a bounded rectangular world. It supports:

- Falling circles and boxes
- Static boundaries and obstacles
- Clicking the canvas to create objects
- Random / circle / box spawn modes
- Pause, reset, and removal of dynamic bodies

Entry points:

- [`App.kt`](composeApp/src/commonMain/kotlin/xyz/thewind/kbox2d/App.kt)
- [`SimpleDropDemoScreen.kt`](composeApp/src/commonMain/kotlin/xyz/thewind/kbox2d/SimpleDropDemoScreen.kt)

## Project status

KBox2D is an **ongoing Kotlin Multiplatform physics-engine port**, rather than a finished or API-stable library release. It is intended as a runnable, testable foundation for experimentation, further porting, and eventual packaging and integration.

## License

Original KBox2D contributions are released under the [MIT License](LICENSE). The project draws on JBox2D / Box2D; any upstream copyrights and licenses applicable to adapted third-party materials remain in effect. See [Third-party notices](THIRD_PARTY_NOTICES.md).

## References

- [Box2D documentation](https://box2d.org/documentation/)
- [JBox2D website](http://www.jbox2d.org/)
- [JBox2D on GitHub](https://github.com/jbox2d/jbox2d)
