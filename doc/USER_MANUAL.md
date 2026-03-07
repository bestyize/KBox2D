# KBox2D 用户手册

> 本手册面向“第一次接触这个仓库的人”，帮助你快速理解 KBox2D 是什么、怎么跑、先看哪里、如何写出第一个可运行世界。

## 1. KBox2D 是什么

KBox2D 是一个基于 Kotlin Multiplatform 的 2D 物理引擎工程，目标是把 Box2D / JBox2D 的经典能力迁移到 Kotlin 多平台环境中，并在当前仓库里通过 Compose Multiplatform 提供可运行演示。

当前适用场景包括：

- Android / iOS / Desktop 共用 2D 物理逻辑
- 在 Compose Multiplatform 中接入物理世界
- 从 JBox2D 项目迁移到 Kotlin Multiplatform
- 学习刚体、关节、连续碰撞和粒子系统的 Kotlin 写法

## 2. 仓库里有什么

```text
KBox2D/
├── kbox2d/      # 引擎核心
├── composeApp/  # 演示应用
├── iosApp/      # iOS 容器工程
├── doc/         # 文档
├── PLAN.md      # 迁移计划
└── EXEC.md      # 执行时间线
```

最值得先看的文件：

- `README.md`
- `doc/API_OVERVIEW.md`
- `composeApp/src/commonMain/kotlin/xyz/thewind/kbox2d/SimpleDropDemoScreen.kt`
- `kbox2d/src/commonTest/kotlin/xyz/thewind/kbox2d/dynamics/WorldStepTest.kt`

## 3. 如何快速跑起来

### 编译核心与测试

```bash
./gradlew :composeApp:compileKotlinJvm :kbox2d:jvmTest
```

### 运行桌面演示

```bash
./gradlew :composeApp:run
```

### 平台入口

- Android：在 IDE 中运行 `composeApp`
- Desktop：运行 `:composeApp:run`
- iOS：用 Xcode 打开 `iosApp/`

## 4. 先理解这几个概念

### 4.1 World

`World` 是整个物理世界，负责保存刚体、关节和粒子，并执行每一帧的物理解算。

```kotlin
val world = World(gravity = Vec2(0f, -10f))
```

### 4.2 Body

`Body` 是刚体实例，常见类型有：

- `BodyType.STATIC`
- `BodyType.KINEMATIC`
- `BodyType.DYNAMIC`

一般来说：

- 地面、墙体、障碍物用静态体
- 需要下落、碰撞、受力的物体用动态体

### 4.3 Shape 与 Fixture

Shape 描述几何，Fixture 把几何附着到 Body 并附带物理参数。

常用参数：

- `density`
- `friction`
- `restitution`

常见形状：

- `CircleShape`
- `PolygonShape`
- `EdgeShape`
- `ChainShape`

### 4.4 Joint

Joint 用来约束两个 body 的相对运动，例如距离、转轴、绳索、滑轨。

### 4.5 Particle

当前仓库也支持粒子系统，包括：

- 粒子创建
- 粒子组
- query / raycast
- destroy-in-shape
- 深度计算与多类 flags

## 5. 第一个可运行世界

```kotlin
import xyz.thewind.kbox2d.collision.shapes.CircleShape
import xyz.thewind.kbox2d.common.Vec2
import xyz.thewind.kbox2d.dynamics.BodyDef
import xyz.thewind.kbox2d.dynamics.BodyType
import xyz.thewind.kbox2d.dynamics.FixtureDef
import xyz.thewind.kbox2d.dynamics.World

val world = World(gravity = Vec2(0f, -10f))

val body = world.createBody(BodyDef().apply {
    type = BodyType.DYNAMIC
    position = Vec2(0f, 10f)
})!!

body.createFixture(FixtureDef().apply {
    shape = CircleShape().apply { m_radius = 1f }
    density = 1f
    friction = 0.2f
    restitution = 0.3f
})

repeat(60) {
    world.step(1f / 60f, 8, 3)
    println(body.getPosition())
}
```

## 6. 推荐的步进方式

参考 Box2D 官方建议，当前项目也推荐固定时间步长：

```kotlin
world.step(1f / 60f, 8, 3)
```

推荐理由：

- 结果更稳定
- 与测试一致
- 更容易调参和复现问题

## 7. 当前桌面示例能做什么

当前 demo 是一个矩形区域内的掉落场景，支持：

- 球体和方块掉落
- 静态边界与障碍物
- 点击画布生成新物体
- 随机 / 球 / 方块三种投放模式
- 暂停、重置、清空动态物体

入口位于：

- `composeApp/src/commonMain/kotlin/xyz/thewind/kbox2d/App.kt`
- `composeApp/src/commonMain/kotlin/xyz/thewind/kbox2d/SimpleDropDemoScreen.kt`

## 8. 你接下来应该看什么

- 想写代码：看 `doc/API_OVERVIEW.md`
- 想迁移旧项目：看 `doc/MIGRATION_GUIDE.md`
- 想了解当前迭代历史：看 `EXEC.md`
- 想了解剩余工作：看 `PLAN.md`

## 9. 参考来源

本手册参考了以下官方资料，并结合当前仓库源码、测试和 demo 进行了本地化整理：

- Box2D Manual: https://box2d.org/documentation/
- Box2D Hello World: https://box2d.org/documentation/hello.html
- JBox2D 官方站点: http://www.jbox2d.org/
- JBox2D GitHub: https://github.com/jbox2d/jbox2d

