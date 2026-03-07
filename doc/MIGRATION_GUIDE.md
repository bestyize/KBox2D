# JBox2D / Box2D 到 KBox2D 迁移指南

> 本文档面向“准备把现有 JBox2D / Box2D 代码迁到 KBox2D”的开发者，重点说明术语映射、迁移顺序、模块对照和当前项目的实际边界。

## 1. 迁移目标

当前仓库的目标不是机械式改名，而是：

- 保留 Box2D / JBox2D 的核心概念模型
- 将实现迁移到 Kotlin Multiplatform
- 让 Android / iOS / Desktop 共用同一套物理核心
- 在迁移过程中用测试和 demo 持续验证主链可运行

## 2. 迁移时应该怎样理解三者关系

建议这样理解：

- **Box2D 官方手册**：概念和最佳实践的第一来源
- **JBox2D**：Java / JVM 环境下的 API 习惯与旧工程参考
- **KBox2D**：当前仓库中的 Kotlin Multiplatform 落地实现

发生差异时，建议优先级：

1. 当前仓库源码与测试
2. Box2D 官方概念模型
3. JBox2D 历史代码习惯

## 3. 常见命名映射

多数情况下，你可以按接近 1:1 的方式迁移：

- `World` -> `World`
- `BodyDef` -> `BodyDef`
- `BodyType` -> `BodyType`
- `FixtureDef` -> `FixtureDef`
- `CircleShape` -> `CircleShape`
- `PolygonShape` -> `PolygonShape`
- `JointDef` -> `JointDef`
- `DistanceJointDef` -> `DistanceJointDef`
- `Vec2` -> `Vec2`

这意味着从 JBox2D 迁移时，很多业务层代码结构都可以保留。

## 4. 推荐迁移顺序

### 第 1 步：先迁移数据建模

优先迁移这些对象：

- `Vec2`
- `BodyDef`
- `FixtureDef`
- 常用 `Shape`
- 业务里自己的 world wrapper

### 第 2 步：再迁移 world 创建与 step 流程

先把下面这一层跑通：

- 创建 world
- 创建 body
- 创建 fixture
- 固定步长 `step(...)`
- 读回位置与角度

### 第 3 步：迁移静态边界与主要动态体

优先迁移：

- 地面
- 墙壁
- 平台
- 球 / 箱子 / 主角 body

### 第 4 步：迁移 joint

如果旧工程大量依赖 joint，建议按使用频率迁移，而不是一次性全部替换。

推荐优先级：

1. `DistanceJoint`
2. `RevoluteJoint`
3. `PrismaticJoint`
4. `RopeJoint`
5. `WheelJoint`
6. 其他业务相关 joint

### 第 5 步：最后迁移粒子系统与高级特性

粒子、复杂过滤、特殊回调、debug draw 等可以放在后面。

## 5. 从 Java / JBox2D 到 Kotlin 的写法变化

JBox2D 代码往往更偏 Java 风格；迁移到 KBox2D 后，常见变化包括：

- 更多使用 `apply { ... }`
- 更多使用不可空与空安全
- 更自然地复用共享逻辑到 KMP 的 `commonMain`
- 更适合用测试驱动去验证迁移正确性

例如：

```kotlin
val body = world.createBody(BodyDef().apply {
    type = BodyType.DYNAMIC
    position = Vec2(0f, 10f)
})!!
```

## 6. 迁移时最常见的坑

### 6.1 直接把像素当物理单位

这会导致：

- 数值范围异常
- 抖动
- 速度和尺寸失真

建议继续遵循 Box2D 的 MKS 风格思路。

### 6.2 使用不稳定的可变时间步长

建议优先固定：

```kotlin
world.step(1f / 60f, 8, 3)
```

### 6.3 忽略高速物体的连续碰撞需求

对于高速小物体，建议：

```kotlin
bullet = true
```

### 6.4 让静态障碍物承担“会被推走”的业务语义

静态体本质上应该代表世界几何或不可动边界，不应依赖碰撞去改变位置。

### 6.5 不先读测试，直接大面积改业务

在当前仓库里，最准确的 API 示例不是 README，而是测试与 demo。

## 7. 当前仓库里哪些内容适合作为迁移参照

### 刚体主链

- `kbox2d/src/commonTest/kotlin/xyz/thewind/kbox2d/dynamics/WorldStepTest.kt`

### Joint 主链

- `kbox2d/src/commonTest/kotlin/xyz/thewind/kbox2d/dynamics/joints/JointRuntimeTest.kt`

### 粒子系统

- `kbox2d/src/commonTest/kotlin/xyz/thewind/kbox2d/particle/ParticleSystemTest.kt`

### UI 集成方式

- `composeApp/src/commonMain/kotlin/xyz/thewind/kbox2d/SimpleDropDemoScreen.kt`

## 8. 当前项目的迁移现状

当前仓库已经完成并接通了：

- 刚体世界主链
- 多类 joint runtime
- 连续碰撞主链
- 粒子系统主链
- Compose 演示应用

这意味着它已经适合：

- 作为迁移参考项目
- 作为二次封装基础
- 作为业务试接入原型

## 9. 如何评估你的旧项目适不适合迁移

如果你的旧项目满足以下条件，就很适合迁移到 KBox2D：

- 物理逻辑主要依赖 Box2D / JBox2D 的标准对象模型
- Android 与 iOS / Desktop 之间想共享同一套物理逻辑
- 愿意把部分旧的 Java 风格写法重构成 Kotlin 风格
- 有测试或可视化 demo 可用来做迁移验证

## 10. 推荐迁移策略

最稳妥的策略通常不是“一次性全量替换”，而是：

- 先做最小可运行 world
- 再接通主场景刚体
- 再接通核心 joint
- 再接通粒子和高级特性
- 每一步都跑测试或可视化 demo 验证

## 11. 相关本地文档

- `doc/USER_MANUAL.md`
- `doc/API_OVERVIEW.md`
- `PLAN.md`
- `EXEC.md`

## 12. 官方参考链接

- Box2D Manual: https://box2d.org/documentation/
- Box2D Hello World: https://box2d.org/documentation/hello.html
- Box2D Simulation: https://box2d.org/documentation/md_simulation.html
- JBox2D 官方站点: http://www.jbox2d.org/
- JBox2D GitHub: https://github.com/jbox2d/jbox2d

