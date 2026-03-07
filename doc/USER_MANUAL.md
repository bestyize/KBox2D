# KBox2D 用户手册

> 本手册参考了 Box2D 官方用户手册与 JBox2D 官方项目说明，并结合当前仓库里 `kbox2d` 的实际 API、测试用例和 Compose 示例整理而成。
>
> 目标不是逐字翻译上游文档，而是帮助你在这个仓库里**快速理解、运行并使用 KBox2D**。

## 1. KBox2D 是什么

KBox2D 是一个基于 Kotlin Multiplatform 的 2D 物理引擎项目，目标是把 JBox2D / Box2D 的经典能力迁移到 Kotlin 多平台环境中，供以下场景复用：

- Compose Multiplatform 演示与应用
- Android / iOS / Desktop 共享物理逻辑
- 2D 刚体、关节、连续碰撞、粒子系统实验
- 从 JBox2D 项目迁移到 Kotlin Multiplatform

当前仓库中：

- 核心引擎模块在 `kbox2d/`
- 演示应用在 `composeApp/`
- iOS 容器工程在 `iosApp/`

## 2. 参考来源与术语基础

本手册主要参考以下官方资料：

- Box2D 官方手册：刚体、fixture、joint、仿真步进、连续碰撞等核心概念
- Box2D 官方 Hello World：最小世界、body 创建、step 方式
- JBox2D 官方说明：Java / JVM 侧 API 风格与项目组织

KBox2D 的多数核心命名与 Box2D / JBox2D 保持一致，例如：

- `World`
- `BodyDef`
- `BodyType`
- `FixtureDef`
- `CircleShape`
- `PolygonShape`
- `JointDef`
- `DistanceJointDef`

如果你熟悉 Box2D 或 JBox2D，上手 KBox2D 的成本会很低。

## 3. 仓库结构速览

```text
KBox2D/
├── kbox2d/      # 物理引擎核心
├── composeApp/  # Compose 多平台演示
├── iosApp/      # iOS 容器工程
├── doc/         # 本地文档
├── PLAN.md      # 迁移计划
└── EXEC.md      # 执行纪要
```

重点路径：

- 核心源码：`kbox2d/src/commonMain/kotlin/xyz/thewind/kbox2d`
- 测试用例：`kbox2d/src/commonTest/kotlin/xyz/thewind/kbox2d`
- Demo 入口：`composeApp/src/commonMain/kotlin/xyz/thewind/kbox2d/App.kt`
- 当前演示：`composeApp/src/commonMain/kotlin/xyz/thewind/kbox2d/SimpleDropDemoScreen.kt`

## 4. 快速开始

### 4.1 编译核心与测试

```bash
./gradlew :composeApp:compileKotlinJvm :kbox2d:jvmTest
```

### 4.2 运行桌面演示

```bash
./gradlew :composeApp:run
```

### 4.3 Android / iOS

- Android：使用 IDE 运行 `composeApp`
- iOS：用 Xcode 打开 `iosApp/` 运行

## 5. 使用前需要理解的几个关键概念

根据 Box2D 官方手册，2D 物理引擎最重要的不是“画什么”，而是“怎样组织仿真对象”。在 KBox2D 里，建议先理解以下对象层级：

### 5.1 World

`World` 是整个物理世界，负责：

- 保存所有 body、joint、particle
- 执行仿真步进 `step(...)`
- 处理碰撞、约束与粒子求解

典型创建方式：

```kotlin
val world = World(gravity = Vec2(0f, -10f))
```

### 5.2 Body

`Body` 是刚体实例。常见类型：

- `BodyType.STATIC`：静态体，不参与常规动力学积分，通常用于地面、墙、障碍物
- `BodyType.KINEMATIC`：运动学刚体，通常由代码直接控制速度
- `BodyType.DYNAMIC`：动态体，受重力、碰撞、力和约束影响

### 5.3 Shape 与 Fixture

Shape 只描述几何；Fixture 把形状附着到 body，并附带物理属性，例如：

- `density`
- `friction`
- `restitution`
- 过滤信息和传感器属性

常见形状：

- `CircleShape`
- `PolygonShape`
- `EdgeShape`
- `ChainShape`

### 5.4 Joint

Joint 用来约束两个刚体之间的相对运动，例如：

- 距离约束
- 转轴
- 滑轨
- 绳索
- 轮子
- 齿轮

### 5.5 Particle

当前项目也支持粒子系统能力，包括：

- 单粒子创建
- 粒子组
- 多种粒子 flags
- group 深度、query、raycast、destroy-in-shape

## 6. 第一个刚体世界

下面是一个最小可运行思路：创建一个世界、一个动态圆，然后不断 step。

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
    world.step(1f / 60f, velocityIterations = 8, positionIterations = 3)
    println(body.getPosition())
}
```

这也是当前测试中使用的基本模式。

## 7. 创建边界与障碍物

地面、墙壁、平台等通常使用静态体：

```kotlin
val ground = world.createBody(BodyDef().apply {
    type = BodyType.STATIC
    position = Vec2(0f, -1f)
})!!

ground.createFixture(FixtureDef().apply {
    shape = PolygonShape().apply { setAsBox(10f, 1f) }
    density = 0f
    friction = 0.4f
})
```

这类对象通常：

- 不需要设置密度
- 不应该依赖碰撞被“推走”
- 更适合作为边界、平台、障碍、地形

当前项目已经针对“静态障碍物长期受撞击后漂移”做过专门修复。

## 8. 推荐的 step 策略

Box2D 官方长期推荐固定时间步长。对于 KBox2D，也建议：

- 固定 `dt = 1f / 60f`
- `velocityIterations` 使用 `8`
- `positionIterations` 使用 `3`

示例：

```kotlin
world.step(1f / 60f, 8, 3)
```

这样做的好处：

- 更稳定
- 行为更可预测
- 与测试和现有 demo 保持一致

## 9. 连续碰撞与 bullet

对于高速运动物体，普通离散碰撞可能发生“穿透”。

KBox2D 已支持连续碰撞主链，典型做法是为高速物体设置：

```kotlin
val bullet = world.createBody(BodyDef().apply {
    type = BodyType.DYNAMIC
    position = Vec2(-10f, 0f)
    linearVelocity = Vec2(120f, 0f)
    bullet = true
})!!
```

这适用于：

- 子弹
- 高速小球
- 窄小高速道具

## 10. Joint 用法

### 10.1 Distance Joint

```kotlin
val bodyA = world.createBody(BodyDef().apply {
    type = BodyType.DYNAMIC
    position = Vec2(0f, 10f)
})!!

val bodyB = world.createBody(BodyDef().apply {
    type = BodyType.DYNAMIC
    position = Vec2(3f, 10f)
})!!

val joint = world.createJoint(DistanceJointDef().apply {
    initialize(bodyA, bodyB, bodyA.getPosition().clone(), bodyB.getPosition().clone())
})
```

用途：

- 吊绳
- 弹性连接
- 保持两个点之间的距离

### 10.2 Rope Joint

```kotlin
world.createJoint(RopeJointDef().apply {
    bodyA = bodyA
    bodyB = bodyB
    maxLength = 2f
})
```

用途：

- 限制最大距离
- 做绳索或拉拽上限

### 10.3 其他当前已迁移的关节

当前仓库已迁移并具备运行能力的 joint 包括但不限于：

- `DistanceJoint`
- `RevoluteJoint`
- `PrismaticJoint`
- `WheelJoint`
- `GearJoint`
- `PulleyJoint`
- `RopeJoint`
- `WeldJoint`
- `MotorJoint`
- `MouseJoint`
- `FrictionJoint`

如果你想看更贴近真实代码的示例，优先参考：

- `kbox2d/src/commonTest/kotlin/xyz/thewind/kbox2d/dynamics/joints/JointRuntimeTest.kt`

## 11. 粒子系统

当前 KBox2D 已支持粒子系统的一批核心能力。

### 11.1 创建单个粒子

```kotlin
val world = World(gravity = Vec2(0f, -10f))

val index = world.createParticle(ParticleDef().apply {
    position.set(1f, 5f)
    velocity.set(0f, 0f)
})
```

### 11.2 设置粒子半径

```kotlin
world.setParticleRadius(0.3f)
```

### 11.3 创建粒子组

```kotlin
val group = world.createParticleGroup(ParticleGroupDef().apply {
    shape = CircleShape().apply {
        m_p.setZero()
        m_radius = 0.7f
    }
    position.set(0f, 0f)
})
```

### 11.4 已支持的典型能力

- `createParticle(...)`
- `createParticleGroup(...)`
- `joinParticleGroups(...)`
- `destroyParticlesInShape(...)`
- `queryParticleAABB(...)`
- `raycastParticle(...)`
- `computeDepthForGroup(...)`
- contact / body-contact buffer 访问
- destruction listener

### 11.5 已支持的粒子语义

当前项目已迁移并验证过一批常见 flags：

- `b2_wallParticle`
- `b2_viscousParticle`
- `b2_powderParticle`
- `b2_tensileParticle`
- `b2_springParticle`
- `b2_elasticParticle`
- solid group / rigid group
- color mixing

如果你要写粒子相关功能，推荐直接先看：

- `kbox2d/src/commonTest/kotlin/xyz/thewind/kbox2d/particle/ParticleSystemTest.kt`

## 12. 查询、销毁与回调

### 12.1 粒子 AABB 查询

```kotlin
val queried = mutableListOf<Int>()
world.queryParticleAABB(ParticleQueryCallback { index ->
    queried += index
    true
}, AABB().apply {
    lowerBound.set(-0.5f, -0.5f)
    upperBound.set(0.5f, 0.5f)
})
```

### 12.2 粒子 Raycast

```kotlin
world.raycastParticle(ParticleRaycastCallback { index, _, _, fraction ->
    println("hit particle=$index fraction=$fraction")
    fraction
}, Vec2(-1f, 0f), Vec2(1f, 0f))
```

### 12.3 按形状销毁粒子

```kotlin
val destroyed = world.destroyParticlesInShape(
    CircleShape().apply {
        m_p.setZero()
        m_radius = 0.6f
    },
    Transform(),
)
```

### 12.4 碰撞与销毁监听

当前世界层已经提供：

- `setContactFilter(...)`
- `setContactListener(...)`
- `ParticleDestructionListener`

如果你需要做：

- 触发器逻辑
- 命中统计
- 自定义过滤
- 粒子销毁回调

这些 API 是主要入口。

## 13. 单位、比例与建模建议

参考 Box2D 官方手册，建议继续使用 MKS 思路：

- 长度用“米”级抽象单位
- 不要直接把像素当物理单位
- 典型动态体尺寸保持在合理范围
- 避免极大或极小的 shape

对于 UI / 画布：

- 物理世界和屏幕坐标建议通过 viewport 做转换
- 当前 `SimpleDropDemoScreen.kt` 已包含一个简化版 `SimpleViewport`

## 14. 当前项目里的建议阅读顺序

如果你是第一次接触本项目，推荐按下面顺序读：

1. `README.md`
2. `doc/USER_MANUAL.md`
3. `composeApp/src/commonMain/kotlin/xyz/thewind/kbox2d/SimpleDropDemoScreen.kt`
4. `kbox2d/src/commonTest/kotlin/xyz/thewind/kbox2d/dynamics/WorldStepTest.kt`
5. `kbox2d/src/commonTest/kotlin/xyz/thewind/kbox2d/dynamics/joints/JointRuntimeTest.kt`
6. `kbox2d/src/commonTest/kotlin/xyz/thewind/kbox2d/particle/ParticleSystemTest.kt`
7. `PLAN.md`
8. `EXEC.md`

## 15. 当前状态与已知范围

当前仓库已经具备：

- 刚体世界主链可运行
- 多类 joint 可运行
- 连续碰撞主链可运行
- 粒子系统主链可运行
- Compose 多平台演示已接通

同时也要明确：

- 这是一个迁移中的工程，而不是“已经发布到 Maven Central 的稳定库”
- 文档优先服务于本仓库当前代码，而不是声称与所有上游细节 100% 完全一致
- 最准确的用法样例，始终是测试与 demo 源码

## 16. 与上游手册的关系

建议这样理解三者关系：

- **Box2D 官方手册**：概念与最佳实践的第一参考
- **JBox2D 项目说明**：JVM / Java 风格 API 的迁移参照
- **KBox2D 本手册**：结合本仓库当前实现的实际落地说明

当三者有细节差异时，优先级建议为：

1. 当前仓库源码与测试
2. Box2D 官方概念模型
3. JBox2D 历史 API 习惯

## 17. 官方参考链接

- Box2D Manual: https://box2d.org/documentation/
- Box2D Hello World: https://box2d.org/documentation/hello.html
- Box2D Simulation: https://box2d.org/documentation/md_simulation.html
- JBox2D 官方站点: http://www.jbox2d.org/
- JBox2D GitHub: https://github.com/jbox2d/jbox2d

