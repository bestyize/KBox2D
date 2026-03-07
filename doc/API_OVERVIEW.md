# KBox2D API 概览

> 本文档面向“准备直接写代码”的开发者，重点说明当前仓库中最重要的 API、对象关系、常见用法与建议入口。

## 1. 核心命名空间

核心代码位于：

- `kbox2d/src/commonMain/kotlin/xyz/thewind/kbox2d/common`
- `kbox2d/src/commonMain/kotlin/xyz/thewind/kbox2d/collision`
- `kbox2d/src/commonMain/kotlin/xyz/thewind/kbox2d/dynamics`
- `kbox2d/src/commonMain/kotlin/xyz/thewind/kbox2d/dynamics/joints`
- `kbox2d/src/commonMain/kotlin/xyz/thewind/kbox2d/particle`
- `kbox2d/src/commonMain/kotlin/xyz/thewind/kbox2d/callbacks`

## 2. World 相关 API

`World` 是主入口，负责：

- 创建 / 销毁刚体
- 创建 / 销毁关节
- 创建 / 管理粒子系统数据
- 执行 `step(...)`
- 设置 contact listener / filter

### 2.1 创建世界

```kotlin
val world = World(gravity = Vec2(0f, -10f))
```

### 2.2 步进

```kotlin
world.step(1f / 60f, velocityIterations = 8, positionIterations = 3)
```

### 2.3 刚体相关入口

- `createBody(...)`
- `destroyBody(...)`
- `getBodyList()`
- `getBodyCount()`

### 2.4 关节相关入口

- `createJoint(...)`
- `destroyJoint(...)`
- `getJointCount()`

### 2.5 粒子相关入口

- `createParticle(...)`
- `createParticleGroup(...)`
- `destroyParticlesInShape(...)`
- `queryParticleAABB(...)`
- `raycastParticle(...)`
- `setParticleRadius(...)`
- `computeDepthForGroup(...)`

## 3. Body / Fixture 基础模式

### 3.1 创建动态体

```kotlin
val body = world.createBody(BodyDef().apply {
    type = BodyType.DYNAMIC
    position = Vec2(0f, 10f)
})!!
```

### 3.2 创建静态体

```kotlin
val wall = world.createBody(BodyDef().apply {
    type = BodyType.STATIC
    position = Vec2(0f, 0f)
})!!
```

### 3.3 给 body 绑定 fixture

```kotlin
body.createFixture(FixtureDef().apply {
    shape = CircleShape().apply { m_radius = 0.5f }
    density = 1f
    friction = 0.2f
    restitution = 0.4f
})
```

### 3.4 常见形状

- `CircleShape`
- `PolygonShape`
- `EdgeShape`
- `ChainShape`

### 3.5 常见调参建议

- 动态体通常设置 `density > 0`
- 静态体通常使用 `density = 0`
- 弹跳感来自 `restitution`
- 滑动/阻力感来自 `friction`

## 4. 连续碰撞与 bullet

对于高速小物体，建议使用：

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
- 高速球
- 小尺寸快速运动道具

可参考：

- `kbox2d/src/commonTest/kotlin/xyz/thewind/kbox2d/dynamics/WorldStepTest.kt`

## 5. Joint API 概览

当前已迁移并可运行的 joint 包括：

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

### 5.1 DistanceJoint 示例

```kotlin
val joint = world.createJoint(DistanceJointDef().apply {
    initialize(bodyA, bodyB, bodyA.getPosition().clone(), bodyB.getPosition().clone())
})
```

### 5.2 RopeJoint 示例

```kotlin
world.createJoint(RopeJointDef().apply {
    this.bodyA = bodyA
    this.bodyB = bodyB
    maxLength = 2f
})
```

参考测试：

- `kbox2d/src/commonTest/kotlin/xyz/thewind/kbox2d/dynamics/joints/JointRuntimeTest.kt`

## 6. 粒子系统 API 概览

### 6.1 创建单个粒子

```kotlin
val index = world.createParticle(ParticleDef().apply {
    position.set(1f, 5f)
    velocity.set(0f, 0f)
})
```

### 6.2 设置粒子半径

```kotlin
world.setParticleRadius(0.3f)
```

### 6.3 创建粒子组

```kotlin
val group = world.createParticleGroup(ParticleGroupDef().apply {
    shape = CircleShape().apply {
        m_p.setZero()
        m_radius = 0.7f
    }
    position.set(0f, 0f)
})
```

### 6.4 查询粒子

```kotlin
world.queryParticleAABB(ParticleQueryCallback { index ->
    println(index)
    true
}, AABB().apply {
    lowerBound.set(-0.5f, -0.5f)
    upperBound.set(0.5f, 0.5f)
})
```

### 6.5 粒子 raycast

```kotlin
world.raycastParticle(ParticleRaycastCallback { index, _, _, fraction ->
    println("hit=$index fraction=$fraction")
    fraction
}, Vec2(-1f, 0f), Vec2(1f, 0f))
```

### 6.6 按形状销毁粒子

```kotlin
val destroyed = world.destroyParticlesInShape(
    CircleShape().apply {
        m_p.setZero()
        m_radius = 0.6f
    },
    Transform(),
)
```

### 6.7 当前支持的粒子能力

- particle contacts
- body contacts
- destruction listener
- query / raycast
- depth for group
- `wall`、`viscous`、`powder`、`tensile`
- `spring`、`elastic`
- rigid group / solid group
- color mixing

参考测试：

- `kbox2d/src/commonTest/kotlin/xyz/thewind/kbox2d/particle/ParticleSystemTest.kt`

## 7. Callback 与扩展点

当前你最常会接触到：

- `ContactListener`
- `ContactFilter`
- `ParticleQueryCallback`
- `ParticleRaycastCallback`
- `ParticleDestructionListener`

这些接口适合做：

- 自定义碰撞过滤
- 命中统计
- 粒子查询
- 粒子命中检测
- 销毁监听

## 8. Demo 代码入口

如果你想看“API 如何在真实界面里接起来”，最值得看的文件是：

- `composeApp/src/commonMain/kotlin/xyz/thewind/kbox2d/SimpleDropDemoScreen.kt`

当前它演示了：

- 世界创建
- 边界与障碍物
- 动态体生成
- 点击交互
- 画布坐标转换
- 逐帧 step 与重绘

## 9. 最推荐的源码阅读顺序

1. `kbox2d/src/commonTest/kotlin/xyz/thewind/kbox2d/dynamics/WorldStepTest.kt`
2. `kbox2d/src/commonTest/kotlin/xyz/thewind/kbox2d/dynamics/joints/JointRuntimeTest.kt`
3. `kbox2d/src/commonTest/kotlin/xyz/thewind/kbox2d/particle/ParticleSystemTest.kt`
4. `composeApp/src/commonMain/kotlin/xyz/thewind/kbox2d/SimpleDropDemoScreen.kt`
5. `kbox2d/src/commonMain/kotlin/xyz/thewind/kbox2d/dynamics/World.kt`

## 10. 设计建议

参考 Box2D 官方手册，在 KBox2D 中也建议：

- 使用固定时间步长
- 不要把像素直接当物理单位
- 动态体尺寸保持在合理范围
- 高速小物体使用 `bullet = true`
- 把边界、平台、障碍物建模为静态体
- 先看测试，再写业务代码

## 11. 官方参考链接

- Box2D Manual: https://box2d.org/documentation/
- Box2D Simulation: https://box2d.org/documentation/md_simulation.html
- JBox2D GitHub: https://github.com/jbox2d/jbox2d

