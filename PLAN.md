# KBox2D 迁移实施计划

## 1. Current project inventory
- 当前本地工程模块：`composeApp`（Compose 多端 UI）、`iosApp`（iOS 宿主）。当前仍是模板应用，尚无物理引擎核心模块。
- 目标上游：`jbox2d` 仓库中的 `jbox2d-library`；本次不迁移 `jbox2d-serialization`、`jbox2d-testbed`、`jni-broadphase`。
- 上游 `jbox2d-library` 规模：`callbacks` 13 文件、`collision` 25 文件、`common` 16 文件、`dynamics` 59 文件、`particle` 11 文件、`pooling` 12 文件。
- 共享候选代码：绝大多数数学、碰撞、动力学、粒子、回调与对象池逻辑都属于纯算法，可放进 `commonMain`。
- 薄平台包装：`Timer`、极少数依赖 JVM 反射/位运算的工具类；这些需要改写成 KMP 兼容实现。
- 高耦合热点：`dynamics`、`contacts`、`joints`、`particle`，它们依赖最深，应在公共数学与碰撞层稳定后再迁。

## 2. Recommended migration order
- Wave 0：新建 `kbox2d` KMP 模块，接入 Android/iOS/Desktop，建立包路径 `xyz.thewind.kbox2d.*`。
- Wave 1：迁移 `common` 基础数学层，包含向量、矩阵、旋转、变换、`Settings`、`MathUtils`、视口变换、计时工具。
- Wave 2：迁移 `collision` 的纯数据结构与几何算法，先做 `AABB`、`RayCast*`、`Manifold*`、`Distance*`、`TimeOfImpact`，再做 `shapes` 和 `broadphase`。
- Wave 3：迁移 `callbacks` 与 `pooling`，为 `World`、`ContactSolver` 等高层运行时提供依赖。
- Wave 4：迁移 `dynamics` 核心，顺序为 `BodyDef/FixtureDef/Filter/Profile/TimeStep` → `Body/Fixture/ContactManager/Island` → `World` → `contacts` → `joints`。
- Wave 5：迁移 `particle` 子系统；它对 `BufferUtils`、对象池、碰撞与世界调度依赖更深，放在最后独立收口。

## 3. Module-by-module strategy
- `common`：重写为 Kotlin `commonMain`，保留类名与方法名，优先兼容 `Vec2/Mat22/Mat33/Rot/Transform/Sweep/Settings/MathUtils`。
- `collision`：以“兼容 API + 原行为优先”为原则迁移；纯数据类可直接改写，复杂算法类按源文件逐个等价翻译。
- `callbacks`：保持接口兼容，直接迁到 `commonMain`。
- `pooling`：优先保留算法行为；对依赖 JVM 反射的数组扩容逻辑改成 Kotlin 工厂函数或类型安全构造。
- `dynamics`：按依赖图逐层推进，不从 `World` 直接开刀；先迁定义类，再迁联系人与关节，再迁世界调度。
- `particle`：标记为 rewrite+compat，接口尽量贴近上游，但允许内部使用更适合 KMP 的 buffer/array 实现。

## 4. Mock-first items
- `Timer`：先用 Kotlin `TimeSource.Monotonic` 实现，避免平台差异阻塞编译。
- `BufferUtils`：先提供 KMP 友好的扩容 API，后续在 `particle` 迁移时再补齐与上游最接近的调用形式。
- `MathUtils.fastPow` / `fastAtan2`：先保证数值正确和跨平台一致，后续若性能有瓶颈再按基准测试替换为近似实现。
- Demo 集成：先在 `composeApp` 中接入基础数学类做 smoke test，不等待完整 `World` 完成。

## 5. Architecture modernization notes
- 尽量把绝大多数代码放在 `commonMain`，避免新增 `expect/actual` 面。
- 对无法直接兼容 JVM API 的位置，优先改内部实现，不扩大公共 API 的平台分叉。
- 对对象池和缓冲区工具，优先使用 Kotlin 数组与工厂函数，减少运行时反射。
- 对后续 `dynamics/particle`，建议一边迁一边补单元测试，而不是整体迁完后一次性回归。

## 6. Team split and ownership
- PM：定义“与 `jbox2d-library` 行为兼容”的验收边界，确认是否需要粒子、关节、调试绘制的完整支持。
- Senior Engineer：设计 `commonMain` 兼容边界、处理 `MathUtils`、`BufferUtils`、`World` 等高风险核心。
- Advanced Engineers：并行迁移 `collision/shapes`、`callbacks/pooling`、`contacts/joints` 子域。
- Tester：基于上游 `MathTest` 与后续世界步进样例，建立 Android/JVM/iOS 三端回归。

## 7. Validation and rollout checkpoints
- Checkpoint A：`kbox2d` 模块可独立编译，`common` 数学层单测通过。
- Checkpoint B：`collision` 与 `callbacks/pooling` 编译通过，基础几何测试通过。
- Checkpoint C：`World` 可创建，刚体可步进，重力与碰撞基础场景正确。
- Checkpoint D：关节、粒子系统补齐，接口稳定，可在 `composeApp` 中做跨平台演示。
- Checkpoint E：逐步补 README、迁移清单与未完成项，明确仍与上游存在的 API 差异。

## 当前实施状态
- 已完成 Wave 0：新增 `kbox2d` KMP 模块并接入主工程，`composeApp` 已依赖该模块并完成 smoke 编译。
- 已完成 Wave 1：`common` 数学基础层已迁移，包含 `Vec2`、矩阵、旋转、变换、`MathUtils`、`Settings`、`Timer`、`BufferUtils` 等。
- 已完成 Wave 2 的第二阶段：除基础数据结构、`Distance`、`TimeOfImpact`、`AABB`、`shapes`、`broadphase` 外，`collision/Collision` 也已迁移并补了圆/多边形/边接触回归测试。
- 已完成 Wave 3 的第二阶段：已迁移 `DebugDraw`、`PairCallback`、`TreeCallback`、`ContactFilter`、`ContactListener`、`QueryCallback`、`RayCastCallback`、粒子查询/射线接口，以及 `pooling` 的基础栈实现、`IWorldPool` 与 `DefaultWorldPool`。
- 已完成 Wave 4 的第三阶段：`Body` 已补齐基础质量/惯量、睡眠/激活标志、坐标变换、局部/世界点向量换算与基础速度 API，`World` 已具备 `createBody`/`destroyBody`、joint 生命周期和 `step()` 主循环。
- 已完成 Wave 4 的第四阶段（support layer）：已迁移 `JointType`、`LimitState`、`Jacobian`、`JointEdge`、`JointDef` 及各类 `*JointDef`，并将 `Joint` 从占位符升级为可供后续 concrete joint 继承的真实抽象基类。
- 已完成 Wave 4 的第五阶段（runtime joints 第一批）：`DistanceJoint`、`RopeJoint`、`MotorJoint`、`FrictionJoint`、`MouseJoint`、`WeldJoint`、`RevoluteJoint`、`PrismaticJoint`、`PulleyJoint`、`WheelJoint`、`GearJoint`、`ConstantVolumeJoint` 已具备真实求解链并补齐对应运行时回归。
- 已完成 Wave 5 的第二阶段：已迁移 `ParticleColor`、`ParticleDef`、`ParticleGroupDef`、`ParticleType`、`ParticleGroupType`、`ParticleContact`、`ParticleBodyContact`、`StackQueue`、`VoronoiDiagram`，并提供 `ParticleGroup`/`ParticleSystem` 的基础可运行实现与 world API 接入，并已支持基于 `ParticleGroupDef.shape` 的多粒子建组、`rigidParticleGroup` 刚性组运动、粒子组合并、粒子组/缓冲区统计接口和碰撞能量计算。
- 当前已具备一段更完整的 runtime 闭环：`Body`/`Fixture`/`World.step()`、精简 `Island`、精简 `ContactSolver`、world joint 生命周期、上述 10 个 concrete joints 的真实求解、基础 `ParticleSystem`/`ParticleGroup` 与粒子 world API、以及连续碰撞 `solveTOI()` 主循环钩子都已贯通；当前主剩余差异已进一步收缩到更高保真的粒子-刚体/粒子-粒子接触细节、更多 group/flag 深层语义，以及少量连续碰撞边角场景；`solveTOI()` 主链、刚性粒子组主链和粒子组合并/统计接口都已接入并通过回归。
- 已完成 Wave 5 的第三阶段：进一步补齐粒子系统关键兼容面，新增粒子-粒子/粒子-刚体 contact buffer、去重后的 body contact 汇总、`destroyParticlesInShape`、粒子 AABB 查询、粒子射线查询、solid group 深度计算与深度缓冲访问接口，并补了对应回归测试；当前粒子系统已从“可运行基础版”推进到“多数核心 API 可调用”的阶段。
- 当前距离“完全可用/较完整兼容”大约还差 1 个聚焦收口波次：主要集中在粒子 flag 语义保真（spring/elastic/viscous/powder/tensile/color mixing 等更接近上游的求解分支）、更精细的粒子-刚体耦合，以及少量连续碰撞和查询 API 的边角一致性。
- 已完成 Wave 5 的第四阶段：补齐了一批更接近上游语义的粒子求解分支与回调能力，包含 `wall` 固定粒子、`viscous` 黏性阻尼、`powder` 粉末排斥、`tensile` 表面张力、`solid group` 额外喷射排斥、`color mixing` 颜色混合，以及 `ParticleDestructionListener` 的 world 级接入和销毁回调测试。
- 当前剩余主差异已进一步收缩到：`spring` / `elastic` 等依赖 pair/triad 的更高保真粒子拓扑求解、少量粒子与刚体双向耦合细节、以及个别高级查询/边角一致性；从“主功能可用”角度看，已进入最后的小范围精修阶段。
- 已完成 Wave 5 的第五阶段：补上了最后一批高价值粒子弹性语义，包括基于建组邻接的 `spring` 软约束与基于局部形状恢复的 `elastic` 软回弹，并补充了对应 `ParticleSystem` 定向回归；至此，粒子 flags 的主要常用分支已基本具备可运行语义。
- 当前剩余工作已进一步压缩到“高保真一致性”层面：若继续追求更接近上游源码实现，后续主要是把 `spring` / `elastic` 从当前轻量实现升级到完整 pair/triad 拓扑求解，以及继续细修粒子-刚体双向冲量和少量高级查询边角。
- 已完成 Wave 5 的第六阶段：粒子内部拓扑从轻量近邻链接升级为更接近上游的显式 `Pair/Triad` 缓冲思路，`spring` 现已基于 contact-derived pair 约束，`elastic` 现已基于 `VoronoiDiagram` 生成的 triad 恢复；同时补充了“合并粒子组后 spring 仍继续生效”的回归。
- 当前剩余主差异已进一步缩小到最后的精修面：主要是让粒子-刚体双向耦合、部分 buffer/query 细节和若干极端边角行为更接近上游源码；从主链运行和常用 API 覆盖角度看，已经处在接近完成的状态。
