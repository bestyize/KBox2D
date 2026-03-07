# KBox2D 执行纪要（时间线版）

> 说明：本文件按本轮对话的实际推进顺序，记录迁移、演示开发、问题修复与验证过程，便于后续快速回溯。

## 阶段 1：迁移任务启动
- 用户要求读取 `DESC.md`，并按 `PLAN.md` 持续迁移 `jbox2d-library` 到 `kbox2d`。
- 用户明确要求：不要反复询问意见，直接持续推进，直到主要阻塞点被清除。
- 迁移目标被确定为：先实现 Kotlin Multiplatform 版本的“主功能可用”，再继续提高与上游实现的一致性。

## 阶段 2：KMP 核心框架落地
- 新建并接入 `kbox2d` KMP 模块，统一包名为 `xyz.thewind.kbox2d`。
- 完成 `common`、`collision`、`callbacks`、`pooling` 等基础模块迁移。
- 打通刚体世界主链：`World`、`Body`、`Fixture`、`ContactManager`、`Island`、`ContactSolver`。
- 完成一批 joint runtime 迁移，包括 `DistanceJoint`、`RevoluteJoint`、`PrismaticJoint`、`WheelJoint`、`GearJoint`、`PulleyJoint` 等。

## 阶段 3：连续碰撞与粒子系统补齐
- 完成 `solveTOI()` 主链，建立连续碰撞的可运行基础。
- 补齐粒子系统主链、粒子组、粒子 contact/body contact、深度计算、shape destroy、query/raycast、listener。
- 增补粒子 flags 主要语义：`wall`、`viscous`、`powder`、`tensile`、`solid group`、`color mixing`、`spring`、`elastic`。
- 将粒子 `spring` / `elastic` 从轻量邻域逻辑升级为更接近上游的 `Pair/Triad` 建模方式。

## 阶段 4：开始制作 Compose 实际演示
- 用户要求把迁移成果落到一个“真实可运行”的 Compose 示例上。
- 应用入口切换到一个简单、直观的掉落演示页面。
- 第一版演示实现为：
  - 一个矩形框内的刚体掉落场景
  - 同时包含球体和矩形
  - 带基础弹性碰撞效果

## 阶段 5：演示交互与玩法增强
- 用户继续要求把示例做得更可玩、更直观。
- 逐步加入以下能力：
  - 点击画布任意位置生成新形状
  - 生成模式切换：`随机模式`、`只生成球`、`只生成方块`
  - 操作按钮：`暂停/继续`、`重置`、`清空掉落物`
- 同时加入障碍物设计：
  - 一个贴着墙的障碍物
  - 多个不贴墙的中心障碍物

## 阶段 6：视觉样式与布局优化
- 用户要求墙面只保留线条，不再绘制厚重的实体墙。
- 页面视觉整体调整为偏“小红书化”的暖粉色系。
- 继续优化页面布局：
  - 顶部信息卡片
  - 控制区卡片
  - 圆角画布容器
  - 安全区与留白优化
  - 底部简短提示文案
- 到这一阶段，演示已具备“可展示、可点击、可重置、可持续生成”的基础体验。

## 阶段 7：暴露核心稳定性问题
- 用户反馈两个关键异常：
  - 墙壁和障碍物运行一段时间后像是被撞飞了
  - 在此之后新生成的球体会向上飞
- 这说明问题不只在演示层，而是物理求解过程中存在更底层的状态污染。

## 阶段 8：定位并修复静态体漂移问题
- 对 `Island` 求解流程进行了重点检查。
- 问题最终定位为：静态体在解算过程中，其临时位置/速度数组可能被接触或约束路径污染，最后又被错误写回。
- 对 `Island.solve(...)` 的修复：
  - 静态体跳过积分
  - 在位置求解前后恢复静态体原始位姿
  - 最终写回时强制清零静态体线速度和角速度
- 对 `Island.solveTOI(...)` 也加入同样的保护逻辑。
- 这一轮修复后，静态边界和障碍物不再因长期碰撞而漂移。

## 阶段 9：补充回归测试防止再次退化
- 新增回归测试 `staticObstacleRemainsFixedAfterRepeatedImpacts()`。
- 该测试覆盖两个重点：
  - 静态障碍物在持续冲击后位置与角度仍保持不变
  - 长时间碰撞之后新创建的动态体依然向下掉落，不会出现反向异常
- 这一步把演示层遇到的问题上升为了引擎层的自动化保障。

## 阶段 10：出现“看起来不掉落”的新问题
- 在上一轮修复后，用户反馈“改动后无法掉落了”。
- 继续排查后确认：物理步进实际上还在执行，但画面没有持续刷新。
- 根因不是物理停止，而是 `Canvas` 区域没有显式依赖逐帧状态，因此没有随着 `world.step(...)` 的推进持续重绘。

## 阶段 11：修复画布不刷新的显示问题
- 让画布区域显式绑定 `frame` 状态。
- 每次 world 步进后，帧号递增，从而触发画布子树重绘。
- 这一步恢复了用户可见的掉落动画。
- 最终结果是：
  - 物理仍正常运行
  - 画面也恢复持续刷新
  - 用户主观体验重新一致

## 阶段 12：验证当前演示状态
- 多次执行以下命令验证当前成果：

```bash
/bin/zsh -lc 'GRADLE_USER_HOME=$PWD/.gradle-user ./gradlew :composeApp:compileKotlinJvm :kbox2d:jvmTest'
```

- 在需要启动桌面演示时执行：

```bash
/bin/zsh -lc 'GRADLE_USER_HOME=$PWD/.gradle-user ./gradlew :composeApp:run'
```

- 当前验证结论：
  - `composeApp` 可编译
  - `kbox2d` 回归测试通过
  - 桌面示例可以启动
  - 掉落、碰撞、点击生成、障碍物与控制按钮均可继续迭代

## 阶段 13：整理执行纪要
- 用户要求把对话保存为执行记录文件，并改名为 `EXEC.md`。
- 已将原先的执行摘要整理并转换为当前这个时间线版本。
- 本文件用于后续继续演进时快速了解“做过什么、为什么这么做、当前停在哪里”。

## 当前状态总结
- `kbox2d` 已具备核心可运行能力，且不是停留在空壳迁移状态。
- `composeApp` 已具备一个可直接运行的刚体掉落演示。
- 本轮对话中已解决的主阻塞点包括：
  - 静态障碍物漂移
  - 新生成球体异常向上飞
  - 画面不刷新的“看起来不掉落”问题
- 当前后续工作已从“主功能修复”转入“体验增强与继续完善”。

## 关键文件
- `DESC.md`
- `PLAN.md`
- `composeApp/src/commonMain/kotlin/xyz/thewind/kbox2d/App.kt`
- `composeApp/src/commonMain/kotlin/xyz/thewind/kbox2d/SimpleDropDemoScreen.kt`
- `kbox2d/src/commonMain/kotlin/xyz/thewind/kbox2d/dynamics/Island.kt`
- `kbox2d/src/commonTest/kotlin/xyz/thewind/kbox2d/dynamics/WorldStepTest.kt`

