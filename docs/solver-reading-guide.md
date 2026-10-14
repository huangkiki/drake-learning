# 读懂 DexLab 的 Drake 配置：SAP、接触模型与离散近似

[Sim Learning](https://github.com/huangkiki/sim-learning) · [课程目录](curriculum.md) · [E3 接触模型](contact-models.md) · [E3 求解过程](contact-solvers.md) · [版本](versions.md)

面向已有 VLA、WAM 或 RL 经验的工程师、硕士和博士：读完应能把一行实验标签拆成原生配置，解释哪些变化会改变模型，哪些只改变求解预算，并判断现有证据能支持什么。共同先修按总仓的 [Physics-Based Simulation 学习主线](https://github.com/huangkiki/sim-learning/blob/main/docs/physics-based-simulation.md) 阅读；书中的 IPC 与本章 Drake 接触近似分别核对，不能因为都使用优化就视为同一算法。

本文核对 **Drake 1.57.0**，固定源码 `1e1466ba466e7ce8fa9fcca4e086ce1383e5427d`。DexLab 记录冻结在 `7e4a0774efb4f8b332e866e1495e032b4969c8aa`，版本名称与本课程一致；这不证明课程源码与历史 wheel 的全部构建选项相同。下列片段只有语法、源码与绑定检查，**本课程没有运行原生物理**。[课程源身份](sources.json) · [DexLab 术语登记](https://github.com/huangkiki/Dexlab/blob/7e4a0774efb4f8b332e866e1495e032b4969c8aa/docs/solver-taxonomy.json)

## 1. 一行标签包含四种不同问题

| DexLab 标签 | 它回答什么 | Drake 原生归属 | 不能据此推断 |
| --- | --- | --- | --- |
| `SAP` | 如何求解当前离散约束问题？ | `DiscreteContactSolver.kSap` | 所有接触都使用 `kSap` 近似 |
| `point` | 接触几何与柔顺力如何表示？ | `ContactModel.kPoint` | 理想无穿透刚体接触 |
| `hydroelastic` | 如何由接触面和压力场描述接触？ | 严格模式 `ContactModel.kHydroelastic`；另有显式 fallback 模式 | 标签本身已经记录是否启用 fallback |
| `kSap` | 如何离散接触材料与摩擦？ | `DiscreteContactApproximation.kSap` | 一个与 SAP 并列的新求解器 |
| `kSimilar` | 同上，使用 Similar 近似 | `DiscreteContactApproximation.kSimilar` | 当前 SAP 求解器被替换 |
| `kLagged` | 同上，使用 Lagged 近似 | `DiscreteContactApproximation.kLagged` | 所有力都延迟了一整个物理步 |
| `not qualified` | 某项实验协议的准入证据到哪里？ | DexLab 证据状态，不是引擎字段 | Drake 从未运行，或不支持所登记模型 |

本固定版本的三种离散近似都交给 SAP。原生 getter 直接返回 `DiscreteContactSolver.kSap`；因此“SAP vs kLagged”不是同级算法比较。需要比较的是，例如“相同模型与步长下，SAP 分别解 Lagged 和 Similar 近似所生成的问题”。[枚举及其职责](https://github.com/RobotLocomotion/drake/blob/1e1466ba466e7ce8fa9fcca4e086ce1383e5427d/multibody/plant/multibody_plant.h#L143-L246) · [实际 getter/setter](https://github.com/RobotLocomotion/drake/blob/1e1466ba466e7ce8fa9fcca4e086ce1383e5427d/multibody/plant/multibody_plant.cc#L870-L892)

## 2. 先选接触表示，再看材料和近似

**Point** 用接触位置和局部穿透等量建立柔顺接触力；几何对支持、刚度和耗散仍需确认。**Hydroelastic** 从相交几何的压力场构造接触面，并将面上的作用汇总为力和力矩。它不是“把 point 的迭代次数增加”，也不自动保证比 point 更接近你的真实物体。[接触表示与材料课](contact-models.md)

用一个只保留法向弹性的教学近似对比量纲：

$$f_n\approx k\,\delta,\qquad F_n\approx\int_{S}p(\mathbf{x})\,dA.$$

左式的局部刚度 `k` 是 N/m，穿透 `δ` 是 m；右式的压力 `p` 是 Pa，面积是 m²。Hydroelastic 的 `hydroelastic_modulus` 是 Pa，不能与 point 的 `point_contact_stiffness` [N/m] 直接设成同一个数来宣称“等刚度”。上式省略耗散、摩擦和离散化，仅帮助辨认量纲；实际离散约束还取决于接触面、梯度和材料组合。[材料与几何背景](https://github.com/RobotLocomotion/drake/blob/1e1466ba466e7ce8fa9fcca4e086ce1383e5427d/multibody/plant/contact_model_doxygen.h) · [面片到约束的推导](contact-solvers.md#22-hydroelastic-由面片变成离散约束)

严格 `kHydroelastic` 遇到不支持的接触几何会报错。`kHydroelasticWithFallback` 会在可用时使用 hydroelastic，对部分其他碰撞对尝试 point；它不是“所有失败都能恢复”的承诺。固定源码默认是 fallback 模式，但 DexLab 冻结斜面批次显式选择**严格 hydroelastic**，不能拿默认值补写该批次。[模式声明](https://github.com/RobotLocomotion/drake/blob/1e1466ba466e7ce8fa9fcca4e086ce1383e5427d/multibody/plant/multibody_plant.h#L143-L164) · [冻结斜面报告](https://github.com/huangkiki/Dexlab/blob/7e4a0774efb4f8b332e866e1495e032b4969c8aa/docs/drake-incline-results.zh-CN.md)

三种近似主要改变装入离散问题的接触法则：

| 近似 | 材料与摩擦的关键区别 | 读配置时检查 |
| --- | --- | --- |
| `kSap` | 法向耗散采用线性 Kelvin–Voigt 形式；摩擦正则化由数值参数控制 | `relaxation_time` [s]，以及 near-rigid 数值限制 |
| `kSimilar` | 对 Hunt–Crossley 接触与正则化摩擦的凸近似，法向和切向仍有耦合 | `hunt_crossley_dissipation` [s/m]、stiction tolerance 与步长 |
| `kLagged` | 同一类接触模型的另一近似，摩擦限额使用旧状态计算的法向冲量尺度 | 同上，并检查冲击/强瞬态中的耦合误差 |

`kSap` 与 `kSimilar` 的滑动可能带有随步长变化的 gliding；Lagged 避免该项，但对强瞬态的摩擦耦合有另一种近似。这个模型取舍不等于对任何任务的性能或精度排名；具体方程与适用假设见 [E3 三种近似](contact-solvers.md#3-三种近似的实际法则)。[固定版本的近似说明](https://github.com/RobotLocomotion/drake/blob/1e1466ba466e7ce8fa9fcca4e086ce1383e5427d/multibody/plant/multibody_plant.h#L189-L246)

## 3. SAP 在这个层级解什么

先忽略未参与接触的实现细节，离散一步的动量关系可以写成

$$A(v_{next}-v^*)=J^T\gamma.$$

`v*` 是没有约束冲量时的自由运动速度，`A` 是本步离散动力学的对称正定矩阵，`J` 将广义速度映射到约束速度，`γ` 包含接触及其他约束冲量。平移接触冲量单位 N·s；混合转动/平移的广义量不能都标成 N·s。**SAP 负责找到满足离散模型的速度和冲量；改变近似会改变冲量与约束速度之间的关系。**[原生问题定义](https://github.com/RobotLocomotion/drake/blob/1e1466ba466e7ce8fa9fcca4e086ce1383e5427d/multibody/contact_solvers/sap/sap_contact_problem.h#L29-L55)

优化过程中，Newton 方向来自 Hessian 线性系统；固定默认线性求解类型是 block sparse Cholesky。这里的 Newton、Cholesky、SAP 和 `kLagged` 属于不同层级。`max_iterations` 是本步优化预算，不是子步数；收敛于离散问题也不能证明材料正确或时间步足够细。[参数结构](https://github.com/RobotLocomotion/drake/blob/1e1466ba466e7ce8fa9fcca4e086ce1383e5427d/multibody/contact_solvers/sap/sap_solver.h#L30-L169) · [线性方向求解](https://github.com/RobotLocomotion/drake/blob/1e1466ba466e7ce8fa9fcca4e086ce1383e5427d/multibody/contact_solvers/sap/sap_solver.cc#L701-L715)

这些内部 C++ 参数不都对应公开 pydrake setter；不要从字段名构造不存在的 Python 调参接口。要深入外层迭代、停止条件与线性系统，继续 [E3 求解与时间](contact-solvers.md)。

## 4. 把标签落到原生配置

下面只展示配置层；没有注册物体、材料或 SceneGraph，不能用于运行一个接触实验。`0.001` s 是示例物理步长，不是所有 DexLab 工况的值。

```python
from pydrake.multibody.plant import (
    ContactModel,
    DiscreteContactApproximation,
    MultibodyPlant,
)

plant = MultibodyPlant(time_step=0.001)
plant.set_contact_model(ContactModel.kHydroelastic)
plant.set_discrete_contact_approximation(DiscreteContactApproximation.kLagged)

recorded_choices = {
    "time_step_s": plant.time_step(),
    "contact_model": plant.get_contact_model(),
    "discrete_approximation": plant.get_discrete_contact_approximation(),
    "solver": plant.get_discrete_contact_solver(),
}
# Continue with native geometry/material/model construction, then Finalize().
# This source-reading snippet has not been executed.
```

必须在 `Finalize()` 前设置接触模型和近似；近似 setter 还要求离散 Plant，即 `time_step > 0`。这几个 setter/getter 均可在固定 Python 绑定中核对；不能把连续 Plant 的 Simulator 积分器选择当作这里的离散近似。[setter 前置条件](https://github.com/RobotLocomotion/drake/blob/1e1466ba466e7ce8fa9fcca4e086ce1383e5427d/multibody/plant/multibody_plant.cc#L870-L892) · [Python 绑定](https://github.com/RobotLocomotion/drake/blob/1e1466ba466e7ce8fa9fcca4e086ce1383e5427d/bindings/pydrake/multibody/plant_py.cc#L1233-L1245)

完整的原生配置阅读例子在 [contact_inspection.py](../examples/contact_inspection.py)。它选择 fallback 模式，与上面的严格模式有意不同；阅读时应记录差异，而非把所有 `hydroelastic` 标签折叠为一个实验身份。

## 5. 从故障现象追到应检查的层

| 现象 | 先核对 | 为什么增加迭代可能无效 |
| --- | --- | --- |
| 切到严格 hydroelastic 后报错 | 两侧形状、hydro 属性、刚/柔配对、过滤与模式 | 几何支持问题可能发生在求解器之前 |
| 切换近似后耗散似乎变了 | 近似实际读取 `relaxation_time` 还是 Hunt–Crossley 参数 | 输入模型已经变化，容差不是材料转换器 |
| 优化成功但物体缓慢滑动 | 摩擦正则化、接触法则、材料组合、控制与采样 | 成功只证明达到该次优化的停止条件 |
| 相同数值的 point/hydro 参数产生不同响应 | 单位、接触几何与压力面积积分 | N/m 与 Pa 描述不同对象 |
| 更小步长没有改善误差 | 保持物理工况与观察时间一致，核对模型和实际误差趋势 | 时间分辨率变化不能自动消除模型误差 |
| 日志只有 `SAP`，无法复现 | 补充近似、严格/fallback、步长、材料、版本、控制和观测协议 | 一个算法名不足以确定实验配置 |

与学习流程衔接时，把这组配置与控制频率、动作缩放、随机种子和 observation 时间戳一起保存；策略回报变化不能单独归因于 solver。训练与评估协议继续由 [Robot Learning](https://github.com/huangkiki/robot-learning) 负责。

## 6. 如何读 `not qualified`

冻结 DexLab 斜面报告已有 SAP / kLagged / 严格 hydroelastic 的原生运行：九个正例中六个通过、三个滑动工况失败；无地面负例被正确拒绝。报告同时明确没有取得 `coverage-v1` 可靠任务覆盖资格。因此 `not qualified` 不能改写为“从未运行”，六个通过也不能改写为“六种任务覆盖”。该批次未运行的 `kSap`、`kSimilar` 和其他适用接触模式仍是未运行。[原报告、失败与协议边界](https://github.com/huangkiki/Dexlab/blob/7e4a0774efb4f8b332e866e1495e032b4969c8aa/docs/drake-incline-results.zh-CN.md)

本教程复用该历史来源，不重跑、不新增资格，也不把其滑动误差归因为某个已被证明的 solver 缺陷。确认本次结论时同时读版本、工况、验收协议与失败记录。课程的静态检查记录见 [Learning 迁移与导读验收](evidence/learning-migration-validation.md)。E7 完整安装、原生运行与全路线审校保持独立暂停待办。

## 7. 阅读练习

1. 同事把配置表写成 `solver = SAP / kSap / kLagged`，你如何改列？
2. 从 Lagged 改成 `kSap`，其余材料数字完全不动，是否一定保持同一耗散模型？
3. 日志写 `hydroelastic`，课程例子使用 fallback，能否直接重现冻结的严格模式批次？
4. 将 Newton 上限从 100 改为 200，能否把 1 ms 物理步长视为 0.5 ms？
5. 六个斜面工况通过，但标记 `not qualified`，是否自相矛盾？

<details><summary>答案</summary>

1. 分为 solver（此版本 SAP）、discrete approximation（kSap/kSimilar/kLagged），另列 contact model、step、材料和证据状态。
2. 不是。Lagged 使用 Hunt–Crossley 耗散 [s/m]；kSap 使用 relaxation time [s]，法向模型不同。
3. 不能。先读取实际枚举、几何和材料；fallback 的路径可能与严格模式不同。不能用默认配置覆盖明确登记值。
4. 不能。前者改变同一步优化预算，后者改变时间离散；需要分别记录和评估。
5. 不矛盾。工况通过与满足完整覆盖协议是不同结论，且失败、未运行配置和任务分母都必须保留。

</details>

## English summary

This reading bridge targets engineers and graduate students experienced in VLA, WAM or RL. In pinned Drake 1.57.0, SAP is the solver; `kSap`, `kSimilar` and `kLagged` are discrete contact approximations. Point and hydroelastic describe contact models, with strict hydroelastic distinct from fallback. The native configuration snippet is source-reviewed and syntax-checked only. Frozen DexLab incline evidence includes six passing and three failing positive cases, while reliable task-coverage qualification remains unmet. No new native execution or qualification is claimed; E7 stays paused.
