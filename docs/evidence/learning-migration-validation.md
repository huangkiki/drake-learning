# Learning 迁移与 Drake 配置导读：静态验收

日期：2026-10-14。基线 `f3e069ace46b4d6a83ca132528a206c993550e77`；本记录覆盖当前本地文档修改，不宣称提交、PR 或合入完成。关联 [Learning 迁移](https://github.com/huangkiki/sim-learning/issues/1)、[solver 教程](https://github.com/huangkiki/sim-learning/issues/3)；[E7 #8](https://github.com/huangkiki/drake-learning/issues/8) 保持暂停。

## 修改与来源

- 将现行标题和导航更新为 Drake Learning / Sim Learning，现行 URL 使用改名后路径。
- 新增[配置导读](../solver-reading-guide.md)，覆盖 SAP、point、hydroelastic、kSap、kSimilar、kLagged 与 not qualified；接触表示、离散近似、求解算法与资格状态分别解释。
- 中英文 README、课程、版本与 roadmap 提供入口，并保留 E7 暂停状态。共同教材作为先修链接；未将 IPC 等同于 Drake 接触模型。
- Drake 固定源码仍为 `1e1466ba466e7ce8fa9fcca4e086ce1383e5427d`，沿用 [sources.json](../sources.json)；没有改变已有源身份。
- DexLab 引用固定为 `7e4a0774efb4f8b332e866e1495e032b4969c8aa`。报告存在斜面原生运行记录，但 coverage-v1 仍未资格；这是引用历史证据，不是本课程新增运行。

## 已完成检查

| 检查 | 结果与边界 |
| --- | --- |
| `python3 scripts/check_docs.py` | 相对 Markdown 目标存在；不代表远端链接或锚点全库验收 |
| `python3 -m compileall -q examples scripts` | Python 语法编译通过；没有 import 或执行例子 |
| 新导读 Python fenced block | 1 个片段经 `ast.parse` 通过；原生方法对照固定绑定 |
| 新导读 Drake 链接 | 10 次固定源码引用，逐项核对 pin、目标文件和行号范围；正文相关段落已阅读 |
| 下载源身份 | 下列 9 个文件按 Git blob 格式计算 SHA-1，与既有清单一致 |
| `git diff --check` | 通过 |

核对文件为 `multibody/plant/{multibody_plant.h,multibody_plant.cc,contact_model_doxygen.h,sap_driver.cc,contact_properties.cc}`、`multibody/contact_solvers/sap/{sap_solver.h,sap_solver.cc,sap_contact_problem.h}`、`bindings/pydrake/multibody/plant_py.cc`。下载和检查缓存未纳入交付。

## 迁移例外

`docs/evidence/e1-validation.md` 至 `e6-validation.md` 保留当时记录的 `drake-atlas` Issue URL、基线 SHA 和分支名称，文件字节未修改。这六份历史验收不是现行导航；现行入口均采用新仓库路径。本记录保留旧名称仅用于说明迁移例外。

原生示例、API、模型标识和源清单均未因品牌迁移重命名。当前跟踪文件中没有其他需要保留的 Atlas 原生标识例外。

## 未开展

未安装或导入 pydrake，未构建原生模型、求解接触、调用 Simulator、渲染、训练或跑性能基准。没有重跑或改变 DexLab 数据，也没有通过本次静态检查补发物理资格。E7 完整安装、运行和全路线审校继续暂停。
