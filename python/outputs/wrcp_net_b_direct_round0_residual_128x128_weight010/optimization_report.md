# Network B Round 0 训练与优化报告

## 数据边界

- 训练：`network_b_direct_dataset_v3_round0`，8 个完整 seed，共 542,464 行。
- 验证：`network_b_direct_dataset_v3_validation`，seed `20260721`，67,808 行。
- 测试：seed `20260722` 保持封存，本轮未采集、未读取、未用于调参。
- 正式标签均为 Cal 阶段积分器 accepted step（接受步）的最终
  `STRIPES + Kalker` 教师结果。
- 当前数据只覆盖单接触斑，不代表多接触斑能力。

## 优化过程

| 候选 | 隐层 | 法向力建模 | 验证合力 NRMSE |
| --- | --- | --- | ---: |
| 绝对力基线 | 64 × 64 | 直接预测 | 0.207220% |
| 赫兹残差 | 64 × 64 | 预测教师相对赫兹残差 | 0.153966% |
| 固定赫兹 | 128 × 128 | 法向力使用解析赫兹值 | 0.152857% |
| 赫兹残差 | 128 × 128 | 等权损失 | 0.151474% |
| 赫兹残差（选定） | 128 × 128 | 法向力损失权重 0.10 | 0.148910% |

选择依据：赫兹残差将已知接触物理作为基线，网络只学习传统教师与解析
赫兹力的修正；降低法向力损失权重后，蠕滑力与力矩精度改善，同时法向力
精度没有显著退化。

## 选定模型

- 模型：`model.npz`
- schema：`wrcp-net-b-direct-force-residual-v3`
- SHA256：`b0774086e7f65c3e8d2390ce689aae9ddd098e8f018ebde73e2766db6ad563c6`
- 最佳 epoch：121
- 隐层：128 × 128
- 验证合力向量 RMSE：105.343932 N
- 验证合力向量 NRMSE：0.148910%
- 验证法向力 MAE / RMSE / NRMSE：10.693053 N / 103.377905 N / 0.146088%
- 验证 OOD 比例（阈值 4）：0.4631%
- 训练 OOD 比例（阈值 4）：0.0929%

OOD（out-of-distribution，分布外）尺度由训练集绝对标准化距离的
0.9999 分位数单独校准。闭环运行中，超出阈值的输入只回退到
`Hertz + Kalker`，不会回退或改写 Network A-1 的接触几何。

## 验证 seed 闭环短跑

使用冻结的 V22 Network A-1、验证 seed `20260721` 和本模型完成全传统
Preload 后的 600 个 Cal 接受步：

- Cal 前端里程：`47.609722 m → 53.433333 m`。
- 600 步均覆盖 `FF/FR/RF/RR × L/R`，共 4,800 个接触斑行。
- 时间和前端里程严格递增，所有数值有限。
- 非线性迭代：362 步为 2 次，237 步为 3 次，1 步为 4 次。
- Network B OOD 回退：0 行、0 步。
- 最大接触斑序号：1，仍只证明单接触斑。

闭环证据位于
`outputs/network_ab_round0_validation_seed20260721_rollout600/closed_loop_summary.json`
和同目录的 `closed_loop_report.md`。该短跑不替代完整里程验证或测试
seed `20260722` 的一次性盲测。

## 复现训练

```bash
cd python
./.venv/bin/python scripts/train_network_b.py \
  --train-dataset-dir outputs/network_b_direct_dataset_v3_round0 \
  --validation-dataset-dir outputs/network_b_direct_dataset_v3_validation \
  --train-seeds \
    20260716 20260717 20260718 20260719 \
    20260720 20260723 20260724 20260725 \
  --validation-seeds 20260721 \
  --hidden-sizes 128 128 \
  --normal-force-mode hertz_residual \
  --loss-weights 0.10 1 1 1 1 1 1 \
  --epochs 150 \
  --patience 30 \
  --batch-size 256 \
  --ood-training-quantile 0.9999 \
  --ood-calibration-threshold 4 \
  --friction-limit 0.40 \
  --output-dir outputs/wrcp_net_b_direct_round0_residual_128x128_weight010
```

## 尚未越过的门槛

- 在锁定模型、OOD 阈值和回退策略前，不读取测试 seed `20260722`。
- 一次盲测通过后，仍需做 held-out seed（留出随机种子）的完整里程闭环验证。
- 完整多接触斑教师数据出现前，不声明多接触斑覆盖或生产可用。
