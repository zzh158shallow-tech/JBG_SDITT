# Python Code

Place all Python code for this project in this directory.

This includes new Python scripts, modules, packages, notebooks converted to
Python files, and future generated Python code.

## Layout

- `sditt/config`: project path configuration.
- `sditt/io`: MATLAB `.mat` and numeric text readers.
- `sditt/vehicle`: MATLAB vehicle-parameter reader and CRH380A vehicle matrix
  reproduction.
- `sditt/track`: raw turnout/track data readers.
- `sditt/profiles`: wheel/rail profile discovery and loading.
- `sditt/contact`, `sditt/integrators`, `sditt/simulation`: reserved package
  namespaces for later physical-model migration.
- `tests`: reader-level tests.

## Scope

The current code reads source data and reproduces the linear vehicle dynamics
and flexible turnout modal matrix skeletons from:

- `Par_Vehicle_CRH380A_v6.m`
- `Matrix_Vehicle_RW_230409.m`
- `Matrix_Modal_FT_230313.m`

It now includes the first linear time-integration loop without wheel-rail
contact. It does not yet perform wheel-rail contact or full coupled
vehicle-track simulation.

The migration handoff plan for the full default coupled model is in
[`docs/full_case_migration_plan.md`](docs/full_case_migration_plan.md).

Useful smoke check for the raw MATLAB/text/profile data layer:

```bash
python -m sditt.io.inspect_data
```

Programmatic entry point:

```python
from sditt.io import inspect_raw_inputs, load_sditt_raw_inputs

manifest = inspect_raw_inputs()
raw = load_sditt_raw_inputs(max_profiles_per_kind=2)
```

This reads/summarizes `Mat_FT_S8b.mat`, loads `ModeFreq.FT_All`, parses
`Par_Vehicle_CRH380A_v6.m`, discovers wheel/rail profile files, and loads
numeric text tables such as damping-ratio and mileage files. It is intentionally
only a data layer.

Operating-case layouts:

```python
from sditt.config import DEFAULT_OPERATING_CASE, MATLAB_FULL_DEFAULT_CASE

interval_inp_par = DEFAULT_OPERATING_CASE.to_inp_par()
turnout_inp_par = MATLAB_FULL_DEFAULT_CASE.to_inp_par()
```

`DEFAULT_OPERATING_CASE` uses two constant basic-rail contact slots, `L1/R1`.
`MATLAB_FULL_DEFAULT_CASE` preserves the original turnout `L1/R1/R2/R3`
contact layout aligned with `SDITT_CR400_NoStrTIrr_250728_Face.m`.

Build and export the CRH380A vehicle matrices:

```bash
python -m sditt.vehicle.export_matrices vehicle_matrices_rw_230409.npz
```

The archive stores both Python-facing names (`M_vehicle`, `K_vehicle`,
`C_vehicle`) and MATLAB-compatible names (`Mlc`, `Klc`, `Clc`, `Clc_0`).

Build and export the flexible turnout modal matrices:

```bash
python -m sditt.track.export_matrices modal_track_matrices_ft_230313.npz
```

The archive stores `M_track`, `K_track`, `C_track`, `DR`, `ModeFreq_FT`, and
`omega`. With the default `--cut-freq 2000`, the dense modal matrices are
`6197 x 6197`.

Build and export the uncoupled full system block matrices:

```bash
python -m sditt.simulation.export_system_matrices system_matrices_modal_rw.npz
```

This follows the main-script ordering
`[track DOFs, flexible wheel DOFs, rigid vehicle DOFs]`. For the current RW
vehicle route, `nm_fw = 0`, so `Mxt/Kxt/Cxt` are the track block followed
directly by the 51-DOF vehicle block.

For large FT-Modal runs, prefer the sparse builder when a downstream solver can
consume SciPy sparse matrices:

```python
from sditt.simulation import build_default_sparse_modal_rw_system_matrices

system, track, vehicle = build_default_sparse_modal_rw_system_matrices()
Mxt = system.Mxt
Kxt = system.Kxt
Cxt = system.Cxt
```

The sparse route keeps modal track `M/K/C` as diagonal sparse matrices and
assembles the full system with sparse block diagonals instead of dense zero
blocks.

Run the first no-contact forced-response loop:

```python
from sditt.simulation import run_default_no_contact_smoke

result = run_default_no_contact_smoke(cut_freq=50.0, dt=1e-4, n_steps=100)
time = result.history.time
displacement = result.history.displacement
velocity = result.history.velocity
acceleration = result.history.acceleration
force = result.force
```

This uses a simple prescribed harmonic force and advances the uncoupled
vehicle-track system with the Park/Newmark integrator. It does not compute
wheel-rail contact forces.

Run the current full-default driver:

```python
from sditt.simulation import FullDefaultCaseSettings, run_default_full_case_driver

result = run_default_full_case_driver(
    settings=FullDefaultCaseSettings(
        cut_freq=50.0,
        dt=1e-4,
        n_steps_per_stage=2,
    )
)
stages = result.stages
missing = result.preparation.missing_stages
```

This builds the default `07(009)` / `Face` / `350 km/h` /
`FT-Modal` / `CRH380A_v6` system and executes the two-stage
`Preload -> Cal` coupled loop with the modal FT gravity preload
applied as the baseline external force. The default contact selector uses the
same constant measured basic-rail section on both sides. The matrices, gravity
preload, and structural dynamics still come from the flexible-turnout model,
so this is an interval contact-geometry surrogate rather than a complete
interval track dynamics model.

The original turnout contact route remains available from the command line:

```bash
python -m sditt.validation.full_case_short_run --rail-layout turnout --steps 2 --cut-freq 50
```

Track irregularity is off by default so existing baselines remain unchanged.
Enable the Chinese high-speed ballastless-track spectrum with a reproducible
random-phase trigonometric-series reconstruction as follows:

```bash
python -m sditt.validation.full_case_short_run \
  --rail-layout interval \
  --track-irregularity china-ballastless \
  --irregularity-seed 20260716 \
  --full-size \
  --matlab-mileage-endpoints \
  --save-progress
```

The model synthesizes vertical profile (高低), alignment (轨向), cross level
(水平), and gauge (轨距) over wavelengths 2–200 m. It applies both displacement
and the corresponding velocity to wheel–rail contact. With `--save-progress`,
the `progress/` directory also contains `track_irregularity.csv`,
`track_irregularity_spectrum.csv`, and `track_irregularity.svg`.

For a locked production run using the traditional contact model, full modal
system, full MATLAB mileage endpoints, track irregularity, checkpoints, and the
realtime window, run this command from the repository root:

```bash
python python/scripts/run_traditional_fullcase_irregularity.py
```

The window opens before computation; click `开始计算`. Use
`--resume-checkpoint` to continue from the latest compatible checkpoint. The
default output directory is
`python/outputs/traditional_fullcase_irregularity_seed20260716/`. Its
`progress/` directory contains:

- `wheel_rail_forces.csv`: one row per accepted integration step and physical
  wheel (`FF/FR/RF/RR` x `L/R`), including wheelset mileage, lateral force,
  vertical force, and the Y-Z resultant.
- `contact_patch_forces.csv`: one row per accepted integration step and
  configured rail contact slot (`L1/R1` for the interval case).
- `progress.csv` and `progress_final.svg`: run-level convergence/progress and
  contact-force overview.
- `track_irregularity.csv`, `track_irregularity_spectrum.csv`, and
  `track_irregularity.svg`: the generated irregularity realization and its
  spectrum.

All force values are in N. `lateral_force_y_N` and `vertical_force_z_N` retain
the model global Y/Z signs; `resultant_yz_force_N` is their non-negative vector
magnitude. Only integration steps accepted by the nonlinear coupled solver are
written, so rejected retry candidates are not mixed into the physical output.

## Network A training data

The interval-layout contact-geometry surrogate dataset uses one accepted
time-step / wheelset / wheel-side environment per sample. Generate the
2,000-sample pilot first, then extend the same output to the 20,000-sample
production dataset:

```bash
python -m sditt.training_data.network_a generate --mode pilot
python -m sditt.training_data.network_a generate --mode production
```

Outputs are written under `outputs/network_a_dataset_v1/` and are ignored by
Git. The production dataset contains 6,000 accepted coupled samples and 14,000
parameter samples: 7,600 ordinary contact, 800 targeted flange-related
multi-contact, 3,500 contact/separation boundary, and 2,100 no-contact samples.
The targeted samples use Sobol perturbations around profile-specific discovery
anchors and are accepted only after the traditional geometry teacher confirms
two separated contact patches with at least one flange-angle patch. All samples
are grouped into `14,000 / 3,000 / 3,000` train/validation/test splits. Inspect
the manifest and quality results with:

```bash
python -m sditt.training_data.network_a inspect outputs/network_a_dataset_v1
```

Programmatic loading converts floating arrays to `float32` by default while
preserving masks and metadata:

```python
from sditt.training_data import load_network_a_dataset

train = load_network_a_dataset("outputs/network_a_dataset_v1", split="train")
```

## WRCP-Net A1 training

`WRCP-Net A1` means Wheel–Rail Contact Point Network A1 (轮轨接触点网络A1).
It is the first network-A baseline: a dependency-free NumPy multi-task MLP
that predicts the contact-patch count and two masked patch-label slots. Train
and inspect it from the `python/` directory:

```bash
python -m sditt.models.wrcp_net_a1 train
python -m sditt.models.wrcp_net_a1 inspect outputs/wrcp_net_a1
```

The ignored `outputs/wrcp_net_a1/` directory contains the compressed model,
training history, validation/test metrics, test predictions, and provenance
manifest. The first version uses class-weighted cross entropy for contact state
and masked Huber loss (胡贝尔损失，兼顾平方误差与异常值鲁棒性) for continuous
patch labels, with additional weight on the second contact slot.

## WRCP-Net A2G geometry-aware training

`WRCP-Net A2G` is the geometry-aware successor. It predicts a 257-point
canonical gap/penetration field and the contact topology, then reconstructs
patch boundaries, wheel/rail points, penetration and contact angle from the
fixed profiles. When the predicted topology agrees with the native profile
topology, a deterministic profile-consistency refinement removes compact-grid
quantisation from the final coordinates without overriding the predicted
contact class. Generate the field cache and train in one command:

```bash
python -m sditt.models.wrcp_net_a2g train
python -m sditt.models.wrcp_net_a2g inspect outputs/wrcp_net_a2g
```

The field cache is written to `outputs/network_a_gap_field_v1/`; the trained
model and comparison metrics are written to `outputs/wrcp_net_a2g/`. Both are
ignored by Git.

Use A2G as a strict replacement for the traditional multi-point geometry
search in the interval full case with:

```bash
python -m sditt.validation.full_case_short_run \
  --rail-layout interval \
  --contact-geometry-mode network-a \
  --network-a-model outputs/wrcp_net_a2g/model.npz \
  --steps 2 \
  --cut-freq 50
```

This mode has no traditional-geometry fallback. A predicted/reconstructed
patch-count mismatch, non-finite geometry, a missing model, or a non-interval
layout raises an error. STRIPES/Hertz/Kalker force calculation and the coupled
dynamics remain unchanged after A2G supplies the contact geometry.

To establish the preload equilibrium with traditional geometry and switch to
strict A2G geometry only for the subsequent `Cal` stage, use:

```bash
python -m sditt.validation.full_case_short_run \
  --rail-layout interval \
  --contact-geometry-mode network-a-after-preload \
  --network-a-model outputs/wrcp_net_a2g/model.npz \
  --network-a-trace-dir outputs/network_a_runtime_trace \
  --track-irregularity china-ballastless \
  --irregularity-seed 20260716 \
  --steps 2 \
  --cut-freq 50
```

Production runs should omit `--network-a-trace-dir`. Runtime tracing is then
fully disabled, which avoids geometry-label conversion, NPZ compression, and
trace-disk I/O inside the contact-geometry timing path.

When a trace directory is supplied, the default `selective` mode always saves
the final nonlinear iteration accepted by the time integrator. It additionally
saves every iteration of a step when the step was retried, the maximum class
probability falls below `0.95`, more than one contact patch is predicted or
reconstructed, or a 1 m absolute-mileage sampling boundary is crossed. The
threshold and interval can be adjusted, and `0` disables mileage sampling:

```bash
python -m sditt.validation.full_case_short_run \
  --contact-geometry-mode network-a-after-preload \
  --network-a-model outputs/wrcp_net_a2r_continuity/model.npz \
  --network-a-trace-dir outputs/network_a_runtime_trace_selective \
  --network-a-trace-mode selective \
  --network-a-trace-low-confidence 0.95 \
  --network-a-trace-sample-interval-m 1.0 \
  --steps 2 \
  --cut-freq 50
```

Use `--network-a-trace-mode full` only for explicit nonlinear-iteration
debugging. Trace shards include a `selection_reason` field, while
`accepted_steps.jsonl` remains the authoritative accepted-step index. All
retained rows are diagnostics rather than teacher labels: rerun the traditional
geometry teacher before promoting any row into a training dataset.

## WRCP-Net A2R continuity refinement

`WRCP-Net A2R` adds independent left/right bounded penetration-residual heads,
accepted-step continuity state, contact-branch hysteresis, and candidate-region
fixed-profile interpolation to the frozen A2G gap-field model. Build its v2
teacher dataset and micron-scale perturbation pairs, then train with higher
weights for high-iteration, reduced-`dt`, left-side, and accepted states:

```bash
python -m sditt.models.wrcp_net_a2r build-dataset \
  --trace outputs/network_a_runtime_trace_irregularity_full_seed20260716 \
  --output outputs/network_a_runtime_teacher_v2_continuity \
  --repo-root .. \
  --base-model outputs/wrcp_net_a2g/model.npz

python -m sditt.models.wrcp_net_a2r train \
  --dataset outputs/network_a_runtime_teacher_v2_continuity \
  --base-model outputs/wrcp_net_a2g/model.npz \
  --output outputs/wrcp_net_a2r_continuity \
  --left-residual-gain 1.0 \
  --right-residual-gain 1.0 \
  --smooth-weight 0.25 \
  --rate-reference-dataset outputs/network_a_dataset_v1/all_samples.npz
```

The deployed continuity envelope uses the 99.5th percentile of adjacent
traditional coupled steps. Ordinary changes pass through unchanged; only
out-of-envelope changes activate `alpha=0.3` under-relaxation and rate clipping.
Every nonlinear retry remains anchored to the preceding accepted step, and
history is committed only after the integrator accepts the step. The model is
still an A2G-compatible artifact and introduces no traditional fallback.

Run it after traditional preload with a new v2 trace directory:

```bash
python -m sditt.validation.full_case_short_run \
  --rail-layout interval \
  --contact-geometry-mode network-a-after-preload \
  --network-a-model outputs/wrcp_net_a2r_continuity/model.npz \
  --network-a-trace-dir outputs/network_a_runtime_trace_a2r_continuity \
  --track-irregularity china-ballastless \
  --irregularity-seed 20260716 \
  --steps 200 \
  --cut-freq 50 \
  --save-progress
```

The v2 runtime trace separately records base penetration, bounded residual,
slew-limited residual, network penetration, final used penetration, branch
hysteresis, and continuity-limit flags. Do not append v2 records to a v1 trace
directory.

## WRCP-Net A1 Direct Set (experimental)

`WRCP-Net A1 Direct Set`（轮轨接触斑直接集合网络）直接输出最多两个最终
接触斑。它使用 Deep Sets（深度集合）编码上一接受步的无序接触斑历史，
不生成 257/501/1001 点间隙场，不执行候选区细化、边界搜索或准弹性修正。
新接口使用 `DirectContactGeometry`，因此不会用空廓形数组或虚构索引伪装
传统 `MultiPointContactGeometry`。

从现有传统教师数据生成 2,000 样本 pilot（小规模试验），训练并执行静态
验收：

```bash
python -m sditt.training_data.network_a_direct \
  --source-dir outputs/network_a_dataset_v1 \
  --output-dir outputs/network_a_direct_set_v1 \
  --repo-root .. \
  --mode pilot

python scripts/train_network_a1_direct.py \
  --dataset-dir outputs/network_a_direct_set_v1 \
  --output-dir outputs/wrcp_net_a1_direct

python scripts/validate_network_a1_direct.py \
  --dataset outputs/network_a_direct_set_v1/test.npz \
  --model outputs/wrcp_net_a1_direct/model.npz \
  --output outputs/wrcp_net_a1_direct/static_validation.json
```

完整 20,000 样本训练必须使用阶段感知历史：同一轨迹、轮对和侧别先排列
全部 Preload（预加载）接受步，再排列 Cal（计算）接受步，因此 Cal-1 的
历史严格来自最后一个 Preload 接受步。训练后 40% 使用 pushforward
curriculum（前推式滚动课程）：滚动长度由 1 增至 8，自身预测历史的概率
由 0 增至 50%，并向教师历史注入按闭环残差标定的扰动。早停只能在进入
自身历史训练后生效。Preload 行始终保留传统教师历史，Cal-1 强制以最后
一个传统 Preload 接受斑为锚点，仅从 Cal-2 开始采用 A-1 自身历史。
运行时的 OOD（分布外）阈值保持为 4，只用训练集逐特征 99.95% 经验包络
校准尺度，不能通过放宽门控掩盖历史分布缺口。

```bash
python -m sditt.training_data.network_a_direct \
  --source-dir outputs/network_a_dataset_v1 \
  --output-dir outputs/network_a_direct_set_v2_history_fixed \
  --repo-root .. \
  --mode production

python scripts/train_network_a1_direct.py \
  --dataset-dir outputs/network_a_direct_set_v2_history_fixed \
  --output-dir outputs/wrcp_net_a1_direct_stageaware \
  --epochs 120 \
  --batch-size 256
```

静态报告中的 `history_continuity` 会单独给出 Preload→Cal 首过渡的严格
门控通过率，以及闭环深度 0、1–8、9 步以上的拓扑准确率，用于区分
“过渡分布缺口”与“自回归误差累积”。

当闭环失败来自 A-1 自身历史分布时，使用 DAgger（数据集聚合）增量集，
不得把验证或测试种子并入训练。采样器保持传统 Preload 和教师锚定的
Cal-1，随后闭环运行 A-1；它选择低置信度、拓扑误判以及中心、压入量、
角度和形状超限的“误差生产者”状态。严格门控失败后使用教师斑重置下一步
采样，历史距离超过上限的极端消费者状态只保留为诊断，不参与训练：

```bash
python -m sditt.training_data.network_a_direct_dagger \
  --dataset-dir outputs/network_a_direct_set_v2_history_fixed \
  --model outputs/wrcp_net_a1_direct_stageaware_v7/model.npz \
  --output-dir outputs/network_a_direct_set_v3_dagger_round1_geometry \
  --low-confidence 0.95 \
  --maximum-history-distance 8 \
  --round 1

python scripts/train_network_a1_direct.py \
  --dataset-dir outputs/network_a_direct_set_v3_dagger_round1_geometry \
  --output-dir outputs/wrcp_net_a1_direct_stageaware_v8_dagger \
  --epochs 120 \
  --batch-size 256
```

`manifest.json` 必须显示训练、验证、测试轨迹组零重叠；
`dagger_selection.jsonl` 保存每个增量样本的教师/预测拓扑、置信度、历史
距离、几何误差和选择原因。这个旧入口复制既有标签，并未在新状态上重新
调用传统教师，因此当前监督策略把其行标为审计数据，拓扑和几何训练权重
均为 0；正式局部几何增量应使用后文的 runtime DAgger 教师重标注入口。

若旧耦合轨迹的 Preload 长度与当前验证运行不同，先用当前传统代码重新
采集不同阶段长度的 Cal 接受步。下面的命令只使用训练种子，并保持
20260721/20260722 为验证/测试留出种子：

```bash
python -m sditt.training_data.network_a_direct_fresh \
  --base-dataset-dir outputs/network_a_direct_set_v2_history_fixed \
  --output-dir outputs/network_a_direct_set_v4_fresh_transitions \
  --repo-root .. \
  --seeds 20260716 20260717 20260718 20260719 20260720 20260723 \
  --stage-steps 1 2 5 20 200
```

长阶段的全部接受步会压倒原有短阶段分布。完成教师采集后，只保留首过渡、
早期连续步和少量长程锚点，再与原训练集组合：

```bash
python -m sditt.training_data.network_a_direct_rebalance \
  --base-dataset-dir outputs/network_a_direct_set_v6_runtime_dagger_multilength_fixed \
  --candidate-dataset-dir outputs/network_a_direct_set_v7_fresh_200step_transition \
  --output-dir outputs/network_a_direct_set_v8_balanced_preload200 \
  --group-substring stage-steps-200 \
  --retained-step-indexes 1 2 3 4 5 20 50 100 150 200
```

正式 Cal 会在非线性迭代候选状态上调用 A-1，而接受步数据不一定覆盖首次
迭代。可使用关闭 OOD/置信度门控但保留硬几何约束的审计制品生成 v1
运行 trace（轨迹记录），再离线调用当前传统几何教师标注。此类数据只能
标记为局部监督，`accepted_step_label` 必须为 `false`：

```bash
python -m sditt.training_data.network_a_direct_runtime_dagger \
  --base-dataset-dir outputs/network_a_direct_set_v4_fresh_transitions \
  --trace-dir outputs/network_a1_direct_audit_seed20260716 \
  --trace-dir outputs/network_a1_direct_audit_seed20260717 \
  --trace-dir outputs/network_a1_direct_audit_seed20260718 \
  --trace-dir outputs/network_a1_direct_audit_seed20260719 \
  --trace-dir outputs/network_a1_direct_audit_seed20260720 \
  --trace-dir outputs/network_a1_direct_audit_seed20260723 \
  --output-dir outputs/network_a_direct_set_v5_runtime_dagger \
  --repo-root ..
```

fail-fast（快速失败）的 trace 本来就不会标记为完整。只有在明确用于局部
教师重标注时，才能增加 `--allow-incomplete`；manifest 会逐条记录 trace
是否完整，失败后的状态不会被虚构：

```bash
python -m sditt.training_data.network_a_direct_runtime_dagger \
  --base-dataset-dir outputs/network_a_direct_set_v8_balanced_preload200 \
  --trace-dir outputs/network_a1_direct_failed_trace \
  --output-dir outputs/network_a_direct_set_v9_balanced_runtime_dagger \
  --repo-root .. \
  --maximum-step-index 3 \
  --allow-incomplete
```

运行时迭代样本不进入正式拓扑损失。只有传统教师已重新标注、且接触斑数
与同 seed 的 full-Cal accepted-step 参考一致时，它才以不高于 1.0 的权重
进入局部几何回归；其余候选只作审计，也不能用于校准验证/测试阈值。

为避免长阶段样本造成 catastrophic forgetting（灾难性遗忘，即新数据破坏
旧工况能力），增量轮次可从已验证制品 warm start（热启动）并保留原归一化：

```bash
python scripts/train_network_a1_direct.py \
  --dataset-dir outputs/network_a_direct_set_v9_balanced_runtime_dagger \
  --output-dir outputs/wrcp_net_a1_direct_warm_balanced \
  --initial-model outputs/wrcp_net_a1_direct_stageaware/model.npz \
  --learning-rate 5e-5 \
  --epochs 40 \
  --batch-size 256 \
  --repo-root ..
```

新制品会嵌入固定 LMA 轮廓和 L1/R1 钢轨表。网络只预测最终横向中心、
区间和峰值位置；车轮纵向坐标、钢轨高度、轮廓局部坐标和接触角通过两个
最终横向位置的标量反求生成。每个点使用 30 次二分，分辨率低于 0.2 nm；
不会生成 257/501/1001 点间隙场，不执行候选搜索或准弹性修正。

`train` 可选依赖包含 PyTorch（张量训练框架），安装命令为
`python -m pip install -e ".[train]"`；导出的正式推理制品仍是只依赖
NumPy/SciPy 的 `.npz`。数据集和模型 schema（格式版本）分别是
`network-a-direct-set-dataset-v1` 与 `wrcp-net-a1-direct-set-v1`。

在已有直接 A1 制品、但尚无 Network B Direct 制品时，可用 Hertz（赫兹）
力维持直接几何闭环，同时在相同耦合状态下并行运行传统
STRIPES + Kalker shadow teacher（影子教师）。短程 pilot 也会先完成完整
传统 Preload，再采指定数量的 Cal 接受步：

```bash
python scripts/collect_network_b_dataset.py \
  --seeds 20260716 \
  --steps 2 \
  --cut-freq 50 \
  --output-dir outputs/network_b_direct_dataset_v3_pilot
```

正式 Round 0 使用冻结的 V22 A-1 模型，按完整 seed 生成全模态、全里程
数据；训练、验证、测试 seed 必须保持完整隔离：

```bash
python scripts/collect_network_b_dataset.py \
  --seeds \
    20260716 20260717 20260718 20260719 20260720 \
    20260723 20260724 20260725 \
  --full-size \
  --matlab-mileage-endpoints \
  --output-dir outputs/network_b_direct_dataset_v3_round0

python scripts/collect_network_b_dataset.py \
  --seeds 20260721 \
  --full-size \
  --matlab-mileage-endpoints \
  --output-dir outputs/network_b_direct_dataset_v3_validation
```

每个 seed 按 10 m 写 NPZ 分片和求解器 checkpoint（检查点）；中断后向同一
命令增加 `--resume`。manifest 记录真实 Cal accepted-step 数、终点、A-1/
型面/代码哈希、教师匹配和质量检查，不再从受
`history_retention_steps` 限制的内存历史推断步数。拓扑不一致的影子教师
样本只计入诊断，不会退化成 Hertz 伪标签。

该数据集使用 `network-b-direct-dataset-v3`，包含 53 维 Direct 特征、
7 维教师目标，以及 step/time/dt、轮/侧、patch key、retry、教师匹配距离
和 `accepted_step_label`。它删除离散网格采样点数特征，并从 A1 的连续
形状矩计算面积、1.5 阶面积、均值和标准差。直接联合模式接受绝对力模型
`wrcp-net-b-direct-force-v2`、赫兹残差模型
`wrcp-net-b-direct-force-residual-v3`，以及固定赫兹法向力模型
`wrcp-net-b-direct-force-hertz-fixed-v3`。Round 0 优先使用残差模型：

```bash
python -m sditt.validation.full_case_short_run \
  --rail-layout interval \
  --contact-geometry-mode network-a1-direct-after-preload \
  --network-a-model outputs/wrcp_net_a1_direct/model.npz \
  --network-a-force-mode network-b \
  --network-b-model outputs/wrcp_net_b_direct_round0_residual_128x128_weight010/model.npz \
  --network-b-ood-fallback hertz \
  --track-irregularity china-ballastless \
  --irregularity-seed 20260716 \
  --steps 200 \
  --cut-freq 50
```

A1 的低拓扑置信度、分布外输入或硬约束违规会立即报错并写诊断，不回退
传统几何；B 的分布外输入只允许回退 Hertz + Kalker。直接模式在所有静态、
闭环、全里程和性能门槛通过前始终标记为实验模式，不得视为生产可用。

使用当前 V22 DAgger 严格制品和 Round 0 Network B 运行“全模态尺寸 +
完整 MATLAB 里程 + 实时窗口”的 full-case 验证：

```bash
cd python
./.venv/bin/python scripts/run_network_a1_direct_fullsize_live.py \
  --network-b-model outputs/wrcp_net_b_direct_round0_residual_128x128_weight010/model.npz \
  --network-b-ood-threshold 4 \
  --network-b-ood-fallback hertz \
  --irregularity-seed 20260721 \
  --output-dir outputs/network_ab_v22_b0_fullsize_validation_seed20260721 \
  --no-resume-checkpoint
```

该入口默认加载
`outputs/wrcp_net_a1_direct_multiseed_topology_v22_dagger_strict_stabilized/model.npz`，
启用 `--full-size --matlab-mileage-endpoints --live-window`。窗口打开后点击
“开始计算”。Preload 全程使用传统接触模型；只有进入 Cal 后才由 Network A1
Direct 替换接触几何，并由 Network B 预测接触力；B 的 OOD 输入回退到
Hertz + Kalker。入口保留 256 步内存诊断，但最终 JSON 每阶段只序列化最后
1 步，完整接受步仍写入进度和接触关键数据 CSV，避免全里程快照膨胀。
省略 `--network-b-model` 可恢复为纯 Hertz + Kalker 力。需要诊断时显式增加
`--enable-trace`；默认完全关闭 Network A trace。首次运行使用
`--no-resume-checkpoint`，后续续算改用 `--resume-checkpoint`。测试 seed
`20260722` 只应在模型结构、OOD 阈值和回退策略锁定后执行一次盲测。

使用当前 V10 固定型面投影制品运行“全模态尺寸 + 完整 MATLAB 里程 +
实时窗口”的观察入口：

```bash
cd python
./.venv/bin/python scripts/run_network_a1_v10_fullsize_live.py
```

该入口默认启用 `--full-size --matlab-mileage-endpoints --live-window`，并保存
进度、逐轮接触关键数据、版本 4 checkpoint（检查点）和选择性 A1 trace。
窗口打开后点击“开始计算”。断点续算使用：

```bash
./.venv/bin/python scripts/run_network_a1_v10_fullsize_live.py \
  --resume-checkpoint
```

只有显式增加 `--preview-steps N` 才会缩短里程。V10 仍是实验制品；全尺寸
全里程遇到置信度、OOD 或硬约束门控时会真实停止并保存诊断，不会回退传统
几何。

V11 的 full-Cal（完整 Cal 里程）训练数据使用传统接触链的 accepted-step
（积分器接受步）作为正式标签。V11 失败迭代只作为审计证据，不混入正式
轨迹标签。单 seed 审计入口如下：

```bash
cd python
./.venv/bin/python scripts/prepare_network_a1_v11_full_cal_data.py
```

单 seed 默认产物位于
`outputs/network_a_direct_set_v11_full_cal_seed20260716/`。其中
`traditional_accepted_steps.npz` 是全量传统候选，`full_cal_selected.npz` 是
经过稀疏连续窗口选择的增量。它不再作为正式训练集，因为单条轨迹不足以
覆盖闭环分布。

正式拓扑数据使用 8 个训练 seed（随机种子）的 full-Cal accepted step；验证
只使用 `20260721`，测试只使用 `20260722`，三个集合按完整 seed 隔离：

```bash
cd python
./.venv/bin/python scripts/prepare_network_a1_multiseed_full_cal_data.py
```

默认训练 seed 为 `20260716`–`20260720`、`20260723`–`20260725`。入口会复用
已经存在的传统存档，其余 seed 重新运行传统 full-case（全尺寸算例）；最终
生成严格隔离的 `train.npz`、`validation.npz`、`test.npz` 和带哈希的
`manifest.json`。若非线性 retry 曾缩小时间步，求解器会增加 accepted step
补足损失的里程；Preload 和 Cal 都必须在 `1e-9 m` 容差内到达 MATLAB 阶段
终点，才能保存为正式 full-Cal 存档。
显式传入 `--source-archive SEED=PATH` 时也会执行同一终点检查，因此旧版
固定 accepted-step 数量、遇 retry 后提前停止的存档会被拒绝。

训练器从此使用两套独立权重：正式拓扑损失以多 seed full-Cal accepted step
为主，`accepted_step_label=false` 的 DAgger（数据集聚合）迭代样本拓扑权重
恒为 0；DAgger 仅在传统教师重新标注、找到同 seed/轮对/侧/里程的正式轨迹
参考且接触斑数一致时参与局部几何回归，权重为 1.0。训练前会硬性检查训练
seed 数量以及验证/测试 seed，旧的混合 Sobol 验证集不会被误当作正式评估。
全量 accepted-step 拓扑索引保存在 `accepted_topology_reference.npz`，DAgger
核对的默认同轨迹里程容差为 0.02 m；不满足条件的候选只留在审计 JSONL 中，
不会进入训练 NPZ。

## WRCP-Net B force surrogate (experimental)

Network B predicts patch-level normal force plus three creep-force and three
creep-moment components. Its teacher is the existing Python
`STRIPES + Kalker` chain. Formal datasets contain only final `Cal` steps
accepted by the coupled integrator, and train/validation/test splits use whole,
disjoint irregularity seeds.

Round 0 keeps training and validation in separate, auditable directories.
The training command cannot read a final test split:

```bash
python scripts/train_network_b.py \
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

`hertz_residual`（赫兹残差）不让网络从零学习法向力，而是学习传统教师与
解析赫兹力之差；`--loss-weights` 将法向力损失权重设为 0.10，使有限模型
容量更多用于蠕滑力和力矩。OOD（out-of-distribution，分布外）尺度由训练
集绝对标准化距离的 `0.9999` 分位数独立校准，不再直接复用特征标准差。

如需检查教师匹配距离尾部的敏感性，可增加
`--max-train-teacher-match-um 50`。该筛选只作用于训练行，验证分布保持
完整。模型结构、训练设置、OOD 阈值和回退策略锁定后，才单独采集测试
seed `20260722` 并执行一次盲测：

```bash
python scripts/collect_network_b_dataset.py \
  --seeds 20260722 \
  --full-size \
  --matlab-mileage-endpoints \
  --output-dir outputs/network_b_direct_dataset_v3_test

python scripts/evaluate_network_b.py \
  --model outputs/wrcp_net_b_direct_round0_residual_128x128_weight010/model.npz \
  --training-metrics outputs/wrcp_net_b_direct_round0_residual_128x128_weight010/metrics.json \
  --dataset-dir outputs/network_b_direct_dataset_v3_test \
  --test-seeds 20260722 \
  --ood-threshold 4
```

`evaluate_network_b.py` verifies the model hash and seed isolation before
loading the test data. It refuses to overwrite an existing final-test report
unless `--overwrite` is explicit.

在正式盲测前，可用冻结的 V22 A-1 和验证 seed 做有界闭环短跑。该入口先
完成全传统 Preload，再运行指定数量的 Cal 接受步，并在关键接触数据 CSV
中记录每个接触斑是否触发 Network B 回退：

```bash
python scripts/run_network_a1_direct_strict_rollout_validation.py \
  --model outputs/wrcp_net_a1_direct_multiseed_topology_v22_dagger_strict_stabilized/model.npz \
  --network-b-model outputs/wrcp_net_b_direct_round0_residual_128x128_weight010/model.npz \
  --network-b-ood-threshold 4 \
  --irregularity-seed 20260721 \
  --nominal-cal-steps 600 \
  --output-dir outputs/network_ab_round0_validation_seed20260721_rollout600
```

The out-of-distribution guard falls back only when a feature exceeds the
configured standardized training envelope. For Direct A1, the formal labels
come from the same-state STRIPES + Kalker shadow teacher; topology-mismatched
patches are retained only as diagnostics. The guarded Direct route uses Hertz
fallback. Full-stage speed and pointwise closed-loop force agreement remain
seed-dependent. Do not declare the surrogate a production replacement until a
full-mileage, held-out-seed validation passes.

### Full run with Network B disabled and key contact data only

Use the following production-oriented route when Network A should provide the
contact geometry but Network B must remain completely unloaded. `traditional`
is the explicitly selected primary `STRIPES + Kalker` force law in this route;
it is not an out-of-distribution fallback. Omitting `--network-a-trace-dir`
also keeps Network A iteration tracing disabled.

```bash
cd python
./.venv/bin/python scripts/run_full_model_without_network_b.py
```

The equivalent complete command is:

```bash
./.venv/bin/python -m sditt.validation.full_case_short_run \
  --rail-layout interval \
  --contact-geometry-mode network-a-after-preload \
  --network-a-model outputs/wrcp_net_a2r_continuity/model.npz \
  --disable-network-b \
  --contact-key-data-only \
  --live-window \
  --track-irregularity china-ballastless \
  --irregularity-seed 20260716 \
  --full-size \
  --matlab-mileage-endpoints \
  --history-retention-steps 1 \
  --contact-force-tolerance 0.0025 \
  --output-dir outputs/full_model_network_a_without_network_b
```

This mode writes only
`outputs/full_model_network_a_without_network_b/wheel_rail_contact_key_data.csv`.
Rows correspond only to time steps accepted by the coupled integrator. Each
contact-patch row contains the wheelset and mileage, force acting on the wheel
in track coordinates (`Fx/Fy/Fz`), resultant and normal force, wheel and rail
contact-point coordinates, contact angle, and vertical/normal penetration.
Candidate iterations, profile arrays, Network B features, progress plots,
validation snapshots, timing reports, and Network A traces are not written.
The realtime window still displays progress, wheel/rail force histories, and
the current wheel/rail profile contact; its display payload remains in memory
and is discarded when the window closes.

For the straight-layout route, the current driver evaluates the
CRH380A_v6 nonlinear vehicle damper stage, the default rigid-wheel contact
route, and the MATLAB-style iteration/output storage path. For this straight
FT-Modal route, `missing_stages` is expected to be empty; the remaining
guarded branch is the non-default curve/layout external-force stage, which the
main MATLAB default case does not enter because `Type_Layout = Straight`.

Install the package in editable mode from this directory when using a fresh
Python environment:

```bash
python -m pip install -e ".[test]"
```
