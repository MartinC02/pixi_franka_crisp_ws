

  

Local pipeline stays where it is. Only the **converted LeRobot dataset** travels to

the cluster (~19 MB of encoded video per episode, so ~3 GB for 150 episodes; the

9.9 GB of mcaps stay here), and only the **policy weights** travel back (~8.8 GB,

or 62 MB for a LoRA adapter).

The mcaps, ROS and the conversion toolchain never need to exist on the cluster.

  

lab PC: record -> mcap -> convert (needs ROS + rosbags)

cluster: lerobot-train on H200 (needs lerobot + HF cache)

lab PC: policy_server + RobiClient on the robot (needs the checkpoint)

  

---

  

## 0. Convert locally (before anything else)

  

If you are converting with the **subtask** config, label first — the converter

drops any bag with no row in the label CSV, so a batch recorded after the last

labelling run would vanish silently:

  

cd ~/projects/data_postprocessing

micromamba run -n lerobot python labels/label_subtasks.py labels/raw_labels.json

  

Then convert. Use `convert_batched.py`, never `convert_rosbag_to_lerobot.py`

(that one is the old single-process converter, cannot resume, and has twice

died deep into a run). Both the raw dir and the config are arguments, because

neither is fixed any more — recordings up to 2026-08 sit in `data/BMW Task`,

the 2026-08-05-and-later campaign in `data/v1`:

  

micromamba run -n lerobot python scripts/convert_batched.py \

--raw-dir "$HOME/projects/data/v1" \

--config configs/lerobot_bimanual_absolute.yaml

  

Pick the config deliberately, they produce different datasets:

  

- `lerobot_bimanual_absolute.yaml` → `repo_id: bmw_ecu_assembly`, ONE

full-assembly prompt for every episode, no idle trimming.

- `lerobot_ecu_subtasks.yaml` → `repo_id: bmw_ecu_subtasks`, per-subtask

prompts from `labels/subtask_labels.csv`, and it carries a `drop_idle`

block. Keep that OFF — see the V4 note in `train_pi05_v4.sbatch`: the

position-only idle test deleted the settle-then-close grasps and cost the

policy 10-20 pick attempts where V2 needed 1-2.

  

Output lands at `<raw-dir>/<repo_id>_<ts>/`, nested inside the scanned dir —

do not delete it with the raw episodes. Check before shipping:

  

D="$HOME/projects/data/v1/bmw_ecu_assembly_<ts>"

micromamba run -n lerobot python - <<PY

import json, pyarrow.parquet as pq

i = json.load(open("$D/meta/info.json"))

print(i["total_episodes"], "eps", i["total_frames"], "frames", i["fps"], "fps")

print({k: v.get("shape") for k, v in i["features"].items()})

print(pq.read_table("$D/meta/tasks.parquet").to_pylist())

PY

  

`meta/info.json` must read `action [20]`, `observation.state [30]` and the

three `observation.images.zed_*` features, and `meta/tasks.parquet` must hold

exactly the prompts you expect — one row for the absolute config, one per

subtask for the subtask config. A subtask conversion that shows a single

full-assembly row means the relabelling never ran

(`scripts/conversion/relabel_subtask_prompts.py`), which you would otherwise

only notice as odd rollout behaviour.

  

**Then fix the normalisation stats — always, and only once everything is in:**

  

micromamba run -n lerobot python scripts/conversion/fix_global_quantile_stats.py "$D" --dry-run

micromamba run -n lerobot python scripts/conversion/fix_global_quantile_stats.py "$D"

  

LeRobot aggregates per-episode quantiles as a frame-weighted mean, which is not

a quantile. On V1 that put the right gripper's `action.q99` at 0.709 against a

true 1.100, so pi0.5's QUANTILES normalisation mapped "fully closed" to +2.10

and pushed 24% of action frames outside [-1, 1]. Quantiles are a property of

the whole dataset, so this must run AFTER the last episode is converted.

  

**How to tell whether it already ran on a given dataset:** the script writes

`meta/stats.json.bak` once, so no `.bak` means no fix. The fingerprint is also

visible in the numbers — compare the two gripper dimensions (action index 9 =

right, 19 = left) against their own `max`:

  

micromamba run -n lerobot python -c "

import json; a = json.load(open('$D/meta/stats.json'))['action']

print('q99', [round(float(a['q99'][i]), 3) for i in (9, 19)])

print('max', [round(float(a['max'][i]), 3) for i in (9, 19)])"

  

A gripper that reaches `max 1.1` but reports `q99 0.73` has not been fixed: a

gripper is closed for far more than 1% of the frames, so its true q99 sits at

the closed value. Both grippers landing near 1.09 is what a fixed dataset looks

like.

  

## 1. One-time cluster setup

  

Requires TU Darmstadt network or VPN.

  

ssh hessian

  

**SSH.** `hessian` is an alias in `~/.ssh/config` on the lab PC; the key is

already installed. `IdentitiesOnly yes` in that block is load-bearing — the

gnome-keyring agent holds three unrelated keys (martin_github, jetson,

theo_gitlab) and without it ssh offers them first and can exhaust the server's

`MaxAuthTries` before ever reaching the right one. To redo it on another

machine: `ssh-copy-id hessian`.

  

**Make your workspace directory.** Group convention is a **first-name** dir

(`christian`, `emil`, `fabian`, `ken`, `theo`, …), not the TU id — the scripts

here use `martin`, so this is not `$USER`:

  

mkdir -p /mnt/vast/workspaces/VLA_Reinforcement/martin/ecu/{data,models,runs}

  

**Environment.** `spack install` fails on this cluster (the shared Spack DB is

read-only: `Can't take write lock ... .spack-db/prefix_lock`), so use a private

conda/micromamba install in `$HOME`:

  

# miniconda into $HOME, then

conda create -n lerobot python=3.12 -y && conda activate lerobot

  

**LeRobot.** Copy the local editable checkout (it is pinned to the version the

dataset and the preprocessor patch were built against):

  

rsync -avz ~/projects/lerobot-0.5.1 hessian:/mnt/vast/workspaces/VLA_Reinforcement/martin/ecu/

  

# on the cluster

cd $WS/martin/ecu/lerobot-0.5.1 && pip install -e ".[pi]"

  

The `[pi]` extra is required — without it pi05 dies on

`CONFIG_MAPPING["paligemma"] = None`. It pins `transformers==5.3.0`.

  

**HuggingFace cache and token.** The group convention is to share the cache so

pi05_base (14 GB) is stored once. Share the *hub cache*, not `HF_HOME`:

  

export HF_HUB_CACHE=/mnt/vast/workspaces/VLA_Reinforcement/huggingface/hub

export HF_HOME=$HOME/.cache/huggingface # keep private

  

`hf auth login` writes your token into `$HF_HOME`. If `HF_HOME` points at the

shared workspace, your personal HF token becomes readable by everyone with

access to it. Keep `HF_HOME` private and share only `HF_HUB_CACHE`.

  

pi05's tokenizer pulls the **gated** `google/paligemma-3b-pt-224`, which

normally needs a token (HF account `MartinC02` has accepted the license). But as

of 2026-08-12 the shared cache **already holds it** — 21 MB, tokenizer files

only (`tokenizer.json`, `tokenizer.model`, …), which is exactly the part pi05

needs. So try `HF_HUB_OFFLINE=1` first and only fall back to:

  

hf auth login # paste the token from the lab PC's ~/.cache/huggingface/token

  

**Base model + the preprocessor patch.** pi05_base ships a

`policy_preprocessor.json` naming a step `relative_actions_processor`, which

this LeRobot renamed to `delta_actions_processor`; loading fails without the

rename. pi05_base (14 GB) is **already in the shared cache**, so the

`snapshot_download` below is a local copy, not a download.

  

Do NOT patch the shared cache in place. That cache is `drwxrws---` and **group-

writable by you**, so a `sed -i` there would silently break pi05 for everyone

else in VLA_Reinforcement — and any re-download would revert it anyway. Take a

private snapshot and patch that:

  

python - <<'PY'

from huggingface_hub import snapshot_download

import os, shutil

dst = os.path.expandvars("$WS/martin/ecu/models/pi05_base_patched")

src = snapshot_download("lerobot/pi05_base")

shutil.copytree(src, dst, dirs_exist_ok=True)

print(dst)

PY

  

cd $WS/martin/ecu/models/pi05_base_patched

cp policy_preprocessor.json policy_preprocessor.json.bak

sed -i 's/relative_actions_processor/delta_actions_processor/' policy_preprocessor.json

  

The step is `enabled: false` anyway — it is the action−state transform that must

**not** be used for our DeltaCartesian actions.

  

## 2. Ship the dataset

  

`$D` is the local dataset dir from step 0.

  

WS=/mnt/vast/workspaces/VLA_Reinforcement

rsync -avz --progress "$D/" hessian:$WS/martin/ecu/data/bmw_ecu_v5/

  

Note the **trailing slash on the source** — it copies the dataset's *contents*

into `data/bmw_ecu_v5/`, dropping the local `_<ts>` suffix so the path matches

`DATASET` in the batch script. Without it you get a nested

`bmw_ecu_v5/bmw_ecu_assembly_<ts>/` and the job cannot find `meta/`.

  

Ship each revision to its OWN directory rather than overwriting an earlier one

— `bmw_ecu_assembly` is V1, then `bmw_ecu_v2`, `bmw_ecu_v3`, `bmw_ecu_v4`:

rsync without `--delete` would merge the new files into the old tree and leave

a dataset that is neither. Each generation's three runs read their own dataset

dir, so earlier data and runs stay reproducible.

  

~4 GB over the VPN takes a while; run it under `tmux`/`screen` so a dropped

connection does not kill it. rsync is resumable — just re-run the same command.

  

Datasets and checkpoints go in the **workspace** (10 TB), never in `$HOME`

(200 GB, and the docs explicitly forbid checkpoints/caches there).

  

## 3. Train

  

**Derive a batch script for the new generation** rather than editing the old

one — copy the newest (`train_pi05_v4.sbatch`) and change exactly three lines:

  

cp train_pi05_v{4,5}.sbatch

  

DATASET=$PROJECT/data/bmw_ecu_v5

OUTPUT=$PROJECT/runs/pi05_v5_$VARIANT

--dataset.repo_id=bmw_ecu_v5

  

The dataset and output dirs are versioned **together** on purpose: the resume

block keys off `$OUTPUT`, so a new run left pointing at the previous `runs/`

dir finds `checkpoints/last` and silently RESUMES the old training instead of

starting clean. Everything else — batch 8, seed 1000, 30000 steps, the

per-variant LR and the `rename_map` — stays untouched, or the three-way

comparison stops being one.

  

Check the environment first — this catches a missing `[pi]` extra, an unpatched

base model or a half-transferred dataset on the login node, instead of two

minutes into a GPU allocation:

  

bash preflight.sh bmw_ecu_v5

  

sbatch -J pi05_v5_a1 train_pi05_v5.sbatch a1_full

sbatch -J pi05_v5_a2 train_pi05_v5.sbatch a2_frozen

sbatch -J pi05_v5_a3 train_pi05_v5.sbatch a3_lora

squeue -u $USER

tail -f pi05_v5_a1_<jobid>.out

  

`train_pi05.sbatch` is the V1 single-variant script, kept for reference only;

`train_pi05_v2/v3/v4.sbatch` are the generations that have actually run.

  

Notes on the batch script:

  

- **Capacity is thin.** The `h200` partition has only two nodes and one is down:

`cn36` (8× H200, 224 CPUs, 2 TB RAM) is the only usable one, and `cn37` has

been `DRAIN` since 2026-07-10 (`NVSwitch1 thermal throttling SXid

10004/10005`). Expect to queue behind other group members; `sinfo -R` shows

the current drain reasons.

- **Preemption.** `h200` + `qos=normal` is preemptible. `--requeue`,

`--open-mode=append` and the resume block mean a preemption costs you at most

the work since the last `save_freq`. Set `save_freq` low enough that losing

that much is acceptable, and high enough not to drown in 11 GB checkpoints.

- **Checkpoints are 11 GB each** (8.8 GB weights + 2.1 GB optimizer state).

`steps=30000, save_freq=5000` is 6 checkpoints ≈ 66 GB. LeRobot does not prune

old ones — delete them yourself as the run progresses.

- **Batch size is pinned at 8 for all three variants, deliberately.** The V1

script scaled it to the card (16 on an H200, 8 on an H100). That is fine for a

single run and fatal for a three-way comparison: which GPU Slurm happens to

hand you would silently become an experimental variable. Do not "optimise" it

per variant unless a variant cannot fit — and if one cannot, report that as a

result rather than quietly changing it.

- **A1 memory.** A full fine-tune is ~8.3 GB bf16 weights + 8.3 GB grads +

~33 GB fp32 AdamW moments ≈ 50 GB before activations, so `a1_full` is the one

variant that turns gradient checkpointing on. If it still OOMs on an 80 GB

H100, say so in the report with the hardware named; that is information.

- **LoRA targets are set explicitly in the script.** LeRobot's built-in pi0.5

defaults name `state_proj` and `action_time_mlp_in/out`, none of which exist

in this checkpoint (pi0.5 has no `state_proj` and calls them `time_mlp_*`), so

the defaults would adapt 38 modules / 1.29 M params — 500× less than

`a2_frozen` and not a meaningful comparison.

- **Trainable parameters**, measured from the checkpoint (4143.4 M total):

`a1_full` 4143.4 M (100%), `a2_frozen` 693.5 M (16.7%), `a3_lora` 16.0 M

(0.39% — 13.86 M adapters at r=32 plus the 2.17 M action/time projections).

- **How many steps.** Epochs are `steps * batch_size / total_frames`, so

`steps=30000` at `batch_size=8` is 240 k samples: ~0.7 epochs on V3/V4's 201

episodes (~340 k frames), ~1.5 epochs on a 163 k-frame dataset. Read the

epoch count off `meta/info.json` before deciding, judge the length from the

loss curve rather than raising it blind, and remember a longer run costs

11 GB per extra checkpoint.

- **Learning rate.** 2.5e-5 (the pi05 default) for `a1_full` and `a2_frozen`;

1e-4 for `a3_lora`, because LoRA wants a higher LR than a full fine-tune.

Declared per-variant in the script rather than silently inherited.

- Do **not** set `--policy.use_relative_actions=true` — it computes

action−state, meaningless for absolute Cartesian EE actions vs joint-position

state. (V2 uses `AbsoluteCartesian`; V1 used `DeltaCartesian`. Either way this

stays off.)

- Interactive `srun`/`salloc` silently get QoS `interactive` (7-day cap). Use it

for debugging the env, not for the run.

  

## 4. Bring the policy back and deploy

  

Only `pretrained_model/` is needed for inference; leave `training_state/` behind.

  

rsync -avz --progress \

hessian:$WS/martin/ecu/runs/pi05_v5_a1_full/checkpoints/030000/pretrained_model/ \

~/projects/train_outputs/pi05_v5_a1_full/checkpoints/030000/pretrained_model/

  

**Repoint the tokenizer immediately after every pull, before the first

rollout.** Every checkpoint's `policy_preprocessor.json` still names the

CLUSTER tokenizer path (`$WS/martin/ecu/models/paligemma_tokenizer`, the local

snapshot the compute nodes need because they have no internet):

  

CKPT=~/projects/train_outputs/pi05_v5_a1_full/checkpoints/030000/pretrained_model

sed -i.bak-claude \

's#/mnt/vast/workspaces/[^"]*paligemma_tokenizer#'"$HOME"'/projects/train_outputs/paligemma_tokenizer#' \

"$CKPT/policy_preprocessor.json"

grep tokenizer_name "$CKPT/policy_preprocessor.json"

  

Skipping this fails in a maximally confusing way: `make_pre_post_processors`

throws inside the server's `SendPolicyInstructions` RPC, nothing wraps that RPC

in try/except, so `self.preprocessor` stays `None` and every later `GetActions`

dies with `TypeError: 'NoneType' object is not callable` at

`policy_server.py:394`, forever, with the timestep stuck at 0. The handshake

error itself is logged exactly once.

  

Point the client at the checkpoint in

`pixi_franka_crisp_ws/src/franka_policy/config/robi_client.yaml`, then:

  

# policy server (lerobot env, gRPC on 127.0.0.1:8080)

micromamba run -n lerobot python franka_policy/server/policy_server.py

  

# client (humble env, run from the client/ dir -- it resolves imports by CWD)

pixi shell -e humble

cd src/franka_policy/franka_policy/client && python client.py

  

The same `rename_map` must be in the client handshake as in training — it is

already in `robi_client.yaml`. Keys: `r` run, `s` stop, `h` home.

  

**Known gap before you trust a rollout:** `RobiClient._send_home_pose` is

single-arm and publishes to the `target_pose` *topic*, but that is a

`SetPoseStamped` **service** — the publish is silently ignored. Port the

service-based bimanual homing from `replay_client.py` before relying on `h`.

Within-chunk execution is open-loop (50 steps = 1 s at 50 Hz), so the

measured-pose anchoring in the client's action processors matters.