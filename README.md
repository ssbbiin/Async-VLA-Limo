# AsyncVLA-Gemma-LIMO

Gemma 기반 AsyncVLA 비동기 추론 구조 및 ROS 2 기반 AgileX LIMO Pro 실물 로봇 적용

## 목차

- [1. 연구 배경 및 목표](#1-연구-배경-및-목표)
- [2. 전체 시스템 구성](#2-전체-시스템-구성)
- [3. Gemma 기반 VLA 모델 구성](#3-gemma-기반-vla-모델-구성)
- [4. AsyncVLA Slow / Fast 구조](#4-asyncvla-slow--fast-구조)
- [5. 데이터셋 및 학습 구성](#5-데이터셋-및-학습-구성)
- [6. Gazebo Sync / Async 비교 실험](#6-gazebo-sync--async-비교-실험)
- [7. 추론 지연 및 Observation Freshness 분석](#7-추론-지연-및-observation-freshness-분석)
- [8. ROS 2 및 LIMO Pro 통합](#8-ros-2-및-limo-pro-통합)
- [9. 실물 로봇 주행 실험](#9-실물-로봇-주행-실험)
- [10. 실험 결과 및 한계](#10-실험-결과-및-한계)
- [11. 향후 진행 계획](#11-향후-진행-계획)
- [12. 소스코드](#12-소스코드)

## 프로젝트 개요

Vision-Language-Action(VLA) 모델은 이미지와 자연어 명령을 입력받아 로봇의 행동을 직접 생성할 수 있지만, 대규모 멀티모달 모델의 높은 추론 지연으로 인해 실제 로봇 제어에서는 최신 관측을 즉시 반영하기 어렵다는 문제가 있다.

본 프로젝트에서는 AsyncVLA의 **Slow VLA / Fast Edge Adapter** 구조를 기반으로 Gemma 계열 멀티모달 모델을 모바일 로봇 행동 생성 파이프라인에 연결하였다.

Slow VLA는 과거 RGB 이미지와 자연어 명령으로 semantic/action feature를 생성하고, Fast Edge Adapter는 해당 feature와 최신 RGB observation을 결합하여 미래 주행 궤적을 빠르게 갱신한다.

최종적으로 다음 항목을 구현 및 검증하는 것을 목표로 한다.

- Gemma 기반 multimodal feature와 Action Adapter 연결
- AsyncVLA Slow / Fast 비동기 추론 구조 구현
- 공식 Edge Adapter 기반 미래 주행 궤적 생성
- Sync / Async 구조의 trajectory update rate 비교
- Observation age 및 inference latency 분석
- ROS 2 기반 카메라·제어 파이프라인 구성
- Robot-local Safety Guard 구현
- AgileX LIMO Pro 실물 로봇 적용

현재 **Model → ROS 2 → Safety Guard → LIMO Pro 실제 구동**까지 연결한 상태이며, 실제 환경에서의 자연어 기반 목표 도착과 반복적인 Sync / Async 비교 실험은 진행 중이다.

---

## 1. 연구 배경 및 목표

일반적인 synchronous VLA 구조에서는 하나의 observation에 대한 VLA 추론이 완료된 이후에 새로운 행동 또는 trajectory를 생성한다.

따라서 모델 추론 시간이 길어질 경우 로봇이 실제 행동을 수행하는 시점에는 모델이 사용한 observation이 이미 오래된 상태가 될 수 있다.

예를 들어 Slow VLA의 추론 시간이 약 1초 이상인 경우,

`현재 RGB → VLA 추론 → trajectory 생성 → 로봇 제어`

과정이 완료되는 동안 실제 환경과 로봇 상태는 계속 변화한다.

본 프로젝트에서는 이러한 문제를 줄이기 위해 AsyncVLA의 비동기 구조를 적용하였다.

핵심 아이디어는 다음과 같다.

`Slow VLA → Semantic / Action Feature`

와

`Latest RGB + Feature → Fast Edge Adapter → Trajectory`

를 분리하여 Slow VLA가 새로운 feature를 생성하는 동안에도 Fast Edge Adapter가 최신 RGB observation을 이용해 새로운 trajectory를 생성하도록 구성한다.

이를 통해 다음 두 항목을 중심으로 Sync / Async 구조를 비교한다.

- 새로운 trajectory가 실제 controller에 적용되는 주기
- trajectory 생성에 사용되는 최신 observation의 시간 지연

이후 해당 구조를 ROS 2와 AgileX LIMO Pro에 연결하여 실제 모바일 로봇 환경에서도 적용 가능성을 검증한다.

---

## 1. 데이터셋 및 실험 환경

본 프로젝트에서는 AsyncVLA의 비동기 VLA 구조를 Gemma 기반 모델에 적용하기 위해 공개 모바일 로봇 내비게이션 데이터셋을 사용하였다.

학습 및 평가에는 **RECON/GNM, LeLaN, HuRoN** 데이터를 사용하였으며, 공식 AsyncVLA/OmniVLA 계열 모델의 action feature를 teacher로 활용하여 Gemma 기반 student 모델을 학습하였다.

### 데이터셋 구성

| Dataset | 활용 목적 |
|---|---|
| RECON / GNM | 일반적인 visual navigation trajectory 학습 |
| LeLaN | 자연어 명령이 포함된 language-conditioned navigation 학습 및 평가 |
| HuRoN | 시간적으로 연속된 실제 로봇 주행 데이터 기반 temporal adaptation |

HuRoN 데이터는 원본 데이터를 그대로 사용하는 것이 아니라 실제 비동기 추론 구조에 맞게 시간축을 재구성하였다.

주요 전처리 조건은 다음과 같다.

- RGB sampling: 3 Hz
- Frame stride: 1
- Timestamp 기반 XY 위치 보간
- Shortest-angle 방식의 yaw 보간
- 연속 frame 사이의 시간 간격이 0.457027초를 초과하는 구간 제거

본 프로젝트는 원 AsyncVLA 논문의 전체 학습 데이터와 환경을 그대로 재현한 것이 아니라, **공개 데이터셋과 사용 가능한 하드웨어 환경에 맞춰 재구성한 adaptation 연구**이다.

---

### 학습 데이터 구성

주 학습 단계에서는 GNM, LeLaN, HuRoN을 함께 사용하였다.

Effective batch는 다음과 같이 구성하였다.

| Dataset | Batch 내 sample 수 |
|---|---:|
| GNM | 60 |
| LeLaN | 60 |
| HuRoN | 60 |
| **Total** | **180** |

주 학습 300 update 동안 총 **54,000개의 training identity**를 사용하였다.

이후 언어 조건과 action representation을 보강하기 위해 여러 후속 학습 단계를 진행하였으며, 현재 실험에 사용하는 최종 선택 모델은 **Coverage30**이다.

Coverage30은 전체 학습을 30회만 수행한 모델을 의미하지 않는다. 이전 단계에서 학습된 Adapter, projector, Edge 및 Gemma parameter를 이어받은 뒤 마지막 단계에서 Gemma LoRA를 추가 학습한 모델이다.

---

### 실험 환경

#### AI / Training

| 항목 | 구성 |
|---|---|
| Base VLM | Gemma 4 E4B |
| Framework | PyTorch |
| GPU | NVIDIA RTX 4060 Ti 8GB |
| Training | Teacher Feature Distillation, LoRA |
| Teacher | AsyncVLA / OmniVLA 계열 공식 모델 |
| Simulation | Gazebo |

#### Robot

| 항목 | 구성 |
|---|---|
| Robot Platform | AgileX LIMO Pro |
| Middleware | ROS 2 |
| Vision Sensor | RGB Camera |
| Safety Sensor | LiDAR |
| Robot State | Wheel Odometry / Chassis State |
| Inference Device | Desktop GPU |
| Robot Control | ROS 2 기반 local safety guard + base driver |

Gemma 기반 VLA 추론은 LIMO Pro 내부 보드가 아니라 **RTX 4060 Ti가 장착된 외부 데스크톱**에서 수행한다.

데스크톱에서 생성한 trajectory 기반 command를 ROS 2를 통해 로봇으로 전달하고, LIMO Pro에서는 별도의 local safety guard가 센서 상태와 명령 유효성을 확인한 뒤 실제 속도 명령을 전달하도록 구성하였다.

---

## 2. 입력 데이터 구성

본 시스템의 입력은 크게 **자연어 명령, RGB 이미지, 시간 정보, 로봇 상태 정보**로 구성된다.

Async 구조에서는 Slow VLA와 Fast Edge Adapter가 서로 다른 시점의 visual information을 사용하므로, 각 입력이 어느 시점의 정보인지 구분하는 것이 중요하다.

---

### 2.1 자연어 명령

사용자는 로봇이 수행해야 할 행동을 영어 자연어 명령으로 입력한다.

예시는 다음과 같다.

```text
"Go to the black chair."
"Approach the cabinet."
"Move toward the red chair."
```

자연어 명령은 RGB observation과 함께 Gemma 기반 Slow VLA에 입력되며, 현재 장면에서 어떤 대상과 행동이 중요한지 표현하는 semantic/action feature 생성에 사용된다.

```text
Natural Language Command
          +
      RGB Observation
          ↓
      Gemma VLA
          ↓
Semantic / Action Feature
```

---

### 2.2 RGB Observation

실물 환경에서는 LIMO Pro의 RGB camera에서 다음 ROS 2 topic을 통해 이미지를 수신한다.

```text
/camera/color/image_raw
```

메시지 타입은 다음과 같다.

```text
sensor_msgs/Image
```

실제 LIMO Pro 연결 시험에서는 **640 × 480 RGB image**를 정상적으로 수신하였다.

카메라 입력은 Async 구조에서 두 가지 역할로 사용된다.

#### Slow VLA Input

Slow VLA는 특정 시점의 RGB frame과 자연어 명령을 입력으로 받아 semantic/action feature를 생성한다.

```text
RGB(t_slow)
    +
Language Command
        ↓
      Gemma
        ↓
Action Adapter
        ↓
Semantic / Action Feature
```

#### Fast Edge Input

Fast Edge Adapter는 Slow VLA가 사용했던 과거 RGB가 아니라, 가능한 한 **현재 시점에 가까운 최신 RGB observation**을 추가로 사용한다.

```text
Slow Feature
      +
Latest RGB(t_now)
      ↓
Edge Adapter
      ↓
Updated Trajectory
```

이 구조를 통해 Slow VLA inference가 진행되는 동안에도 최신 visual information을 trajectory 생성 과정에 반영할 수 있도록 구성하였다.

---

### 2.3 Past Observation / Current Observation

AsyncVLA 구조에서는 동일한 이미지 입력을 계속 사용하는 것이 아니라, 두 시점의 visual observation을 구분한다.

- **Past Observation**  
  Slow VLA가 semantic/action feature를 생성할 때 사용한 RGB observation

- **Current Observation**  
  Fast Edge Adapter가 trajectory를 갱신하는 시점의 최신 RGB observation

개념적으로 다음과 같이 구성된다.

```text
Past RGB
   │
   ├──→ Slow VLA
   │       ↓
   │    Feature
   │
   └───────────────┐
                   │
Current RGB ───────┼──→ Edge Adapter
                   │
              Slow Feature
                   ↓
               Trajectory
```

현재 구현에서는 Slow VLA가 사용한 과거 visual information과 생성된 feature를 함께 보관하여 서로 다른 시점의 정보가 잘못 결합되지 않도록 구성하였다.

---

### 2.4 Timestamp

비동기 구조에서는 **어떤 feature가 어떤 RGB observation에서 생성되었는가**를 유지하는 것이 중요하다.

따라서 Slow VLA가 생성한 packet에는 다음 정보가 함께 저장된다.

```text
Feature
Past Observation
Source Timestamp
```

실물 runtime에서는 카메라 메시지의 원본 header timestamp를 사용하여 observation 시점을 기록한다.

Fast Edge Adapter가 실행될 때는 최신 RGB timestamp와 Slow feature의 timestamp 차이를 검사한다.

현재 live runtime의 조건은 다음과 같다.

```text
0 ≤ Latest RGB Timestamp - Feature Timestamp ≤ 3 s
```

즉, 너무 오래된 semantic feature는 Fast Edge Adapter 입력으로 사용하지 않는다.

이 시간 차이는 **두 카메라 observation의 source timestamp 차이**이며, 네트워크 통신 지연 자체를 의미하지 않는다.

---

### 2.5 Action Feature

Gemma의 multimodal representation은 외부 Action Adapter와 projector를 거쳐 AsyncVLA Edge Adapter와 호환되는 feature로 변환된다.

현재 모델의 feature 흐름은 다음과 같다.

```text
Gemma Multimodal Representation
            ↓
    32-query Action Adapter
            ↓
       32 × 4096
            ↓
     Action Projector
            ↓
        8 × 1024
```

최종적으로 생성되는 Slow VLA feature의 shape은 다음과 같다.

```text
[Batch, Horizon, Feature]

[1, 8, 1024]
```

이 feature는 미래 행동에 필요한 semantic/action information을 표현하며, Fast Edge Adapter에서 최신 RGB observation과 함께 사용된다.

---

### 2.6 Trajectory Output

Fast Edge Adapter는 Slow VLA feature와 최신 RGB observation을 입력으로 받아 다음 shape의 delta trajectory를 생성한다.

```text
[1, 8, 4]
```

각 trajectory는 이후 SE(2) 기반 pose representation으로 변환한 뒤 PD controller에 전달된다.

전체 흐름은 다음과 같다.

```text
Gemma
  ↓
Action Feature
[1, 8, 1024]
  +
Latest RGB
  ↓
Edge Adapter
  ↓
Delta Trajectory
[1, 8, 4]
  ↓
SE(2) Pose
  ↓
PD Controller
```

---

### 2.7 Robot-side Input

실제 LIMO Pro에서는 모델이 생성한 trajectory만으로 바로 로봇을 움직이지 않는다.

Robot-local Safety Guard가 다음 정보를 추가로 확인한다.

- RGB 상태
- LiDAR scan
- Wheel odometry
- Chassis state
- Planned command
- Command timestamp
- Session / sequence
- STOP condition

따라서 전체 입력 구조는 단순한

```text
RGB + Language → Action
```

이 아니라 실제로는 다음과 같다.

```text
                 ┌─ Natural Language
                 │
RGB Observation ─┼─→ VLA / Edge Adapter
                 │
                 └─ Timestamp
                         ↓
                 Planned Trajectory
                         ↓
             ┌───────────┴───────────┐
             │                       │
           LiDAR                  Odometry
             │                       │
        Chassis State          Command State
             └───────────┬───────────┘
                         ↓
                    Safety Guard
                         ↓
                     Robot Motion
```

이를 통해 AI model inference와 실제 로봇 actuation을 분리하였다.

---

## 3. 전체 처리 과정

전체 시스템은 **Slow VLA inference**, **Fast Edge trajectory generation**, **ROS 2 command transmission**, **Robot-local Safety Guard**의 네 단계로 구성된다.

전체 처리 흐름은 다음과 같다.

```text
RGB Image + Natural Language Command
                ↓
        Gemma Multimodal Model
                ↓
        32-query Action Adapter
                ↓
          Action Projector
                ↓
        Semantic / Action Feature
                ↓
      ┌─────────────────────┐
      │   Slow VLA Output   │
      └──────────┬──────────┘
                 │
                 │ Feature + Past RGB + Timestamp
                 ▼
      ┌─────────────────────┐
      │   Fast Edge Path    │
      │                     │
      │ Feature + Latest RGB│
      │          ↓          │
      │    Edge Adapter     │
      │          ↓          │
      │  Delta Trajectory   │
      └──────────┬──────────┘
                 ↓
          SE(2) Trajectory
                 ↓
           PD Controller
                 ↓
    /asyncvla/planned_command
                 ↓
       Robot-local Safety Guard
                 ↓
   /asyncvla/guarded_cmd_vel
                 ↓
          LIMO Base Driver
                 ↓
          AgileX LIMO Pro
```

---

### 3.1 RGB 및 자연어 명령 입력

LIMO Pro의 RGB camera에서 실시간 이미지를 수신하고, 사용자가 입력한 자연어 명령과 함께 Slow VLA의 입력으로 사용한다.

```text
RGB Observation
       +
Language Instruction
       ↓
Gemma Multimodal Model
```

실물 환경에서 RGB image는 ROS 2의 다음 topic을 통해 수신한다.

```text
/camera/color/image_raw
```

카메라 message의 원본 timestamp도 함께 저장하여 이후 Slow feature와 최신 observation 사이의 시간 차이를 계산한다.

---

### 3.2 Gemma Multimodal Feature 추출

RGB 이미지와 자연어 명령은 Gemma의 native multimodal forward를 통과한다.

본 프로젝트에서는 Gemma 내부 attention 구조 자체를 변경하기보다, Gemma에서 생성한 multimodal representation을 외부 **Action Adapter**와 연결하는 방식을 사용하였다.

```text
Image + Language
       ↓
Gemma Native Multimodal Forward
       ↓
Multimodal Representation
```

이를 통해 pretrained Gemma의 visual-language representation을 유지하면서 로봇 행동 생성에 필요한 representation으로 변환한다.

---

### 3.3 Action Adapter

Gemma의 multimodal representation을 로봇 행동 생성에 사용할 수 있도록 **32개의 learnable action query**를 사용하는 외부 Action Adapter를 구성하였다.

```text
Gemma Representation
        ↓
32-query Action Adapter
        ↓
32 × 4096 Action Representation
```

Action Adapter는 Gemma가 생성한 visual-language representation에서 미래 행동 생성에 필요한 정보를 추출하는 역할을 한다.

이 단계에서는 직접적인 linear/angular velocity를 생성하지 않고, 이후 trajectory prediction에 사용할 latent action representation을 생성한다.

---

### 3.4 Action Projector

Action Adapter의 출력은 기존 AsyncVLA Edge Adapter와 연결할 수 있도록 projector를 통해 변환한다.

```text
32 × 4096
     ↓
Action Projector
     ↓
8 × 1024
```

최종적으로 Slow path에서 생성되는 feature는 다음 형태를 가진다.

```text
[1, 8, 1024]
```

이 feature는 미래 8개 step에 대한 행동 정보를 표현하며 Fast Edge Adapter의 입력으로 전달된다.

---

### 3.5 Slow Packet 생성

Slow VLA inference가 끝나면 feature만 단독으로 저장하지 않고 해당 feature를 생성할 때 사용한 observation 정보와 함께 하나의 packet으로 저장한다.

```text
Slow Packet
├── Action Feature
├── Past RGB Observation
└── Source Timestamp
```

이를 통해 Fast Edge Adapter가 오래된 feature를 최신 RGB와 결합할 때 **feature가 어떤 시점의 observation에서 생성되었는지** 추적할 수 있다.

현재 live runtime에서는 긴 FIFO queue를 사용하지 않고 가장 최근에 생성된 packet 하나를 유지하는 **single-slot 방식**을 사용한다.

Slow thread와 Fast loop 사이의 packet 교환에는 `threading.Lock`을 사용한다.

---

### 3.6 Latest RGB 갱신

Slow VLA가 새로운 feature를 계산하는 동안에도 RGB camera는 계속 새로운 observation을 제공한다.

```text
t0        Slow VLA 시작
│
├─ RGB(t0)
├─ RGB(t1)
├─ RGB(t2)
├─ RGB(t3)
│
t4        Slow VLA 완료
```

Synchronous 방식에서는 `RGB(t0)`를 기반으로 생성된 trajectory를 그대로 사용하게 된다.

반면 Async 방식에서는 Slow VLA가 생성한 semantic/action feature를 유지하면서 **현재 시점에 가까운 RGB(t4)**를 Fast Edge Adapter에 입력한다.

```text
Feature from RGB(t0)
        +
Latest RGB(t4)
        ↓
Fast Edge Adapter
```

이 구조가 본 프로젝트에서 다루는 비동기 처리의 핵심이다.

---

### 3.7 Fast Edge Adapter

Fast Edge Adapter는 다음 두 정보를 사용한다.

1. Slow VLA가 생성한 semantic/action feature
2. 현재 시점의 최신 RGB observation

```text
Slow Feature
     +
Latest RGB
     ↓
Edge Adapter
     ↓
Delta Trajectory
```

Edge Adapter의 출력은 다음 shape의 delta trajectory이다.

```text
[1, 8, 4]
```

Fast Edge Adapter는 Slow VLA 전체를 다시 실행하는 것보다 훨씬 빠르게 동작하기 때문에 새로운 visual observation을 더 높은 주기로 trajectory에 반영할 수 있다.

---

### 3.8 Delta Trajectory → SE(2) Pose

Edge Adapter가 출력한 delta representation은 누적 변환을 통해 실제 로봇 주행에 사용할 수 있는 SE(2) pose trajectory로 변환한다.

전체 과정은 다음과 같다.

```text
Delta Trajectory
       ↓
delta_to_pose
       ↓
SE(2) Future Poses
```

이를 통해 현재 로봇 위치를 기준으로 미래에 따라가야 할 상대적인 2D trajectory를 구성한다.

---

### 3.9 PD Controller

생성된 future trajectory를 실제 differential-drive 로봇 명령으로 변환하기 위해 AsyncVLA의 기존 PD controller를 사용한다.

```text
Future Trajectory
       ↓
PD Controller
       ↓
Linear Velocity
Angular Velocity
```

실물 LIMO Pro에서는 안전한 초기 검증을 위해 출력 속도에 추가적인 저속 제한을 적용하였다.

모델이나 Edge Adapter가 직접 `/cmd_vel`을 발행하지 않고, 먼저 계획된 command 형태로 ROS 2에 전달하도록 구성하였다.

---

### 3.10 Planned Command 전송

데스크톱에서 계산된 제어 명령은 다음 topic으로 전달한다.

```text
/asyncvla/planned_command
```

메시지는 trajectory 기반 제어 정보와 session, sequence, timestamp 등의 정보를 포함한다.

구조적으로는 다음과 같다.

```text
Desktop GPU
    │
    │ VLA / Edge inference
    ↓
PD Controller
    ↓
Planned Command
    ↓
ROS 2 Network
    ↓
LIMO Pro
```

이 단계까지는 **AI가 제안한 명령**이며 아직 실제 motor command는 아니다.

---

### 3.11 Robot-local Safety Guard

실제 로봇에서는 별도의 Safety Guard가 planned command를 검사한다.

Guard는 다음 정보를 동시에 확인한다.

```text
Planned Command
RGB Freshness
LiDAR Scan
Odometry
Chassis State
Session / Sequence
STOP Condition
        ↓
    Safety Guard
```

모든 조건을 만족한 경우에만 최종 velocity command를 생성한다.

```text
/asyncvla/planned_command
          ↓
      Safety Guard
          ↓
/asyncvla/guarded_cmd_vel
```

이를 통해 데스크톱의 AI inference와 실제 로봇의 물리적 actuation을 분리하였다.

---

### 3.12 LIMO Pro 구동

Safety Guard를 통과한 command는 vendor base driver로 전달한다.

```text
/asyncvla/guarded_cmd_vel
          ↓
      Base Driver
          ↓
      Motor Control
          ↓
     AgileX LIMO Pro
```

기존 base driver의 `/cmd_vel` subscriber를 `guarded_cmd_vel` topic으로 remap하여 Safety Guard를 우회한 속도 명령이 직접 로봇으로 전달되지 않도록 구성하였다.

따라서 최종 실물 시스템은 다음과 같은 계층으로 분리된다.

```text
Perception / Language
        ↓
Slow VLA
        ↓
Fast Edge
        ↓
Trajectory
        ↓
Controller
        ↓
Planned Command
        ↓
Safety Guard
        ↓
Physical Command
        ↓
LIMO Pro
```

이 구조를 통해 **AI inference layer와 physical safety layer를 독립적으로 구성**하였다.

---

## 4. Gemma 기반 VLA 모델 구성 및 학습

본 프로젝트에서는 기존 AsyncVLA 계열 모델의 action representation을 그대로 활용하면서, Slow VLA 부분을 **Gemma 기반 multimodal model**로 재구성하였다.

핵심 목표는 Gemma가 RGB 이미지와 자연어 명령에서 생성한 representation을 기존 AsyncVLA의 Edge Adapter가 사용할 수 있는 형태로 변환하는 것이다.

전체 모델 구조는 다음과 같다.

```text
RGB Image + Language Instruction
              ↓
        Gemma 4 E4B
              ↓
   Multimodal Representation
              ↓
    32-query Action Adapter
              ↓
       32 × 4096 Feature
              ↓
      Action Projector
              ↓
        8 × 1024 Feature
              ↓
      AsyncVLA Edge Adapter
              ↓
     8 × 4 Delta Trajectory
```

---

### 4.1 Gemma 기반 Slow VLA

Slow VLA는 **Gemma 4 E4B**를 기반으로 구성하였다.

Gemma 내부의 native multimodal attention 구조를 크게 변경하지 않고, 모델이 생성한 hidden representation을 외부 Action Adapter에 전달하는 방식을 사용하였다.

```text
Image
  +
Language
  ↓
Gemma Native Multimodal Forward
  ↓
Multimodal Hidden Representation
```

이를 통해 pretrained visual-language representation을 유지하면서 로봇 행동 생성에 필요한 추가 representation을 학습하도록 구성하였다.

Gemma의 hidden dimension은 다음과 같다.

```text
Hidden Dimension : 2560
```

---

### 4.2 Action Adapter

Gemma의 multimodal representation을 직접 trajectory로 변환하지 않고, 중간에 **32-query Action Adapter**를 추가하였다.

```text
Gemma Representation
        ↓
32 Learnable Action Queries
        ↓
Self / Cross Attention
        ↓
Action Representation
```

Action Adapter의 출력은 다음 크기를 가진다.

```text
32 × 4096
```

이 representation은 RGB와 자연어 명령에서 추출된 정보를 미래 행동 생성에 적합한 latent representation으로 변환하는 역할을 한다.

본 프로젝트에서는 Gemma 본체의 attention mask 구조를 변경하는 대신 **외부 Adapter 방식**을 사용하였다.

---

### 4.3 Action Projector

Action Adapter의 `32 × 4096` representation은 기존 AsyncVLA Edge Adapter가 사용할 수 있는 feature dimension으로 다시 변환한다.

```text
32 × 4096
     ↓
Action Projector
     ↓
8 × 1024
```

최종 Slow VLA feature는 다음 형태를 가진다.

```text
[1, 8, 1024]
```

여기서 8은 미래 trajectory horizon에 대응하며, 이후 Fast Edge Adapter의 입력으로 사용된다.

---

### 4.4 Teacher-Student 학습

Gemma 기반 student가 기존 AsyncVLA 계열 모델의 행동 representation을 학습할 수 있도록 **teacher feature distillation**을 적용하였다.

전체적인 학습 구조는 다음과 같다.

```text
            Public Navigation Data
                      │
          ┌───────────┴───────────┐
          ↓                       ↓
   Official Teacher          Gemma Student
          ↓                       ↓
  Action Feature           Student Feature
          └───────────┬───────────┘
                      ↓
               Feature Loss
                      +
              Trajectory Loss
```

Teacher는 기존 AsyncVLA / OmniVLA 계열의 공식 action representation을 제공하고, Gemma 기반 student는 동일한 입력에서 유사한 feature와 trajectory를 생성하도록 학습한다.

---

### 4.5 주 학습 단계

초기 주 학습에서는 GNM, LeLaN, HuRoN 데이터를 함께 사용하였다.

Effective batch는 다음과 같이 구성하였다.

| Dataset | Samples |
|---|---:|
| GNM | 60 |
| LeLaN | 60 |
| HuRoN | 60 |
| **Total** | **180** |

총 300 update 동안 누적 **54,000 training identities**를 사용하였다.

```text
180 samples / update
×
300 updates
=
54,000 identities
```

이 단계에서는 공식 teacher의 action feature를 Gemma 기반 student에 distillation하면서 trajectory loss도 함께 사용하였다.

---

### 4.6 후속 학습 단계

주 학습 이후에는 language conditioning과 action representation을 보강하기 위해 여러 후속 학습 단계를 진행하였다.

전체 계보는 다음과 같다.

```text
Main Distillation
        ↓
Language Diagnosis
        ↓
Adapter Paired Coverage
        ↓
Adapter Training
        ↓
Joint Training
        ↓
Public Edge Paired KD
        ↓
Output-pair KD
        ↓
Native Gemma LoRA Coverage30
```

각 단계는 서로 독립적인 전체 학습이 아니라 이전 단계의 checkpoint와 parameter를 이어받아 진행하였다.

---

### 4.7 Coverage30

현재 시뮬레이션 및 실물 테스트에 사용하는 선택 모델은 **Coverage30**이다.

Coverage30이라는 이름은 전체 모델을 30 update만 학습했다는 의미가 아니다.

이 모델은 이전의 여러 학습 단계를 모두 상속한 뒤 마지막 단계에서 **Gemma LoRA-only fine-tuning을 30 update 추가로 수행한 모델**이다.

마지막 Coverage30 단계의 구성은 다음과 같다.

| 항목 | 값 |
|---|---:|
| Update | 30 |
| Instruction Records | 5,400 |
| Anchors | 4,500 |
| LeLaN | 900 images × 2 prompts |
| GNM | 1,800 records |
| HuRoN | 1,800 records |

이 데이터는 완전히 새로운 5,400개의 독립 sample이 아니라 기존 TRAIN 데이터에서 구성한 replay 기반 학습 데이터이다.

따라서 Coverage30의 성능을 해석할 때는 마지막 30 update뿐 아니라 이전 학습 단계 전체를 함께 고려해야 한다.

---

### 4.8 LoRA Fine-tuning

Coverage30 단계에서는 Gemma 전체 parameter를 다시 학습하지 않고 **LoRA 기반 fine-tuning**을 사용하였다.

이 단계에서는 기존에 학습된 Adapter, projector 및 Edge 구조를 유지하면서 Gemma의 multimodal representation이 현재 navigation task에 더 적합하도록 조정하였다.

```text
Pretrained / Previously Trained Gemma
              ↓
          LoRA Update
              ↓
Navigation-conditioned Representation
```

마지막 학습 단계에서 기록된 peak GPU memory는 약 다음 수준이었다.

```text
Peak Allocated VRAM : 2.832 GiB
```

RTX 4060 Ti 8GB 환경에서도 학습이 가능하도록 parameter-efficient fine-tuning 방식을 사용하였다.

---

### 4.9 Checkpoint 구조

현재 선택 모델은 단일 checkpoint 파일 하나만으로 전체 시스템이 완전히 복원되는 구조가 아니다.

현재 모델은 이전 parent checkpoint와 공식 backbone / Edge component에 대한 의존성을 가진다.

개념적으로는 다음과 같다.

```text
Parent Model
    +
Adapter / Projector
    +
Gemma LoRA Overlay
    +
Official Edge Component
        ↓
Current Coverage30 Model
```

따라서 공개용 repository에서는 checkpoint 하나만 제공하면 모든 모델이 재현된다고 설명하지 않고, 각 component와 parent dependency를 별도로 명시한다.

---

### 4.10 학습 결과 해석

현재 Gemma 기반 모델은 단순한 정적 Gazebo 환경에서는 높은 목표 접근 성공률을 보였지만, 실제 이미지 기반 평가에서는 공식 모델과 여전히 성능 차이가 존재한다.

따라서 현재 연구의 목적은

```text
Official Model을 완전히 대체
```

하는 것이 아니라,

```text
Gemma 기반 VLA 구조를 구성하고
        ↓
Async inference 구조를 적용하고
        ↓
실제 로봇까지 배포 가능한지 검증
```

하는 데 있다.

이후 실험에서는 모델 자체의 절대 성능뿐 아니라 **Slow / Fast 분리가 trajectory update rate와 observation freshness에 어떤 영향을 주는지**를 중심으로 분석한다.

---

## 5. AsyncVLA Slow / Fast 비동기 구조

기존 synchronous 방식에서는 새로운 trajectory를 생성하기 위해 매번 Slow VLA inference가 완료될 때까지 기다려야 한다.

본 프로젝트에서는 이러한 추론 지연을 줄이기 위해 **Slow VLA와 Fast Edge Adapter를 서로 다른 실행 흐름으로 분리**하였다.

핵심 아이디어는 Slow VLA가 새로운 semantic/action feature를 생성하는 동안에도, Fast Edge Adapter가 가장 최근에 생성된 feature와 최신 RGB observation을 이용해 trajectory를 계속 갱신하는 것이다.

---

### 5.1 Synchronous 구조

Synchronous 방식에서는 하나의 observation에 대해 전체 inference pipeline을 순차적으로 수행한다.

```text
RGB(t0)
   +
Language Command
   ↓
Slow VLA
   ↓
Action Feature
   ↓
Edge Adapter
   ↓
Trajectory
   ↓
Controller
```

Slow VLA inference가 끝나기 전까지 새로운 trajectory를 생성할 수 없기 때문에 trajectory update frequency가 Slow VLA의 추론 속도에 직접 제한된다.

예를 들어 Slow VLA inference가 약 1초 이상 걸릴 경우 다음 trajectory가 생성될 때까지 controller는 기존 trajectory를 계속 사용하게 된다.

```text
t0                    t1
│                     │
RGB(t0) → Slow VLA ───┤
                      ↓
                New Trajectory
```

따라서 새로운 trajectory가 생성되는 시점에는 처음 사용한 RGB observation이 이미 오래된 상태가 될 수 있다.

---

### 5.2 Asynchronous 구조

Async 방식에서는 Slow VLA와 Fast Edge Adapter를 분리한다.

```text
                  Slow Thread
                      │
RGB(t0) + Language ───┤
                      ↓
                  Slow VLA
                      ↓
               Action Feature
                      │
                      ▼
               Latest Packet
                      │
                      │
RGB(t1) ──────────────┤
RGB(t2) ──────────────┤
RGB(t3) ──────────────┤
                      ▼
               Fast Edge Loop
                      ↓
                Trajectory
```

Slow VLA가 새로운 feature를 생성하는 동안에도 RGB stream은 계속 갱신된다.

Fast Edge Adapter는 가장 최근에 생성된 semantic/action feature를 유지한 상태에서 새로운 RGB observation을 입력받아 trajectory를 반복적으로 갱신한다.

즉,

```text
Slow VLA
= 느린 semantic / language reasoning

Fast Edge
= 빠른 visual correction / trajectory update
```

의 역할로 분리한다.

---

### 5.3 Slow Thread

Slow VLA는 별도의 thread에서 실행한다.

Slow thread는 새로운 RGB observation과 자연어 명령을 입력받아 다음 과정을 수행한다.

```text
RGB + Language
      ↓
Gemma
      ↓
Action Adapter
      ↓
Projector
      ↓
Action Feature
```

Slow inference가 완료되면 다음 정보를 하나의 packet으로 저장한다.

```text
Slow Packet

├── feature
├── past RGB
└── source timestamp
```

이 packet은 Fast Edge Adapter가 다음 trajectory를 생성할 때 사용한다.

---

### 5.4 Fast Loop

Fast loop는 Slow VLA가 완료될 때까지 기다리지 않는다.

가장 최근의 Slow packet이 존재하면 최신 RGB observation과 결합하여 Edge Adapter를 반복 실행한다.

```text
Latest Slow Feature
        +
Latest RGB
        ↓
Edge Adapter
        ↓
Delta Trajectory
        ↓
Controller
```

따라서 하나의 Slow feature가 생성된 이후에도 여러 개의 최신 RGB observation을 이용해 trajectory를 다시 계산할 수 있다.

개념적으로 다음과 같이 동작한다.

```text
Slow Feature F0
      │
      ├── RGB(t1) → Trajectory 1
      │
      ├── RGB(t2) → Trajectory 2
      │
      ├── RGB(t3) → Trajectory 3
      │
      └── RGB(t4) → Trajectory 4
```

이후 Slow VLA에서 새로운 feature `F1`이 생성되면 Fast loop는 새로운 feature를 사용한다.

```text
F0 → F0 → F0 → F0
               ↓
              F1
```

---

### 5.5 Single-slot Buffer

실물 ROS 2 runtime에서는 긴 FIFO queue를 사용하지 않는다.

대신 다음 두 개의 최신 상태만 유지한다.

```text
latest observation

latest slow packet
```

새로운 observation이나 Slow feature가 생성되면 이전 값을 새로운 값으로 교체한다.

```text
Old Packet
    ↓
New Packet
    ↓
Keep Latest Only
```

실시간 로봇 제어에서는 오래된 inference 결과를 순서대로 모두 처리하는 것보다 **현재 시점과 가장 가까운 정보를 유지하는 것**이 중요하기 때문에 이러한 구조를 사용하였다.

Slow thread와 Fast loop가 동시에 같은 데이터를 접근할 수 있으므로 packet 교환 과정에는 `threading.Lock`을 사용한다.

---

### 5.6 Timestamp 기반 Freshness 검사

Async 구조에서는 최신 RGB와 너무 오래된 semantic feature가 결합되는 것을 방지해야 한다.

이를 위해 각 Slow packet에 원본 camera source timestamp를 함께 저장한다.

Fast loop에서는 다음 조건을 확인한다.

```text
0 ≤ latest_timestamp - feature_timestamp ≤ 3 s
```

즉, feature가 현재 RGB보다 미래 timestamp를 가지거나 3초 이상 오래된 경우 해당 조합을 사용하지 않는다.

여기서 측정하는 값은

```text
Latest RGB Source Timestamp
        -
Slow Feature Source Timestamp
```

이며, 네트워크 통신 latency 자체를 의미하지 않는다.

---

### 5.7 Past RGB와 Feature의 정합성

Slow VLA가 생성한 feature는 해당 feature 생성에 사용한 past RGB와 함께 보관한다.

```text
RGB(t0)
   ↓
Slow VLA
   ↓
Feature(t0)

Packet
├── RGB(t0)
├── Feature(t0)
└── Timestamp(t0)
```

따라서 feature가 어떤 observation에서 생성되었는지를 유지할 수 있다.

Fast Edge Adapter에서는 다음 조합을 사용한다.

```text
Past RGB(t0)
Feature(t0)
Latest RGB(t_now)
```

이를 통해 오래된 feature와 관계없는 과거 frame이 잘못 결합되는 문제를 줄인다.

---

### 5.8 Fast Loop와 Robot Guard의 주기

현재 live system에서는 각 component의 실행 주기가 서로 다르다.

```text
Slow VLA
   ↓
약 1초 이상의 inference latency

Fast Edge Loop
   ↓
최대 약 10 Hz 목표

Robot Safety Guard
   ↓
명목 20 Hz
```

중요한 점은 이 세 값을 동일한 의미의 frequency로 해석하면 안 된다는 것이다.

다음 값들은 각각 다른 의미를 가진다.

- Camera FPS
- Slow VLA inference frequency
- Edge Adapter execution frequency
- New trajectory application frequency
- Planned command publish frequency
- Guard loop frequency
- `/cmd_vel` publish frequency

본 프로젝트의 Sync / Async 비교에서 사용하는 핵심 지표는 이 중 **실제로 새로운 trajectory가 controller에 적용된 빈도**이다.

---

### 5.9 Sync / Async 동작 비교

전체적인 차이는 다음과 같이 정리할 수 있다.

| 항목 | Sync | Async |
|---|---|---|
| Slow VLA | 매 trajectory마다 실행 | 별도 thread에서 실행 |
| Edge Adapter | Slow VLA 이후 실행 | 최신 RGB로 반복 실행 |
| RGB observation | Slow inference 시작 시점 | 가능한 최신 observation |
| Feature | 현재 inference 결과 | 최근 Slow feature 재사용 |
| Trajectory update | Slow inference에 제한 | Fast Edge 속도에 가까움 |
| Observation freshness | 상대적으로 낮음 | 상대적으로 높음 |

---

### 5.10 Simulation에서 확인한 차이

동일한 Gemma 모델과 controller를 사용한 Gazebo 실험에서 실제로 controller에 적용된 새로운 trajectory의 갱신률은 다음과 같았다.

| Method | Run 1 | Re-evaluation |
|---|---:|---:|
| Gemma Sync | 0.794 Hz | 0.808 Hz |
| Gemma Async | **7.937 Hz** | **7.909 Hz** |

Async 구조에서 새로운 trajectory의 적용 빈도가 약 **0.8 Hz에서 7.9 Hz 수준으로 증가**하였다.

이 값은 camera FPS나 `/cmd_vel` publish frequency가 아니라 **새로운 trajectory가 controller에 적용된 빈도**이다.

---

### 5.11 Observation Freshness

Trajectory 생성에 사용되는 최신 observation의 age도 함께 측정하였다.

| Method | Run 1 | Re-evaluation |
|---|---:|---:|
| Gemma Sync | 1.757 s | 1.749 s |
| Gemma Async | **0.195 s** | **0.196 s** |

Sync 구조에서는 Slow VLA inference로 인해 약 1.75초 이전의 observation을 기반으로 trajectory가 생성되었다.

반면 Async 구조에서는 Fast Edge Adapter가 최신 RGB를 사용하면서 observation age가 약 0.20초 수준으로 감소하였다.

```text
Sync
Observation ───────────────→ Trajectory
            ~1.75 s

Async
Slow Feature ──────────────→
                  Latest RGB → Trajectory
                     ~0.20 s
```

---

### 5.12 비동기 구조의 한계

Async 구조가 모든 종류의 latency를 제거하는 것은 아니다.

Fast Edge Adapter는 최신 RGB를 빠르게 반영할 수 있지만, semantic/action feature 자체는 여전히 Slow VLA의 과거 observation을 기반으로 생성된다.

즉,

```text
Visual Information
→ Fast Edge에서 빠르게 갱신 가능

Semantic / Language Information
→ Slow VLA latency의 영향을 계속 받음
```

실제 LIMO Pro 로그에서도 Slow VLA latency가 약 **1.405초**로 측정되었으며, semantic anchor와 최신 RGB 사이에는 평균 약 **2.061초**의 시간 차이가 존재하였다.

따라서 현재 Async 구조는 **visual observation freshness와 trajectory update frequency를 개선하는 구조**로 해석하며, semantic latency 자체를 완전히 제거한 구조로 해석하지 않는다.

---

## 6. Gazebo Sync / Async 비교 실험

Async 구조가 실제 trajectory 갱신 주기와 observation freshness에 어떤 영향을 주는지 확인하기 위해 Gazebo 환경에서 Sync / Async 비교 실험을 수행하였다.

동일한 모델, 동일한 controller, 동일한 scene 조건을 유지한 상태에서 실행 방식만 Sync / Async로 변경하여 비교하였다.

---

### 6.1 실험 목적

본 실험의 주요 목적은 다음과 같다.

- Sync와 Async의 목표 도착 성능 비교
- 새로운 trajectory가 controller에 적용되는 주기 비교
- trajectory 생성에 사용되는 최신 observation의 age 비교
- 동일 환경에서 Gemma 기반 모델과 공식 모델의 차이 확인
- Async 구조의 효과가 성공률 때문인지, 반응성 때문인지 구분

본 실험에서는 단순히 성공률만 비교하지 않고 **trajectory update rate와 observation age**를 함께 측정하였다.

---

### 6.2 실험 환경

Gazebo 내 정적 환경에 서로 다른 위치의 chair target을 배치하였다.

평가에는 새롭게 선정한 **20개의 chair position**을 사용하였다.

각 위치에서는 두 종류의 자연어 목표를 사용하였다.

```text id="b1u9w0"
Red Chair
Blue Chair
```

평가 대상은 다음 네 가지 방식이다.

| Model | Runtime |
|---|---|
| Gemma | Sync |
| Gemma | Async |
| Official Model | Sync |
| Official Model | Async |

동일한 20개 위치에서 각 방식을 평가하고, 동일한 batch를 다시 한 번 재평가하였다.

---

### 6.3 전체 Episode 수

평가 구성은 다음과 같다.

```text id="1bplqr"
20 chair positions
×
2 target colors
×
4 methods
×
2 evaluation rounds
=
320 episodes
```

총 **320개의 episode**를 실행하였다.

단, 독립적으로 선정한 scene position 자체는 20개이며, 나머지는 동일한 scene에서 target, model, runtime 방식을 변경한 평가이다.

---

### 6.4 성공 판정 기준

하나의 chair position을 성공한 것으로 판정하기 위해서는 해당 위치에서

```text id="aq7aqd"
Red Chair Task
+
Blue Chair Task
```

두 개를 모두 성공해야 한다.

즉, 각 position은 하나의 **success set**으로 평가하였다.

```text id="iw5k4h"
Red 성공 + Blue 성공
        ↓
1 Successful Set
```

목표 도착은 로봇이 target center 기준 **0.75 m 이내**에 진입한 경우 성공으로 판정하였다.

---

### 6.5 목표 도착 결과

평가 결과는 다음과 같다.

| Model | Runtime | 1차 평가 | 동일 Batch 재평가 |
|---|---|---:|---:|
| Gemma | Sync | **20 / 20** | **20 / 20** |
| Gemma | Async | **20 / 20** | **20 / 20** |
| Official Model | Sync | 15 / 20 | 13 / 20 |
| Official Model | Async | **20 / 20** | 19 / 20 |

Gemma 기반 모델은 본 Gazebo 환경에서 Sync와 Async 모두 20개의 test set을 성공하였다.

따라서 이 실험 결과만으로는

```text id="k88gfn"
Async 구조가 Gemma의 성공률을 향상시켰다
```

고 주장할 수 없다.

본 환경에서는 두 방식 모두 이미 20/20 성공했기 때문에 성공률 측면에서는 ceiling에 가까운 결과가 나타났다.

---

### 6.6 Trajectory Update Rate

Sync / Async의 가장 큰 차이는 **새로운 trajectory가 controller에 적용되는 빈도**에서 나타났다.

#### 1차 평가

| Method | Trajectory Update Rate |
|---|---:|
| Gemma Sync | 0.794 Hz |
| Gemma Async | **7.937 Hz** |

#### 동일 Batch 재평가

| Method | Trajectory Update Rate |
|---|---:|
| Gemma Sync | 0.808 Hz |
| Gemma Async | **7.909 Hz** |

두 실험 모두 Async 구조에서 약 **7.9 Hz** 수준의 새로운 trajectory 갱신이 발생하였다.

반면 Sync 구조에서는 약 **0.8 Hz** 수준으로 제한되었다.

대략적인 비율은 다음과 같다.

```text id="zclaa7"
Async / Sync
≈ 7.9 / 0.8
≈ 9.9 ×
```

즉, 본 실험에서는 Async 구조가 실제 controller에 적용되는 새로운 trajectory의 갱신 빈도를 약 10배 수준으로 증가시켰다.

> 이 값은 Camera FPS, Edge Adapter 실행 횟수, `/cmd_vel` publish frequency가 아니라 **새로운 trajectory가 실제 controller에 적용된 frequency**이다.

---

### 6.7 Observation Age

Trajectory를 생성할 때 사용된 최신 observation이 현재 시점에서 얼마나 오래된 정보인지 측정하였다.

#### 1차 평가

| Method | Latest Observation Age |
|---|---:|
| Gemma Sync | 1.757 s |
| Gemma Async | **0.195 s** |

#### 동일 Batch 재평가

| Method | Latest Observation Age |
|---|---:|
| Gemma Sync | 1.749 s |
| Gemma Async | **0.196 s** |

Sync 방식에서는 Slow VLA inference의 영향을 받아 약 1.75초 이전의 observation을 기반으로 새로운 trajectory가 생성되었다.

반면 Async 방식에서는 Fast Edge Adapter가 최신 RGB를 사용하면서 observation age가 약 **0.20초** 수준으로 감소하였다.

```text id="4w9js4"
Sync  : ~1.75 s
Async : ~0.20 s
```

즉, 약 **1.55초 더 최신의 visual observation**을 trajectory 생성 과정에 반영할 수 있었다.

---

### 6.8 평균 도착 시간

목표에 성공적으로 도착한 episode의 평균 도착 시간도 비교하였다.

#### 1차 평가

| Method | Mean Arrival Time |
|---|---:|
| Gemma Sync | 8.158 s |
| Gemma Async | **7.488 s** |

#### 동일 Batch 재평가

| Method | Mean Arrival Time |
|---|---:|
| Gemma Sync | 8.663 s |
| Gemma Async | **7.643 s** |

두 평가 모두 Async 방식에서 평균 도착 시간이 더 짧게 나타났다.

다만 이 결과는 단순한 정적 Gazebo 환경에서 측정한 값이며, 실제 로봇 환경의 navigation 성능을 직접 의미하지 않는다.

또한 해당 시간은 초기 model preparation 시간을 제외하고 성공 조건을 만족할 때까지의 시간이다.

---

### 6.9 결과 요약

Gemma Sync / Async의 핵심 결과를 정리하면 다음과 같다.

| Metric | Sync | Async | 변화 |
|---|---:|---:|---:|
| Success Set | 20 / 20 | 20 / 20 | 동일 |
| Trajectory Update Rate | ~0.8 Hz | **~7.9 Hz** | 약 10배 증가 |
| Observation Age | ~1.75 s | **~0.20 s** | 약 1.55 s 감소 |
| Mean Arrival Time | 8.16 s | **7.49 s** | 감소 |

동일 batch 재평가에서도 유사한 경향을 확인하였다.

---

### 6.10 실험 결과 해석

본 Gazebo 실험에서 Async 구조의 가장 명확한 효과는 **성공률 향상보다 반응성 향상**으로 나타났다.

Gemma Sync와 Async 모두 20/20 성공했기 때문에 목표 도착 성공률만으로는 두 방법의 차이를 설명하기 어렵다.

반면 trajectory update rate와 observation age에서는 뚜렷한 차이가 나타났다.

```text id="wi7mp5"
Sync
Slow VLA 속도
    ↓
낮은 trajectory update rate
    ↓
오래된 observation 사용

Async
Slow VLA / Fast Edge 분리
    ↓
높은 trajectory update rate
    ↓
최신 observation 반영
```

따라서 본 실험에서 확인한 Async 구조의 핵심 효과는 다음 두 가지이다.

1. **새로운 trajectory를 더 높은 빈도로 controller에 제공한다.**
2. **trajectory 생성에 더 최신의 visual observation을 반영한다.**

---

### 6.11 실험의 한계

본 실험에는 다음과 같은 한계가 있다.

- 단순한 정적 chair target 환경을 사용하였다.
- 실제 AgileX LIMO Pro의 완전한 digital twin 환경이 아니다.
- 움직이는 장애물이나 복잡한 실내 구조를 중심으로 평가한 benchmark가 아니다.
- Gemma Sync / Async 모두 20/20 성공하여 성공률 개선 여부를 판단하기 어렵다.
- 실제 로봇에서 동일한 Sync / Async 반복 비교는 아직 수행하지 않았다.
- 원 AsyncVLA 논문의 공식 benchmark 결과와 직접 비교할 수 있는 실험은 아니다.

따라서 현재 결과는 **Async 구조가 실제 trajectory update rate와 observation freshness를 개선한다는 제한된 실험적 근거**로 해석한다.

---

## 7. 실제 이미지 기반 평가 및 일반화 성능

Gazebo의 정적 chair 환경에서 Sync / Async 비교를 수행한 이후, 단순한 시뮬레이션 환경 밖에서도 모델이 유사한 행동 표현을 생성할 수 있는지 확인하기 위해 실제 이미지 기반 평가를 수행하였다.

본 평가는 크게 두 가지로 구성하였다.

- LeLaN 고정 이미지 기반 object endpoint 평가
- 동적 장애물이 포함된 별도 진단 평가

---

### 7.1 평가 목적

Gazebo 환경은 target과 배경이 비교적 단순하고, 조명과 카메라 조건도 일정하다.

따라서 시뮬레이션에서 높은 성공률을 보이더라도 실제 이미지에서는 다음과 같은 차이가 발생할 수 있다.

- 배경 복잡도 증가
- 조명 변화
- 물체 appearance 변화
- camera viewpoint 차이
- 실제 texture와 simulation texture 차이
- 자연어 명령과 visual target의 ambiguity

이를 확인하기 위해 공개 LeLaN 이미지와 별도 동적 장애물 환경에서 모델의 일반화 성능을 평가하였다.

---

### 7.2 LeLaN 고정 이미지 평가

LeLaN 데이터에서 고정한 **100개의 image pair**와 총 **200개의 language command**를 사용하여 object endpoint error를 평가하였다.

평가 대상은 다음 두 모델이다.

- Gemma 기반 student model
- Official model

결과는 다음과 같다.

| Model | Object Endpoint Error |
|---|---:|
| Official Model | **0.530 m** |
| Gemma | 0.759 m |

Gemma 기반 모델은 실제 이미지에서 일정 수준의 language-conditioned trajectory를 생성했지만, 공식 모델보다 endpoint error가 크게 나타났다.

두 모델의 차이는 다음과 같다.

```text
Gemma - Official
=
0.759 m - 0.530 m
=
0.229 m
```

즉, 본 평가에서는 Gemma 기반 student가 공식 모델보다 평균 약 **0.229 m 큰 endpoint error**를 보였다.

---

### 7.3 결과 해석

정적 Gazebo 환경에서는 Gemma Sync와 Async 모두 20/20 set을 성공했지만, 실제 이미지 평가에서는 공식 모델과의 성능 차이가 나타났다.

이를 통해 다음과 같은 차이를 확인하였다.

```text
Gazebo
→ 단순한 정적 환경
→ Gemma도 높은 목표 접근 성공률

Real Image
→ 실제 appearance와 background
→ Official model 대비 오차 증가
```

따라서 Gazebo의 높은 성공률만으로 실제 환경 generalization 성능을 판단하기 어렵다.

본 결과는 현재 Gemma 기반 student representation이 공식 model의 action representation을 일정 부분 학습했지만, 실제 이미지 domain에서는 추가적인 개선이 필요함을 의미한다.

---

### 7.4 동적 장애물 평가

정적 target 환경 외에도 이동 장애물이 존재하는 조건에서 모델의 행동 변화를 확인하기 위한 별도 진단 평가를 수행하였다.

Gemma 기반 모델의 결과는 다음과 같다.

| Runtime | Goal Arrival |
|---|---:|
| Gemma Sync | 16 / 40 |
| Gemma Async | 11 / 40 |

해당 결과에서는 Async가 Sync보다 높은 성공률을 보이지 않았다.

따라서 본 실험만으로는

```text
Async = Dynamic Obstacle Avoidance 성능 향상
```

이라고 해석할 수 없다.

오히려 현재 모델과 실험 조건에서는 **최신 RGB를 빠르게 반영하는 것만으로 복잡한 동적 장애물 회피 문제가 해결되지 않는다**는 점을 확인하였다.

---

### 7.5 Async 구조와 Obstacle Avoidance의 차이

Async 구조의 목적은 기본적으로 Slow VLA와 Fast Edge를 분리하여

- 최신 visual observation을 더 빠르게 반영하고
- trajectory를 더 높은 빈도로 갱신하는 것

이다.

이는 obstacle avoidance planner 자체와는 다른 문제이다.

```text
AsyncVLA
→ Fresh Observation
→ Faster Trajectory Update

Obstacle Avoidance
→ Obstacle Detection
→ Collision Prediction
→ Alternative Path Generation
```

현재 시스템의 LiDAR safety layer 역시 장애물을 발견했을 때 새로운 우회 경로를 생성하는 구조가 아니라 **정지시키는 safety guard**이다.

따라서 현재 단계에서는 Async 구조의 반응성 개선과 obstacle avoidance 능력을 분리해서 해석한다.

---

### 7.6 Simulation-to-Real Gap

현재까지의 결과를 종합하면 simulation과 실제 이미지 환경 사이에 성능 차이가 존재한다.

#### Simulation

- Gemma Sync: 20 / 20
- Gemma Async: 20 / 20
- Async에서 trajectory update rate 증가
- Async에서 observation age 감소

#### Real Image

- Gemma endpoint error: 0.759 m
- Official model endpoint error: 0.530 m
- Official model 대비 정확도 격차 존재

#### Dynamic Environment

- Gemma Sync: 16 / 40
- Gemma Async: 11 / 40
- Async 성공률 개선 미확인

이를 통해 현재 가장 큰 과제 중 하나는 **비동기 실행 구조 자체보다 model representation의 실제 환경 generalization 성능을 개선하는 것**임을 확인하였다.

---

### 7.7 현재 확인된 한계

현재 실제 이미지 및 동적 환경 평가에서는 다음과 같은 한계가 존재한다.

- 실제 이미지에서 공식 모델보다 큰 endpoint error
- 실제 camera domain에 대한 충분한 adaptation 부족
- moving obstacle에 대한 명시적인 prediction 구조 부재
- Async의 최신 observation 사용이 항상 더 나은 행동으로 이어지지는 않음
- 학습 데이터의 scene distribution과 실제 환경의 차이
- 일부 historical dataset preprocessing 조건이 현재 평가와 완전히 동일하지 않음

특히 과거 LeLaN 전처리 과정과 현재 평가 조건이 완전히 동일하지 않은 구간이 존재하므로, 서로 다른 시점의 evaluation result를 단순 합산하여 비교하지 않는다.

---

### 7.8 평가 결과 요약

실제 이미지 기반 평가에서 확인한 핵심 결과는 다음과 같다.

| Evaluation | Gemma | Official | 해석 |
|---|---:|---:|---|
| LeLaN Object Endpoint Error | 0.759 m | **0.530 m** | Gemma의 실제 이미지 오차가 더 큼 |
| Dynamic Obstacle Goal Arrival | Sync 16/40 | - | 제한적인 성공 |
| Dynamic Obstacle Goal Arrival | Async 11/40 | - | Async 개선 미확인 |

현재 결과는 Gemma 기반 Async pipeline의 구조적 동작 가능성을 확인한 단계이며, **실제 환경에서 공식 모델 수준의 정확도나 robust navigation 성능을 확보한 단계는 아니다.**

---

## 8. ROS 2 및 LIMO Pro 실물 로봇 통합

Simulation에서 검증한 Gemma 기반 AsyncVLA pipeline을 실제 모바일 로봇에 적용하기 위해 **AgileX LIMO Pro + ROS 2 기반 실물 시스템**을 구성하였다.

실물 환경에서는 VLA inference와 실제 motor command를 분리하였다.

전체 구조는 다음과 같다.

```text
LIMO RGB Camera
      ↓
ROS 2 Image Topic
      ↓
Desktop GPU
      ↓
Gemma Slow VLA
      ↓
Fast Edge Adapter
      ↓
Trajectory
      ↓
PD Controller
      ↓
/asyncvla/planned_command
      ↓
Robot-local Safety Guard
      ↓
/asyncvla/guarded_cmd_vel
      ↓
LIMO Base Driver
      ↓
AgileX LIMO Pro
```

---

### 8.1 시스템 배치 구조

Gemma 기반 VLA inference는 LIMO Pro 내부 컴퓨팅 보드가 아니라 **외부 데스크톱 GPU**에서 수행한다.

구성은 다음과 같이 분리하였다.

| Device | 역할 |
|---|---|
| Desktop PC | Gemma inference, Action Adapter, Edge Adapter, trajectory 생성, PD control |
| LIMO Pro | RGB / LiDAR / Odometry 제공, safety guard, 실제 motor command 실행 |

즉, 연산량이 큰 AI inference와 실제 물리 제어를 서로 다른 시스템에서 수행한다.

```text
Desktop
┌──────────────────────┐
│ Gemma                │
│ Action Adapter       │
│ Edge Adapter         │
│ PD Controller        │
└──────────┬───────────┘
           │ ROS 2
           ▼
LIMO Pro
┌──────────────────────┐
│ Safety Guard         │
│ Base Driver          │
│ Motor Control        │
└──────────────────────┘
```

---

### 8.2 RGB Camera Input

LIMO Pro의 RGB camera는 다음 ROS 2 topic을 통해 수신한다.

```text
/camera/color/image_raw
```

message type은 다음과 같다.

```text
sensor_msgs/Image
```

desktop의 policy node는 해당 topic을 subscribe하여 최신 RGB frame과 원본 message timestamp를 저장한다.

실제 연결 시험에서는 다음 조건으로 RGB stream을 확인하였다.

```text
Resolution          : 640 × 480
Captured Frames     : 60
Source Timestamp FPS: 12.542 FPS
Maximum Receive Gap : 0.352 s
```

이 RGB stream은 Slow VLA와 Fast Edge 모두에서 사용된다.

---

### 8.3 Desktop Policy Node

desktop에서는 `coverage30_live_proposal_only` 형태의 policy node가 동작한다.

주요 역할은 다음과 같다.

```text
ROS Image Receive
      ↓
Latest RGB 저장
      ↓
Slow Thread
      ↓
Gemma + Action Adapter
      ↓
Slow Feature 생성
      ↓
Fast Loop
      ↓
Latest RGB + Feature
      ↓
Edge Adapter
      ↓
Trajectory
      ↓
PD Controller
```

중요한 점은 desktop policy node가 **실제 `/cmd_vel`을 직접 publish하지 않는다는 것**이다.

모델이 생성한 결과는 먼저 planned command 형태로 전달한다.

---

### 8.4 Planned Command

desktop에서 계산된 trajectory와 control 결과는 다음 topic으로 전달한다.

```text
/asyncvla/planned_command
```

message는 JSON 형태의 `std_msgs/String`을 사용한다.

planned command는 단순 linear/angular velocity뿐 아니라 실행 상태를 확인하기 위한 session / sequence / timestamp 정보를 함께 전달한다.

개념적으로는 다음과 같다.

```text
{
    session,
    sequence,
    timestamp,
    linear_velocity,
    angular_velocity,
    ...
}
```

이 단계의 command는 아직 실제 motor로 전달되는 최종 명령이 아니다.

---

### 8.5 Robot-local Safety Guard

LIMO Pro 측에서는 별도의 `asyncvla_robot_local_guard` node를 실행한다.

Guard는 planned command뿐 아니라 실제 로봇의 여러 sensor 상태를 함께 확인한다.

주요 입력은 다음과 같다.

```text
Planned Command
RGB
LiDAR Scan
Odometry
Chassis Status
STOP Condition
```

이를 이용해 command가 현재 로봇 상태에서 실행 가능한지 판단한다.

---

### 8.6 Freshness 검사

실제 로봇에서는 오래된 command나 sensor data가 사용되지 않도록 freshness 조건을 설정하였다.

현재 주요 조건은 다음과 같다.

| Data | Maximum Age |
|---|---:|
| Planned Command | 0.30 s |
| RGB | 0.50 s |
| LiDAR Scan | 0.30 s |
| Odometry | 0.20 s |

예를 들어 command가 0.30초 이상 오래된 경우 해당 명령은 실행하지 않는다.

```text
Current Time
    -
Command Receive Time
    >
0.30 s

→ Reject
```

이를 통해 desktop inference 또는 network 상태에 문제가 발생했을 때 오래된 속도 명령이 계속 실행되는 것을 방지한다.

---

### 8.7 LiDAR Safety Gate

LiDAR는 로봇 전방 및 주변 장애물과의 거리를 확인하는 데 사용한다.

현재 설정된 주요 거리 조건은 다음과 같다.

```text
Front Distance Threshold : 0.45 m
Global Minimum Distance  : 0.32 m
```

전방 obstacle이 설정된 거리보다 가까워지면 로봇은 이동 명령을 중단한다.

```text
Obstacle Distance
      ↓
Threshold Check
      ↓
Too Close
      ↓
STOP
```

현재 구조는 obstacle을 감지한 뒤 새로운 우회 경로를 생성하는 planner가 아니라 **collision 방지를 위한 stop-only safety layer**이다.

---

### 8.8 Session / Sequence 검사

네트워크를 통해 전달되는 command가 이전 실행에서 생성된 오래된 명령인지 확인하기 위해 session과 sequence 정보를 함께 관리한다.

```text
Session ID
Sequence Number
Timestamp
```

Guard는 현재 실행 session과 맞지 않거나 순서가 잘못된 command를 거부한다.

이를 통해 이전 실행에서 남아 있던 command가 새로운 실험에서 다시 실행되는 것을 방지한다.

---

### 8.9 Shadow Mode와 Execute Mode

Safety Guard는 기본적으로 **Shadow Mode**로 동작한다.

Shadow Mode에서는

```text
AI Command
   ↓
Guard Evaluation
   ↓
Log / Validation
```

까지만 수행하고 실제 velocity publisher를 생성하지 않는다.

실제 로봇을 움직이는 경우에만 별도의 승인된 **Execute Mode**를 사용한다.

```text
Shadow Mode
→ 분석 및 검증
→ 실제 motor command 없음

Execute Mode
→ 승인된 조건
→ guarded velocity publish
```

이를 통해 model inference와 physical actuation을 단계적으로 분리하여 검증하였다.

---

### 8.10 Guarded Velocity

모든 safety condition을 통과한 경우 최종 command는 다음 topic으로 publish한다.

```text
/asyncvla/guarded_cmd_vel
```

message type은 다음과 같다.

```text
geometry_msgs/Twist
```

이 topic이 실제 base driver로 전달되는 최종 velocity command이다.

---

### 8.11 Base Driver Remap

기존 LIMO base driver의 `/cmd_vel` subscriber를 safety guard가 출력하는 topic으로 remap하였다.

```text
기존

/cmd_vel
    ↓
Base Driver


변경

/asyncvla/guarded_cmd_vel
    ↓
Base Driver
```

따라서 desktop model이나 다른 ROS node가 일반 `/cmd_vel`을 publish하더라도 해당 명령이 바로 robot motor에 전달되지 않도록 구성하였다.

---

### 8.12 전체 Safety 구조

최종적인 실물 제어 구조는 다음과 같다.

```text
AI Model
   ↓
Trajectory
   ↓
PD Controller
   ↓
Planned Command
   ↓
┌────────────────────┐
│   Safety Guard     │
│                    │
│ Command Freshness │
│ RGB Freshness     │
│ LiDAR Freshness   │
│ Odom Freshness    │
│ Chassis State     │
│ Session / Seq     │
│ Obstacle Distance │
│ STOP Condition    │
└─────────┬──────────┘
          ↓
Guarded cmd_vel
          ↓
Base Driver
          ↓
LIMO Pro
```

이 구조를 통해 **AI model의 출력과 실제 물리 구동 사이에 독립적인 safety layer**를 배치하였다.

---

### 8.13 실물 적용 단계

실물 시스템은 한 번에 전체 자율주행을 시도하지 않고 다음 순서로 단계적으로 검증하였다.

```text
1. RGB Camera 연결
        ↓
2. ROS 2 Topic 확인
        ↓
3. 정지 상태 Gemma 추론
        ↓
4. Planned Command 생성
        ↓
5. Shadow Guard 검증
        ↓
6. 저속 Command Sign 확인
        ↓
7. 실제 LIMO Motion
        ↓
8. Learned Policy 접근 실험
```

이를 통해 각 단계에서 발생하는 문제를 모델 문제, ROS 2 문제, controller 문제, safety 문제로 분리하여 확인할 수 있도록 구성하였다.

---

### 8.14 현재 실물 시스템 상태

현재까지 다음 항목을 실제 LIMO Pro에서 확인하였다.

- RGB camera stream 수신
- ROS 2 image subscription
- Gemma 기반 language-conditioned inference
- Slow / Fast pipeline 실행
- Planned command publish
- Robot-local guard 동작
- Guarded velocity publish
- Base driver remap
- 저속 실물 구동
- LiDAR threshold 기반 safety stop

따라서 현재 시스템은

```text
Model
  ↓
ROS 2
  ↓
Safety Guard
  ↓
Physical Robot Motion
```

까지 연결된 상태이다.

다만 **자연어 명령에 따른 안정적인 실제 목표 도착은 아직 완료되지 않았다.**

---

## 10. 결과 요약 및 한계

본 프로젝트에서는 Gemma 기반 VLA를 AsyncVLA의 Slow / Fast 구조에 연결하고, simulation과 실제 AgileX LIMO Pro 환경에서 비동기 inference pipeline을 검증하였다.

현재까지 확인한 핵심 결과는 다음과 같다.

---

### 10.1 핵심 결과 요약

| 항목 | 결과 |
|---|---:|
| Gemma Sync 성공 세트 | 20 / 20 |
| Gemma Async 성공 세트 | 20 / 20 |
| Sync Trajectory Update Rate | 약 0.8 Hz |
| Async Trajectory Update Rate | 약 7.9 Hz |
| Sync Observation Age | 약 1.75 s |
| Async Observation Age | 약 0.20 s |
| Official Real-image Endpoint Error | 0.530 m |
| Gemma Real-image Endpoint Error | 0.759 m |
| Real Robot Motion | 약 0.312 m |
| Real Robot Stop Distance | 0.448 m |
| Safety Threshold | 0.450 m |
| Slow VLA Latency | 약 1.405 s |
| Edge Adapter Latency | 약 0.108 s |

---

### 10.2 Async 구조에서 확인한 효과

Gazebo Sync / Async 비교에서 가장 명확한 차이는 **성공률이 아니라 trajectory update rate와 observation freshness**에서 나타났다.

```text
Trajectory Update Rate

Sync  : ~0.8 Hz
Async : ~7.9 Hz
```

Async 구조에서는 Slow VLA가 새로운 semantic/action feature를 생성하는 동안에도 Fast Edge Adapter가 최신 RGB를 이용해 trajectory를 갱신할 수 있었다.

그 결과 실제 controller에 적용되는 새로운 trajectory의 빈도가 약 10배 수준으로 증가하였다.

Observation age 역시 다음과 같이 감소하였다.

```text
Observation Age

Sync  : ~1.75 s
Async : ~0.20 s
```

즉, Async 구조는 Slow VLA inference 자체를 빠르게 만든 것이 아니라 **느린 semantic inference와 빠른 visual correction을 분리하여 더 최신의 visual observation을 trajectory 생성에 반영한 구조**이다.

---

### 10.3 성공률 결과 해석

Gemma 기반 모델은 정적 Gazebo chair 환경에서 다음 결과를 보였다.

```text
Gemma Sync  : 20 / 20
Gemma Async : 20 / 20
```

두 방식 모두 동일하게 20/20 성공했기 때문에 본 실험만으로는 Async 구조가 목표 도착 성공률을 향상시켰다고 판단할 수 없다.

따라서 현재 확인한 효과는 다음 두 가지로 제한한다.

- trajectory update rate 증가
- observation freshness 개선

---

### 10.4 실제 이미지 성능

Gazebo의 정적 환경에서는 높은 성공률을 보였지만, 실제 이미지 기반 LeLaN 평가에서는 공식 모델과 성능 차이가 나타났다.

```text
Official Model : 0.530 m
Gemma          : 0.759 m
```

Gemma 기반 student의 object endpoint error가 약 **0.229 m 더 크게 나타났다.**

이를 통해 현재 모델이 simulation 환경에서는 안정적인 trajectory를 생성할 수 있지만, 실제 이미지 domain에서는 공식 모델 수준의 generalization 성능에 도달하지 못했음을 확인하였다.

---

### 10.5 동적 환경에서의 한계

이동 장애물이 포함된 별도 평가에서는 다음 결과가 나타났다.

| Runtime | Goal Arrival |
|---|---:|
| Gemma Sync | 16 / 40 |
| Gemma Async | 11 / 40 |

이 평가에서는 Async가 Sync보다 높은 성공률을 보이지 않았다.

따라서

```text
Higher Update Rate
≠
Better Dynamic Obstacle Avoidance
```

로 해석한다.

현재 Async 구조는 최신 visual observation을 더 빠르게 반영할 수 있지만, moving obstacle의 future motion을 예측하거나 새로운 우회 경로를 생성하는 planner 자체를 포함하지 않는다.

---

### 10.6 실물 로봇 적용 결과

실제 AgileX LIMO Pro에서는 다음 pipeline을 연결하였다.

```text
RGB Camera
    ↓
Gemma Slow VLA
    ↓
Fast Edge Adapter
    ↓
Trajectory
    ↓
PD Controller
    ↓
ROS 2
    ↓
Safety Guard
    ↓
Physical Robot Motion
```

검정 의자 접근 실험에서는 약 **0.312 m**를 실제 주행하였다.

이후 전방 LiDAR가 **0.448 m**를 측정하면서 설정한 safety threshold인 **0.450 m**에 도달하였고, Robot-local Safety Guard가 로봇을 정지시켰다.

따라서 현재까지는

**Model-to-Robot pipeline의 실제 동작**

까지 확인하였다.

다만 해당 정지는 목표 도착이 아니라 **Safety Guard에 의한 정지**였으므로 자연어 기반 목표 도착 성공으로 평가하지 않는다.

---

### 10.7 현재 확인한 주요 문제

#### 1. Slow Semantic Latency

Slow VLA inference는 실제 live log 기준 약 **1.405초**의 latency를 가진다.

Fast Edge Adapter가 최신 RGB를 사용할 수 있어도 semantic/action feature 자체는 과거 observation을 기반으로 한다.

따라서 visual freshness는 개선되지만 semantic freshness 문제는 남아 있다.

---

#### 2. Real-image Generalization

실제 이미지 평가에서 Gemma 기반 모델은 공식 모델보다 큰 endpoint error를 보였다.

Simulation에서의 성공 결과가 실제 환경 성능으로 그대로 이어지지 않았다.

---

#### 3. Initial Steering Bias

실제 검정 의자 접근 실험에서 첫 waypoint가 약 **18.64° 우측**을 가리키는 현상이 나타났다.

현재 다음 원인을 후보로 분석하고 있다.

- visual representation
- training distribution
- camera calibration
- trajectory scale
- 실제 환경과 학습 환경의 차이

단일 원인은 아직 확정하지 않았다.

---

#### 4. Controller Saturation

기존 PD controller에서 linear / angular velocity를 독립적으로 clipping하면서 원래 trajectory curvature가 변할 수 있는 문제를 확인하였다.

대안 clipping 방식도 offline에서 시험했지만 tracking error가 증가하여 현재 실제 로봇에는 적용하지 않았다.

---

#### 5. Stop-only Safety

현재 LiDAR Safety Guard는 장애물을 감지하면 로봇을 정지시키는 구조이다.

```text
Obstacle
   ↓
STOP
```

따라서 장애물을 피해서 새로운 경로를 생성하는 obstacle avoidance planner는 아직 구현하지 않았다.

---

### 10.8 현재까지 완료한 범위

현재까지 완료한 항목은 다음과 같다.

- Gemma multimodal representation 연동
- 32-query Action Adapter 구현
- AsyncVLA-compatible action representation 구성
- Teacher feature distillation
- Gemma LoRA fine-tuning
- Slow / Fast inference 분리
- Timestamp 기반 feature packet
- Gazebo Sync / Async 비교
- Trajectory update rate 측정
- Observation age 측정
- ROS 2 camera integration
- Planned command 구조
- Robot-local Safety Guard
- LIMO Pro 저속 실물 구동
- Model → ROS 2 → Robot pipeline 연결

---

### 10.9 아직 완료하지 않은 범위

다음 항목은 현재 진행 중이다.

- 실제 자연어 목표 도착
- 실물 환경의 반복적인 Sync / Async 비교
- 실제 camera extrinsic 검증
- trajectory scale 검증
- 안정적인 closed-loop object approach
- obstacle avoidance
- dynamic environment 성능 개선
- semantic feature latency 감소
- 독립 환경에서의 전체 재현성 검증

---

### 10.10 현재 연구 상태

현재 프로젝트는 다음 단계까지 진행하였다.

```text
Model Architecture
      ↓
Teacher Distillation
      ↓
Gemma Adaptation
      ↓
Async Slow / Fast Runtime
      ↓
Gazebo Evaluation
      ↓
ROS 2 Integration
      ↓
Safety Layer
      ↓
Real Robot Motion
      ↓
[ Current Stage ]
```

최종적인 **실물 자연어 기반 목표 도착과 반복적인 Sync / Async 실험은 진행 중이다.**

본 프로젝트는 현재까지 완성된 연구 결과만을 주장하기보다, simulation과 real-world deployment 사이에서 발생하는 문제를 분석하고 개선하는 과정 자체를 포함하는 연구 프로젝트로 진행하고 있다.

---

## 11. 향후 진행 계획

현재까지 Gemma 기반 AsyncVLA 구조를 ROS 2 및 AgileX LIMO Pro에 연결하고 실제 저속 구동까지 검증하였다.

이후에는 단순한 pipeline 연결을 넘어 **실제 환경에서의 반복 가능한 정량 평가**를 진행할 예정이다.

---

### 11.1 실물 환경 목표 도착 검증

현재 실물 로봇에서는 모델 기반 trajectory가 실제 LIMO Pro motion까지 연결되는 것을 확인하였지만, 자연어 명령에 따른 안정적인 목표 도착은 아직 완료하지 않았다.

다음 단계에서는 보다 넓은 복도 환경에서

```text
Natural Language Command
        ↓
Target Recognition
        ↓
Trajectory Generation
        ↓
Closed-loop Navigation
        ↓
Goal Arrival
```

전체 과정을 반복 검증할 예정이다.

목표물 접근 성공과 LiDAR Safety Guard에 의한 정지는 서로 다른 결과로 구분하여 평가한다.

---

### 11.2 실물 Sync / Async 비교

Gazebo에서는 Sync와 Async 구조의 trajectory update rate와 observation age 차이를 확인하였다.

다음 단계에서는 동일한 비교를 실제 LIMO Pro에서도 수행한다.

주요 평가 항목은 다음과 같다.

- Goal success rate
- Trajectory update rate
- Observation age
- Slow VLA latency
- Edge Adapter latency
- Goal arrival time
- Safety stop frequency
- Command drop / timeout

동일한 환경, 동일한 명령, 동일한 모델과 controller를 사용하여 Sync / Async 실행 방식만 변경하는 방식으로 비교한다.

---

### 11.3 Timestamp 및 Runtime Logging 개선

현재 live runtime에서는 source timestamp와 command freshness를 사용하고 있지만, 전체 pipeline의 각 이벤트가 실제로 어느 시점에 발생했는지 더 세밀하게 기록할 필요가 있다.

향후 다음 timestamp를 분리하여 기록한다.

```text
Image Source Time
        ↓
Image Receive Time
        ↓
Slow Inference Start
        ↓
Slow Feature Ready
        ↓
Edge Inference
        ↓
Trajectory Ready
        ↓
Command Publish
        ↓
Command Receive
        ↓
Command Applied
```

이를 통해 실제 시스템에서 발생하는

- model inference latency
- network latency
- queue / scheduling latency
- control latency

를 구분하여 분석할 예정이다.

---

### 11.4 Camera Extrinsic 및 Trajectory Scale 검증

Simulation과 실제 로봇 환경 사이의 차이를 줄이기 위해 camera coordinate와 robot coordinate 사이의 관계를 추가로 검증한다.

특히 다음 항목을 확인할 예정이다.

- Camera extrinsic calibration
- Image-space와 robot-space 방향 관계
- Predicted trajectory scale
- 실제 odometry displacement
- Steering direction consistency

이를 통해 실제 환경에서 관찰된 initial steering bias의 원인을 분석한다.

---

### 11.5 Controller 개선

현재 시스템에서는 AsyncVLA 계열의 기존 PD controller를 사용하고 있다.

실물 테스트에서는 linear / angular velocity의 독립적인 saturation이 trajectory curvature를 변화시킬 수 있는 문제를 확인하였다.

향후에는

- curvature-preserving velocity scaling
- trajectory tracking controller
- velocity smoothing
- acceleration limit
- robot dynamics 반영

등을 검토할 예정이다.

단, controller 개선이 model 자체의 trajectory prediction 성능과 혼동되지 않도록 별도의 실험으로 평가한다.

---

### 11.6 Dynamic Environment 평가

현재 Async 구조는 trajectory update rate와 observation freshness를 개선하지만, 동적 장애물 환경에서 성공률 개선은 확인하지 못하였다.

향후에는 단순한 최신 RGB 반영을 넘어

```text
Obstacle Detection
        ↓
Temporal Motion Estimation
        ↓
Collision Risk Estimation
        ↓
Trajectory Adaptation
```

과 같은 동적 환경 대응 방식을 검토할 예정이다.

현재 LiDAR Safety Guard는 obstacle avoidance planner가 아니라 collision 방지를 위한 stop-only safety mechanism으로 유지한다.

---

### 11.7 Semantic Latency 개선

현재 실제 runtime에서 Slow VLA는 약 1초 이상의 inference latency를 가진다.

Fast Edge Adapter가 최신 RGB를 반영하더라도 semantic/action feature 자체는 과거 observation에 기반한다.

향후 다음 방법을 검토한다.

- Slow VLA inference 최적화
- feature update frequency 증가
- smaller / quantized multimodal model
- semantic feature reuse 전략
- stale feature 분석
- temporal feature update

목표는 visual freshness뿐 아니라 **semantic freshness 역시 개선하는 것**이다.

---

### 11.8 재현 가능한 공개 환경 구성

현재 연구 workspace에는 학습 결과, checkpoint, 실험 로그 및 내부 환경 정보가 함께 존재한다.

공개 repository에서는 필요한 코드만 선별하여 다음 형태로 정리할 예정이다.

```text
asyncvla-gemma-limo/
├── training/
├── scripts/
├── configs/
├── tests/
├── docs/
├── assets/
└── README.md
```

공개 전 다음 항목을 제거하거나 별도로 관리한다.

- API Key / Token
- 내부 IP 및 계정 정보
- 개인 환경의 절대경로
- 대용량 checkpoint
- rosbag
- 비공개 연구 데이터
- 재배포가 제한된 제3자 코드

최종적으로 새로운 환경에서도 동일한 simulation 및 ROS 2 pipeline을 실행할 수 있도록 재현 절차를 정리한다.

---

### 11.9 현재 진행 상태

현재 프로젝트의 진행 상태는 다음과 같다.

```text
Gemma 기반 VLA 구성                 ✅
Teacher Feature Distillation        ✅
Async Slow / Fast 구조              ✅
Gazebo Sync / Async 비교            ✅
ROS 2 Integration                   ✅
Robot-local Safety Guard            ✅
LIMO Pro 실제 저속 구동             ✅

실물 자연어 목표 도착               진행 중
실물 Sync / Async 정량 비교         진행 예정
Controller 개선                     진행 예정
Dynamic Obstacle 대응               진행 예정
Independent Reproduction            진행 예정
```

본 프로젝트는 현재 **진행 중인 연구 프로젝트**이다.

---

## 12. 소스코드 및 참고자료

### 주요 구현 파일

현재 프로젝트의 핵심 구현은 다음 파일을 중심으로 구성되어 있다.

| File | Description |
|---|---|
| `training/student.py` | Gemma 기반 student model 및 전체 학습/추론 연결 |
| `training/action_adapter.py` | 32-query Action Adapter |
| `training/official.py` | AsyncVLA 계열 trajectory representation, loss, SE(2) 관련 기능 |
| `training/split_candidate.py` | Slow feature / Fast Edge inference interface |
| `training/joint_public_checkpoint.py` | 선택 모델 checkpoint loading |
| `scripts/train_native_coverage30.py` | Coverage30 Gemma LoRA 학습 |
| `scripts/limo_live_policy.py` | LIMO RGB 기반 live VLA / trajectory 생성 |
| `scripts/limo_live_guard.py` | Robot-local Safety Guard |
| `scripts/limo_control_shadow.py` | PD controller 및 safety 제한 검증 |
| `scripts/run_selected_runtime_fresh20_gemma.py` | Gazebo Sync / Async 평가 |

---

### Repository Structure

```text
asyncvla-gemma-limo/
│
├── training/
│   ├── student.py
│   ├── action_adapter.py
│   ├── official.py
│   ├── split_candidate.py
│   └── joint_public_checkpoint.py
│
├── scripts/
│   ├── train_native_coverage30.py
│   ├── limo_live_policy.py
│   ├── limo_live_guard.py
│   ├── limo_control_shadow.py
│   └── run_selected_runtime_fresh20_gemma.py
│
├── configs/
├── tests/
├── docs/
├── assets/
└── README.md
```

---

### References

본 프로젝트는 AsyncVLA / OmniVLA 계열 연구의 아이디어와 일부 공개 component를 기반으로 한다.

원 논문, 공식 구현, pretrained model 및 공개 dataset의 저작권은 각 원 저자에게 있다.

본 repository는 해당 연구를 그대로 재현하는 것이 아니라 다음 환경에 맞춰 구조를 분석하고 확장한 연구 프로젝트이다.

- Gemma 기반 multimodal model
- RTX 4060 Ti 8GB 환경
- ROS 2
- AgileX LIMO Pro
- 실제 모바일 로봇 deployment

공식 논문 및 repository 링크는 공개 코드 정리가 완료된 이후 본 항목에 추가한다.

---

## Current Status

> **Research Prototype — Work in Progress**

현재 **Gemma 기반 AsyncVLA pipeline → ROS 2 → Safety Guard → AgileX LIMO Pro 실제 구동**까지 검증하였다.

실제 환경에서의 자연어 기반 목표 도착, 반복적인 Sync / Async 비교 및 실제 환경 generalization 개선은 계속 진행 중이다.

---
