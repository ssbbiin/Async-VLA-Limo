# AsyncVLA-Gemma-LIMO

> **Gemma 기반 비동기 Vision-Language-Action 파이프라인과 ROS 2 기반 AgileX LIMO Pro 실물 로봇 통합**

## 프로젝트 개요

본 프로젝트는 Vision-Language-Action(VLA) 모델의 높은 추론 지연으로 인해 실제 로봇 제어에서 발생하는 **관측 노후화(Observation Staleness)**와 **낮은 행동 갱신 주기** 문제를 다룹니다.

AsyncVLA의 **Slow VLA / Fast Edge Adapter** 구조를 기반으로, Gemma 계열 멀티모달 모델을 모바일 로봇 행동 생성 파이프라인에 연결했습니다.

Slow VLA는 RGB 이미지와 자연어 명령을 바탕으로 semantic/action feature를 생성하고, Fast Edge Adapter는 최신 RGB observation을 이용해 trajectory를 빠르게 갱신합니다.

최종적으로 해당 파이프라인을 ROS 2 기반의 **AgileX LIMO Pro**와 연결하여 시뮬레이션뿐 아니라 실제 로봇 환경에서도 모델 추론부터 안전 제어 및 구동까지 검증하는 것을 목표로 합니다.

현재까지 다음을 수행했습니다.

- Gemma 기반 multimodal representation과 Action Adapter 연동
- AsyncVLA Slow / Fast inference 구조 구현
- 공식 Edge Adapter 및 trajectory controller 연결
- Gazebo 환경에서 Sync / Async 비교 실험
- ROS 2 카메라 및 로봇 제어 파이프라인 구축
- Robot-local Safety Guard 구성
- AgileX LIMO Pro 실제 저속 구동 검증

> 현재 **Model → ROS 2 → Safety Guard → Real Robot Motion**까지 연결되었으며, 실제 환경에서 자연어 기반 목표 도착 및 반복적인 Sync / Async 비교 실험은 진행 중입니다.

---

## Tech Stack

| Category | Technologies |
|---|---|
| Language | Python |
| Deep Learning | PyTorch |
| Vision-Language Model | Gemma |
| Robotics | ROS 2 |
| Simulation | Gazebo |
| Robot Platform | AgileX LIMO Pro |
| Sensors | RGB Camera, LiDAR, Odometry |
| AI / Robotics | Vision-Language-Action, Knowledge Distillation, LoRA |
| Hardware | NVIDIA RTX 4060 Ti 8GB |
