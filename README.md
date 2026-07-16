# lerobot-tools — SO-101 / Jetson Thor pipeline
HuggingFace lerobot 기반 SO-101 수집·학습·추론 파이프라인 웹툴 (본 repo는 HF lerobot 자체가 아님).

- lrweb.py : 통합 웹툴 (Datasets/Collect/Training/Rollout/Control, port 8080)
- activate.sh : conda env + HF_HOME
- urdf/ : SO-101 URDF/meshes — TheRobotStudio SO-ARM100 (Apache 2.0) 기반

Env: lerobot commit <해시>, torch 2.9 cu130 (Thor: jetson-ai-lab sbsa / x86: pytorch.org),
torchcodec 제거 + av>=15,<16
