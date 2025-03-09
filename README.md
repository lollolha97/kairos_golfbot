# golfbot
[![Kairos Golfbot](http://img.youtube.com/vi/Ou8qCmr9eWY/0.jpg)](https://www.youtube.com/watch?v=Ou8qCmr9eWY&list=PLX4MtaacavIiuLv5J_uSyMYdoqZit-67E&index=2)
# GolfBot - AI 기반 골프 캐디 로봇
https://www.youtube.com/watch?v=Ou8qCmr9eWY&list=PLX4MtaacavIiuLv5J_uSyMYdoqZit-67E&index=2

## 📌 프로젝트 개요
**GolfBot**은 AI 및 자율주행 기술을 활용하여 골프장에서 플레이어를 보조하는 스마트 캐디 로봇입니다. YOLO 기반의 객체 추적 및 LegTracker를 활용하여 플레이어를 자동으로 따라가며, 분석을 위한 가장 적합한 위치로 이동하여 사용자의 골프 스윙을 분석하여 피드백을 제공합니다.

## 🚀 주요 기능
### 🔹 **AI 기반 플레이어 Tracking**
- **카메라**: YOLO를 이용한 플레이어 객체 탐지 및 추적
- **LiDAR**: LegTracker 기반의 다리 추적을 통한 트랙킹

### 🔹 **스마트 정지 & 최적 촬영 위치 이동**
- **정면 인식 후 정지**: Haarcascade face frontal 모델 사용 (프레임 70% 이상 얼굴 감지 시 정지)
- **Mediapipe 마커 데이터 활용**: 카메라 각도 및 촬영 높이 자동 조절

### 🔹 **자세 분석 및 피드백 제공**
- 촬영 후 **자세 평가** 기능 제공
- ChatGPT API를 활용한 'Quantitative Golf Swing Analysis based on Kinematic Mining Approach' 논문 기반 피드백 제공 


## 🛠 기술 스택
| 기술 | 설명 |
|------|------|
| **YOLO** | 플레이어 객체 감지 및 추적 |
| **LegTracker** | LiDAR 기반 다리 추적 |
| **ROS2** | 로봇 운영 시스템 |
| **Mediapipe** | 자세 분석 및 카메라 제어 |

## 📌 실행 방법
```bash
# ROS2 환경 설정
source /opt/ros/humble/setup.bash

# 패키지 빌드 및 실행

## Build
```bash
git clone https://github.com/lollolha97/kairos_golfbot.git
cd kairos_golfbot
catkin_make
```

## launch
```bash
roslaunch myagv_controller golf_bot.launch
python ./streamlit/gui.py
```

## 📚 참고 자료
- YOLO v5 ROS 
- LegTracker
- GolfDB: A Video Database for Golf Swing Sequencing
- Quantitative Golf Swing Analysis based on Kinematic Mining Approach 

- yolov5 파일만 다운 후 압축해제해서 사용해주세요.
