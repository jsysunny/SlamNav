# Doosan Robotics Boot Camp(2025.01.06 ~ 2025.07.06)
## 4. ROKEY B-3조 지능-1 Project (SLAM(위치추정 및 공간지도생성) 기반 자율주행 로봇 시스템 구현) SlamNav

&nbsp;

## 🧠 로봇 자동화 주차 시스템

&nbsp;

## 📑 목차

1. [📌 프로젝트 개요](#1--프로젝트-개요)
2. [📘 프로젝트 수행 절차 및 방법](#2--프로젝트-수행-절차-및-방법)  
3. [🔧 구성 요소](#3--구성-요소)  
4. [💻 사용 기술](#4--사용-기술)  
5. [🧭 동작 흐름 요약](#5--동작-흐름-요약)  
6. [💻 코드 실행 방법](#6--코드-실행-방법)  
7. [📷 시연 영상/이미지](#7--시연-영상--이미지)  
8. [🌟 기대 효과/ 한계점 및 개선점](#8--기대-효과)  

   
&nbsp;
## 1. 📌 프로젝트 개요

전기차 보급 확대와 함께 주차장 이용 문제, 충전 문제, 안내 인력 문제 등이 대두되고 있으며,
본 프로젝트는 AI Vision과 자율 시스템 기반의 **스마트 주차 솔루션**을 통해 이러한 문제를 해결하고 차세대 EV 주차 환경을 조성하고자 합니다.

&nbsp;
### 🎯 기획 의도

- EV(전기차) 판매 증가로 인한 충전기 부족, 혼잡, 관리 이슈에 대응
- 주차장 위치 확인, 잔여 공간 파악, 충전 위치 안내 등에서의 불편 해소
- AI Vision 기반 실시간 주차 공간 인식, 자율주차, 비대면 결제 및 안내 시스템 도입
- 고령층, 초보 운전자, 외부 방문객 등의 주차 스트레스 최소화 및 효율적 공간 활용 목적
  
&nbsp;
### 🏭 기존 기술의 활용과 협동로봇의 확장 가능성
기존에는 주차장이 단순 차량 보관 공간에 불과했으나, AI 기반 기술을 접목함으로써 다양한 기능 확장이 이루어지고 있습니다.

####  전기차 수요 폭증
- IEA 발표에 따르면 2024년 전기차 판매량 1,700만대 돌파
- 2030년 전체 차량의 40% 이상이 전기차가 될 것으로 전망

#### 사용자 불편 사례
- 실시간 위치 기반 정보 제공 부족
- 충전구역에 일반 차량 주차로 인한 충전 불가 문제 발생

#### 안내/관리자 의존 문제
- 안내원에 의존한 수동 제어 시스템
- 폭염, 근무환경 열악화 등으로 인한 인력 운영 문제 발생

#### 벤치마킹: 입출차 자동화 시스템
- 카카오 T주차 등 스마트 결제 및 출입 제어 시스템 확산
- 차량번호 인식 → 자동 요금 계산 → 무정차 출차

#### 벤치마킹: 자율 주차 로봇
- HL로보틱스 등에서 자율 주차로봇 도입
- 리프트 방식, AGV 방식 등 다양하게 발전 중
  
#### 충전 위치 제어 필요성
- 두산로보틱스 등에서 자율 충전 로봇 도입
- 충전기의 정확한 위치 파악 및 자동 정렬 기술 필요
- 케이블이 차량 위치에 맞지 않는 불편 해소를 위한 자동 위치 보정 기술 필요

&nbsp;
## 2. 📘 프로젝트 수행 절차 및 방법 
<img width="831" height="600" alt="image" src="https://github.com/user-attachments/assets/c10eef7a-fd77-44fa-a385-bdc52f606d61" />

&nbsp;

### 팀원: 정서윤, 나승원, 홍진규, 이동기, 이세현, 강인우, 이형연

&nbsp;
## 3. 🔧 구성 요소

<img width="1222" height="491" alt="image" src="https://github.com/user-attachments/assets/b7e9aff5-f273-4eda-9e13-5ce00ca27e6a" />


&nbsp;
## 4. 💻 사용 기술

<img width="816" height="725" alt="image" src="https://github.com/user-attachments/assets/d563c55c-7f65-46f4-bcb0-f74a7fb04136" />

<img width="1222" height="725" alt="image" src="https://github.com/user-attachments/assets/9b252f34-52d7-4b75-a57b-5354d9edc214" />


&nbsp;
### 📷 Vision model 
**1. Object Detection (YOLOv11)**  
- 목적: 약 서랍의 라벨 텍스트(예: dermatitis, cold 등)를 박스 단위로 탐지  
- 모델: `yolov11n.pt`  
- Dataset: 20장 → 증강하여 총 60장 (Train 70% / Val 30%)  
- 하이퍼파라미터:  
  - Epoch: 200  
  - Batch size: 16  
  - IOU threshold: 0.5  
- 성능 지표:  
  - mAP@0.5 = **0.995**  
- 결과: 약 서랍 위에 부착된 라벨을 정확히 탐지하여 위치 기반 분류 가능  

<img width="600" height="300" alt="image" src="https://github.com/user-attachments/assets/66f84a6b-4087-4709-824d-bd150fb0c091" />

<img width="400" height="400" alt="image" src="https://github.com/user-attachments/assets/8e911d69-6535-48f5-a8ee-90e72b922055" />

&nbsp;

**2. Text Classification (ResNet18)**  
- 목적: 탐지된 라벨 이미지(text 박스)를 4종류 약 분류로 분류  
- 약 종류: cold, dermatitis, dyspepsia, diarrhea  
- 모델: `ResNet18`  
- Dataset: 20장 → 증강하여 총 80장  
- 하이퍼파라미터:  
  - Epoch: 22  
- 성능 지표:  
  - Accuracy = **1.00**  
- 결과: OCR된 라벨 이미지를 정확하게 약 카테고리로 분류

<img width="600" height="300" alt="image" src="https://github.com/user-attachments/assets/e71ab87e-50ae-4790-8da9-262e386c6833" />

<img width="400" height="400" alt="image" src="https://github.com/user-attachments/assets/0881a9b6-bd4d-4b7a-87de-723dd089fecd" />

&nbsp;

**3. Segmentation (YOLOv11s)**  
- 목적: 약 서랍 내부 의약품 패키지를 탐지 및 회전 각도 추정  
- 모델: `yolov11s.pt`  
- Dataset: 20장 → 증강하여 총 60장 (Train 70% / Val 30%)  
- 하이퍼파라미터:  
  - Epoch: 200  
  - Batch size: 16  
  - IOU threshold: 0.5  
- 성능 지표:  
  - mAP@0.5 ≈ **0.992 ~ 0.993**  

###  🤧 [1. Cold]  
- 탐지 클래스: `amoxicile_tab`, `ponstar_tab`  
- mAP@0.5 = **0.993**

<img width="600" height="300" alt="image" src="https://github.com/user-attachments/assets/8b037393-0f9a-4d26-9057-0d45f7e7565d" />

<img width="400" height="400" alt="image" src="https://github.com/user-attachments/assets/5c5dcb3d-b395-46d7-a0c6-c1176115ed90" />

---

### 🤕 [2.Dermatitis]  
- 탐지 클래스: `monodoxy_cap`, `ganakan_tab`  
- mAP@0.5 = **0.992**

<img width="600" height="300" alt="image" src="https://github.com/user-attachments/assets/307c1f0a-282a-494d-b599-4d93ee1b6a0a" />

<img width="400" height="400" alt="image" src="https://github.com/user-attachments/assets/1057fded-ca00-4450-acf2-76cc6cb6fdb8" />


---

### 🤢 [3.Dyspepsia]  
- 탐지 클래스: `mogum_tab`, `medicostenter`, `nexilen_tab`  
- mAP@0.5 = **0.992**

<img width="600" height="300" alt="image" src="https://github.com/user-attachments/assets/e7440bc4-85fe-4044-a7bc-4d44d5a025e5" />

<img width="400" height="400" alt="image" src="https://github.com/user-attachments/assets/ba31c0f3-0e59-4341-ab07-5a2da85ebbef" />

---

### 💩 [4.Diarrhea]  
- 탐지 클래스: `otillen_tab`, `famodine`, `somnux_scop`  
- mAP@0.5 = **0.992**

<img width="600" height="300" alt="image" src="https://github.com/user-attachments/assets/65cdba23-eee7-4705-8c98-2864ebea89bc" />

<img width="400" height="400" alt="image" src="https://github.com/user-attachments/assets/12101f88-5f75-426c-90d3-277298caa9a6" />


&nbsp;

### 🗣 Voice model 

#### 🎙 1. OpenWakeWord  
- **모델명**: `hello_rokey_8332_32.tflite`  
- **기능**: 웨이크워드 “hello rokey” 감지를 위한 TFLite 기반 모델  
- **동작 방식**:  
  - 0.1초 간격으로 마이크 입력 버퍼에서 오디오 청크 수신  
  - `model.predict()`를 통해 inference 수행  
  - confidence score ≥ 0.6 → 웨이크워드 감지로 간주  

---

#### 📝 2. OpenAI Whisper  
- **모델명**: `whisper-1`  
- **기능**: 녹음된 오디오 파일 (예: `input.wav`)을 텍스트로 변환 (STT)  

---

#### 🤖 3. GPT-4o  
- **모델명**: `gpt-4o`  
- **기능**:  
  - 사용자의 음성 명령에서 의약품 이름 및 수량 추출  
  - 의약품 종류 분류 (전문의약품 vs 일반의약품)  
  - 증상 입력 시 약 추천  
  - 약 설명 요청 시 효능·주의사항 안내  

---

#### 🔊 4. Microsoft Edge TTS  
- **모델명**: `ko-KR-SunHiNeural`  
- **기능**:  
  - TTS(Text-to-Speech)를 통해 사용자에게 음성 안내 출력
  
&nbsp;
## 5. 🧭 동작 흐름 요약

<img width="488" height="689" alt="image" src="https://github.com/user-attachments/assets/8ae44e59-4aa9-436d-88e9-9e8f1de6b939" />

&nbsp;

<img width="1298" height="495" alt="image" src="https://github.com/user-attachments/assets/3c15599b-e267-49eb-9749-136d88901461" />

&nbsp;

### 입차하기 동작과정
1. **nav 초기위치에서 undock**
2. **YOLO 번호판 및 차종 인식 + OCR**
3. **GUI에서 '주차하기' 클릭**
4. **대기 위치로 이동**
5. **차 수레 걸어주기**
6. **주차 공간으로 이동**
7. **Depth 및 TF 발행**
8. **앞으로 전진 (base_link 좌표 기준 TF으로부터 0.5m)**
9. **180도 회전 → '주차 진행 중' 음성 발령**
10. **주차 완료 → '주차 완료' 음성 발령**
11. **차 수레 빼주기**
12. **대기 위치 및 초기 위치로 복귀 → Dock**

&nbsp;

### 출차하기 동작과정
1. **nav 초기위치에서 undock**
2. **GUI에서 '출차하기' 클릭**
3. **주차 공간으로 이동**
4. **차 수레 걸어주기**
5. **출차 공간으로 이동**
6. **차 수레 빼주기**
7. **초기 위치로 이동 → Dock**

&nbsp;

<img width="1045" height="493" alt="image" src="https://github.com/user-attachments/assets/614cca63-d65f-492d-9b61-81855565e3ac" />

&nbsp;

### 입차하기 : 번호판 OCR-> GUI -> Navi
<img width="874" height="493" alt="image" src="https://github.com/user-attachments/assets/307bedd0-bf9e-45b1-aa59-aadf4cb2999d" />

&nbsp;

### 입차하기 : Depth-> TF
<img width="874" height="546" alt="image" src="https://github.com/user-attachments/assets/d026d956-f472-44b8-b319-6be050f94df7" />

&nbsp;

### 입차하기 : Navigation
<img width="1101" height="501" alt="image" src="https://github.com/user-attachments/assets/e1232e2a-d816-428f-89b7-e8ffdd6f8977" />

&nbsp;

### GUI 사용자: 입차하기
<img width="1178" height="463" alt="image" src="https://github.com/user-attachments/assets/a458544a-5495-4a30-a055-bb5e231e9691" />

&nbsp;

### GUI 사용자: 출차하기
<img width="1178" height="463" alt="image" src="https://github.com/user-attachments/assets/00a16dc4-80ba-471f-85a5-077a60096ac0" />

&nbsp;

### GUI 관리자
<img width="514" height="463" alt="image" src="https://github.com/user-attachments/assets/983601b6-a792-4d2e-b03d-c1b8142697ed" />

&nbsp;

## 6. 💻 코드 실행 방법

🅿️ 주차 공간 탐지 (YOLO + Depth)
- 모델: 320_v8n.pt
- 학습 코드: train.py
- 실행 코드: yolo_detect.py

- 코드: [`yolo_detect`](./rokey_ws/src/rokey_pjt/rokey_pjt/yolo_detect.py)
```bash
ros2 run rokey_pjt yolo_detect
```

&nbsp;

📷 번호판 인식 (YOLO + OCR)
- 모델: best_320.pt
- 학습 코드: yolo_train.ipynb
- 실행 코드: detect_car_info2.py

- 코드: [`detect_car_info2`](./rokey_ws/src/rokey_pjt/rokey_pjt/detect_car_info2.py)
```bash
ros2 run rokey_pjt detect_car_info2
```

&nbsp;

🧠 TF Classifier - 전면
- 코드: [`detect_car_info2`](./rokey_ws/src/rokey_pjt/rokey_pjt/detect_ps_front.py)
  
```bash
ros2 run rokey_pjt detect_ps_front
```

&nbsp;


🖥️ GUI
- 코드: [`detect_car_info2`](./rokey_ws/src/rokey_pjt/rokey_pjt/User_GUI/parking_gui.py)
  
```bash
python3 parking_qui.py
```

&nbsp;


🚗 입차 주행 제어
- 코드: [`sc_follow_waypoints2_1`](./rokey_ws/src/rokey_pjt/rokey_pjt/sc_follow_waypoints2_1.py)
  
```bash
ros2 run rokey_pjt sc_follow_waypoints2_1
```

&nbsp;

🚙 출차 주행 제어
- 코드: [`sc_follow_waypoints`](./rokey_ws/src/rokey_pjt/rokey_pjt/sc_follow_waypoints.py)
  
```bash
ros2 run rokey_pjt sc_follow_waypoints
```

&nbsp;

## 7. 📷 시연 영상 / 이미지
### 시연영상
> https://youtu.be/YpOET5k4NcU

### 발표영상
>

&nbsp;
## 8. 🌟 기대 효과

- **AI Vision과 Navigation을 통한 실시간 자율 입·출차 구현**  
- **YOLO 객체 인식, TF, Depth 계산, 장애물 회피까지 수업 핵심 요소 모두 반영**  
- **주차 공간 내 정밀 위치 지정 → 안정적인 180도 회전 및 전진 주차 성공률 향상**  
- **GUI와 음성 피드백, DB 연동 등 사용자 친화성 강화 요소 적극 적용**

&nbsp;

### ⚙️ 활용 방안

- **AI 자율주행 기반 스마트 주차 시스템**
- **비대면 주차 서비스 로봇**
- **지하 주차장 자동 안내 및 충전 공간 정렬**
- **로봇-서버-DB 연동을 통한 실시간 주차관리 시스템**

&nbsp;

### ⚠️ 잘한 점

- **[1. AI Vision 인식 정확도 및 Depth 계산]**  
  → 객체 인식 정확도 90~95%, 3x3 거리 계산 적용으로 거리 추정 오차 최소화

- **[2. TF Transform 기반 위치 이동 수행]**  
  → base_link에서 목표 좌표까지 정확하게 TF 계산 → Nav2로 목적지 주행 성공률 높음

- **[3. 로봇 간 협업 연동]**  
  → 입차/출차용 로봇의 DB 연동 및 상태 공유 구현 → 병렬 처리 기반 협업 가능

- **[4. One Take 시연 영상]**  
  → 실제 입차부터 출차까지 논스톱 데모 영상 제작 → 실전 수준 데모 구현 능력 입증

- **[5. 시스템 구성 및 요구사항 충실 반영]**  
  → 아키텍처, 시나리오, 노드 간 연결 흐름이 명확하고 문서화 완성도 높음

- **[6. Flowchart 및 시나리오 구성]**  
  → 전체 기능 흐름도와 각 기능별 상세 설명 및 시나리오 스크립트 명확 작성

- **[7. 발표 시간 및 형식 준수]**  
  → 자료 통일성 확보, 제한 시간 내 발표 및 질의응답 대응 능력 우수

- **[8. 헬퍼 기능 추가]**  
  → GUI, 경고음, DB 저장 등 실사용 시 고려되는 확장 기능 탑재


&nbsp;


🧩 **한계 및 개선점**

- **네트워크 품질 저하 시 시스템 안정성 저하**  
  → 실내 와이파이 환경에서 로봇이 끊김 없이 이동하려면 추가적 예외처리 필요

- **TF 계산 시 간헐적 오차**  
  → 실시간 센서 노이즈에 의해 소폭 오차 발생 → 필터링 기법 도입 고려

- **하드웨어 의존성**  
  → 수레 걸기/빼기 동작의 정확도는 물리적인 제약을 크게 받음 → 그리퍼 보완 필요

&nbsp;
