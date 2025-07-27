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


### 📋 전문의약품 노드 

1. **사람 감지**  
   - 초음파 센서 → robot  
   - 상태 메시지: `state="detected"` ROS topic publish

2. **QR 및 안내 음성 출력**  
   - robot → vision (`detect_qr`)  
   - robot → voice (처방전 안내 음성 출력)  
   - `robot_state = "check_qr"`

3. **QR 코드 인식**

4. **서랍 텍스트 인식 위치 지정**  
   - vision → robot (`qr_info`)

5. **서랍 텍스트 인식 실행**  
   - robot → vision  
   - `robot_state = "check_text"`

6. **서랍 열기 및 약 탐지 위치로 이동**  
   - robot → 서랍 앞

7. **약 위치 탐지 (YOLOv11)**  
   - robot → vision  
   - `robot_state = "detect_pill"`

8. **약 위치로 이동 및 자세 정보 추출**  
   - vision → robot  
   - 결과값: (x, y, theta)

9. **약 집기 + 비닐봉투에 담기 + 서랍 닫기**

10. **복약 설명 음성 출력**  
    - robot → voice (`task_state`)

&nbsp;

### 📋 일반의약품 노드

1. **사람 감지**  
   - 초음파 센서 → robot  
   - 상태 메시지: `state="detected"`

2. **QR 인식 및 일반약 요청 음성 출력**  
   - robot → vision / voice  
   - `robot_state = "check_qr"`

3. **음성 명령 수신 (의약품 요청)**  
   - voice → robot  
   - topic: `medicine`

4. **로봇이 선반 앞으로 이동**

5. **선반 인식 준비**  
   - robot → vision  
   - `robot_state = "shelf_state"`

6. **약 위치 탐지 (YOLOv11)**  
   - vision → robot  
   - 결과: `medicine_loc`

7. **약 pick & place**  
   - robot → vision  
   - `robot_state = "pick_medicine"`

8. **외력 인식 시 로봇 순응 제어 실행 (Gripper Release)**

9. **약 설명 음성 출력**  
   - robot → voice (`task_state`)

&nbsp;

### 💊 전문의약품 동작 과정 

1. **초음파로 사람 감지**  
   - 5~37cm 거리에서 사용자가 3초 이상 머물면 감지  
   - moving average 필터 사용  
   - 로봇에 메시지 전송: state="detected"

2. **처방전 QR 인식 자세 및 안내 음성 출력**  
   - 로봇의 동작은 모두 moves로 자연스럽게 연결
   - 예:  
      ```
      voice: "안녕하세요 rokey약국입니다. QR을 스캔하거나 hello rokey를 말해주세요."
      ```

3. **QR 찍기 (처방전 인식)**  
   - 처방전 파싱 → json 코드
   
   <img width="600" height="300" alt="image" src="https://github.com/user-attachments/assets/1fa064a8-69eb-4dfa-a694-9cf27adfd11f" />

   - 예시:
     ```json
     {
       "이름": "홍길동",
       "생년": "010204",
       "atc": "A02XA01",
       "1회투약량": 1,
       "1일투약횟수": 3,
       "총투약일수": 1
     }
     ```

4. **서랍 바라보는 모션**  

5. **서랍 text 인식 후 열기**  
   - 처방된 ATC 코드, 이름, 증상을 딕셔너리로 저장
   - 처방전 atc 코드 -> 증상 == yolo text 증상
   - text 증상 (x,y) 가 4분면 중 하나 해당 -> 지정 좌표 이동

6. **서랍 안에 바라보는 위치 조정**  
   - 4개의 서랍 segmentation 좌표로 이동

7. **약 탐지**  
   - 약 segmentation mask 적용  
   - A02XA01 1 1 1 → nexilan_tab 탐지  
   - 타원형: x, y, theta 추출  
   - camera calibration 후 3D 좌표로 로봇 이동
   - segmentation mask 사용하여 타원형 알약의 포즈 계산 (deg) -> 로봇의 6축이 해당 deg 만큼 회전 -> girp

8. **약 이동 및 집음**  
   - gripper 알약 크기에 맞춰 조정 -> force control 과 compliance로 알약 세밀하게 집기
   - `A02X1 1 1 1 -> 1번 1번 투약 1일치`
   - nexilan_tab( index=1/ total 1) -> 점심 이동
   - 흔드는 모션 -> 약이 잘 떨어지지 않음 방지  

9. **다른 약도 반복 탐지 및 분류**
   - A07FA01 1 2 1 -> medilacsenteric_tab 탐지
   - 타원형 캡슐-> center x, y, theta 전송 -> camera calibration -> 3d 좌표로 로봇 이동

10. **약 이동 및 집음**
    - gripper 알약 크기에 맞춰 조정
    - `A07FA01 1 2 1 ->1번 2번 투약 1일치`
    - nexilan_tab( index=1/ total 2)-> 아침
    - nexilan_tab( index=2/ total 2)-> 저녁

11. **다른 약도 반복 탐지 및 분류**
    - gripper 알약 크기에 맞춰 조정
    - A02AA04 1 3 1 ->1번 3번 투약 1일치
    - A02AA04 1 3 1 -> magmil_tab 탐지
    - 원형 캡슐-> center x, y 전송 -> camera calibration -> 3d 좌표로 로봇 이동 (원형은 theta 무시)

12. **약 이동 및 집음**
    - gripper 알약 크기에 맞춰 조정
    - `A02AA04 1 3 1 ->1번 3번 투약 1일치`
    - magmil_tab( index=1/ total 3)-> 아침
    - magmil_tab( index=2/ total 3)-> 저녁
    - magmil_tab( index=3/ total 3)-> 저녁
      
13. **선반 넣기**  
    - 로봇이 약을 force control로 집음 -> 선반이 밑으로 내려가게됨  
    - 선반을 잡고 올리기 -> 밀어넣기

14. **포장 대기 상태 이동**  
    - 모든 약 개수 약 주걱으로 이동 후 포장 상태로 전환

15. **약사가 약 포장 후 외력 감지**
    - 약사가 약 검사 및 약 비닐 → 외력으로 포장 완료 알림   
    - 외력 감지 시: `check_force_condition x축 = true`  
    - 외력 감지 후 외력 해제 

17. **약 봉투로 이동 및 설명**  
    - 외력 해제 시 약 설명 voice 출력  
    - 예:  
      ```
      voice: "nexilan_tab은 위염치료제이며 다른 약 복용시 위 손상을 막아줍니다. 아침 점심 저녁 하루 3번 복용하세요.
             감사합니다 안녕히 가세요"
      ```

### 🧾 일반의약품 동작 과정

1. **초음파로 사람 감지**  
   - 5~37cm 거리에서 사용자가 3초 이상 머물면 감지  
   - moving average 필터 사용  
   - 로봇에 메시지 전송: `state="detected"`

2. **QR 인식 자세 안내 및 음성 출력**  
   - 로봇의 동작은 모두 `moves`로 자연스럽게 연결  
   - 음성 안내:  
     ```
     voice: "안녕하세요 rokey약국입니다. QR을 스캔하거나 hello rokey를 말해주세요."
     ```

3. **증상 입력 (voice: hello rokey)**  
   - 사용자가 음성으로 증상 입력  
   - 예시:  
     ```
     "나 머리 아파 / 열이 나 / 감기 걸린 것 같아"
     ```

4. **voice 기반 의약품 추천**  
   - 증상 기반 추천 음성 출력  
   - 예시:  
     ```
     voice: "추천약은 타이레놀(해열진통제)과 판콜에이(감기약)입니다. 증상이 계속되면 병원에 방문하세요."
     ```

5. **voice로 추천 의약품 구매 의사 확인**  
   - 사용자 예시:  
     ```
     "타이레놀 줄래요?"
     ```  
   - 로봇 응답:  
     ```
     "추천약은 타이레놀입니다. 어떤 약을 드릴까요?"
     ```

6. **voice로 의약품 구매 명령**  
   - 사용자 예시:  
     ```
     "타이레놀 한 개 줘"
     ```  
   - 로봇 응답:  
     ```
     "타이레놀을 준비하겠습니다."
     ```

7. **선반 위치 이동 및 의약품 인식**  
   - 선반 이동 후 YOLO로 의약품 위치 인식  
   - 약품 좌표 (x, y)가 4분면 중 하나에 해당 → 지정 좌표로 이동  

8. **의약품 꺼내기 및 가져다주기**  
   - 물품 높이에 따라 그리퍼 너비 조정  
   - 2층: `movec`로 문턱 넘기 -> movej
   - 1층: `movej`  

9. **x축 외력 감지 시 약 놓기 및 설명 출력**  
   - 사용자가 손 흔들기 등으로 외력 제공  
   - 조건 감지: `check_force_condition == true` (x축 외력 감지)  
   - 로봇이 약 내려놓고 약 설명 출력  
   - 예시:  
     ```
     voice: "해당 약은 해열진통제이며 진통 완화 및 열 내림 효과가 있습니다. 감사합니다 안녕히 가세요."
     ```

## 6. 💻 코드 실행 방법

### 🤖 Robot Control Node
- 코드: [`main_robot_control`](./Rokey_Pharmacy-main/rokey_project/rokey_project/main_robot_control.py)

```bash
ros2 run rokey_project main_robot_control
```

### 👁️ Vision Node (Realsense)
- 코드: [`main_vision_realsense`](./Rokey_Pharmacy-main/rokey_project/rokey_project/main_vision_realsense.py)

```bash
ros2 run rokey_project main_vision_realsense
```

&nbsp;
## 7. 📷 시연 영상 / 이미지
> https://youtu.be/qz6bvLREzT4

> https://youtu.be/YkDVQ3afCMA

&nbsp;
## 8. 🌟 기대 효과

- 약물 사고 예방 → 사망 사고, 부작용 최소화
- 약사의 단순 반복업무 감소 → 핵심 업무 집중 가능
- 복약 실수 줄이고, 독립적인 복약 가능
- 팬데믹 등 상황에서 비대면 복약 시스템 활용 가능

### ⚙️ 활용 방안

- 약국 내 조제 공정
- 의료 어시스턴트(수술, 차트)
- 창고 정리
- 선반/서랍 정리


### ⚠️ 잘한 점 / 아쉬운 점

- **약 모양 다양함 → segmentation으로 위치, 자세 추정 → 로봇 세밀하고 정밀한 조정 가능**
- **다양한 센서 이용 → voice, vision, sleep → 저전력 구동**
- **각 detection에 맞는 모델 사용 → ai 판단 → 증상, 전문 의약품, 일반의약품 다양**

🧩 **한계 및 개선점**:
- 그리퍼의 크기가 약을 집기에 커서 겹쳐있는 약이나, 붙어있는 약을 집을 때 정확히 집지 못하는 점
- 기능 추가 필요 (증상 → 처방전 → 약 종류 자동 판단 등 통합 처리)
- 예외처리 부족 (약 부족 상황에서 대체 약 제안, 의약품 추가 구매 유도 등)


### 🤝 팀 완성도 평가 및 느낀 점

- **백종하**: 팀원과의 활발한 소통이 중요함을 느낌. 커뮤니케이션이 순조롭게 진행됨  
- **정서윤**: 계획과 실제 운영에서 발생한 문제를 유연하게 대응하며 실무형 문제 해결 역량 체감  
- **정민섭**: 협력 과정에서 중요한 점은 팀원 간 보완과 통합이었음을 실감함  
- **서형원**: 협동 로봇 기능을 잘 활용하고 추가 의견을 반영하며 팀워크 완성도 향상

&nbsp;
