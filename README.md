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

---

### 🖥️ 1. GUI 관리자- 차량 감지 dataset 
<img width="1245" height="525" alt="image" src="https://github.com/user-attachments/assets/78002b5b-3bd6-4f4b-b06f-a6e4219b3b1a" />

&nbsp;

---

### 📷 2. Vision- 주차공간 object detection

<img width="1245" height="525" alt="image" src="https://github.com/user-attachments/assets/7046baad-fd9c-4c19-a83f-5dea54de9ae9" />

<img width="1245" height="540" alt="image" src="https://github.com/user-attachments/assets/97346b06-e019-4ced-b0e8-60c5f698fada" />

<img width="1292" height="281" alt="image" src="https://github.com/user-attachments/assets/758070db-1cad-4e2a-88fe-ca8a3ec3cd86" />

&nbsp;

### - 주차(Disabled, Normal, EV) - 모델 선정 과정

<img width="1133" height="511" alt="image" src="https://github.com/user-attachments/assets/e015f439-020c-4a47-842a-0ffe2b53592b" />

<img width="1225" height="553" alt="image" src="https://github.com/user-attachments/assets/f287d0a9-5db4-4dd0-82fa-049192a576fb" />

&nbsp;

### - 주차(Disabled, Normal, EV) 경량화 - size 640
<img width="1225" height="290" alt="image" src="https://github.com/user-attachments/assets/fa39804e-624d-4cfa-b4fb-55dd2f57ee89" />


&nbsp;

### - 주차(Disabled, Normal, EV) 경량화 - size 320
<img width="1225" height="546" alt="image" src="https://github.com/user-attachments/assets/37a35c70-9827-4d82-8b70-d7393793dc4b" />

&nbsp;

### - 주차(Disabled, Normal, EV) 경량화 - compressed
<img width="1281" height="546" alt="image" src="https://github.com/user-attachments/assets/5c3c9a39-06de-4003-b8c7-eadc7678c5a0" />

<img width="1281" height="546" alt="image" src="https://github.com/user-attachments/assets/97f71119-ff49-4416-afcc-30236d2897d3" />

&nbsp;
### - 주차 object detection 결론 : size 320, compressed

&nbsp;

---

### 📷 3. Vision- 차량 번호판 object detection
<img width="1278" height="546" alt="image" src="https://github.com/user-attachments/assets/eda5c1a1-e60b-49a5-8fc3-6667fee682f3" />

<img width="1276" height="546" alt="image" src="https://github.com/user-attachments/assets/fd33bc6c-d3a3-42e7-810b-08c46911b246" />

&nbsp;

### - 차량 번호판 경량화 - raw 640
<img width="1278" height="546" alt="image" src="https://github.com/user-attachments/assets/19eb473f-384b-4668-8adc-f2b3f1a786cf" />

&nbsp;

### - 차량 번호판 경량화 - raw 320
<img width="1300" height="546" alt="image" src="https://github.com/user-attachments/assets/0ce6920a-7282-4823-9221-9ece1383a116" />


&nbsp;

### - 차량 번호판 경량화 - compressed
<img width="1300" height="546" alt="image" src="https://github.com/user-attachments/assets/0c9bb116-6004-4fa7-9e70-7e6fd96b6d31" />

<img width="1300" height="546" alt="image" src="https://github.com/user-attachments/assets/2d71119b-c871-4044-a123-076e09c872c0" />

&nbsp;
### - 차량번호판 object detection 결론 : size 320, compressed

&nbsp;

---

### 📷 3. Vision- 차량 번호판 + OCR
<img width="1300" height="546" alt="image" src="https://github.com/user-attachments/assets/c97afccf-e41b-4736-b577-93d821f8a290" />

<img width="1320" height="546" alt="image" src="https://github.com/user-attachments/assets/a7499120-9edb-4bd6-8a20-58bf8e5fd962" />

<img width="1320" height="546" alt="image" src="https://github.com/user-attachments/assets/e680fc4c-0e18-42de-aa97-57f0ae59813a" />

<img width="1364" height="546" alt="image" src="https://github.com/user-attachments/assets/ea11f3f8-f139-429e-90b2-53474e94d82b" />

<img width="1364" height="546" alt="image" src="https://github.com/user-attachments/assets/26142f5d-a608-4831-8dab-68a0a6e00605" />


&nbsp;

---

### 🧠 4. Depth + TF
<img width="1364" height="546" alt="image" src="https://github.com/user-attachments/assets/226e7082-2ba2-4ff3-a945-b2d8a164cee2" />

<img width="1364" height="546" alt="image" src="https://github.com/user-attachments/assets/f8240b0a-4c88-4f21-badf-53e3651b29f9" />

<img width="1364" height="546" alt="image" src="https://github.com/user-attachments/assets/ec5e113c-55c1-4ed2-aa1c-1be165851815" />

<img width="1364" height="546" alt="image" src="https://github.com/user-attachments/assets/54d01ef7-3afd-4649-8e55-ef0d4c4d2f5f" />


&nbsp;

---

### 🗺️ 5. Slam, Nav
<img width="1364" height="546" alt="image" src="https://github.com/user-attachments/assets/87b5314d-fca2-478d-a7b5-283c7b271388" />

<img width="1364" height="546" alt="image" src="https://github.com/user-attachments/assets/30861604-9caa-4a8a-b1e2-45dbe2c5f56e" />

<img width="1364" height="546" alt="image" src="https://github.com/user-attachments/assets/4595b276-eb61-4c39-bbf7-dbfcc4d88db5" />

<img width="1364" height="546" alt="image" src="https://github.com/user-attachments/assets/9f526667-39c7-487f-a731-27251a419362" />


&nbsp;

---

### 📊 6. DB
<img width="1364" height="546" alt="image" src="https://github.com/user-attachments/assets/a6e40f7c-52eb-47a3-b789-a64cb4645f40" />

<img width="1364" height="546" alt="image" src="https://github.com/user-attachments/assets/df0259da-f53d-4727-8930-63ed553353ce" />

&nbsp;


### 📦 Structure

- **Database Name**: `parking`
- **Measurement**: `parking` (주차 데이터 저장)

&nbsp;


### 🧩 Schema

- **Tag Keys**  
  - `car_type` : 차량 종류 (`normal`, `ev`, `disabled`)  
  - `license_plate` : 차량 번호  
  - `location` : 차량 위치 (`A-1`, `B-2` 등)

- **Status Field**  
  - `status` : `parked` (입차), `exit` (출차)

- **Timestamp**  
  - 입차 및 출차 시간 자동 기록

&nbsp;


### 🅿️ Parking Location (한정 영역)

| 차량 종류     | 위치     |
|--------------|----------|
| `normal`     | A-1, A-2 |
| `ev`         | B-1, B-2 |
| `disabled`   | C-1, C-2 |

&nbsp;

### 🔄 Data Flow

- **입차 시**
  - 차량 번호, 차량 타입, 위치, `status: parked`, 입차 시간 기록

- **출차 시**
  - 차량 번호, 차량 타입, 위치, `status: exit`, 출차 시간 기록

- **관리 방식**
  - 시간 순으로 입출차 기록 → 효율적인 조회 및 관리

&nbsp;


### 🛠️ DB Initialize

- 클라이언트 및 관리자 초기화
- 기존 데이터 로드 및 조회
- 최근 30일 이내 입출차 이력 조회

&nbsp;


### 🔁 DB Synchronize

- **주기**: 5초
- **기능**:
  - DB 주기적 갱신
  - 주차된 차량 정보 최신 상태로 유지

&nbsp;


### 🚘 주차된 차량 정보 조회 로직

#### 1. WITH 절 (임시 테이블 `latest_status`)
- 차량 번호별 가장 최근 기록만 조회
- 30일 이내 데이터 필터링
- 차량 번호로 그룹화

#### 2. Main Query
- `parking` 테이블과 `latest_status` 조인
- 조건: 차량 번호 & 최신 시간 일치
- 결과: 최신순(`DESC`) 정렬

#### 3. 출력 결과
- `license_plate`, `car_type`, `location`, `status`, `time`

&nbsp;


### 🚗 출차 기록 조회

- 기간: 최근 7일  
- 조건: `status = exit`  
- 정렬: `time DESC`  
- 제한: 최근 20개만 반환  

&nbsp;


### 🧾 전체 기록 조회

- 기간: 최근 30일  
- 정렬: `time DESC`  
- 제한: 100개 레코드 반환  
- 필드: 차량 번호, 타입, 위치, 상태, 작업 시간  

&nbsp;


### 🔍 차량 번호로 조회

- 조건: `license_plate = 차량번호` AND `status = parked`
- 정렬: `time DESC`
- 반환: 1건

&nbsp;


### 📍 위치 기반 조회

- 조건: `location = 특정위치` AND `status = parked`
- 정렬: `time DESC`
- 반환: 1건

&nbsp;


### 🔢 차량 번호 뒷자리 4자리로 검색

- 조건: `license_plate LIKE '%1234%'` AND `status = parked`
- 정렬: `time DESC`

&nbsp;


### 🧾 데이터 작성 (입차/출차 이벤트 기록)

- **InfluxDB는 시계열 비정형 DB로 SQL의 INSERT 대신 직접 작성**
- **형식**: `line protocol`

&nbsp;

#### ✅ 예시
```plaintext
parking,car_type=normal,license_plate=12가3456,location=A-1 status="parked" 1722337200000000000
```
- Measurement: parking
- Tags: car_type, license_plate, location
- Field: status
- Timestamp: ns 단위 시간


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
- 코드: [`detect_ps_front`](./rokey_ws/src/rokey_pjt/rokey_pjt/detect_ps_front.py)
  
```bash
ros2 run rokey_pjt detect_ps_front
```

&nbsp;


🖥️ GUI
- 코드: [`parking_qui`](./rokey_ws/src/rokey_pjt/rokey_pjt/User_GUI/parking_gui.py)
  
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
> https://youtu.be/5f-ziTxzPM4

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
