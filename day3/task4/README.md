# ATmega128 과제 및 프로젝트 템플릿

> **광운대학교 (로봇학부)**  
> **작성자:** (예희원)
> **제출일:** (8/2)

---

## 1. 개요 (Overview)
본 과제는 ATmega128 마이크로컨트롤러의 PORTD와 DDRD만을 사용하여 uart 데이터를 전송하는 것이 목표이다.

### 핵심 목표
* UART 관련 레지스터 사용하지 않고 PORTD와 DDRD만을 사용하여 uart 데이터 전송 가능하게 하기
* "HelloWorld!" 1초마다 보내기
* Baudrate 9600으로 할 것

---

## 2. 개발 환경 (Environment)

| 항목 | 내용 |
| :--- | :--- |
| **MCU** | ATmega128A (16MHz External Crystal) |
| **IDE / Compiler** | Microchip Studio 7.0 / Microchip AVR GCC |
| **Flasher Tool** | USBISP / STK500 |
| **언어** | C Language |
| **주요 부품** | ATmega128 개발보드, ISP |

---

## 3. 하드웨어 구성 및 핀 맵 (Hardware Structure)

### Pin Configuration

```text
[ATmega128]                 [Target Component]
------------------------------------------------------------
[PORTD] -----> Software UART (Bit-Banging)
  PD3 : Software UART TX (HelloWorld! 1초마다 전송, 9600bps)
```

### 주요 회로 특징
* **전원:** 5V DC 안정화 전원 공급
* **uart** uart1에 연결할 것
* **주의사항:** ISP 다운로드 시 SPI 핀 타겟 전원 및 리셋 회로 간섭 주의

---

## 4. 프로젝트 구조 (Directory Structure)
> 구현부(.c), 선언부(.h)만 구조에 표기함.
```text
└── task4
    ├── task4
    │   ├── main.c
    │   └── task4.cproj
    └── task4.atsln
```

---

## 5. 핵심 코드 및 레지스터 설정 (Key Implementation)

### UART 초기화 예시
```c
#include <avr/io.h>
#include <util/delay.h>
#include <stdint.h>

	DDRD  |= (1 << TX_PIN);     // TX_PIN 출력으로 설정
	PORTD |= (1 << TX_PIN);     // Idle 상태 = High (UART는 안 보낼 때 항상 High)
```

---

## 6. 동작 설명 및 결과 (Results)

### 동작 시나리오
1. start 신호를 보내줌
2. "Helloworld!"를 한문자씩 보냄
3. end 신호를 보냄
### 동작 사진 / 영상

| 정면 동작 모습 | 센서 측정 및 시리얼 출력 |
| :---: | :---: |
| ![Hardware Setup](https://drive.google.com/file/d/1FSbACL0YsxUwq95ukgD038Zol1bZzogh/view?usp=drive_link)
---

## 7. AI 툴 활용 명시 (AI Tools Declaration)
본 과제 작성 및 구현 과정에서 활용한 AI 도구(Generative AI)의 사용 현황 및 목적은 다음과 같음.

| 도구명 (Tool) | 활용 영역 | 세부 사용 목적 및 내용 |
| :--- | :--- | :--- |
| **Claude** | 코드 디버깅 | - 빌드 에러 및 문법 오류 원인 분석<br>- 레지스터 설정 주석 작성|



### AI 활용 및 검증 원칙
1. **코드 검증:** AI가 생성한 레지스터 설정 및 함수 코드는 데이터시트(ATmega128 Datasheet)와 비교 검증한 후 실제 시리얼 모니터링을 거쳐 직접 수정 및 테스트하였습니다.
2. **학습 주도성:** uart1이 PD2, PD3을 사용하기 때문에 uart관련 레지스터 없이 통신할 수 있다는 것을 알게됨