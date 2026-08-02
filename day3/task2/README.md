# ATmega128 과제 및 프로젝트 템플릿

> **광운대학교 (로봇학부)**  
> **작성자:** (예희원)
> **제출일:** (8/2)

---

## 1. 개요 (Overview)
본 과제는 ATmega128 마이크로컨트롤러를 활용하여 UART 통신으로 LED를 켜고 이동시키는 것을 목표로 함

### 핵심 목표
* ATmega128 레지스터 설정을 통한 UART 제어
* 센서 및 외부 모듈과의 통신 (UART)및 데이터 처리
* LED 제어

---

## 2. 개발 환경 (Environment)

| 항목 | 내용 |
| :--- | :--- |
| **MCU** | ATmega128A (16MHz External Crystal) |
| **IDE / Compiler** | Microchip Studio 7.0 / Microchip AVR GCC |
| **Flasher Tool** | USBISP / STK500 |
| **언어** | C Language |
| **주요 부품** | ATmega128 개발보드, LED, 택트 스위치, ISP |

---

## 3. 하드웨어 구성 및 핀 맵 (Hardware Structure)

### Pin Configuration

```text
[ATmega128]                 [Target Component]
 PORTA (PA0 ~ PA7)   ----->   8-Bit LED Output
 PD2 (INT2)          ----->   SW1 Button (External Interrupt)
 PE0 (RXD0)          ----->   UART0 Serial Receive (from PC)
 PE1 (TXD0)          ----->   UART0 Serial Transmit (to PC)
```

### 주요 회로 특징
* **전원:** 5V DC 안정화 전원 공급
* **주의사항:** ISP 다운로드 시 SPI 핀 타겟 전원 및 리셋 회로 간섭 주의

---

## 4. 프로젝트 구조 (Directory Structure)
> 구현부(.c), 선언부(.h)만 구조에 표기함.
```text
├── task2
    ├── README.md
    ├── task2
    │   ├── main.c
    │   └── task2.cproj
    └── task2.atsln
```

---

## 5. 핵심 코드 및 레지스터 설정 (Key Implementation)

### UART 초기화 예시
```c
#include <avr/io.h>
#include <avr/interrupt.h>

// ---- UART 초기화 ----
	UBRR0L = 16;    // Baud Rate : 57600bps (16MHz)
	UBRR0H = 0;
	UCSR0B = 0x18;  // 송신, 수신 기능 활성화 (RXEN0, TXEN0)
	UCSR0C = 0x06;  // START 1비트 / DATA 8비트 / STOP 1비트
	DDRE = 0x02;    // RXD0(E0) 입력, TXD0(E1) 출력
```

---

## 6. 동작 설명 및 결과 (Results)

### 동작 시나리오
1. switch1을 누르면 모든 상태 초기화 후, UART로 "RESET" 문자열 출력
2. PC에서 0 ~ 7 숫자 입력 시 해당 번호의 LED 켜기
3. PC에서 8 입력 시, LED 왼쪽으로 이동, PC에 "LEFT" 문자열 전송
4. PC에서 9 입력 시, LED 오른쪽으로 이동, PC에 "RIGHT" 문자열 전송

### 동작 사진 / 영상

| 정면 동작 모습 | 센서 측정 및 시리얼 출력 |
| :---: | :---: |
|[Hardware Setup](https://drive.google.com/file/d/1SqbjaTcVUgyNdwEXCQdhHUbixRf6UqSa/view?usp=drive_link)|
---

## 7. AI 툴 활용 명시 (AI Tools Declaration)
본 과제 작성 및 구현 과정에서 활용한 AI 도구(Generative AI)의 사용 현황 및 목적은 다음과 같음.

| 도구명 (Tool) | 활용 영역 | 세부 사용 목적 및 내용 |
| :--- | :--- | :--- |
| **Claude** | 코드 디버깅 | - 빌드 에러 및 문법 오류 원인 분석<br>- 레지스터 설정 주석 작성|

### AI 활용 및 검증 원칙
1. **코드 검증:** AI가 생성한 레지스터 설정 및 함수 코드는 데이터시트(ATmega128 Datasheet)와 비교 검증한 후 실제 시리얼 모니터링을 거쳐 직접 수정 및 테스트하였습니다.
2. **학습 주도성:** 코드의 핵심 제어 로직 설계는 직접 했지만, 오류가 발생했을 때 빠진 부분을 보완하고 로직 오류에 대한 피드백을 받아가며 코드를 작성했습니다.
