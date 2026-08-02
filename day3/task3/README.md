# ATmega128 과제 및 프로젝트 템플릿

> **광운대학교 (로봇학부)**  
> **작성자:** (예희원)
> **제출일:** (8/2)

---

## 1. 개요 (Overview)
본 과제는 ATmega128 마이크로컨트롤러를 활용하여 Dynamixel을 목표 위치로 돌리는 것이 목표이다.

### 핵심 목표
* 가변저항 값에 따라 Dynamixel 목표 위치 설정(0 ~ 1023)
* PC에서 전송 받은 값(0~9)에 따라서 Dynamixel 목표 속도 설정
* Dynamixel 목표 속도 (첫 번째 줄)와 목표 위치 (두 번째 줄) LCD에 표시

---

## 2. 개발 환경 (Environment)

| 항목 | 내용 |
| :--- | :--- |
| **MCU** | ATmega128A (16MHz External Crystal) |
| **IDE / Compiler** | Microchip Studio 7.0 / Microchip AVR GCC |
| **Flasher Tool** | USBISP / STK500 |
| **언어** | C Language |
| **주요 부품** | ATmega128 개발보드, Dynamixel, ADC 센서 모듈, LCD, ISP |

---

## 3. 하드웨어 구성 및 핀 맵 (Hardware Structure)

### Pin Configuration

```text
[ATmega128]                 [Target Component]
[PORTA] -----> LED Output
  PA0~PA7 : 8-Bit LED Output

[PORTD] -----> SW1 Button (External Interrupt)
  PD2 (INT2) : SW1 Button Input

[PORTE] -----> UART0 Serial Communication
  PE0 (RXD0) : UART0 Serial Receive (from PC)
  PE1 (TXD0) : UART0 Serial Transmit (to PC)
```

### 주요 회로 특징
* **전원:** 5V DC 안정화 전원 공급
* **주의사항:** ISP 다운로드 시 SPI 핀 타겟 전원 및 리셋 회로 간섭 주의

---

## 4. 프로젝트 구조 (Directory Structure)
> 구현부(.c), 선언부(.h)만 구조에 표기함.
```text
├── task3
│   ├── task3
│   │   ├── i2c.c
│   │   ├── i2c.h
│   │   ├── i2c_lcd.c
│   │   ├── i2c_lcd.h
│   │   ├── main.c
│   │   └── task3.cproj
│   └── task3.atsln
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
//  ----ADC 설정------
	ADMUX  = (1 << REFS0);                          // 기준전압 AVCC, 채널 ADC0(PF0)
	ADCSRA = (1 << ADEN)                            // ADC 활성화
	| (1 << ADPS2) | (1 << ADPS1) | (1 << ADPS0);   // 128분주 -> 125kHz
```

---

## 6. 동작 설명 및 결과 (Results)

### 동작 시나리오
1. 가변 저항 값에 따라 Dynamixel 목표 위치 설정( 0 ~ 1023)
2. PC에서 0 ~ 9값을 입력받아 Dynamixel 목표 속도 설정
3. Dynamixel 목표 속도 (첫 번째 줄)와 목표 위치(두 번째 줄) LCD에 표시

### 동작 사진 / 영상

| 정면 동작 모습 | 센서 측정 및 시리얼 출력 |
| :---: | :---: |
| ![Hardware Setup](https://drive.google.com/file/d/1Vwxf26oCHbrQc77UAOjMGULT6wbblv7F/view?usp=drive_link)
---

## 7. AI 툴 활용 명시 (AI Tools Declaration)
본 과제 작성 및 구현 과정에서 활용한 AI 도구(Generative AI)의 사용 현황 및 목적은 다음과 같음.

| 도구명 (Tool) | 활용 영역 | 세부 사용 목적 및 내용 |
| :--- | :--- | :--- |
| **Claude** | 코드 디버깅 | - 빌드 에러 및 문법 오류 원인 분석<br>- 레지스터 설정 주석 작성|
| **Claude** | 코드 예제 작성 | - |Dynamixel 프로토콜 2.0에 대한 예제 작성을 요청해 공부했다|


### AI 활용 및 검증 원칙
1. **코드 검증:** AI가 생성한 레지스터 설정 및 함수 코드는 데이터시트(ATmega128 Datasheet)와 비교 검증한 후 실제 시리얼 모니터링을 거쳐 직접 수정 및 테스트하였습니다.
2. **학습 주도성:** Dynamixel 제어에 필요한 레지스터들과 과 예제를 통해 Dynamixel 제어 방법을 익혔다.