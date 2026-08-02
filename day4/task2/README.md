# ATmega128 과제 및 프로젝트 템플릿

> **광운대학교 (로봇학부)**  
> **작성자:** (예희원)
> **제출일:** (8/2)

---

## 1. 개요 (Overview)
본 과제는 ATmega128 마이크로컨트롤러의 uart, ADC, interrupt 기능을 활용해 달력, 시계를 만드는 것이 목표이다.

### 핵심 목표
* 가변저항, 스위치, LCD를 활용해 달력, 시계의 값을 정한다.
* LCD에 연 월 일 시 분 초 밀리초 형태로 출력
* sw2를 누르면 시간이 흐름

---

## 2. 개발 환경 (Environment)

| 항목 | 내용 |
| :--- | :--- |
| **MCU** | ATmega128A (16MHz External Crystal) |
| **IDE / Compiler** | Microchip Studio 7.0 / Microchip AVR GCC |
| **Flasher Tool** | USBISP / STK500 |
| **언어** | C Language |
| **주요 부품** | ATmega128 개발보드, ISP , LCD, 택트 스위치|

---

## 3. 하드웨어 구성 및 핀 맵 (Hardware Structure)

### Pin Configuration

```text
[ATmega128]                 [Target Component]
------------------------------------------------------------
[PORTD] -----> Button Input (SW1, SW2)
  PD2 (SW1) : 값 확정 버튼 (외부 풀업, 누르면 LOW)
  PD3 (SW2) : 시간 시작 버튼 (외부 풀업, 누르면 LOW)
  ※ PD0(SCL)/PD1(SDA)은 I2C 전용이라 건드리지 않음

[PORTF] -----> Analog Input
  PF0 (ADC0) : 가변저항 (연도/월/일/시/분/초 값 설정용)

[I2C (PD0/PD1)] -----> LCD Display
  SCL/SDA : I2C
```

### 주요 회로 특징
* **전원:** 5V DC 안정화 전원 공급
* **uart** uart1에 연결할 것
* **주의사항:** 

---

## 4. 프로젝트 구조 (Directory Structure)
> 구현부(.c), 선언부(.h)만 구조에 표기함.
```text
├── task2
│   ├── README.md
│   ├── task2
│   │   ├── i2c.c
│   │   ├── i2c.h
│   │   ├── i2c_lcd.c
│   │   ├── i2c_lcd.h
│   │   ├── main.c
│   │   └── task2.cproj
│   └── task2.atsln
```

---

## 5. 핵심 코드 및 레지스터 설정 (Key Implementation)

### UART 초기화 예시
```c
#include <avr/io.h>
#include <avr/interrupt.h>
#include <util/delay.h>
#include <stdint.h>
#include <stdio.h>

#include "i2c.h"
#include "i2c_lcd.h"

    //ADC 설정
	ADMUX  = (1 << REFS0);                                  // AVCC 기준, ADC0(PF0)
    ADCSRA = (1 << ADEN)
           | (1 << ADPS2) | (1 << ADPS1) | (1 << ADPS0);
    
    //timer 설정
    TCCR1A = 0x00;
    TCCR1B = (1 << WGM12) | (1 << CS11) | (1 << CS10);      // CTC, 분주비 64
    OCR1A  = 2499;                                          // 10ms
    TCNT1  = 0;
    TIMSK |= (1 << OCIE1A);
```

---

## 6. 동작 설명 및 결과 (Results)

### 동작 시나리오
1. 가변저항 값과 swith1을 이요해 날짜, 시간 설정한다
2. LCD에 연 월 일 시 분 초 밀리초 형태로 출력한다
3. switch2를 누르면 시간이 흐른다
4. 윤년, 달마다 일수가 다른 예외를 처리해야 한다.
### 동작 사진 / 영상

| 정면 동작 모습 | 센서 측정 및 시리얼 출력 |
| :---: | :---: |
[Hardware Setup](https://drive.google.com/file/d/1alQWbD4x8l8y9jwWoNuvbab50tPLULFI/view?usp=drive_link)
---

## 7. AI 툴 활용 명시 (AI Tools Declaration)
본 과제 작성 및 구현 과정에서 활용한 AI 도구(Generative AI)의 사용 현황 및 목적은 다음과 같음.

| 도구명 (Tool) | 활용 영역 | 세부 사용 목적 및 내용 |
| :--- | :--- | :--- |
| **Claude** | 코드 디버깅 | - 빌드 에러 및 문법 오류 원인 분석<br>- 레지스터 설정 주석 작성|



### AI 활용 및 검증 원칙
1. **코드 검증:** AI가 생성한 레지스터 설정 및 함수 코드는 데이터시트(ATmega128 Datasheet)와 비교 검증한 후 실제 시리얼 모니터링을 거쳐 직접 수정 및 테스트하였습니다.
2. **학습 주도성:** enum을 사용하면 일일이 define하지 않고도 상태 변수를 설정할 수 있다는 것을 LLM의 예제를 통해 알 수 있었다.
