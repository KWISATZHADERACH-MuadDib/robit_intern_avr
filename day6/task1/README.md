# ATmega128 과제 및 프로젝트 템플릿

> **광운대학교 (로봇학부)**  
> **작성자:** (예희원)
> **제출일:** (8/9)

---

## 1. 개요 (Overview)
본 과제는 ATmega128 마이크로컨트롤러의 usart, IR센서, LED를 활용하여 IR 필터를 구현하는 과제이다.

### 핵심 목표
* IR 센서값을 이용하여 UART를 통해 터미널에 각각의 값을 띄운다.
 (IR센서 6개의 원본값, 필터를 적용한 값, min값, max값, 정규화값)
* IR센서로 입력된 값을 필터로 걸러내고 정규화한 값이 0.8이상일 경우 같은 번호의 LED를 켜고 그 이하일 때는 끈다.
* LCD에 IR센서 6개의 정규화된 값을 띄운다.

---

## 2. 개발 환경 (Environment)

| 항목 | 내용 |
| :--- | :--- |
| **MCU** | ATmega128A (16MHz External Crystal) |
| **IDE / Compiler** | Microchip Studio 7.0 / Microchip AVR GCC |
| **Flasher Tool** | USBISP / STK500 |
| **언어** | C Language |
| **주요 부품** | ATmega128 개발보드, ISP , LCD, LED, IR센서|

---

## 3. 하드웨어 구성 및 핀 맵 (Hardware Structure)

### Pin Configuration

```text
[ATmega128]                 [Target Component]
------------------------------------------------------------
[PORTF] -----> Analog Input (IR 수광부 6채널)
  PF2 (ADC2) : IR 0
  PF3 (ADC3) : IR 1
  PF4 (ADC4) : IR 2
  PF5 (ADC5) : IR 3
  PF6 (ADC6) : IR 4
  PF7 (ADC7) : IR 5

[PORTA] -----> LED Output (IR 번호와 1:1 대응)
  PA0 : LED 0    PA3 : LED 3
  PA1 : LED 1    PA4 : LED 4
  PA2 : LED 2    PA5 : LED 5

[PORTD] -----> Button Input (내부 풀업, 누르면 LOW)
  PD2 (SW1) : 캘리브레이션 시작 / 재시작 (min·max 초기화 후 학습)
  PD3 (SW2) : 캘리브레이션 종료 -> 출력 모드 진입 (min·max 고정)

[PORTD] -----> I2C (TWI) LCD Display
  PD0 (SCL) : I2C 클럭
  PD1 (SDA) : I2C 데이터

[PORTE] -----> UART Debug (USART0)
  PE0 (RXD0) : 수신
  PE1 (TXD0) : 송신
```

### 주요 회로 특징
* **전원:** 5V DC 안정화 전원 공급
* **uart** uart0에 연결할 것
* **주의사항:** 

---

## 4. 프로젝트 구조 (Directory Structure)
> 구현부(.c), 선언부(.h)만 구조에 표기함.
```text
├── task1
│   ├── README.md
│   ├── task1
│   │   ├── i2c.c
│   │   ├── i2c.h
│   │   ├── i2c_lcd.c
│   │   ├── i2c_lcd.h
│   │   ├── main.c
│   │   └── task1.cproj
│   └── task1.atsln
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

    //uart 설정
    uint16_t ubrr = (uint16_t)((F_CPU / (16UL * UART_BAUD)) - 1);

	UBRR0H = (uint8_t)(ubrr >> 8);
	UBRR0L = (uint8_t)(ubrr & 0xFF);
	UCSR0B = (1 << TXEN0) | (1 << RXEN0);
	UCSR0C = (1 << UCSZ01) | (1 << UCSZ00);      // 8-N-1

	stdout = &uart0_stdout;

    //adc 설정
    DDRF  &= (uint8_t)~(0xFF << IR_ADC_START);   // PF2~PF7 입력
	PORTF &= (uint8_t)~(0xFF << IR_ADC_START);   // 내부 풀업 off

	ADMUX  = (1 << REFS0);                       // 기준전압 AVCC
	ADCSRA = (1 << ADEN) | (1 << ADPS2) | (1 << ADPS1) | (1 << ADPS0);

	ADCSRA |= (1 << ADSC);                       // 더미 변환
	while (ADCSRA & (1 << ADSC));

    //버튼 설정
    // PD2, PD3 입력 + 내부 풀업. PD0/PD1 은 I2C 가 쓰므로 건드리지 않는다.
	DDRD  &= (uint8_t)~BTN_MASK;
	PORTD |= BTN_MASK;

    //led 설정
    LED_DDR |= LED_MASK;
	#if LED_ACTIVE_LOW
	LED_PORT |= LED_MASK;
	#else
	LED_PORT &= (uint8_t)~LED_MASK;
	#endif
```

---

## 6. 동작 설명 및 결과 (Results)

### 동작 시나리오
1. 버튼 1을 누르면 캘리브레이션을 시작한다.
2. 버튼 2를 누르면 IR 센서값을 이용해 터미널에 각각의 값을 띄우고 정규화된 값이 0.8이상일 경우 같은 번호의 LED가 켜진다. 그리고 LCD에 IR센서 6개의 정규화된 값을 띄운다.
### 동작 사진 / 영상

| 정면 동작 모습 | 센서 측정 및 시리얼 출력 |
| :---: | :---: |
[Hardware Setup](https://drive.google.com/file/d/15K5N_oKLo1qoeHLa0qSWhyuX07KBJ0n5/view?usp=sharing)
---

## 7. AI 툴 활용 명시 (AI Tools Declaration)
본 과제 작성 및 구현 과정에서 활용한 AI 도구(Generative AI)의 사용 현황 및 목적은 다음과 같음.

| 도구명 (Tool) | 활용 영역 | 세부 사용 목적 및 내용 |
| :--- | :--- | :--- |
| **Claude** | 코드 디버깅 | - 빌드 에러 및 문법 오류 원인 분석<br>- 레지스터 설정 주석 작성|



### AI 활용 및 검증 원칙
1. **코드 검증:** AI가 생성한 레지스터 설정 및 함수 코드는 데이터시트(ATmega128 Datasheet)와 비교 검증한 후 실제 시리얼 모니터링을 거쳐 직접 수정 및 테스트하였습니다.
2. **학습 주도성:** IR센서 필터를 사용할 때 중앙값 필터를 잘 사용하지 않는 이유가 IR센서는 크게 튀는 값이 나오지 않고 값이 일정하게 변하기 때문이라는 것을 알게되었습니다. 그래서 IR센서 필터로 평균값 필터를 사용했습니다.
