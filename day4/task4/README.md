# ATmega128 과제 및 프로젝트 템플릿

> **광운대학교 (로봇학부)**  
> **작성자:** (예희원)
> **제출일:** (8/2)

---

## 1. 개요 (Overview)
본 과제는 과제 3에 이어 ATmega128 마이크로컨트롤러로 PSD를 제어해 거리를 측정하고 필터를 이용해 값을 보정하고 사용한 필터에 대해 보고서를 작성하는 것이 목표이다.

### 핵심 목표
* PSD를 사용해서 거리 측정하기
* 측정한 PSD값을 필터를 이용해 보정한다. 
* 측정된 거리와 필터링한 값을 UART로 출력

---

## 2. 개발 환경 (Environment)

| 항목 | 내용 |
| :--- | :--- |
| **MCU** | ATmega128A (16MHz External Crystal) |
| **IDE / Compiler** | Microchip Studio 7.0 / Microchip AVR GCC |
| **Flasher Tool** | USBISP / STK500 |
| **언어** | C Language |
| **주요 부품** | ATmega128 개발보드, ISP, PSD|

---

## 3. 하드웨어 구성 및 핀 맵 (Hardware Structure)

### Pin Configuration

```text
[ATmega128]                 [Target Component]
------------------------------------------------------------
[PORTE] -----> UART0 Serial Communication
  PE0 (RXD0) : UART0 Serial Receive
  PE1 (TXD0) : UART0 Serial Transmit (to PC)
  PE2        : MAX485 Direction Control (LOW 고정, 485 버스로 송신 안 함)

[PORTF] -----> Analog Sensor Input
  PF1 (ADC1) : PSD 거리 센서 (SHARP GP2Y0A02YK0F)
```

### 주요 회로 특징
* **전원:** 5V DC 안정화 전원 공급
* **uart** uart1에 연결할 것
* **주의사항:** 물체가 PSD와 너무 가까우면 오히려 PSD값이 감소할 수 있음

---

## 4. 프로젝트 구조 (Directory Structure)
> 구현부(.c), 선언부(.h)만 구조에 표기함.
```text
├── task3
│   ├── task3
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
#include <avr/pgmspace.h>
#include <util/delay.h>
#include <stdint.h>
#include <stdio.h>

    //UART
    DDRE |=  (1 << PE1);        // TXD0 출력
    DDRE &= ~(1 << PE0);        // RXD0 입력
    PORTE |= (1 << PE0);        // 풀업 (플로팅 방지)

    DDRE  |=  (1 << DIR_PIN);   // MAX485 방향제어 출력
    PORTE &= ~(1 << DIR_PIN);   // LOW = 485 송신 비활성 (PC로만 나간다)

    UBRR0H = (uint8_t)(UBRR0_VAL >> 8);
    UBRR0L = (uint8_t)(UBRR0_VAL);

    UCSR0A = (1 << U2X0);
    UCSR0C = (1 << UCSZ01) | (1 << UCSZ00);     // 8-N-1
    UCSR0B = (1 << TXEN0);  
    //PSD
      // PF1을 입력으로, 내부 풀업 OFF
    DDRF  &= ~(1 << PF1);
    PORTF &= ~(1 << PF1);

    ADMUX  = (1 << REFS0) | (PSD_ADC_CH & 0x1F);        // AVCC 기준, ADC1
    ADCSRA = (1 << ADEN)
           | (1 << ADPS2) | (1 << ADPS1) | (1 << ADPS0);    // 128분주 -> 125kHz

```

---

## 6. 동작 설명 및 결과 (Results)

### 동작 시나리오
1. ADC(PROTF)를 이용햇서 PSD 센서 값을 읽는다
2. ADC 변환 값을 센서 특성에 맞는 거리(cm)단위로 환산한다.
3. 거리를 중앙값 필터를 활용해 필터링한다. 
4. 계산된 거리 데이터와 필터링된 값을 Uart로 PC 시리널 터미널에서 출력한다.
5. 측정 주기 설정 및 비정상 센서 데이터는 예외처리한다.(너무 가깝거나, 너무 멀 때)
### 동작 사진 / 영상

| 정면 동작 모습 | 센서 측정 및 시리얼 출력 |
| :---: | :---: |
| ![Hardware Setup](https://drive.google.com/file/d/1ocvOF0c98N4diq7-IMNoNuLkL375ujSl/view?usp=drive_link)
---

## 7. AI 툴 활용 명시 (AI Tools Declaration)
본 과제 작성 및 구현 과정에서 활용한 AI 도구(Generative AI)의 사용 현황 및 목적은 다음과 같음.

| 도구명 (Tool) | 활용 영역 | 세부 사용 목적 및 내용 |
| :--- | :--- | :--- |
| **Claude** | 코드 디버깅 | - 빌드 에러 및 문법 오류 원인 분석<br>- 레지스터 설정 주석 작성|



### AI 활용 및 검증 원칙
1. **코드 검증:** AI가 생성한 레지스터 설정 및 함수 코드는 데이터시트(ATmega128 Datasheet)와 비교 검증한 후 실제 시리얼 모니터링을 거쳐 직접 수정 및 테스트하였습니다.
2. **학습 주도성:** PSD가 튀는 값을 줄이기 위해 샘플들의 중앙값을 사용해 튀는 값을 걸러 낼 수 있다.