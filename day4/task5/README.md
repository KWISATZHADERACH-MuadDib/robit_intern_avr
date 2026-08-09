# ATmega128 과제 및 프로젝트 템플릿

> **광운대학교 (로봇학부)**  
> **작성자:** (예희원)
> **제출일:** (8/2)

---

## 1. 개요 (Overview)
본 과제는 ATmega128 마이크로컨트롤러로 서브모터를 PWM제어 하는것이 이 과제의 목표이다.

### 핵심 목표
* UART 시리얼 통신으로 목표 각도를 입력받는다
* 목표 각도 만큼 서브모터를 움직인다
* 시스템 초기화시 90도로 복귀한다
* 유효 범위(180도)를 벗어나면 예외처리(움직이지 않음)한다, 경고 메세지 출력
---

## 2. 개발 환경 (Environment)

| 항목 | 내용 |
| :--- | :--- |
| **MCU** | ATmega128A (16MHz External Crystal) |
| **IDE / Compiler** | Microchip Studio 7.0 / Microchip AVR GCC |
| **Flasher Tool** | USBISP / STK500 |
| **언어** | C Language |
| **주요 부품** | ATmega128 개발보드, ISP, SG90|

---

## 3. 하드웨어 구성 및 핀 맵 (Hardware Structure)

### Pin Configuration

```text
[ATmega128]                 [Target Component]
------------------------------------------------------------
[PORTB] -----> Servo Motor PWM Output
  PB7 (OC1C) : SG90 서보모터 제어 신호 (Timer1 Fast PWM)

[PORTE] -----> UART0 Serial Communication
  PE0 (RXD0) : UART0 Serial Receive (from PC)
  PE1 (TXD0) : UART0 Serial Transmit (to PC)
  PE2        : MAX485 Direction Control (계속 HIGH 유지, PE0을 PC 입력용으로 확보)
```

### 주요 회로 특징
* **전원:** 5V DC 안정화 전원 공급
* **uart** uart1에 연결할 것
* **주의사항:** 0도에서 180도 사이의 값을 입력하지 않으면 오류 메세지 출력됨

---

## 4. 프로젝트 구조 (Directory Structure)
> 구현부(.c), 선언부(.h)만 구조에 표기함.
```text
└── task5
    ├── task5
    │   ├── main.c
    │   └── task5.cproj
    └── task5.atsln
```

---

## 5. 핵심 코드 및 레지스터 설정 (Key Implementation)

### UART 초기화 예시
```c
#include <avr/io.h>
#include <avr/interrupt.h>
#include <util/delay.h>
#include <stdint.h>

    //SERVO
    DDRB |= (1 << PB7);                                     // OC1C 출력

    TCCR1A = (1 << COM1C1) | (1 << WGM11);                  // 채널C 비반전, Fast PWM
    ICR1   = 39999;                                         // TOP 먼저 설정 (20ms)
    TCCR1B = (1 << WGM13) | (1 << WGM12) | (1 << CS11);

    //UART 
    DDRE  |=  (1 << PE1);        // TXD0 출력
    DDRE  &= ~(1 << PE0);        // RXD0 입력
    PORTE |=  (1 << PE0);        // 풀업 (플로팅 노이즈 방지)

    DDRE  |= (1 << DIR_PIN);     // MAX485 방향제어 출력
    DIR_TX();                    // 계속 HIGH 유지 (위 주석 참고 - PE0을 놓아주기 위함)

    UBRR0H = (uint8_t)(UBRR0_VAL >> 8);
    UBRR0L = (uint8_t)(UBRR0_VAL);

    UCSR0A = (1 << U2X0);                                   // 2배속 (보레이트 오차 감소)
    UCSR0C = (1 << UCSZ01) | (1 << UCSZ00);                 // 8bit, 패리티 없음, 스톱 1
    UCSR0B = (1 << TXEN0) | (1 << RXEN0) | (1 << RXCIE0);   // 송신 / 수신 / 수신인터럽트


```

---

## 6. 동작 설명 및 결과 (Results)

### 동작 시나리오
1. UART 시리얼 통신으로 목표 각도를 입력받는다.
2. 목표 각도로 서브모터를 이동시킨다.
3. 시스템 초기화 시 90도로 복귀한다
4. 0~180도 사이 값이 아닌 값이 입력되면 오류 메세지 출력 및 예외처리
### 동작 사진 / 영상

| 정면 동작 모습 | 센서 측정 및 시리얼 출력 |
| :---: | :---: |
[Hardware Setup](https://drive.google.com/file/d/19Xcxvma7yjBmWrMgGbSN5lDn8_j-bpLN/view?usp=drive_link)
---

## 7. AI 툴 활용 명시 (AI Tools Declaration)
본 과제 작성 및 구현 과정에서 활용한 AI 도구(Generative AI)의 사용 현황 및 목적은 다음과 같음.

| 도구명 (Tool) | 활용 영역 | 세부 사용 목적 및 내용 |
| :--- | :--- | :--- |
| **Claude** | 코드 디버깅 | - 빌드 에러 및 문법 오류 원인 분석<br>- 레지스터 설정 주석 작성|



### AI 활용 및 검증 원칙
1. **코드 검증:** AI가 생성한 레지스터 설정 및 함수 코드는 데이터시트(ATmega128 Datasheet)와 비교 검증한 후 실제 시리얼 모니터링을 거쳐 직접 수정 및 테스트하였습니다.
2. **학습 주도성:** 170 같은 숫자를 입력받으면 %10을 해서 1의자리부터 100의자리까지의 값을 한자리씩 받게 되는데 이렇게 값을 받으면 0, 7, 1순으로 거꾸로 받기 때문에 배열의 인덱스를 뒤에서부터 값을 읽어 입력받은 숫자의 값을 알 수 있다. (한번에 한글자밖에 보내지 못하기 때문에 이런 방식을 사용해 값을 전송한다.)
