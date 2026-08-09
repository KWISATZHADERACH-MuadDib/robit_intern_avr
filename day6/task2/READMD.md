# ATmega128 과제 및 프로젝트 템플릿

> **광운대학교 (로봇학부)**  
> **작성자:** (예희원)
> **제출일:** (8/9)

---

## 1. 개요 (Overview)
본 과제는 ATmega128 마이크로컨트롤러와 모터드라이버로 모터를 구동시키는 과제이다

### 핵심 목표
* 모터 드라이버로 모터를 구동시킨다.
---

## 2. 개발 환경 (Environment)

| 항목 | 내용 |
| :--- | :--- |
| **MCU** | ATmega128A (16MHz External Crystal) |
| **IDE / Compiler** | Microchip Studio 7.0 / Microchip AVR GCC |
| **Flasher Tool** | USBISP / STK500 |
| **언어** | C Language |
| **주요 부품** | ATmega128 개발보드, 모터 드라이버, 모터|

---

## 3. 하드웨어 구성 및 핀 맵 (Hardware Structure)

### Pin Configuration

```text
[ATmega128]                 [Target Component]
------------------------------------------------------------
[PORTB] -----> Motor Driver (L298N)
  PB0 : IN1 (모터 A 방향 제어1) -> HIGH 고정
  PB1 : IN2 (모터 A 방향 제어2) -> LOW 고정
  PB2 : IN3 (모터 B 방향 제어1) -> HIGH 고정
  PB3 : IN4 (모터 B 방향 제어2) -> LOW 고정
  PB5 : ENA (모터 A 인에이블/속도) -> HIGH 고정
  PB6 : ENB (모터 B 인에이블/속도) -> HIGH 고정
```

### 주요 회로 특징
* **전원:** 12V DC 안정화 전원 공급
* **주의사항:** 

---

## 4. 프로젝트 구조 (Directory Structure)
> 구현부(.c), 선언부(.h)만 구조에 표기함.
```text
├── task2
│   ├── README.md
│   ├── task2
│   │   ├── main.c
│   └── task2.atsln
```

---

## 5. 핵심 코드 및 레지스터 설정 (Key Implementation)

### UART 초기화 예시
```c
#include <avr/io.h>

    //모터 설정
    // PORTB 0~3 : IN1, IN2, IN3, IN4
    // PORTB 5,6 : ENA, ENB
    DDRB |= (1 << PB0) | (1 << PB1) | (1 << PB2) | (1 << PB3) | (1 << PB5) | (1 << PB6);
```

---

## 6. 동작 설명 및 결과 (Results)

### 동작 시나리오
1. 2개의 모터가 동작한다.
### 동작 사진 / 영상

| 정면 동작 모습 | 센서 측정 및 시리얼 출력 |
| :---: | :---: |
[Hardware Setup](https://drive.google.com/file/d/1iOq8W1MERKFv7QbZ0HPagXduAIW2AxtZ/view?usp=drive_link)
---

## 7. AI 툴 활용 명시 (AI Tools Declaration)
본 과제 작성 및 구현 과정에서 활용한 AI 도구(Generative AI)의 사용 현황 및 목적은 다음과 같음.

| 도구명 (Tool) | 활용 영역 | 세부 사용 목적 및 내용 |
| :--- | :--- | :--- |
| **Claude** | 코드 디버깅 |



### AI 활용 및 검증 원칙
1. **코드 검증:** 
2. **학습 주도성:** 
