# ATmega128 과제 및 프로젝트 템플릿

> **광운대학교 (로봇학부)**  
> **작성자:** (예희원)
> **제출일:** (07/30)

---

## 1. 개요 (Overview)
본 과제는 ATmega128 마이크로컨트롤러를 활용하여 주요 주변장치(Peripherals)를 제어하고 센서 데이터를 수신/처리하는 시스템을 구현하는 것을 목표로 함.

### 핵심 목표
* ATmega128 인터럽트 사용과 LED 사용

---

## 2. 개발 환경 (Environment)

| 항목 | 내용 |
| :--- | :--- |
| **MCU** | ATmega128A (16MHz External Crystal) |
| **IDE / Compiler** | Microchip Studio 7.0 / Microchip AVR GCC |
| **Flasher Tool** | USBISP / STK500 |
| **언어** | C Language |
| **주요 부품** | ATmega128 개발보드, LED, 택트 스위치 |

---

## 3. 하드웨어 구성 및 핀 맵 (Hardware Structure)

### Pin Configuration

```text
[ATmega128]                 [Target Component]
 PORTA (PA0 ~ PA7)   ----->   8-Bit LED
 PORTD (PD2 ~ PD3)   ----->   switch 1, 2
 PORTE (PE4 ~ PE5)   ----->   switch 3, 4
```

### 주요 회로 특징
* **전원:** 5V DC 안정화 전원 공급

---

## 4. 프로젝트 구조 (Directory Structure)
> 구현부(.c), 선언부(.h)만 구조에 표기함.
```text
├── Day01_Task02/
│   ├── main.c # 메인 제어 루프 및 시스템 초기화
└── README.md
```

---

## 5. 핵심 코드 및 레지스터 설정 (Key Implementation)

### LED, 인터럽트 포트 초기설정
```c
#define F_CPU 16000000
#include <avr/io.h>
#include <avr/interrupt.h>
#include <util/delay.h>

int main(void)
{
	DDRA = 0xFF;   // PORTA 전체를 출력으로 설정 
	PORTA = 0b11111111;

	DDRD = 0b11110011;   // SW1(PD2), SW2(PD3) 입력으로 설정
	PORTD |= (1 << PD2) | (1 << PD3);     // 내부 풀업 저항 켜기 

	DDRE = 0b11001111;   // PE4, PE5 입력으로 설정
	PORTE |= (1 << PE4) | (1 << PE5);     // 내부 풀업 저항 켜기

	EICRB |= (1 << ISC41);    // INT4 : falling edge 발생
	EICRB |= (0 << ISC40);

	EICRB |= (1 << ISC51);    // INT5 : falling edge 발생
	EICRB |= (0 << ISC50);

	EIMSK |= (1 << INT4) | (1 << INT5);
```

---

## 6. 동작 설명 및 결과 (Results)

### 동작 시나리오
0. LED 8개가 0.5초 간격으로 깜빡거린다
1. 스위치 1번 누를 경우 -> LED 4 ~ 7번 켜짐
2. 스위치 2번 누를 경우 -> LED 0 ~ 3번 켜짐
3. 스위치 3번 누를 경우 -> 인터럽트 4 발생 (맨 오른쪽 LED 하나 켜고 LED를 외쪽 끝까지 한칸씩 이동)
4. 스위치 4번 누를 경우 -> 인터럽트 5 발생 (맨 왼쪽 LED 하나 켜고 LED를 오른쪽 끝까지 한칸씩 이동)

### 동작 사진 / 영상

| 정면 동작 모습 | 센서 측정 및 시리얼 출력 |
| :---: | :---: |
| ![Hardware Setup](https://drive.google.com/file/d/1-9JamMKYEpO-mjkyjQ5o8wJx26HpLqxb/view?usp=drive_link)

---

## 7. AI 툴 활용 명시 (AI Tools Declaration)
본 과제 작성 및 구현 과정에서 활용한 AI 도구(Generative AI)의 사용 현황 및 목적은 다음과 같음.

| 도구명 (Tool) | 활용 영역 | 세부 사용 목적 및 내용 |
| :--- | :--- | :--- |
| Claude | 버그 수정 | led >> 1을 했을 때 led가 하나만 켜지는게 아닌 누적되어 켜지는 문제가 발생해 해결책을 물어봤고, | 연산자로 누적된 값을 없애주면 된다는 해답을 얻음 |

### AI 활용 및 검증 원칙
1. **코드 검증:** AI가 생성한 레지스터 설정 및 함수 코드는 데이터시트(ATmega128 Datasheet)와 비교 검증한 후 실제 오실로스코프/시리얼 모니터링을 거쳐 직접 수정 및 테스트하였습니다.
2. **학습 주도성:** 코드의 핵심 제어 로직 설계는 직접 작성하였으며, AI는 보조 도구(디버깅, 문서화)로만 활용하였습니다.
