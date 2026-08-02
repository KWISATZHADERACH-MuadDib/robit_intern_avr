TIMER 0/2 보고서
====================
## TIMER란
MCU의 내부 클럭을 이용하여 일정시간 간격의 펄스를 만들어 내거나 일정시간 경과 후에 인터럽트를 발생시키는 기능을 한다.<br>
TIMER에는 TIMER0, TIMER1, TIMER2, TIMER3이 있는데 0,2는 8bit, 1,3은 16bit TIMER이다.
## TIMER interrupt
### 인터럽트의 종류 :
1. overflow interrupt<br>
+ Overflow가 발생할 때 인터럽트 발생
2. compare match interrupt
+ TCNTn이 일정값에 도달했을 때 인터럽트 발생
3. capture event interrupt
## TIMER interrupt register
1. TIMSK(Interrupt Mask Register)
+ Bit 0 - TOIEn(Overflow Interrupt Enable)<br>
-> TOIEN = 1 : Overflow Interrupt 활성화
2. TCCRn(Control Register)<br>
+ Bit 2:0 - COS2, COS1, COS0 : 분주비 선택<br>
+ Bit 6:3 - WGMO1, WGMO0 : 출력 모드 설정(파형 생성 모드 선택(CTC, OVF등)
+ Bit 5:4 - COMO1, COMO0 : 출력 모드 설정(PWM 모드 아닐 경우)
3. TCNTn (TIMER/COUNTER Register)
+ TIMER / COUTNER 0의 8bit 값을 저장하고 있는 ㅈ레지스터
+ TIMER / COUNTER 0의 값이 자동으로 갱신된다.
## TIMER 0/2 동작 방법
1. 카운트하는 입력 펄스 갯수를 TCNTn에 기록한다
2. TCNTn이 255일 때 입력 펄스가 추가되면 TCNTn이 0으로 기록된다.
3. 0xFF에서 0x00으로 넘어갈 때 오버플로우 인터럽트가 발생한다.
## TIMER OVF interrupt 계산 방법
1. TCNT가 증가하는 시간 변경
+ TCNT가 증가하는 시간 변경(Prescale 변경)<br>
TIMER OVF 주기 = Tick * n(256)
2. TCNT의 초기값 변경<br>
+ TIMER OCV 주기 = 1/16000000hz * (n - x)
# TIMER 0/2의 특징
+ OVF interrupt 지원
+ compare match interrupt 지원(OCR)
+ PWM 모드 지원
+ 자체 프리스케일러 지원
## TIMER 0의 특징
+ 클럭 소스: 시스템 클럭(내부) 또는 외부 클럭(T0핀, 비동깉X)
+ 관련 레지스터 : TCCR0, TCNT0, OCR0, TIMSK, TIFR
+ PWM 출력 핀 : OC0(PB4)
+ 일반적으로 간단한 타이밍, PWM 생성용으로 사용한다.
## TIMER 2의 특징
+ 비동기 동작 지원
+ 별도의 32.768kHz 워치 크리스탈을 TOSC1, TOSC2 핀에 연결해서 독립적으로 동작이 가능하다.
+ 관련 레지스터 : TCCR2, TCNT2, OCCR2, ASSR(비동기 상태 레지스터)<br>
-> 비동기 클럭에서 TCNT2, OCR2, TCCR2와 간은 값을 사용하게 되면 비동기 도메인에 반영되기까지 시간지연이 발생한다.  이때 레지스터를 건드리면 오류가 발생할 수 있어서 기다리는 상태를 알려주기 위해 사용한다.
+ PWM 출력 핀 : OC2(PB7)
+ 전전 모드에서도 동작이 가능하다.