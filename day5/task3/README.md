Register 설정
==========
TIMER / COUNTER 1 관련 Register
+ TCCR1A, TCCR1B, TCCR1C
+ TCNT1H, TCNT1L
+ OCR1AH, OCR1AL, OCR1BL, OCR1CH, OCR1CL
+ ICR1H, ICR1L
+ SFIOR
+ TIMSK, ETIMSK, TIFR, ETIFR

## Register 설정 설명
### TCCR1A, TCCR1B, TCCR1C

#### TCCR1A
비트:   7      6      5      4      3      2     1     0
      COM1A1 COM1A0 COM1B1 COM1B0 COM1C1 COM1C0 WGM11 WGM10
+ COM1n1, COM1n0 : 각 채널의 출력모드(비반전, 반전, 미연결)을 설정함
+ WGM11, WGM10 : 파형 생성 모드에 관련된 하위 2개 비트

#### TCCR1B
비트:   7      6      5    4     3     2     1     0
      ICNC1  ICES1   -   WGM13 WGM12  CS12  CS11  CS10
+ ICNC1 : 입력 캡쳐 노이즈를 제거한다. (노이즈 제거 회로가 작동하면 시스템 클럭의 4주기 만큼 지연되어 작동한다.)
+ ICES1 : input capture edge 선택 (0 = falling edge, 1 = rising edge)
+ WGM13, WGM12 : 파형생성 모드의 상위 2비트
+ CS12, CS11, CS10 : 클럭 분주비 선택

#### TCCR1C
비트:   7      6      5     4  3  2  1  0
      FOC1A  FOC1B  FOC1C   -  -  -  -  -
+ FOC1A, FOC1B, FOC1C : 강제로 출력을 비교한다.PWM 모드가 아닌 Normal/ CTC compare 모드에서 소프트웨어가 강제로 비교매치 이벤트를 한 번 발생시키고 싶을 때 사용한다. PWM 모드에서는 반드시 0으로 두어야 예상치 못한 펄스가 나가는 것을 막을 수 있다.

### TCNT1H, TCNT1L
+ Timer/Counter1의 16bit counter 값을 저장하고 있는 Register
+ AVR은 8비트씩만 주고 받지만 Timer/Counter1,3에서는 16비트를 사용한다. 그래서 TCNT1H는 상위비트를, TCNT1L은 하위 비트를 저장한다.

### OCR1xH, OCR1xL
+ TCNT1과 비교하여 OC1x 단자에 출력 신호를 발생하기 위한 16bit Register

### ICR1H, ICR1L
+ Input Capture 신호 ICP1에 의하여 TCNt1 값을 캡쳐하여 저장하는 16th Register
+ Fast PWM 모드에서 타이머의 TOP 값으로 설정될 수 있다.

### SFIOR
비트:   7    6    5    4    3     2      1       0
      TSM   -    -    -   PUD  PSR0  PSR321    -
+ PSR0 / PSR321 : <br>
PSR0 : Timer0의 프리스케일러만 리셋한다
PSR321 : Timer1, Timer2, Timer3이 공유하는 프리스케일러를 한번에 리셋한다.
+ TSM : 여러 타이머를 정확히 동시에 사작하게함
+ PUD : 풀업 저항 전체 스위치<br>
칩 전체의 풀업을 강제로 꺼버리는 마스터 스위치이다.

### TIMSK, ETIMSK, TIFR, ETIFR
#### TIMSK 
비트:   7     6     5      4      3      2     1     0
      OCIE2 TOIE2 TICIE1 OCIE1A OCIE1B TOIE1 OCIE0 TOIE0
+ TICIE1 : Timer1 입력캡처 인터럽트
+ OCIE1A, OCIE1B : Timer1 채널 A/B 비교매치 인터럽트
+ TOIE1 : Timer1 오버플로우 인터럽트

#### ETIMSH
비트:   7      6      5      4     3      2      1   0
      TICIE3 OCIE3A OCIE3B TOIE3 OCIE3C OCIE1C   -   -
+ TICIE3 : Timer3 입력캡처
+ OCIE3A, OCIE3B, OCIE3C : Timer3 채널 A/B/C 비교매치
+ TOIE3 : Timer3 오버플로우
+ OCIE1C : Timer 채널 C 비교매치

#### TIFR / ETIFR 
TIFR :  OCF2  TOV2  ICF1  OCF1A OCF1B TOV1  OCF0  TOV0 <br>
ETIFR:  ICF3  OCF3A OCF3B TOV3  OCF3C OCF1C  -     -
해당 이벤트가 일어나면 하드웨어가 자동처럼 이 비트를 1로 세운다.
