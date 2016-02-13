
#include "defines.h"   // get definitions of ANx ports

//#include "RCOutput.h"
#define CH_1 0  // LEFT_MOT_CH = 0
#define CH_2 1  // RIGHT_MOT_CH = 1

#define FORWARD  0
#define BACKWARD 1

void setup() {

  /* setup serial */
  Serial.begin(9600);
  
  // setup pwm ports:
//  KAL_init_pwm();

}

void loop() {

  int dir, pwidth;

  Serial.print("Enter the positive pulse width : ");
  pwidth = KAL_read_serial_int16();
  
  Serial.print("You entered : ");
  Serial.print(pwidth);
  Serial.println();

  if (pwidth < 0) {
    pwidth *= -1;
    dir = BACKWARD;
  }
  else {
    dir = FORWARD;
  }
  
//  KAL_enable_ch(CH_1);
//  KAL_enable_ch(CH_2);

   

}

int KAL_read_serial_int16()
{
  int value = 0, sign = 1;

  /* wait until something to parse */
  while (Serial.available() == 0);

  /* ok, now parse */
  while (Serial.available()) {
    char c = Serial.read();
    if ( isDigit(c) ) 
      value = value * 10 + (uint16_t)(c - '0');
    else if ( c == '-')
      sign = -1;
    else if ( c == 10 )
      break;
  }
  return(sign * value);
}




/**********************************************/
/* KAL_init_pwm()                             */
/* initialize PWM Timer registers for timer 1 */
/* which drives PB6/OC1B and PB5/OC1A         */
/**********************************************/
void KAL_init_pwm() 
{
  pinMode(12, OUTPUT);  // Left CH_1 (PB6 / OC1B) - Digital pin 12 on Arduino mega
  pinMode(11, OUTPUT);  // Right CH_2 (PB5 / OC1A) - Digital pin 11 on Arduino mega

// Timer 1 is a 16 bit timer
//
// TCCRxA:  COMxA1, COMxA0, COMxB1, COMxB0, -,      -,    WGM11,  WGM10
// TCCRxB:  ICNCx,  ICESx,  -,      WGM13,  WGM12,  CS12, CS11,   CS10
//
// Timer modes from waveform generation bits: WGM13, WGM12, WGM11, WGM10
// 1, 1, 1, 0: Fast PWM, Top=ICR1, Update OCR1x at BOTTOM, TOV1 Flag set on TOP
//
// Clock select and timer frequency from CS12, CS11, CS10
// 0, 1, 0: system clock / 8 (prescaler
//
// 16 MHz system clock, 8 pre-scale -> 2 MHz (0.5us), TOP=40000 -> 20ms (50 Hz)
//

  TCCR1A = (1<<WGM11);                        // Timer Control Register A
  TCCR1B = (1<<WGM13)|(1<<WGM12)|(1<<CS11);   // Timer Control Register B

  ICR1 = 40000;   // Input Capture Register 1 (used for TOP in WGM mode 14)

// Output Compare Registers
// set to maximum 16 bit value
  OCR1A = 0xFFFF; 
  OCR1B = 0xFFFF;

  return;
}



/**********************************************/
/* KAL_enable_ch()                            */
/* activate PWM Timer output                  */
/* which drives PB6/OC1B and PB5/OC1A         */
/**********************************************/
void KAL_enable_ch (uint8_t ch)
{
// [COMxA1, COMxA0] 
//    0, 0 : OCxA disconnected
//    1, 0 : clear OCxA on compare match, set OCxA at BOTTOM (non-inverting mode)
// [COMxB1, COMxB0]
//    0, 0 : OCxB disconnected
//    1, 0 : clear OCxB on compare match, set OCxB at BOTTOM (non-inverting mode)
//
    switch (ch) {
    case 0: 
      TCCR1A |= (1 << COM1B1);  // CH_1 : OC1B
      break;
    case 1:
      TCCR1A |= (1 << COM1A1);  // CH_2 : OC1A
      break;
    default:
      Serial.print("Fell into default case in enable_ch()");
      break;
  }
  return; 
}

void write_pwm(uint8_t ch, uint16_t period_us)
{
  uint16_t pwm = period_us << 1;

  switch (ch) {
    case 0: 
      OCR1B = pwm;
      break;
    case 1:
      OCR1A = pwm;
      break;
    default:
      Serial.print("Fell into default case in write_pwm()");
      break;
  }
  return; 
}

//static inline uint16_t constrain_period(uint16_t p)
//{
//    if (p > RC_INPUT_MAX_PULSEWIDTH) return RC_INPUT_MAX_PULSEWIDTH;
//    if (p > RC_INPUT_MIN_PULSEWIDTH) return RC_INPUT_MIN_PULSEWIDTH;
//    return p
//}

