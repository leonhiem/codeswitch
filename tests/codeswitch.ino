// ZERO demo of IR remote xmit recv
//  version 2 use analogWrite() to set clock,timer,pin
//   Sony remote tests
//  TC5 50us timer and ISR and use any pin
//  pin 12 TCC0 ch3 PORTA 19 PIO_TIMER_ALT pwm(khz)  need PWM out pin
// variant.cpp  PORTA group 0
// 48m/38k  1263 < 16 bits
// can run both xmit/recv at once
// transmit with IR LED, recvr GP1UX311QS
// ref https://github.com/z3t0/Arduino-IRremote

// see: https://arduino-info.wikispaces.com/IR-RemoteControl



#include <RTCZero.h> // Include RTC library - make sure it's installed!
#include "IR_remote.h"
#include "IRremoteInt.h"
#include "pitches.h"


#define DEBUG 1

#define RECVPIN 7
#define PWMPIN 12
#define TCCx TCC0
#define TCCchannel 3

#define TCx TC3
#define TCx_Handler TC3_Handler
#define TCx_IRQn TC3_IRQn
#define TCchannel 0
#define TCGCLK GCM_TCC2_TC3

#define syncTCC while (TCCx->SYNCBUSY.reg & TCC_SYNCBUSY_MASK)
#define syncTC  while (TCx->COUNT16.STATUS.bit.SYNCBUSY)

/*
 * LEDs
 */
const int BLUE_LED = 13; // Blue "stat" LED on pin 13
const int RX_LED = PIN_LED_RXL; // RX LED on pin 25, we use the predefined PIN_LED_RXL to make sure
const int TX_LED = PIN_LED_TXL; // TX LED on pin 26, we use the predefined PIN_LED_TXL to make sure

bool ledState = LOW;

/*
 * RTC
 */
RTCZero rtc; // Create an RTC object
bool alarmTriggered = false;

/*
 * http://www.arduino.cc/en/Tutorial/Tone
 */
// notes in the melody:
int melody[] = {
  //NOTE_C4, NOTE_G3, NOTE_G3, NOTE_A3, NOTE_G3, 0, NOTE_B3, NOTE_C4
  //NOTE_C5, NOTE_G4, NOTE_G4, NOTE_A4, NOTE_G4, 0, NOTE_B4, NOTE_C5
  NOTE_C6, NOTE_G5, NOTE_G5, NOTE_A5, NOTE_G5, 0, NOTE_B5, NOTE_C6
};

// note durations: 4 = quarter note, 8 = eighth note, etc.:
int noteDurations[] = {
  4, 8, 8, 4, 4, 4, 4, 4
};

void play_song(void)
{
  // iterate over the notes of the melody:
  for (int thisNote = 0; thisNote < 8; thisNote++) {

    // to calculate the note duration, take one second
    // divided by the note type.
    //e.g. quarter note = 1000 / 4, eighth note = 1000/8, etc.
    int noteDuration = 1000 / noteDurations[thisNote];
    tone(8, melody[thisNote], noteDuration);

    // to distinguish the notes, set a minimum time between them.
    // the note's duration + 30% seems to work well:
    int pauseBetweenNotes = noteDuration * 1.30;
    delay(pauseBetweenNotes);
    // stop the tone playing:
    noTone(8);
  }
}

void play_toneC(void)
{
  // iterate over the notes of the melody:
  for (int thisNote = 0; thisNote < 1; thisNote++) {

    // to calculate the note duration, take one second
    // divided by the note type.
    //e.g. quarter note = 1000 / 4, eighth note = 1000/8, etc.
    int noteDuration = 1000 / noteDurations[thisNote];
    tone(8, melody[thisNote], noteDuration);

    // to distinguish the notes, set a minimum time between them.
    // the note's duration + 30% seems to work well:
    int pauseBetweenNotes = noteDuration * 1.30;
    delay(pauseBetweenNotes);
    // stop the tone playing:
    noTone(8);
  }
}

/*
 * IR receiver
 */
volatile irparams_t irparams;
// serial port stuff
char rxbuf[100];
int data_receive_idx=0;
volatile bool usart_rx_ready=false;
volatile bool usart_tx_ready=true;

void addchar(char c)
{
    char buf[2]={0,0};
    buf[0]=c;

    if(buf[0]=='*') {
      data_receive_idx=0;     
    } else if(buf[0]=='#' && rxbuf[0]=='*') {
      usart_rx_ready=true;
    } else if(!isdigit(buf[0])) {
        //data_receive_idx=0;
      return;
    }
    
    if(data_receive_idx>80) {
      data_receive_idx=0;
      return;
    }
        
    if(usart_rx_ready) {
        play_song();
    } else {
        play_toneC();
    }

    usart_tx_ready=false;
    Serial1.print(buf);

    strcpy(&rxbuf[data_receive_idx],buf);
    data_receive_idx++;
    //if(rxbuf[0]=='*') gpio_set_pin_level(LED_YELLOW, false);
}

void read_serialport(void)
{
  if(Serial1.available() > 0) {
    addchar(Serial1.read());
  } 
}



void pwm_init(int khz) {
	uint32_t cc;

	analogWrite(PWMPIN,0);  // init clock,timer,pin, and zero
	TCCx->CTRLA.reg &= ~TCC_CTRLA_ENABLE;
	syncTCC;
	// set period and duty cycle
	cc = F_CPU/(khz*1000) - 1;
	TCCx->CC[TCCchannel].reg = cc/2;  // duty 50%
	syncTCC;
	TCCx->PER.reg = cc;               // period
	syncTCC;
}

void enable_pwm() {
	TCCx->CTRLA.reg |= TCC_CTRLA_ENABLE ;
	syncTCC;
}

void disable_pwm() {
	TCCx->CTRLA.reg &= ~TCC_CTRLA_ENABLE;
	syncTCC;
}


void tick_init() {
	// 50us ticks  20khz from 48mhz clock
	uint16_t cc;

	cc = F_CPU/(1000000/USECPERTICK) - 1; // 2399   // 20khz
	GCLK->CLKCTRL.reg = (uint16_t) (GCLK_CLKCTRL_CLKEN | GCLK_CLKCTRL_GEN_GCLK0 | GCLK_CLKCTRL_ID(TCGCLK));
	while (GCLK->STATUS.bit.SYNCBUSY);

	// reset
	TCx->COUNT16.CTRLA.reg &= ~TC_CTRLA_ENABLE;
	syncTC;
	TCx->COUNT16.CTRLA.reg = TC_CTRLA_SWRST;
	while (TCx->COUNT16.CTRLA.bit.SWRST);

	TCx->COUNT16.CTRLA.reg |= TC_CTRLA_MODE_COUNT16;
	TCx->COUNT16.CTRLA.reg |= TC_CTRLA_WAVEGEN_MFRQ;
	TCx->COUNT16.CC[TCchannel].reg = cc;
	syncTC;

	NVIC_EnableIRQ(TCx_IRQn);
	TCx->COUNT16.INTENSET.bit.MC0 = 1;  // enable interrupt
	TCx->COUNT16.CTRLA.reg |= TC_CTRLA_ENABLE;  // enable
	syncTC;
}


unsigned long myticks;

void TCx_Handler() {
	uint8_t irdata = (uint8_t)digitalRead(RECVPIN);
	myticks++;
	TCx->COUNT16.INTFLAG.bit.MC0 = 1;  // clear interrupt
  if (irparams.rawlen >= RAWBUF) {
    // Buffer overflow
    irparams.rcvstate = STATE_STOP;
  }
  switch(irparams.rcvstate) {
  case STATE_IDLE: // In the middle of a gap
    if (irdata == MARK) {
      if (myticks < GAP_TICKS) {
        // Not big enough to be a gap.
        myticks = 0;
      }
      else {
        // gap just ended, record duration and start recording transmission
        irparams.rawlen = 0;
        irparams.rawbuf[irparams.rawlen++] = myticks;
        myticks = 0;
        irparams.rcvstate = STATE_MARK;
      }
     }
    break;
  case STATE_MARK: // timing MARK
    if (irdata == SPACE) {   // MARK ended, record time
      irparams.rawbuf[irparams.rawlen++] = myticks;
      myticks = 0;
      irparams.rcvstate = STATE_SPACE;
    }
    break;
  case STATE_SPACE: // timing SPACE
    if (irdata == MARK) { // SPACE just ended, record it
      irparams.rawbuf[irparams.rawlen++] = myticks;
      myticks = 0;
      irparams.rcvstate = STATE_MARK;
    }
    else { // SPACE
      if (myticks > GAP_TICKS) {
        // big SPACE, indicates gap between codes
        // Mark current code as ready for processing
        // Switch to STOP
        // Don't reset timer; keep counting space width
        irparams.rcvstate = STATE_STOP;
      }
    }
    break;
  case STATE_STOP: // waiting, measuring gap
    if (irdata == MARK) { // reset gap timer
      myticks = 0;
    }
    break;
  }

}

void enableIRIn() {
	tick_init();
	irparams.rcvstate = STATE_IDLE;
	irparams.rawlen = 0;
	pinMode(RECVPIN, INPUT);
}

void irrecv_resume() {
    irparams.rcvstate = STATE_IDLE;
    irparams.rawlen = 0;
}


void mark(int time) {
	//TIMER_ENABLE_PWM;
	enable_pwm();
	if (time > 0) delayMicroseconds(time);
}

void space(int time){
	//TIMER_DISABLE_PWM;
	disable_pwm();
	if (time > 0) delayMicroseconds(time);
}

// Decodes the received IR message
// Returns 0 if no data ready, 1 if data ready.
// Results of decoding are stored in results
int decode(decode_results *results) {
  results->rawbuf = irparams.rawbuf;
  results->rawlen = irparams.rawlen;
  if (irparams.rcvstate != STATE_STOP) {
    return ERR;
  }
  if (decodeNEC(results)) {
    return DECODED;
  }
  // Throw away and start over
  irrecv_resume();
  return ERR;
}

// NECs have a repeat only 4 items long
long decodeNEC(decode_results *results) {
  long data = 0;
  int offset = 1; // Skip first space
  // Initial mark
  if (!MATCH_MARK(results->rawbuf[offset], NEC_HDR_MARK)) {
    return ERR;
  }
  offset++;
  // Check for repeat
  if (irparams.rawlen == 4 &&
    MATCH_SPACE(results->rawbuf[offset], NEC_RPT_SPACE) &&
    MATCH_MARK(results->rawbuf[offset+1], NEC_BIT_MARK)) {
    results->bits = 0;
    results->value = REPEAT;
    results->decode_type = NEC;
    return DECODED;
  }
  if (irparams.rawlen < 2 * NEC_BITS + 4) {
    return ERR;
  }
  // Initial space  
  if (!MATCH_SPACE(results->rawbuf[offset], NEC_HDR_SPACE)) {
    return ERR;
  }
  offset++;
  for (int i = 0; i < NEC_BITS; i++) {
    if (!MATCH_MARK(results->rawbuf[offset], NEC_BIT_MARK)) {
      return ERR;
    }
    offset++;
    if (MATCH_SPACE(results->rawbuf[offset], NEC_ONE_SPACE)) {
      data = (data << 1) | 1;
    }
    else if (MATCH_SPACE(results->rawbuf[offset], NEC_ZERO_SPACE)) {
      data <<= 1;
    }
    else {
      return ERR;
    }
    offset++;
  }
  // Success
  results->bits = NEC_BITS;
  results->value = data;
  results->decode_type = NEC;
  return DECODED;
}

/*
 * Print SAMD chip Serial1 number.
 *
 * http://atmel.force.com/support/articles/en_US/FAQ/Reading-unique-Serial1-number-on-SAM-D20-SAM-D21-SAM-R21-devices
 */
void printChipId() {
  volatile uint32_t val1, val2, val3, val4;
  volatile uint32_t *ptr1 = (volatile uint32_t *)0x0080A00C;
  val1 = *ptr1;
  volatile uint32_t *ptr = (volatile uint32_t *)0x0080A040;
  val2 = *ptr;
  ptr++;
  val3 = *ptr;
  ptr++;
  val4 = *ptr;

  Serial1.print("chip id: 0x");
  char buf[33];
  sprintf(buf, "%8x%8x%8x%8x", val1, val2, val3, val4);
  // my chip replies: 0x67718634504a5230352e3120ff03171c
  Serial1.println(buf);
}

void alarmMatch()
{
  // This function is called when the alarm values match
  // and the alarm is triggered.
  alarmTriggered = true; // Set the global triggered flag
}

void setup() {
    pinMode(13,OUTPUT);
    Serial1.begin(9600);
    enableIRIn();
    Serial1.print("F_CPU=");
    Serial1.println(F_CPU, DEC);
    printChipId();

    play_song();

    // Start RTC:
    rtc.begin(); // To use the RTC, first begin it
    rtc.setTime(0, 0, 0); // set time (hour 0-23, minute 0-59, second 0-59)
    rtc.setDate(0, 0, 0); // set date (day 0-31, month 0-12, year 0-99)
    Serial1.println("RTC Started!");

    // see sketch_feb10b.ino for better alarm example
    // To set an alarm, use the setAlarmTime function.
    // rtc.setAlarmTime(alarmHour, alarmMinute, second);
    rtc.setAlarmTime(0, 2, 0);
    // After the time is set, enable the alarm, configuring
    // which time values you want to trigger the alarm
    rtc.enableAlarm(rtc.MATCH_HHMMSS); // Alarm when hours, minute, & second match
    // When the alarm triggers, alarmMatch will be called:
    rtc.attachInterrupt(alarmMatch);


    // Set the LEDs:
    pinMode(BLUE_LED, OUTPUT);
    pinMode(RX_LED, OUTPUT);
    pinMode(TX_LED, OUTPUT);
    digitalWrite(RX_LED, HIGH);
    digitalWrite(TX_LED, HIGH);
    digitalWrite(BLUE_LED, LOW);
}

void loop() {
  decode_results results;            // create instance of 'decode_results'

  delay(5);   // let gap time grow

  if (irparams.rcvstate == STATE_STOP) {
    if (decode(&results) ) {
        char str[128];
        //sprintf(str,"decoded. value %0x  %d bits",results.value, results.bits);
        //Serial1.println(str);

        switch(results.value) {
            case /*0xFF629D*/0xfd8877: Serial1.println(" FORWARD"); break;
            case /*0xFF22DD*/0xfd28d7: Serial1.println(" LEFT");    break;
            case /*0xFF02FD*/0xfda857: Serial1.println(" -OK-");    break;
            case /*0xFFC23D*/0xfd6897: Serial1.println(" RIGHT");   break;
            case /*0xFFA857*/0xfd9867: Serial1.println(" REVERSE"); break;
            case /*0xFF6897*/0xfd00ff: Serial1.println(" 1"); addchar('1'); break;
            case /*0xFF9867*/0xfd807f: Serial1.println(" 2"); addchar('2'); break;
            case /*0xFFB04F*/0xfd40bf: Serial1.println(" 3"); addchar('3'); break;
            case /*0xFF30CF*/0xfd20df: Serial1.println(" 4"); addchar('4'); break;
            case /*0xFF18E7*/0xfda05f: Serial1.println(" 5"); addchar('5'); break;
            case /*0xFF7A85*/0xfd609f: Serial1.println(" 6"); addchar('6'); break;
            case /*0xFF10EF*/0xfd10ef: Serial1.println(" 7"); addchar('7'); break;
            case /*0xFF38C7*/0xfd906f: Serial1.println(" 8"); addchar('8'); break;
            case /*0xFF5AA5*/0xfd50af: Serial1.println(" 9"); addchar('9'); break;
            case /*0xFF42BD*/0xfd30cf: Serial1.println(" *"); addchar('*'); break;
            case /*0xFF4AB5*/0xfdb04f: Serial1.println(" 0"); addchar('0'); break;
            case /*0xFF52AD*/0xfd708f: Serial1.println(" #"); addchar('#'); break;
            case 0xFFFFFFFF: Serial1.println(" REPEAT");break;  
            default: Serial1.println(" other button   ");
        }
    } else {
        Serial1.print("Unable to decode, however: results.value: ");
        Serial1.println(results.value, HEX);
    }
    irrecv_resume();
  }
  read_serialport();
  if(usart_rx_ready) {
    char str[128];
    usart_rx_ready=false;
    sprintf(str,"\nReceived command:%s",rxbuf);
    Serial1.println(str);
    memset(rxbuf,0,sizeof(rxbuf));
  }

  delay(5);

  if (alarmTriggered) { // If the alarm has been triggered
      Serial1.println("Alarm!"); // Print alarm!
      play_toneC();
  }


  // LED example:
  /*
  digitalWrite(RX_LED, LOW); // RX LED on
  delay(133);
  digitalWrite(RX_LED, HIGH); // RX LED off
  digitalWrite(TX_LED, LOW); // TX LED on
  delay(133);
  digitalWrite(TX_LED, HIGH); // TX LED off
  digitalWrite(BLUE_LED, HIGH); // Blue LED on
  delay(333);
  digitalWrite(BLUE_LED, LOW); // Blue LED off
  */
}

