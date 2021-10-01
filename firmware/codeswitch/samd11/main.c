/*
 * Kamworks Codeswitch firmware
 *
 * Author: Leon Hiemstra <l.hiemstra@gmail.com>
 *
 * Compiler tool: Atmel Studio 7.0.1417
 *
 * Microcontroller: ATSAMD11C14A
 * Required fuse settings: none (keep default)
 *
 * (Arduino IR decoder derived from: ...)
 */

//-----------------------------------------------------------------------------
#include <stdlib.h>
#include <stdio.h>
#include <stdint.h>
#include <stdbool.h>
#include <string.h>
#include "samd11.h"
#include "hal_gpio.h"
#include "utils.h"
#include "rtc.h"
#include "tone.h"
#include "flash.h"

// comment this out after debug/testing:
//#define TESTING 1

#define VERSION     1
#define SUBVERSION  10


volatile bool alarmTriggered = false;

volatile bool usart_rx_ready=false;
volatile bool usart_tx_ready=true;
#define RXBUF_SIZE 100
char rxbuf[RXBUF_SIZE];
int data_receive_idx=0;
#define RAWBUF 70 // Length of raw duration buffer (65 is too small)
// information for the interrupt handler
typedef struct {
	uint8_t rcvstate;          // state machine
	unsigned int rawbuf[RAWBUF]; // raw data
	uint8_t rawlen;         // counter of entries in rawbuf
} irparams_t;
volatile irparams_t irparams;
unsigned long myticks;
#define USECPERTICK 50  // microseconds per clock interrupt tick
// receiver states
#define STATE_IDLE     2
#define STATE_MARK     3
#define STATE_SPACE    4
#define STATE_STOP     5
// IR detector output is active low
#define MARK  0
#define SPACE 1
#define _GAP 100000 // Minimum gap between transmissions  us
#define GAP_TICKS (_GAP/USECPERTICK)
// Decoded value for NEC when a repeat code is received
#define REPEAT 0xffffffff
#define IRERR 0
#define DECODED 1
#define TOLERANCE 25  // percent tolerance in measurements
#define LTOL (1.0 - TOLERANCE/100.)
#define UTOL (1.0 + TOLERANCE/100.)
#define TICKS_LOW(us) (int) (((us)*LTOL/USECPERTICK))
#define TICKS_HIGH(us) (int) (((us)*UTOL/USECPERTICK + 1))
#define MATCH(measured_ticks, desired_us) ((measured_ticks) >= TICKS_LOW(desired_us) && (measured_ticks) <= TICKS_HIGH(desired_us))
#define MATCH_MARK(measured_ticks, desired_us) MATCH(measured_ticks, (desired_us) + MARK_EXCESS)
#define MATCH_SPACE(measured_ticks, desired_us) MATCH((measured_ticks), (desired_us) - MARK_EXCESS)
#define NEC 1
#define NEC_BITS 32
#define NEC_HDR_MARK	9000
#define NEC_HDR_SPACE	4500
#define NEC_BIT_MARK	560
#define NEC_ONE_SPACE	1600
#define NEC_ZERO_SPACE	560
#define NEC_RPT_SPACE	2250
// Marks tend to be 100us too long, and spaces 100us too short
// when received due to sensor lag.
#define MARK_EXCESS 100
// Results returned from the decoder
typedef struct  {
	unsigned long value; // Decoded value
	int bits; // Number of bits in decoded value
	volatile unsigned int *rawbuf; // Raw intervals in .5 us ticks
	int rawlen; // Number of records in rawbuf.
} decode_results;

void irrecv_reset() {
	irparams.rcvstate = STATE_IDLE;
	irparams.rawlen = 0;
}


// NECs have a repeat only 4 items long
long decodeNEC(decode_results *results) {
	long data = 0;
	int offset = 1; // Skip first space
	// Initial mark
	if (!MATCH_MARK(results->rawbuf[offset], NEC_HDR_MARK)) {
		return IRERR;
	}
	offset++;
	// Check for repeat
	if (irparams.rawlen == 4 &&
	MATCH_SPACE(results->rawbuf[offset], NEC_RPT_SPACE) &&
	MATCH_MARK(results->rawbuf[offset+1], NEC_BIT_MARK)) {
		results->bits = 0;
		results->value = REPEAT;
		return DECODED;
	}
	if (irparams.rawlen < 2 * NEC_BITS + 4) {
		return IRERR;
	}
	// Initial space
	if (!MATCH_SPACE(results->rawbuf[offset], NEC_HDR_SPACE)) {
		return IRERR;
	}
	offset++;
	for (int i = 0; i < NEC_BITS; i++) {
		if (!MATCH_MARK(results->rawbuf[offset], NEC_BIT_MARK)) {
			return IRERR;
		}
		offset++;
		if (MATCH_SPACE(results->rawbuf[offset], NEC_ONE_SPACE)) {
			data = (data << 1) | 1;
			} else if (MATCH_SPACE(results->rawbuf[offset], NEC_ZERO_SPACE)) {
			data <<= 1;
			} else {
			return IRERR;
		}
		offset++;
	}
	// Success
	results->bits = NEC_BITS;
	results->value = data;
	return DECODED;
}

// Decodes the received IR message
// Returns 0 if no data ready, 1 if data ready.
// Results of decoding are stored in results
int decode(decode_results *results) {
	
	results->rawbuf = irparams.rawbuf;
	results->rawlen = irparams.rawlen;
	if(irparams.rcvstate != STATE_STOP) return IRERR;
	if(decodeNEC(results)) return DECODED;
	irrecv_reset();
	return IRERR;
}

//-----------------------------------------------------------------------------
void irq_handler_tc1(void)
{  
  if (TC1->COUNT16.INTFLAG.reg & TC_INTFLAG_MC(1)) {
	// begin my code
	uint8_t irdata = HAL_GPIO_RC_PIN_read();	
	//HAL_GPIO_LED_RED_write(irdata);
	
	myticks++;
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
			} else {
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
		} else { // SPACE
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
	// end my code	
    TC1->COUNT16.INTFLAG.reg = TC_INTFLAG_MC(1);
  }  
}

//-----------------------------------------------------------------------------
static void timer1_init(void)
{
  PM->APBCMASK.reg |= PM_APBCMASK_TC1;
  GCLK->CLKCTRL.reg = GCLK_CLKCTRL_ID(TC1_GCLK_ID) |
                      GCLK_CLKCTRL_CLKEN | GCLK_CLKCTRL_GEN(0);

  TC1->COUNT16.CTRLA.reg = TC_CTRLA_MODE_COUNT16 | TC_CTRLA_WAVEGEN_MFRQ |
                           TC_CTRLA_PRESCALER_DIV8 | TC_CTRLA_PRESCSYNC_RESYNC;

  TC1->COUNT16.COUNT.reg = 0;
  
    TC1->COUNT16.CC[0].reg = 48; // 20kHz
    TC1->COUNT16.COUNT.reg = 0;
	
  TC1->COUNT16.CTRLA.reg |= TC_CTRLA_ENABLE;
  TC1->COUNT16.INTENSET.reg = TC_INTENSET_MC(1);
  NVIC_EnableIRQ(TC1_IRQn);
}


volatile uint8_t serial_rxbuf;
volatile bool serial_rxflag=false;

void irq_handler_sercom1(void)
{
	uint8_t c;
	if (SERCOM1->USART.INTFLAG.bit.RXC) {
		c = SERCOM1->USART.DATA.bit.DATA;
		serial_rxbuf = c;
		serial_rxflag = true;
	}
}



//-----------------------------------------------------------------------------
static void uart_init(uint32_t baud)
{
  uint64_t br = (uint64_t)65536 * (F_CPU - 16 * baud) / F_CPU;

  HAL_GPIO_UART_TX_out();
  HAL_GPIO_UART_TX_pmuxen(PORT_PMUX_PMUXE_C_Val);
  HAL_GPIO_UART_RX_in();
  HAL_GPIO_UART_RX_pullup();
  HAL_GPIO_UART_RX_pmuxen(PORT_PMUX_PMUXE_C_Val);
  PM->APBCMASK.reg |= PM_APBCMASK_SERCOM1;

  GCLK->CLKCTRL.reg = GCLK_CLKCTRL_ID(SERCOM1_GCLK_ID_CORE) |
                      GCLK_CLKCTRL_CLKEN | GCLK_CLKCTRL_GEN(0);

  SERCOM1->USART.CTRLA.reg =
      SERCOM_USART_CTRLA_DORD | SERCOM_USART_CTRLA_MODE_USART_INT_CLK |
      SERCOM_USART_CTRLA_RXPO(1/*PAD1*/) | SERCOM_USART_CTRLA_TXPO(0/*PAD0*/);

  SERCOM1->USART.CTRLB.reg = SERCOM_USART_CTRLB_RXEN | SERCOM_USART_CTRLB_TXEN |
      SERCOM_USART_CTRLB_CHSIZE(0/*8 bits*/);

  SERCOM1->USART.BAUD.reg = (uint16_t)br+1;
  
  // set RX interrupt:
  SERCOM1->USART.INTENSET.reg = SERCOM_USART_INTENSET_RXC;
  
  SERCOM1->USART.CTRLA.reg |= SERCOM_USART_CTRLA_ENABLE;
  NVIC_EnableIRQ(SERCOM1_IRQn);
}

//-----------------------------------------------------------------------------
static void uart_putc(char c)
{
  while (!(SERCOM1->USART.INTFLAG.reg & SERCOM_USART_INTFLAG_DRE));
  SERCOM1->USART.DATA.reg = c;
}

//-----------------------------------------------------------------------------
static void uart_puts(char *s)
{
  while (*s) uart_putc(*s++);
}

//-----------------------------------------------------------------------------
static void sys_init(void)
{
  // Switch to 8MHz clock (disable prescaler)
  SYSCTRL->OSC8M.bit.PRESC = 0;
  // Enable interrupts
  asm volatile ("cpsie i");
}

void print_prompt(void)
{
    uart_puts("\n\rCMD> ");	
}

char addchar(char c)
{
	char buf[2]={0,0};
	buf[0]=c;
	
	if(buf[0]=='*') {
		data_receive_idx=0;
	} else if(buf[0]=='#' && rxbuf[0]=='*') {
	    usart_rx_ready=true;
	} else if(!is_digit(buf[0])) {
	    play_song_error();
	    //data_receive_idx=0;
	    return ' ';
	}
	
	if(data_receive_idx>80) {
	    data_receive_idx=0;
	    return ' ';
	}
	
	play_toneC();	
	
	strcpy(&rxbuf[data_receive_idx],buf);
	data_receive_idx++;
	return c;
}


void relay_switch(bool sw)
{
  HAL_GPIO_RELAY_ON_clr();
  HAL_GPIO_RELAY_OFF_clr();
  
  // provide a 0.5 sec pulse on coil
  
  if(sw) { // on
    HAL_GPIO_RELAY_ON_set();
  } else { // off
	HAL_GPIO_RELAY_OFF_set();  
  }
  delay_ms(300);
  HAL_GPIO_RELAY_ON_clr();
  HAL_GPIO_RELAY_OFF_clr();
}


// returns -1 if passed alarm time already
// returns  >=0 if have still days credit left
int8_t read_print_rtc(bool do_print)
{
    char buf[50];
	Time now,alarm;
	int8_t credit;
	
	getTimeNow(&now);
	getAlarmTimeNow(&alarm);
	
	credit = Days_left(&alarm,&now);
	
	if(do_print) {
	  sprintf(buf,"\n\r      YY-MM-DD hh:mm\n\r");
	  uart_puts(buf);
	  sprintf(buf," time:%02d-%02d-%02d %02d:%02d\n\r",now.year,now.month,now.day,now.hour,now.minute);
	  uart_puts(buf);
	  sprintf(buf,"alarm:%02d-%02d-%02d %02d:%02d\n\r",alarm.year,alarm.month,alarm.day,alarm.hour,alarm.minute);
	  uart_puts(buf);
	  sprintf(buf,"credit=%d days left\n\r",credit);
	  uart_puts(buf);
	}
	return credit;
}
void generate_newalarm(Time *newalarm, Time *now, Time *credit)
{
	Time alarm;
	getAlarmTimeNow(&alarm);
	int8_t oldcredit = Days_left(&alarm,now);
	
	if(oldcredit < 0) {
		// credit is expired; add credit to now	
	    Time_add(newalarm, now, credit);
	} else {
		// credit not yet expired; add credit; move alarm forward		
		Time_add(newalarm, &alarm, credit);
	}
}

uint32_t get_myid(void)
{
	// reserved 19 bits for id.
	// 
	// truncate 128bit id.
		
	// ATSAMD21 replies: 0x67718634504a5230352e3120ff03171c
	// ATSAMD11 replies: 0xe1c74e77514d4b5331202020ff0b4631
		
	// most significant side seems to be more unique. Take 19 bits: 114 downto 95
		
	volatile uint32_t *chip_ptr = (volatile uint32_t *)0x0080A00C; // word0 address
	return (*chip_ptr & 0x7ffff); // mask 19bits
}
void print_myid(void)
{
	char buf[50];
	uint32_t chipval = get_myid();	
	sprintf(buf, "myid=%ld\n\r",chipval);
	uart_puts(buf);
}

#define LED_RED_mode_OFF   0
#define LED_RED_mode_ON    1
#define LED_RED_mode_FLASH 2

//-----------------------------------------------------------------------------
int main(void)
{
  decode_results results;
  char buf[100];
  uint16_t cnt=0;
  char ch='.';
  Eeprom eeprom;
  uint8_t rtc_hour_old=0;
  uint8_t rtc_minutes_old=0;
  uint8_t state_LED_RED=LED_RED_mode_OFF;
  
  sys_init();
  timer1_init();
  timer2_init();
  rtc_init();
  uart_init(9600);  
  
    
  HAL_GPIO_RELAY_ON_out();
  HAL_GPIO_RELAY_ON_clr();
  
  HAL_GPIO_RELAY_OFF_out();
  HAL_GPIO_RELAY_OFF_clr();
  
  HAL_GPIO_PIEZO_PIN_out();
  HAL_GPIO_PIEZO_PIN_clr();

  HAL_GPIO_RC_PIN_in();
  HAL_GPIO_RC_PIN_pullup();
  
  HAL_GPIO_LED_RED_out();
  HAL_GPIO_LED_RED_set(); // LED is inverted: OFF

  //int len;
  //char buf[100];
  //volatile uint32_t *chip_ptr;
  //chip_ptr = (volatile uint32_t *)0x0080A00C; // word0
  //chip_val = *chip_ptr;
  sprintf(buf, "Kamworks Codeswitch\n\rVersion=%d.%02d\n\r",VERSION,SUBVERSION);
  uart_puts(buf);
#ifdef TESTING
  sprintf(buf, "TESTING\n\r");
  uart_puts(buf);
#endif
  print_myid();
  /*
  // print full 128bit chip code:
  len=sprintf(buf, "ATSAMD11: %8lx",*chip_ptr);
  chip_ptr = (volatile uint32_t *)0x0080A040; // word1
  len+=sprintf(&buf[len], "%8lx",*chip_ptr);
  chip_ptr++; // word2
  len+=sprintf(&buf[len], "%8lx",*chip_ptr);
  chip_ptr++; // word3
  len+=sprintf(&buf[len], "%8lx\n\r",*chip_ptr);
  uart_puts(buf);
  */
  sprintf(buf, "\nHelp: *code#\n\rcode=1: read id\n\rcode=2: remaining days\n\rcode=5: read RTC\n\r");
  uart_puts(buf);
#ifdef TESTING
  sprintf(buf, "code=3: relay ON\n\rcode=4: relay OFF\n\r");
  uart_puts(buf);
  sprintf(buf, "code=5yy: set Year\n\rcode=6mm: set Month\n\rcode=7dd: set Day\n\r");
  uart_puts(buf);
  sprintf(buf, "code=8hh: set Hour\n\rcode=9mm: set Minute\n\r");
  uart_puts(buf);
#endif
  sprintf(buf, "else code is credit (10 digits)\n\n\r");
  uart_puts(buf);
  
  flash_config();  
  
  // Read the content of "my_flash_store" into the "owner" variable
  flash_read((void *)&eeprom);
  
  // If this is the first run the "valid" value should be "false"
  if (eeprom.valid == false) {
	  state_LED_RED=LED_RED_mode_ON;
	  relay_switch(false);
	  uart_puts("eeprom empty: relay off\n\r");
	  setTime(0,0,0);// set time (hour 0-23, minute 0-59, second 0-59)
	  setDate(1,1,1);// set date (day 1-31, month 1-12, year 0-99)
	  eeprom.seq=0;
	  play_song_expired();
 } else {
	  int8_t creditleft;
	  uart_puts("eeprom ok: read values");
 
      // restore time, date, alarm:
	  setTime(eeprom.now.hour,eeprom.now.minute,0);
	  setDate(eeprom.now.day,eeprom.now.month,eeprom.now.year);
	  setAlarmTime(eeprom.alarm.hour,eeprom.alarm.minute,0);
	  setAlarmDate(eeprom.alarm.day,eeprom.alarm.month,eeprom.alarm.year);
	  sprintf(buf,"(days=%d) (seq=%d)\n\r",eeprom.days,eeprom.seq);
	  uart_puts(buf);
	  
	  creditleft = read_print_rtc(true);
	  
	  if(eeprom.days==0) {
		  uart_puts("free mode, no alarm: relay on\n\r");
		  play_song_success();
		  state_LED_RED=LED_RED_mode_OFF;
		  relay_switch(true);
	  } else {
		  if(creditleft < 0) { // passed the alarm time
			  uart_puts("already expired, relay off\n\r");
			  play_song_expired(); 
			  state_LED_RED=LED_RED_mode_ON;
			  relay_switch(false);			    
		  } else { // not yet passed the alarm time
			  uart_puts("enable alarm, relay on\n\r");
			  enableAlarm(RTC_MODE2_MASK_SEL_YYMMDDHHMMSS_Val);
			  play_song_success();
			  state_LED_RED=LED_RED_mode_OFF;
			  relay_switch(true);
		  }
	  }
  }

  print_prompt();    
  irrecv_reset();  
    
  while (1) {
	cnt++;
    delay_ms(100);
	
	switch(state_LED_RED) {
		case LED_RED_mode_OFF:   HAL_GPIO_LED_RED_set(); // LED is inverted
		                                                   break;
		case LED_RED_mode_ON:    HAL_GPIO_LED_RED_clr();   break;
		case LED_RED_mode_FLASH: HAL_GPIO_LED_RED_toggle();break;
	}
	__disable_irq();
    if(serial_rxflag) {
		serial_rxflag=false;	
		ch=addchar(serial_rxbuf);
		__enable_irq();
		uart_putc(ch);
	} else __enable_irq();
	
	if (irparams.rcvstate == STATE_STOP) {
		if (decode(&results) ) {			
			switch(results.value) {
				//case 0xFF6897: // from internet example
				case 0x8F742bd:  // Leon's fancy remote controller
				case 0xFFA25D:   // Arduino RC 1 (with bubbles under. newer?)
				case 0xfd00ff:   // Arduino RC 2 
				               ch='1'; addchar('1'); break;
				
				//case 0xFF9867:
				case 0x8F7C23D:
				case 0xFF629D:
				case 0xfd807f: ch='2'; addchar('2'); break;
				
				//case 0xFFB04F:
				case 0x8F7E817:
				case 0xFFE21D:
				case 0xfd40bf: ch='3'; addchar('3'); break;
				
				//case 0xFF30CF:
				case 0x8F702FD:
				case 0xFF22DD:
				case 0xfd20df: ch='4'; addchar('4'); break;
				
				//case 0xFF18E7:
				case 0x8F7827D:
				case 0xFF02FD:
				case 0xfda05f: ch='5'; addchar('5'); break;
				
				//case 0xFF7A85:
				case 0x8F76897:
				case 0xFFC23D:
				case 0xfd609f: ch='6'; addchar('6'); break;
				
				//case 0xFF10EF:
				case 0x8F740BF:
				case 0xFFE01F:
				case 0xfd10ef: ch='7'; addchar('7'); break;
				
				//case 0xFF38C7:
				case 0x8F7C03F:
				case 0xFFA857:
				case 0xfd906f: ch='8'; addchar('8'); break;
				
				//case 0xFF5AA5:
				case 0x8F7A857:
				case 0xFF906F:
				case 0xfd50af: ch='9'; addchar('9'); break;
				
				//case 0xFF42BD:
				case 0x8F700FF:
				case 0xFF6897:
				case 0xfd30cf: ch='*'; addchar('*'); break;
				
				//case 0xFF4AB5:
				case 0x8F7807F:
				case 0xFF9867:
				case 0xfdb04f: ch='0'; addchar('0'); break;
				
				//case 0xFF52AD:
				case 0x8F728D7:
				case 0xFFB04F:
				case 0xfd708f: ch='#'; addchar('#'); break;
				
				//case 0xFF629D:
				case 0x8F7906F:
				case 0xFF18E7:
				case 0xfd8877: ch='^'; break;
				
				//case 0xFF22DD:
				case 0x8F710EF:
				case 0xFF10EF:
				case 0xfd28d7: ch='<'; break;
				
				//case 0xFF02FD:
				case 0x8F7708F:
				case 0xFF38C7:
				case 0xfda857: ch='!'; break;
				
				//case 0xFFC23D:
				case 0x8F7D02F:
				case 0xFF5AA5:
				case 0xfd6897: ch='>'; break;
				
				//case 0xFFA857:
				case 0x8F750AF:
				case 0xFF4AB5:
				case 0xfd9867: ch='v'; break;
				
				case 0xFFFFFFFF: ch='r'; break;
				default: ch='?';
			}
			uart_putc(ch);
			sprintf(buf," [IR:%0lx:%c]",results.value,ch);
			uart_puts(buf);
			print_prompt();
		}
		delay_ms(10);	
		irrecv_reset();
	}
	if(usart_rx_ready) {
		int cmdstat=0;
		int8_t creditleft;		
		uart_puts("Received cmd:");
		uart_puts(rxbuf);
		if(strcmp(rxbuf,"*1#")==0) {
			print_myid();
		} else if(strcmp(rxbuf,"*2#")==0) {
			creditleft=read_print_rtc(true);
			if(creditleft<=0) {
				play_song_expired();
			} else {
				play_nof_tones(creditleft);
			}
	    } else if(strcmp(rxbuf,"*5#")==0) {
			read_print_rtc(true);
#ifdef TESTING
		} else if(strcmp(rxbuf,"*3#")==0) {
		    relay_switch(true); // on
		} else if(strcmp(rxbuf,"*4#")==0) {
		    relay_switch(false); // off
		} else if(strlen(rxbuf) == 5) { // *123#  (set date, time) // disable this command after debug!
			unsigned long givencode;
			char opcode = rxbuf[1];
			rxbuf[0]=' '; // strip '*'
			rxbuf[4]=0;   // strip '#'
			rxbuf[1]=' '; // strip opcode
			givencode = strtoul(rxbuf,NULL,10);		
			if(opcode == '5') { // set year			
				setYear((uint8_t)givencode);
			} else if(opcode == '6') { // set month
			    setMonth((uint8_t)givencode);
		    } else if(opcode == '7') { // set day
		        setDay((uint8_t)givencode);
	        } else if(opcode == '8') { // set hour
	            setHours((uint8_t)givencode);
            } else if(opcode == '9') { // set minute
                setMinutes((uint8_t)givencode);
            }
		    read_print_rtc(true);
			sprintf(buf, "Enter new credit code to write flash!\n\r");
			uart_puts(buf);
#endif
		} else {			
			if(strlen(rxbuf) == 12) { // *1234567890#
				unsigned long givencode, givenid;
				uint8_t givendays, givenseq;
				rxbuf[0]=' '; // strip '*'
				rxbuf[11]=0;  // strip '#'
							
				givencode = strtoul(rxbuf,NULL,10);
				
				// decode the code:
				//sprintf(buf," code=%lu (0x%lx):",givencode,givencode);
				//uart_puts(buf);
				givencode = ~givencode; // invert all bits
				givencode = rotate_left(givencode,2);
				//sprintf(buf," code=%lu (0x%lx):",givencode,givencode);
				//uart_puts(buf);
				// done decoding the code
				
				givenseq = (uint8_t)(givencode&0x7f);
				givendays = (uint8_t)((givencode>>7)&0x3f);
				givenid = givencode>>13;
				sprintf(buf," [seq=%d days=%d id=%ld]",givenseq,givendays,givenid);
				uart_puts(buf);
				
				if(givenid == get_myid()) {
					uart_puts("[id match]");
					if(eeprom.seq==0 || givenseq > eeprom.seq) {
						if(givendays > 0) {
							Time now, credit, newalarm;							
							uart_puts("[setting alarm]");
							
							credit.year=0;
							credit.month=0;
							credit.day=givendays;							
							credit.hour=0;							
							credit.minute=0;
						
							getTimeNow(&now);
							generate_newalarm(&newalarm,&now,&credit);
							setAlarmTime(newalarm.hour,newalarm.minute,0);
							setAlarmDate(newalarm.day,newalarm.month,newalarm.year);
							enableAlarm(RTC_MODE2_MASK_SEL_YYMMDDHHMMSS_Val);
							
							eeprom.alarm.hour   = newalarm.hour;
							eeprom.alarm.minute = newalarm.minute;
							eeprom.alarm.day    = newalarm.day;
							eeprom.alarm.month  = newalarm.month;
							eeprom.alarm.year   = newalarm.year;							
							eeprom.now.hour   = now.hour;
							eeprom.now.minute = now.minute;
							eeprom.now.day    = now.day;
							eeprom.now.month  = now.month;
							eeprom.now.year   = now.year;						
						} else {  // if givendays==0 stay on forever.
							uart_puts("[disable alarm]");
							disableAlarm();
						}
						eeprom.valid = true;
						eeprom.seq   = givenseq;
						eeprom.days  = givendays;
						uart_puts("[write flash]");
						flash_write((void *)&eeprom);
						play_song_success();
	                    relay_switch(true);
						state_LED_RED = LED_RED_mode_OFF;	  
					} else {
						uart_puts("[already used]");
						cmdstat=-1;
					}
				} else {
					uart_puts("[id not match]");
					cmdstat=-1;
				}
			} else cmdstat=-1;
		}
		if(cmdstat < 0) {
			play_song_error();
		}
		memset(rxbuf,0,RXBUF_SIZE);
		usart_rx_ready=false;		
		print_prompt();
	}
	if (alarmTriggered) {   // If the alarm has been triggered
		alarmTriggered=false;
		uart_puts("Alarm! Relay OFF!\n\r");
		play_song_expired(); 
		relay_switch(false);
		state_LED_RED = LED_RED_mode_ON;
	}
	
	if((cnt%10)==0) {		
		uint8_t rtc_hour_new = getHours();
		
		uint8_t rtc_minute_new = getMinutes();		
		if(rtc_minutes_old != rtc_minute_new) {
			rtc_minutes_old = rtc_minute_new;
			if(read_print_rtc(true) == 0) {         // 0 credit left? (last day)
				state_LED_RED = LED_RED_mode_FLASH; // toggling (warning the used that alarm comes soon)
			}
		}
		
		if(((rtc_hour_new%8)==0) && (rtc_hour_old != rtc_hour_new)) { // every 8 hours
			rtc_hour_old = rtc_hour_new;
			getTimeNow(&eeprom.now);
			uart_puts("[write flash]\r\n");
			flash_write((void *)&eeprom);
		} 
	}
  }
  return 0;
}

// get rid of linker error when using linker flag --specs=nano.specs
void _sbrk(void) {}

