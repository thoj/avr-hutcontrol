#include <avr/io.h>
#include <avr/pgmspace.h>
#include <avr/interrupt.h>
#include <avr/eeprom.h>
#include <util/delay.h>
#include <string.h>
#include <stdio.h>
#include <stdint.h>

#include "uart.h"
#include "onewire.h"
#include "ds18x20.h"

#define BAUD 9600

#define CMD_ON 'O'
#define CMD_OUTPUT 'o'
#define CMD_INPUT 'i'
#define CMD_OFF 'F'
#define CMD_SCAN 'S'
#define CMD_READ_ALL 'R'
#define CMD_CHECK 'C'
#define CMD_OUTPUT_STATE 'A'

#define EEPROM_PORTA_STATE (uint8_t*)0x10
#define EEPROM_PORTB_STATE (uint8_t*)0x11
#define EEPROM_PORTC_STATE (uint8_t*)0x12
#define EEPROM_PORTD_STATE (uint8_t*)0x13
#define EEPROM_DDRA_STATE (uint8_t*)0x20
#define EEPROM_DDRB_STATE (uint8_t*)0x21
#define EEPROM_DDRC_STATE (uint8_t*)0x22
#define EEPROM_DDRD_STATE (uint8_t*)0x23

#define MAXSENSORS 7
char buffer[UART_TX_BUFFER_SIZE];
uint8_t gSensorIDs[MAXSENSORS][OW_ROMCODE_SIZE];
uint8_t nSensors;
volatile uint8_t last_command;


//Scan bus for sensors
static uint8_t
search_sensors (void)
{
  uint8_t i;
  uint8_t id[OW_ROMCODE_SIZE];
  uint8_t diff, nSensors;
  ow_reset ();
  nSensors = 0;
  diff = OW_SEARCH_FIRST;
  while (diff != OW_LAST_DEVICE && nSensors < MAXSENSORS) {
    DS18X20_find_sensor (&diff, &id[0]);
    if (diff == OW_PRESENCE_ERR) {
      break;
    }
    if (diff == OW_DATA_ERR) {
      break;
    }
    for (i = 0; i < OW_ROMCODE_SIZE; i++)
      gSensorIDs[nSensors][i] = id[i];
    nSensors++;
  }
  return nSensors;
}

static void
uart_put_temp_maxres (int32_t tval)
{
  char s[10];
  DS18X20_format_from_maxres (tval, s, 10);
  uart_puts (s);
}

static void
do_scan_ds1820 (void)
{
  uart_puts_P ("+INFO Scanning...\r\n");
  nSensors = search_sensors ();
  sprintf (buffer, "+OK Found %d Sensors\r\n", nSensors);
  uart_puts (buffer);
}

void
do_read_ds1820 (void)
{
  if (!(nSensors > 0) || nSensors > 8) {
    do_scan_ds1820 ();
  }

  if (DS18X20_start_meas (DS18X20_POWER_PARASITE, NULL) == DS18X20_OK) {
    _delay_ms (DS18B20_TCONV_12BIT);
    int32_t temp_eminus4;
    uint8_t i;
    for (i = 0; i < nSensors; i++) {
      sprintf (buffer, "+TEMP %d %x%x%x%x%x%x%x%x ", i,
	       gSensorIDs[i][0], gSensorIDs[i][1], gSensorIDs[i][2],
	       gSensorIDs[i][3], gSensorIDs[i][4], gSensorIDs[i][5],
	       gSensorIDs[i][6], gSensorIDs[i][7]);
      uart_puts (buffer);
      if (DS18X20_read_maxres (&gSensorIDs[i][0], &temp_eminus4)
	  == DS18X20_OK) {
	uart_put_temp_maxres (temp_eminus4);
	uart_puts_P ("\r\n");
      }

      else {
	uart_puts_P ("-ERR CRC Error (lost connection?)\r\n");
      }
    }
  }

  else {
    uart_puts_P ("-ERR Start meas. failed (short circuit?)\r\n");
  }
}

void
handle_command (char *command)
{
  if (command[0] == CMD_ON) {
    if (command[1] == 'A') {
#ifdef PORTA
      PORTA |= _BV (command[2] - 0x30);
      eeprom_write_byte (EEPROM_PORTA_STATE, PORTA);
#else
      uart_puts_P ("-ERR Wrong PORT Selection\r\n");
#endif
    }
    else if (command[1] == 'B') {
      PORTB |= _BV (command[2] - 0x30);
      eeprom_write_byte (EEPROM_PORTB_STATE, PORTB);
    }
    else if (command[1] == 'C') {
      PORTC |= _BV (command[2] - 0x30);
      eeprom_write_byte (EEPROM_PORTC_STATE, PORTC);
    }
    else if (command[1] == 'D') {
      PORTD |= _BV (command[2] - 0x30);
      eeprom_write_byte (EEPROM_PORTD_STATE, PORTD);
    }
    else {
      uart_puts_P ("-ERR Wrong PORT Selection\r\n");
      return;
    }
    uart_puts_P ("+OK Turning off output ");
    uart_putc (command[2]);
    uart_puts_P (" On Port ");
    uart_putc (command[1]);
    uart_puts_P ("\r\n");
  }
  else if (command[0] == CMD_OFF) {
    if (command[1] == 'A') {
#ifdef PORTA
      PORTA &= ~_BV (command[2] - 0x30);
      eeprom_write_byte (EEPROM_PORTA_STATE, PORTA);
#else
      uart_puts_P ("-ERR Wrong PORT Selection\r\n");
#endif
    }
    else if (command[1] == 'B') {
      PORTB &= ~_BV (command[2] - 0x30);
      eeprom_write_byte (EEPROM_PORTB_STATE, PORTB);
    }
    else if (command[1] == 'C') {
      PORTC &= ~_BV (command[2] - 0x30);
      eeprom_write_byte (EEPROM_PORTC_STATE, PORTC);
    }
    else if (command[1] == 'D') {
      PORTD &= ~_BV (command[2] - 0x30);
      eeprom_write_byte (EEPROM_PORTD_STATE, PORTD);
    }
    else {
      uart_puts_P ("-ERR Wrong PORT Selection\r\n");
      return;
    }
    uart_puts_P ("+OK Turning off output ");
    uart_putc (command[2]);
    uart_puts_P (" On Port ");
    uart_putc (command[1]);
    uart_puts_P ("\r\n");
  }
  else if (command[0] == CMD_OUTPUT) {
    if (command[1] == 'A') {
#ifdef PORTA
      DDRA |= _BV (command[2] - 0x30);
      eeprom_write_byte (EEPROM_DDRA_STATE, DDRA);
#else
      uart_puts_P ("-ERR Wrong PORT Selection\r\n");
#endif
    }
    else if (command[1] == 'B') {
      DDRB |= _BV (command[2] - 0x30);
      eeprom_write_byte (EEPROM_DDRB_STATE, DDRB);
    }
    else if (command[1] == 'C') {
      DDRC |= _BV (command[2] - 0x30);
      eeprom_write_byte (EEPROM_DDRC_STATE, DDRC);
    }
    else if (command[1] == 'D') {
      DDRD |= _BV (command[2] - 0x30);
      eeprom_write_byte (EEPROM_DDRD_STATE, DDRD);
    }
    else {
      uart_puts_P ("-ERR Wrong PORT Selection\r\n");
      return;
    }
    uart_puts_P ("+OK Set Channel ");
    uart_putc (command[2]);
    uart_puts_P (" On Port ");
    uart_putc (command[1]);
    uart_puts_P (" To Output\r\n");
  }
  else if (command[0] == CMD_INPUT) {
    if (command[1] == 'A') {
#ifdef PORTA
      DDRA &= ~_BV (command[2] - 0x30);
      eeprom_write_byte (EEPROM_DDRA_STATE, DDRA);
#else
      uart_puts_P ("-ERR Wrong PORT Selection\r\n");
#endif
    }
    else if (command[1] == 'B') {
      DDRB &= ~_BV (command[2] - 0x30);
      eeprom_write_byte (EEPROM_DDRB_STATE, DDRB);
    }
    else if (command[1] == 'C') {
      DDRC &= ~_BV (command[2] - 0x30);
      eeprom_write_byte (EEPROM_DDRC_STATE, DDRC);
    }
    else if (command[1] == 'D') {
      DDRD &= ~_BV (command[2] - 0x30);
      eeprom_write_byte (EEPROM_DDRD_STATE, DDRD);
    }
    else {
      uart_puts_P ("-ERR Wrong PORT Selection\r\n");
      return;
    }
    uart_puts_P ("+OK Set Channel ");
    uart_putc (command[2]);
    uart_puts_P (" On Port ");
    uart_putc (command[1]);
    uart_puts_P (" To Input\r\n");
  }
  else if (command[0] == CMD_SCAN) {
    do_scan_ds1820 ();
  }
  else if (command[0] == CMD_READ_ALL) {
    do_read_ds1820 ();
  }
  else if (command[0] == CMD_CHECK) {
    uart_puts ("+OK Running\r\n");
  }
  else if (command[0] == CMD_OUTPUT_STATE) {
    if (command[1] == 'A') {
#ifdef PORTA
      sprintf (buffer, "+OK DDRA=%x PORTA=%x\r\n", DDRA, PORTA);
#else
      uart_puts_P ("-ERR Unknown Command: ");
#endif
    }
    else if (command[1] == 'B') {
      sprintf (buffer, "+OK DDRB=%x PORTB=%x\r\n", DDRB, PORTB);
      uart_puts (buffer);
    }
    else if (command[1] == 'C') {
      sprintf (buffer, "+OK DDRC=%x PORTC=%x\r\n", DDRC, PORTC);
      uart_puts (buffer);
    }
    else if (command[1] == 'D') {
      sprintf (buffer, "+OK DDRD=%x PORTD=%x\r\n", DDRD, PORTD);
      uart_puts (buffer);
    }
    else {
      uart_puts_P ("-ERR Wrong PORT Selection (OUTPUT_STATE)\r\n");
      return;
    }
  }
  else {
    uart_puts_P ("-ERR Unknown Command: ");
    uart_puts (command);
    uart_puts_P ("\r\n");
  }
}

void
setup_timer (void)
{
  DDRC |= (1 << 5);		// Set LED as output
  TCCR1B |= (1 << WGM12);	// Configure timer 1 for CTC mode
  TIMSK1 |= (1 << OCIE1A);	// Enable CTC interrupt
  OCR1A = 46875;		// Set CTC compare value to 1Hz at 12MHz AVR clock, with a prescaler of 256
  TCCR1B |= ((1 << CS12) | (0 << CS11) | (0 << CS10));	// Start timer at Fcpu/256
}

ISR (TIMER1_COMPA_vect)
{
  PORTC ^= (1 << 5);		// Toggle the LED
  if (last_command > 100) {
    uart_puts_P ("+INFO Keep-Alive\r\n");
  }
}

void load_eeprom(void) {
#ifdef PORTA
	DDRA = eeprom_read_byte(EEPROM_DDRA_STATE);
	PORTA = eeprom_read_byte(EEPROM_PORTA_STATE);
#endif
	DDRB = eeprom_read_byte(EEPROM_DDRA_STATE);
	PORTB = eeprom_read_byte(EEPROM_PORTB_STATE);
	DDRC = eeprom_read_byte(EEPROM_DDRA_STATE);
	PORTC = eeprom_read_byte(EEPROM_PORTC_STATE);
	DDRD = eeprom_read_byte(EEPROM_DDRA_STATE);
	PORTD = eeprom_read_byte(EEPROM_PORTD_STATE);
}

int
main (void)
{
  unsigned int c;
  uint8_t counter = 0;
  char buf[8];

  load_eeprom();
  setup_timer ();

  uart_init (UART_BAUD_SELECT (9600, F_CPU));
  ow_set_bus (&PIND, &PORTD, &DDRD, PD6);
  
  sei ();
  
  uart_puts_P ("+INFO Booted!\r\n");

  for (;;) {			// main loop
    c = uart_getc ();
    if (c & UART_NO_DATA) {
      //
    }
    else {
      if (c & UART_FRAME_ERROR) {
	uart_puts_P ("Frame Error: ");
      }
      if (c & UART_OVERRUN_ERROR) {
	uart_puts_P ("Overrun Error: ");
      }
      if (c & UART_BUFFER_OVERFLOW) {
	uart_puts_P ("Overflow Error: ");
      }
      if ((unsigned char) c == '\r' || (unsigned char) c == '\n') {
	buf[counter] = '\0';
	counter = 0;
	handle_command (buf);
      }
      else if ((unsigned char) c > 0) {
	buf[counter++] = (unsigned char) c;
	if (counter == 8) {
	  memset (buf, 8, '\0');
	  counter = 0;
	}
      }
    }
  }
}
