// TinyPlayer - MP3-folder version
//
// TinyPlayer is an IR remote controllable MP3 player based on ATtiny13A
// and DFPlayerMini module. The DFPlayer module is controlled via a
// simple cycle-precise software implementation of the UART protocol
// (8N1, 9600 BAUD) in half-duplex mode. The volume last set as well as
// the track number last played are saved in the EEPROM and automatically
// loaded the next time it is started.
//
// Micro SD cards (TF cards) with a maximum of 32 GB are supported. The
// card should be formatted in the FAT32 file system. For this version of
// the firmware, a folder "mp3" must be created in the root directory of
// the SD card. The MP3 files are stored in this folder, which must be
// numbered in ascending order with four digits starting with 0001.mp3.
// Any name can be appended after the four-digit number if desired 
// (e.g. 0001_MySong.mp3). There must not be any unnecessary files on the
// SD card, such as those stored there by MacOS (e.g. Spotlight). Better
// take Linux or Windows to write to the SD card. Under MacOS you can try
// to clean up with the following command: dot_clean /Volumes/SD-Card
//
//                               +-\/-+
//             --- A0 (D5) PB5  1|°   |8  Vcc
// DFPLAYER RX --- A3 (D3) PB3  2|    |7  PB2 (D2) A1 --- DFPLAYER TX
// IR RECEIVER --- A2 (D4) PB4  3|    |6  PB1 (D1) ------
//                         GND  4|    |5  PB0 (D0) ------
//                               +----+
//
// Controller:  ATtiny13A
// Core:        MicroCore (https://github.com/MCUdude/MicroCore)
// Clockspeed:  1.2 MHz internal
// BOD:         BOD disabled
// Timing:      Micros disabled
//
// Leave the rest on default settings. Don't forget to "Burn bootloader"!
// No Arduino core functions or libraries are used. Use the makefile if 
// you want to compile without Arduino IDE.
//
// Note: The internal oscillator may need to be calibrated for the device
//       to function properly.
//
// 2021 by Stefan Wagner 
// Project Files (EasyEDA): https://easyeda.com/wagiminator
// Project Files (Github):  https://github.com/wagiminator
// License: http://creativecommons.org/licenses/by-sa/3.0/


// Oscillator calibration value (uncomment and set if necessary)
//#define OSCCAL_VAL  0x60

// Libraries
#include <avr/io.h>
#include <avr/power.h>
#include <avr/eeprom.h>
#include <avr/interrupt.h>

// Pins
#define TX_PIN        2       // TX pin for DFPlayer
#define RX_PIN        3       // RX pin for DFPlayer
#define IR_PIN        4       // Pin for IR receiver

// IR codes
#define IR_ADDR       0x1A    // IR device address
#define IR_VOL_P      0x01    // IR code for volume up
#define IR_VOL_M      0x03    // IR code for volume down
#define IR_TRK_P      0x02    // IR code for next track
#define IR_TRK_M      0x04    // IR code for previous track
#define IR_PAUSE      0x05    // IR Code for toggle pause/play
#define IR_FAIL       0xFF    // IR fail code

// -----------------------------------------------------------------------------
// UART Implementation (9600 BAUD, 8N1, Half-Duplex)
// -----------------------------------------------------------------------------
//
// ------+     +-----+     +-----+-----+           +-----+     +-----+----- HIGH
//       |     |     |     |           |           |     |     |
//       |     |     |     |           |           |     |     |
//       +-----+     +-----+           +-----+-----+     +-----+            LOW
//       |START|  1  |  0  |  1  |  1  |  0  |  0  |  1  |  0  | STOP|
//       | BIT | LSB               DATA BYTE               MSB | BIT |
//
// Each bit is 1/9600 seconds long. 8N1 = 8 bits, no parity, 1 stop bit.
// The implementation is for half-duplex, so it is not possible to send and
// receive at the same time. The receive routine is interrupt-triggered and uses
// an 11-byte receive buffer. Unfortunately, it doesn't work out of a sleep mode
// yet (it probably needs a few more timing tweaks for that).

// UART macros
#define UART_TX_setHigh()   PORTB |=  (1<<TX_PIN)
#define UART_TX_setLow()    PORTB &= ~(1<<TX_PIN)
#define UART_RX_isHigh()    PINB  &   (1<<RX_PIN)
#define UART_RX_isLow()     ~PINB &   (1<<RX_PIN)

// UART timings
#define UART_BAUD           9600UL              // UART BAUD rate
#define UART_BC             F_CPU / UART_BAUD   // MCU cycles per UART bit
#define UART_SC             UART_BC * 3 / 2     // MCU cycles after start bit

// UART variables
volatile uint8_t UART_RX_BUF[11];               // UART receive buffer
volatile uint8_t UART_RX_PTR = 0;               // UART receive buffer pointer

// UART init
void UART_init(void) {
  PORTB |= (1<<TX_PIN);                         // TX HIGH
  DDRB  |= (1<<TX_PIN);                         // TX as output
  GIMSK |= (1<<PCIE);                           // enable pin change interrupts
  PCMSK |= (1<<RX_PIN);                         // enable PC interrupt on RX pin
}

// UART transmit byte
void UART_write(uint8_t byte) {
  cli();                                        // disable interrupts
  UART_TX_setLow();                             // transmit start bit
  __builtin_avr_delay_cycles(UART_BC - 6);      // wait till end of start bit
  for (uint8_t i=8; i; i--, byte>>=1) {         // 8 bits, LSB first
    if (byte & (0x01))  { asm("nop"); UART_TX_setHigh(); }      // "1" - bit
    else                { UART_TX_setLow(); asm("rjmp .+0"); }  // "0" - bit
    __builtin_avr_delay_cycles(UART_BC - 11);   // wait till end of bit
  }
  __builtin_avr_delay_cycles(4);                // cycle overhead
  UART_TX_setHigh();                            // transmit stop bit
  __builtin_avr_delay_cycles(UART_BC - 8);      // wait till end of stop bit
  sei();                                        // enable interrupts     
}

// UART receive byte (pin change interrupt service routine)
ISR(PCINT0_vect) {
  if (UART_RX_isLow()) {                        // interrupt caused by start bit ?
    uint8_t byte;                               // byte to be received
    __builtin_avr_delay_cycles((UART_BC/2)-27); // wait for first bit
    for (uint8_t i=8; i; i--) {                 // receive 8 Bits
      byte >>= 1;                               // LSB first
      __builtin_avr_delay_cycles(UART_BC - 10); // wait for next bit
      if (UART_RX_isHigh()) byte |= 0x80;       // read bit
    }
    UART_RX_BUF[UART_RX_PTR++] = byte;          // write byte to buffer
    if (UART_RX_PTR > 10) UART_RX_PTR = 10;     // limit pointer (not a ring buffer)
  }
}

// -----------------------------------------------------------------------------
// DFPlayer Implementation
// -----------------------------------------------------------------------------
//
// Instruction Frame (Command String):
// - Start Byte         0x7E
// - Version            0xFF
// - Length             total number of bytes excluding start/end byte and checksum 
// - Command            command byte (see below)
// - Feedback           0x00: no feedback; 0x01: need feedback
// - Parameter 1        data high byte
// - Parameter 2        data low byte
// - Checksum           not include start/end byte; 2 bytes; can be omitted
// - End Byte           0xEF
//
// Length seems to be always 0x06. Feedback seems to be always 10 bytes in total.

// DFPlayer control commands (it's almost complete; only a few of them are used)
#define DFP_CMD_NEXT      0x01                  // play next file
#define DFP_CMD_PREV      0x02                  // play previous file
#define DFP_CMD_TRACK     0x03                  // play track (fileNumber)
#define DFP_CMD_VOLUP     0x04                  // volume up
#define DFP_CMD_VOLDOWN   0x05                  // volume down
#define DFP_CMD_VOL       0x06                  // set volume (0..30)
#define DFP_CMD_EQ        0x07                  // set equalizer
#define DFP_CMD_MODE      0x08                  // 0:repeat; 1:folder; 2:single; 3:random
#define DDP_CMD_OUTPUT    0x09                  // set output device
#define DFP_CMD_STBY      0x0A                  // enter standby
#define DFP_CMD_NORM      0x0B                  // normal working
#define DFP_CMD_RESET     0x0C                  // reset module
#define DFP_CMD_PLAY      0x0D                  // play
#define DFP_CMD_PAUSE     0x0E                  // pause
#define DFP_CMD_FOLDER    0x0F                  // play folder (folderNumber, fileNumber)
#define DFP_CMD_SETTING   0x10                  // output setting (enable, gain)
#define DFP_CMD_REPEAT    0x11                  // loop all (0:stop; 1:start)
#define DFP_CMD_MP3FOLD   0x12                  // play mp3-folder (fileNumber)
#define DFP_CMD_ADVERT    0x13                  // play advertise (fileNumber)
#define DFP_CMD_STOPADV   0x15                  // stop advertise
#define DFP_CMD_STOP      0x16                  // stop
#define DFP_CMD_LOOPFOLD  0x17                  // loop folder (folderNumber)
#define DFP_CMD_RANDOM    0x18                  // random all
#define DFP_CMD_LOOP      0x19                  // loop (0:enable; 1:disable)
#define DFP_CMD_DAC       0x1A                  // DAC (0:enable; 1:disable)

// DFPlayer query commands
#define DFP_QRY_STATE     0x42                  // query current status
#define DFP_QRY_VOL       0x43                  // query current volume
#define DFP_QRY_EQ        0x44                  // query current equalizer setting
#define DFP_QRY_MODE      0x45                  // query current playback mode
#define DFP_QRY_TRACKS    0x48                  // query total number of files
#define DFP_QRY_TRACK     0x4C                  // query current file number
#define DFP_QRY_FTRACKS   0x4E                  // query total files in (folderNumber)
#define DFP_QRY_FOLDERS   0x4F                  // query total number of folders

// DFPlayer macros
#define DFP_setVolume(x)    DFP_command(DFP_CMD_VOL, x)       // set volume
#define DFP_playTrack(x)    DFP_command(DFP_CMD_MP3FOLD, x)   // play file x in mp3 folder
#define DFP_playFolder(x,y) DFP_command(DFP_CMD_FOLDER, (uint16_t)(x<<8)|y) // file y in folder x
#define DFP_getTracks()     DFP_query(DFP_QRY_TRACKS, 0)      // get total number of files
#define DFP_getFolders()    DFP_query(DFP_QRY_FOLDERS, 0)     // get total number of folders
#define DFP_getFiles(x)     DFP_query(DFP_QRY_FTRACKS, x)     // get total files in folder x
#define DFP_play()          DFP_command(DFP_CMD_PLAY, 0)      // play (resume from pause)
#define DFP_pause()         DFP_command(DFP_CMD_PAUSE, 0)     // pause playing

// DFPlayer command string (checksum omitted)
uint8_t DFP_CMD_STR[] = { 0x7E, 0xFF, 0x06, 0x00, 0x01, 0x00, 0x00, 0xEF };

// DFPlayer init
void DFP_init(void) {
  UART_init();                                  // init UART
  while(UART_RX_PTR < 10);                      // wait for init feedback
  UART_RX_PTR = 0;                              // reset receive buffer pointer 
}

// DFPlayer transmit command
void DFP_command(uint8_t cmd, uint16_t data) {
  DFP_CMD_STR[3] = cmd;                         // set command in transmit buffer
  DFP_CMD_STR[5] = data >> 8;                   // set high byte of data
  DFP_CMD_STR[6] = data;                        // set low byte of data
  do {                                          // repeat ...
    for(uint8_t i=0; i<8; i++) UART_write(DFP_CMD_STR[i]); // transmit command string
    while(UART_RX_PTR < 10);                    // wait for feedback
    UART_RX_PTR = 0;                            // reset receive buffer pointer
  } while(UART_RX_BUF[3] == 0x40);              // ... until no error reply
}

// DFPlayer transmit query and receive reply
uint16_t DFP_query(uint8_t cmd, uint8_t data) {
  DFP_command(cmd, data);
  while(UART_RX_PTR < 10);                      // wait for reply
  UART_RX_PTR = 0;                              // reset receive buffer pointer
  return (((uint16_t)UART_RX_BUF[5] << 8) | UART_RX_BUF[6]);  // return query reply
}

// -----------------------------------------------------------------------------
// IR Receiver Implementation (NEC Protocol)
// -----------------------------------------------------------------------------
//
// ------+              +----------+       +-------+       +----------+     HIGH
//       |              |          |       |       |       |          |
//       |    9000us    |  4500us  |562.5us|562.5us|562.5us| 1687.5us |     ...
//       |              |          |       |       |       |          |
//       +--------------+          +-------+       +-------+          +---- LOW
//
//       |<----- Start Frame ----->|<--- Bit=0 --->|<----- Bit=1 ---->|
//
// IR telegram starts with a 9ms leading burst followed by a 4.5ms pause.
// Afterwards 4 data bytes are transmitted, least significant bit first.
// A "0" bit is a 562.5us burst followed by a 562.5us pause, a "1" bit is
// a 562.5us burst followed by a 1687.5us pause. A final 562.5us burst
// signifies the end of the transmission. The four data bytes are in order:
// - the 8-bit address for the receiving device,
// - the 8-bit logical inverse of the address,
// - the 8-bit command and
// - the 8-bit logical inverse of the command.
// The Extended NEC protocol uses 16-bit addresses. Instead of sending an
// 8-bit address and its logically inverse, first the low byte and then the
// high byte of the address is transmitted.

// IR receiver definitions and macros
#define IR_init()       PORTB |= (1<<IR_PIN)    // pullup on IR pin
#define IR_available()  (~PINB & (1<<IR_PIN))   // return true if IR line is low
#define IR_100us        F_CPU / 10000UL         // MCU cycles per 100us

// IR wait for signal change and measure duration
uint8_t IR_waitChange(uint8_t timeout) {
  uint8_t pinState = PINB & (1<<IR_PIN);        // get current signal state
  uint8_t dur = 0;                              // variable for measuring duration
  while ((PINB & (1<<IR_PIN)) == pinState) {    // measure length of signal
    if (dur++ > timeout) return 0;              // exit if timeout
    __builtin_avr_delay_cycles(IR_100us - 13);  // count every 100us
  }
  return dur;                                   // return time in 100us
}

// IR read data byte
uint8_t IR_readByte(void) {
  uint8_t result;
  uint8_t dur;
  for (uint8_t i=8; i; i--) {                   // 8 bits
    result >>= 1;                               // LSB first
    if (IR_waitChange(11) < 3) return IR_FAIL;  // exit if wrong burst length
    dur = IR_waitChange(21);                    // measure length of pause
    if (dur <  3) return IR_FAIL;               // exit if wrong pause length
    if (dur > 11) result |= 0x80;               // bit "0" or "1" depends on pause duration
  }
  return result;                                // return received byte
}

// IR read data according to NEC protocol
uint8_t IR_read(void) {
  uint16_t addr;                                // variable for received address
  if (!IR_available())        return IR_FAIL;   // exit if no signal
  if (!IR_waitChange(100))    return IR_FAIL;   // exit if wrong start burst length
  if (IR_waitChange(55) < 35) return IR_FAIL;   // exit if wrong start pause length

  uint8_t addr1 = IR_readByte();                // get first  address byte
  uint8_t addr2 = IR_readByte();                // get second address byte
  uint8_t cmd1  = IR_readByte();                // get first  command byte
  uint8_t cmd2  = IR_readByte();                // get second command byte

  if (IR_waitChange(11) < 3)  return IR_FAIL;   // exit if wrong final burst length
  if ((cmd1 + cmd2) < 255)    return IR_FAIL;   // exit if command bytes are not inverse
  if ((addr1 + addr2) == 255) addr = addr1;     // check if it's extended NEC-protocol ...
  else addr = ((uint16_t)addr2 << 8) | addr1;   // ... and get the correct address
  if (addr != IR_ADDR)        return IR_FAIL;   // wrong address
  return cmd1;                                  // return command code
}

// -----------------------------------------------------------------------------
// Main Function
// -----------------------------------------------------------------------------

int main(void) {
  // Set oscillator calibration value
  #ifdef OSCCAL_VAL
    OSCCAL = OSCCAL_VAL;                        // set the value if defined above
  #endif

  // Local variables
  uint8_t playstate = 1;                        // 0:Pause, 1:Play
  uint8_t volume;                               // current volume
  uint16_t track;                               // current track number
  uint16_t tracks;                              // total number of tracks

  // Disable unused peripherals to save power
  ACSR   =  (1<<ACD);                           // disable analog comperator
  DIDR0  = ~((1<<RX_PIN)|(1<<IR_PIN)) & 0x1F;   // disable digital intput buffer except RX/IR pins
  PRR    =  (1<<PRADC)|(1<<PRTIM0);             // shut down ADC and timers
  sei();                                        // enable global interrupts

  // Read user settings from EEPROM
  volume = eeprom_read_byte((const uint8_t*)1); // read volume
  track = eeprom_read_word((const uint16_t*)2); // read last track
  if(volume > 30) volume = 10;                  // set standard value if incorrect reading

  // Setup IR receiver and DFPlayer
  IR_init();                                    // init IR receiver
  DFP_init();                                   // init DFPlayer
  DFP_setVolume(volume);                        // set volume
  tracks = DFP_getTracks();                     // query total number of tracks
  if(track > tracks) track = 1;                 // if incorrect track value
  DFP_playTrack(track);                         // play track

  // Loop
  while(1) {
    // Check and parse IR signal
    if(IR_available()) {
      uint8_t command = IR_read();      
      switch (command) {
        case IR_VOL_P:    if (volume < 30) DFP_setVolume(++volume); break;
        case IR_VOL_M:    if (volume) DFP_setVolume(--volume); break;
        case IR_TRK_P:    if (++track > tracks) track = 1;
                          DFP_playTrack(track); break;
        case IR_TRK_M:    if (--track == 0) track = tracks;
                          DFP_playTrack(track); break;
        case IR_PAUSE:    if (playstate) DFP_play();
                          else           DFP_pause();
                          playstate = !playstate; break;
        default:          break;
      }
      eeprom_update_byte ((uint8_t*)1, volume);   // update EEPROM (volume)
      eeprom_update_word((uint16_t*)2, track);    // update EEPROM (track)
    }

    // Check and parse DFPlayer message via UART
    if (UART_RX_PTR == 10) {                      // complete message received ?
      UART_RX_PTR = 0;                            // reset receiver buffer pointer
      if (UART_RX_BUF[3] == 0x3D) {               // if last track finished
        if (++track > tracks) track = 1;          // calculate next track
        DFP_playTrack(track);                     // play next track
        eeprom_update_word((uint16_t*)2, track);  // update EEPROM (track)
      }  
    }
  }
}
