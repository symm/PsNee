// PsNee / psxdev.net version

// For Arduino and ATtiny
//
// Quick start: Select your hardware via the #defines, compile + upload the code, install in PSX.
// There are some pictures in the development thread ( http://www.psxdev.net/forum/viewtopic.php?f=47&t=1262&start=120 )
// Beware to use the PSX 3.5V / 3.3V power, *NOT* 5V! The installation pictures include an example.
//
// Arduinos:
//  - Arduino Pro Mini @8Mhz and @16Mhz (supported, tested)
//  - Arduino Uno @8Mhz and @16Mhz (supported, tested)
//  - Arduino Pro Micro has a different pin assignment and needs some easy porting. (ToDo)
//  - Use #define ARDUINO_BOARD
// ATtiny:
//  - ATtiny85: Should work the same as ATtiny45 (supported, untested)
//  - ATtiny45: LFUSE 0xE2  HFUSE 0xDF > internal oscillator, full 8Mhz speed (supported, tested)
//  - ATtiny25: Should work the same as ATtiny45 but doesn't have enough Flash nor RAM for PSNEEDEBUG (supported, untested)
//  - Use #define ATTINY_X5
//
// To use ATtiny with the Arduino environment, an ATtiny core has to be installed.
//
// PAL PM-41 consoles are supported with #define APPLY_PSONE_PAL_BIOS_PATCH,
// but only on boards with ATmega chips (Arduinos).
// Also, the Arduino must be flashed using SPI (deleting the bootloader), since I expect a signal ~1 second after power on.
//
// This code defaults to multi-region, meaning it will unlock PAL, NTSC-U and NTSC-J machines.
// You can optimize boot times for your console further. See "// inject symbols now" in the main loop.

// +-------------------------------------------------------------------------------------------+
// |                                  Choose your hardware!                                    |
// +-------------------------------------------------------------------------------------------+
// 2 main branches available:
//  - ATmega based > easy to use, fast and nice features for development, recommended
//  - ATtiny based > for minimal installs

// modchip hardware
#define ARDUINO_BOARD
//#define ATTINY_X5

// PSX configuration
#define FIXED_REGION 'A' // Set PSNee to only work with the specified region. Slightly improves boot times. Values: 'E' (SCEE, PAL), 'A' (SCEA, US) or 'I' (SCEI, JP)
//#define APPLY_PSONE_PAL_BIOS_PATCH

// miscellaneous
#define PSNEEDEBUG

#include <limits.h>

#if defined(ARDUINO_BOARD)
// board pins (code requires porting to reflect any changes)
#define SQCK     6 // connect to PSX HC-05 SQCK pin
#define SUBQ     7 // connect to PSX HC-05 SUBQ pin
#define CEI      8 // connect to point 6 in old modchip diagrams
#define WFCK     9 // connect to point 5 in old modchip diagrams
#define BIOS_A18 4 // connect to PSOne BIOS A18 (pin 31 on that chip)
#define BIOS_D2  5 // connect to PSOne BIOS D2 (pin 15 on that chip)

// MCU I/O definitions
#define SUBQ_PI      PIND     // MCU input port for SQCK/SUBQ sampling inputs
#define SQCK_BIT     SQCK     // PD6 "SQCK" < Mechacon pin 26 (PU-7 and early PU-8 Mechacons: pin 41)
#define SUBQ_BIT     SUBQ     // PD7 "SUBQ" < Mechacon pin 24 (PU-7 and early PU-8 Mechacons: pin 39)
#define CEI_PI       PINB     // MCU input port for WFCK/CEI
#define CEI_PO       PORTB    // MCU output port for CEI
#define CEI_BIT      0        // PB0
#define WFCK_BIT     1        // PB1
#define BIOS_PI      PIND
#define BIOS_PO      PORTD
#define BIOS_PD      DDRD
#define BIOS_A18_BIT BIOS_A18
#define BIOS_D2_BIT  BIOS_D2
#elif defined(ATTINY_X5) // ATtiny 25/45/85
// board pins (Do not change. Changing pins requires adjustments to MCU I/O definitions)
#define SQCK 0
#define SUBQ 1
#define CEI  2
#define WFCK 4
#define debugtx 3

// MCU I/O definitions
#define SUBQ_PI PINB
#define SQCK_BIT 0
#define SUBQ_BIT 1
#define CEI_PI PINB
#define CEI_PO PORTB
#define WFCK_BIT 4
#define CEI_BIT 2

#if defined(APPLY_PSONE_PAL_BIOS_PATCH)
#error "ATtiny does not support PAL PSOne patch yet!"
#endif

// extras
#define USINGSOFTWARESERIAL

#else
#error "Select a board!"
#endif

#if defined(PSNEEDEBUG) && defined(USINGSOFTWARESERIAL)
#include <SoftwareSerial.h>
SoftwareSerial mySerial(-1, 3); // RX, TX. (RX -1 = off)
#define DEBUG_PRINT(x)     mySerial.print(x)
#define DEBUG_PRINTHEX(x)  mySerial.print(x, HEX)
#define DEBUG_PRINTLN(x)   mySerial.println(x)
#define DEBUG_FLUSH        mySerial.flush()
#elif defined(PSNEEDEBUG) && !defined(USINGSOFTWARESERIAL)
#define DEBUG_PRINT(x)     Serial.print(x)
#define DEBUG_PRINTHEX(x)  Serial.print(x, HEX)
#define DEBUG_PRINTLN(x)   Serial.println(x)
#define DEBUG_FLUSH        Serial.flush()
#else
#define DEBUG_PRINT(x)
#define DEBUG_PRINTHEX(x)
#define DEBUG_PRINTLN(x)
#define DEBUG_FLUSH
#endif

// Setup() detects which (of 2) injection methods this PSX board requires, then stores it in pu22mode.
bool pu22mode;

// Timing
const int delay_between_bits = 4000;      // 250 bits/s (microseconds) (ATtiny 8Mhz works from 3950 to 4100)
const int delay_between_injections = 90;  // 72 in oldcrow. PU-22+ work best with 80 to 100 (milliseconds)

// inverted UART bit writing routine
void inject_bit(bool b)
{
	// pinMode(CEI, OUTPUT) is used more than it has to be but that's fine.
	if(b)
	{
		pinMode(CEI, OUTPUT);
		bitClear(CEI_PI, CEI_BIT); // data low
		delayMicroseconds(delay_between_bits);
	}
	else
	{
		if(pu22mode)
		{
			pinMode(CEI, OUTPUT);
			unsigned long now = micros();
			do
			{
				bool wfck_sample = bitRead(CEI_PI, WFCK_BIT);
				bitWrite(CEI_PO, CEI_BIT, wfck_sample); // output wfck signal on data pin
			} while((micros() - now) < delay_between_bits);
		}
		else   // PU-18 or lower mode
		{
			pinMode(CEI, INPUT);
			delayMicroseconds(delay_between_bits);
		}
	}
}

// inverted UART implementation with one start bit and two stop bits, no parity
void inject_byte(uint8_t b)
{
	// START bit
	inject_bit(0);
	
	// data
	for(uint8_t i = 0; i < CHAR_BIT; ++i)
	{
		// no variable shift operation on AVR, that's why we don't do it as b & (1 << i)
		inject_bit(b & 1);
		b >>= 1;
	}
	
	// two STOP bits
	inject_bit(1);
	inject_bit(1);
}

void inject_SCEX(char region)
{
	char SCEx[] = {'S', 'C', 'E', region};

	for(uint8_t i = 0; i < sizeof(SCEx); ++i)
		inject_byte((uint8_t)SCEx[i]);
	
	pinMode(CEI, OUTPUT);
	bitClear(CEI_PI, CEI_BIT); // pull data low
	delay(delay_between_injections);
}

void NTSC_fix()
{
#if defined(APPLY_PSONE_PAL_BIOS_PATCH)
	pinMode(BIOS_A18, INPUT);
	pinMode(BIOS_D2, INPUT);

	delay(100); // this is right after SQCK appeared. wait a little to avoid noise
	while(!bitRead(BIOS_PI, BIOS_A18_BIT))
		;  // wait for stage 1 A18 pulse
	delay(1350); // wait through stage 1 of A18 activity

	noInterrupts(); // start critical section
	while(!bitRead(BIOS_PI, BIOS_A18_BIT))
		;  // wait for priming A18 pulse
	delayMicroseconds(17); // max 17us for 16Mhz ATmega (maximize this when tuning!)
	bitClear(BIOS_PO, BIOS_D2_BIT); // store a low
	bitSet(BIOS_PD, BIOS_D2_BIT); // D2 = output. drags line low now
	delayMicroseconds(4); // min 2us for 16Mhz ATmega, 8Mhz requires 3us (minimize this when tuning, after maximizing first us delay!)
	bitClear(DDRD, BIOS_D2_BIT); // D2 = input / high-z
	interrupts(); // end critical section

	// not necessary but I want to make sure these pins are now high-z again
	pinMode(BIOS_A18, INPUT);
	pinMode(BIOS_D2, INPUT);
#endif
}

// --------------------------------------------------
//     Setup
// --------------------------------------------------

void setup()
{
	pinMode(CEI, INPUT);
	pinMode(WFCK, INPUT);
	pinMode(SUBQ, INPUT); // PSX subchannel bits
	pinMode(SQCK, INPUT); // PSX subchannel clock

#if defined(PSNEEDEBUG) && defined(USINGSOFTWARESERIAL)
	pinMode(debugtx, OUTPUT); // software serial tx pin
	mySerial.begin(115200); // 13,82 bytes in 12ms, max for softwareserial. (expected data: ~13 bytes / 12ms) // update: this is actually quicker
#elif defined(PSNEEDEBUG) && !defined(USINGSOFTWARESERIAL)
	Serial.begin(500000); // 60 bytes in 12ms (expected data: ~26 bytes / 12ms) // update: this is actually quicker
	DEBUG_PRINT("MCU frequency: ");
	DEBUG_PRINT(F_CPU);
	DEBUG_PRINTLN(" Hz");
	DEBUG_PRINTLN("Waiting for SQCK..");
#endif

#if defined(ARDUINO_BOARD)
	pinMode(LED_BUILTIN, OUTPUT); // Blink on injection / debug.
	digitalWrite(LED_BUILTIN, HIGH); // mark begin of setup
#endif

	// wait for console power on and stable signals
	while(!digitalRead(SQCK))
		;
	while(!digitalRead(WFCK))
		;

	// if enabled: patches PAL PSOne consoles so they start all region games
	NTSC_fix();

	// Board detection
	//
	// GATE: __-----------------------  // this is a PU-7 .. PU-20 board!
	//
	// WFCK: __-_-_-_-_-_-_-_-_-_-_-_-  // this is a PU-22 or newer board!

	unsigned int highs = 0, lows = 0;
	unsigned long now = millis();
	do
	{
		if(digitalRead(WFCK) == 1)
			highs++;
		if(digitalRead(WFCK) == 0)
			lows++;
		delayMicroseconds(200);   // good for ~5000 reads in 1s
	} while((millis() - now) < 1000); // sample 1s

	// typical readouts
	// PU-22: highs: 2449 lows: 2377
	if(lows > 100)
		pu22mode = 1;
	else
		pu22mode = 0;

#ifdef ATTINY_X5
	DEBUG_PRINT("m ");
	DEBUG_PRINTLN(pu22mode);
#else
	DEBUG_PRINT("highs: ");
	DEBUG_PRINT(highs);
	DEBUG_PRINT(" lows: ");
	DEBUG_PRINTLN(lows);
	DEBUG_PRINT("pu22mode: ");
	DEBUG_PRINTLN(pu22mode);

	// Power saving
	// Disable the ADC by setting the ADEN bit (bit 7)  of the ADCSRA register to zero.
	ADCSRA = ADCSRA & B01111111;

	// Disable the analog comparator by setting the ACD bit (bit 7) of the ACSR register to one.
	ACSR = B10000000;

	// Disable digital input buffers on all analog input pins by setting bits 0-5 of the DIDR0 register to one.
	DIDR0 = DIDR0 | B00111111;
#endif

#if defined(ARDUINO_BOARD)
	digitalWrite(LED_BUILTIN, LOW); // setup complete
#endif

	DEBUG_FLUSH; // empty serial transmit buffer
}

union SubQ
{
	struct
	{
		union
		{
			uint8_t control_adr;
			
			// adr and control is swapped here due a to a way compiler orders bit fields
			struct
			{
				uint8_t adr     : 4;
				uint8_t control : 4;
			};
		};
		
		union
		{
			struct TOC
			{
				uint8_t track_number;
				uint8_t point;
				
				struct MSF
				{
					uint8_t minute;
					uint8_t second;
					uint8_t frame;
				} address;
				uint8_t reserved;
				uint8_t data[3];
			} toc;
			uint8_t data[9];
		};
		uint16_t unknown;
	};
	
	uint8_t raw[12];
};

void loop()
{
	SubQ scq; // We will be capturing PSX "SUBQ" packets, there are 12 bytes per valid read.
	static unsigned int timeout_clock_counter = 0;
	static byte bitbuf = 0;   // SUBQ bit storage
	static bool sample = 0;
	static byte bitpos = 0;
	byte scpos = 0;           // scq raw position

	// start with a small delay, which can be necessary in cases where the MCU loops too quickly
	// and picks up the laster SUBQ trailing end
	delay(1);

	noInterrupts(); // start critical section
start:

	// Capture 8 bits for 12 runs > complete SUBQ transmission
	bitpos = 0;
	for(; bitpos < 8; bitpos++)
	{
		while(bitRead(SUBQ_PI, SQCK_BIT) == 1)
		{
			// wait for clock to go low..
			// a timeout resets the 12 byte stream in case the PSX sends malformatted clock pulses, as happens on bootup
			timeout_clock_counter++;
			if(timeout_clock_counter > 1000)
			{
				scpos = 0;  // reset SUBQ packet stream
				timeout_clock_counter = 0;
				bitbuf = 0;
				goto start;
			}
		}

		// wait for clock to go high..
		while((bitRead(SUBQ_PI, SQCK_BIT)) == 0)
			;

		sample = bitRead(SUBQ_PI, SUBQ_BIT);
		bitbuf |= sample << bitpos;

		timeout_clock_counter = 0; // no problem with this bit
	}

	// one byte done
	scq.raw[scpos] = bitbuf;
	scpos++;
	bitbuf = 0;

	// repeat for all 12 bytes
	if(scpos < 12)
		goto start;
	interrupts(); // end critical section

	// log SUBQ packets. We only have 12ms to get the logs written out. Slower MCUs get less formatting.
	for(unsigned int i = 0; i < 12; ++i)
	{
		if(scq.raw[i] < 0x10)
			DEBUG_PRINT("0");  // padding
		DEBUG_PRINTHEX(scq.raw[i]);
		DEBUG_PRINT(" ");
	}
	DEBUG_PRINTLN("");

	// check if read head is in wobble area
	// We only want to unlock game discs (0x41) and only if the read head is in the outer TOC area.
	// We want to see a TOC sector repeatedly before injecting (helps with timing and marginal lasers).
	// All this logic is because we don't know if the HC-05 is actually processing a getSCEX() command.
	// Hysteresis is used because older drives exhibit more variation in read head positioning.
	// While the laser lens moves to correct for the error, they can pick up a few TOC sectors.
	static byte hysteresis = 0;
	
	// the sector is in Lead-In TOC area and TOC is valid (garbage protection)
	if(scq.adr == 1 && !scq.toc.track_number && !scq.toc.reserved &&                          // reserved is always 0
	  (scq.toc.point == 0xA0 || scq.toc.point == 0xA1 || scq.toc.point == 0xA2 ||             // it's First Track Number (0xA0), Last Track Number (0xA1) or Lead-Out (0xA2)
	   scq.toc.point == 1 && (scq.toc.address.minute >= 152 || scq.toc.address.minute <= 2))) // or Track 1 located before 2 or after 152 minutes
	{
		// if it's data sector or audio sector where CD has the wobble into CD-DA space (started at 0x41, then went into 0x01)
		if(scq.control & (1 << 2) || hysteresis > 0)
			++hysteresis;
	}
	else if(hysteresis > 0)
		--hysteresis;
	
	// hysteresis value "optimized" using very worn but working drive on ATmega328 @ 16Mhz
	// should be fine on other MCUs and speeds, as the PSX dictates SUBQ rate
	if(hysteresis >= 14)
	{
		// If the read head is still here after injection, resending should be quick.
		// Hysteresis naturally goes to 0 otherwise (the read head moved).
		hysteresis = 11;

		DEBUG_PRINTLN("!");
		
#if defined(ARDUINO_BOARD)
		digitalWrite(LED_BUILTIN, HIGH);
#endif

		pinMode(CEI, OUTPUT);
		digitalWrite(CEI, 0); // pull data low
		if(!pu22mode)
		{
			pinMode(WFCK, OUTPUT);
			digitalWrite(WFCK, 0);
		}

		// HC-05 waits for a bit of silence (pin low) before it begins decoding.
		delay(delay_between_injections);

		// inject symbols now. 2 x 3 runs seems optimal to cover all boards
		for(byte loop_counter = 0; loop_counter < 2; loop_counter++)
		{
#ifdef FIXED_REGION
			inject_SCEX(FIXED_REGION);
			inject_SCEX(FIXED_REGION);
			inject_SCEX(FIXED_REGION);
#else
			inject_SCEX('E'); // E = SCEE, A = SCEA, I = SCEI
			inject_SCEX('A'); // injects all 3 regions by default
			inject_SCEX('I'); // optimize boot time by sending only your console region letter (all 3 times per loop)
#endif
		}

		if(!pu22mode)
			pinMode(WFCK, INPUT); // high-z the line, we're done
		pinMode(CEI, INPUT); // high-z the line, we're done
		
#if defined(ARDUINO_BOARD)
		digitalWrite(LED_BUILTIN, LOW);
#endif
	}

	// keep catching SUBQ packets forever
}
