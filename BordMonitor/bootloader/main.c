/* 
 * File:   main.c
 * Author: tevs
 *
 * Created on November 15, 2010, 10:15 PM
 */
#include <avr/io.h>
#include <string.h>
#include <avr/boot.h>
#include <avr/pgmspace.h>
#include <util/crc16.h>
#include <stdlib.h>
#include <avr/wdt.h>

// From OpenBM
#include <base.h>
#include <leds.h>
#include <ibus.h>
#include <sha256.h>
#include <config.h>

// ----------------------------------------------------------------------------
typedef struct
{
	uint32_t dev_id;                // current device ID (unique)
	uint16_t app_version;           // currently flashed version
	uint16_t crc;                   // full checksum of the current program
        uint16_t pgm_counter;           // how often we have already flashed this device
        sha256_hash_t hash;             // password hash of this device (unique)
} bootldrinfo_t;


uint16_t startcluster;
uint16_t updatecluster;
bootldrinfo_t current_bootldrinfo;

ticks_t g_tickNumber = 0;

enum
{
    WAIT,
    ACTIVE_IDLE,
    NOT_MATCHED,
    MATCHED
}bootloaderMode = WAIT;


// ----------------------------------------------------------------------------
/* Make sure the watchdog is disabled as soon as possible    */
/* Copy this code to your bootloader if you use one and your */
/* MCU doesn't disable the WDT after reset!                  */
void get_mcusr(void) \
      __attribute__((naked)) \
      __attribute__((section(".init3")));
void get_mcusr(void)
{
  MCUSR = 0;
  wdt_disable();
}

// ----------------------------------------------------------------------------
// globale Variablen
static uint16_t flashpage = 0;
static uint8_t page_buffer_pos = 0;
static uint8_t page_buffer[SPM_PAGESIZE];

// ----------------------------------------------------------------------------
/**
 * \brief	starts the application program
 */
void app_start(void)
{
	// relocate interrupt vectors
	uint8_t reg = MCUCR & ~((1 << IVCE) | (1 << IVSEL));

	MCUCR = reg | (1 << IVCE);
	MCUCR = reg;

	// reset SPI interface to power-up state
	SPCR = 0;
	SPSR = 0;

#if FLASHEND > 0xffff
	__asm__ __volatile__(
			"push __zero_reg__" "\n\t"
			"push __zero_reg__" "\n\t"
			"push __zero_reg__" "\n\t");
#else
	__asm__ __volatile__(
			"push __zero_reg__" "\n\t"
			"push __zero_reg__" "\n\t");
#endif

	// when the functions executes the 'ret' command to return to
	// its origin the AVR loads the return address from the stack. Because we
	// pushed null it instead jumps to address null which starts the main
	// application.
}


//------------------------------------------------------------------------------
/**
 * React on messages received over ibus
 **/
void onibusMsg(uint8_t src, uint8_t dst, uint8_t* msg, uint8_t msglen)
{
    if (dst != IBUS_DEV_BMBT) return;

    if (msg[0] == IBUS_MSG_OPENBM_TO)
    {
        if (msg[1] == IBUS_MSG_OPENBM_SPECIAL_REQ && bootloaderMode == WAIT)
        {
            BEGIN_ATOMAR;
            {
                // read password contained in the message
                uint8_t pwd[132];
                memset(&pwd, 0, 132);
                uint8_t i = 0;

                for (i=2; i < msglen && i-2 < 128; i++)
                    pwd[i-2] = msg[i];

                // append device id to the password
                pwd[128] = (current_bootldrinfo.dev_id >> 24) & 0xFF;
                pwd[129] = (current_bootldrinfo.dev_id >> 16) & 0xFF;
                pwd[130] = (current_bootldrinfo.dev_id >> 8) & 0xFF;
                pwd[131] = (current_bootldrinfo.dev_id >> 0) & 0xFF;

                // compute hash value of the password
                sha256_ctx_t shaCtx;
                sha256_init(&shaCtx);
                sha256_lastBlock(&shaCtx, pwd, 132*8);

                sha256_hash_t pwdHash;
                sha256_ctx2hash(&pwdHash, &shaCtx);

                // check if hash of password matches to the stored hash
                for (i=0; i < SHA256_HASH_BYTES; i++)
                    if (pwdHash[i] != current_bootldrinfo.hash[i])
                        break;

                if (i != SHA256_HASH_BYTES)
                    bootloaderMode = NOT_MATCHED;
                else
                    bootloaderMode = MATCHED;
            }
            END_ATOMAR;
        }
    }

}

//------------------------------------------------------------------------------
/*
 * 
 */
int main(void)
{
    // Enable change of interrupt vectors
    MCUCR = (1<<IVCE);
    // Move interrupts to boot flash section
    MCUCR = (1<<IVSEL);

    // first we init only those hardware, which is required for proper bootloader handling
    tick_init();
    led_init();
    ibus_init();
    ibus_setMessageCallback(onibusMsg);
    sei();

    // get current flash information. This indicates current flashed version and other usefull things
    memcpy_P(&current_bootldrinfo, (uint8_t*) FLASHEND - BOOTLDRSIZE - sizeof(bootldrinfo_t) + 1, sizeof(bootldrinfo_t));
    if (current_bootldrinfo.app_version == 0xFFFF)
        current_bootldrinfo.app_version = 0; //application not flashed yet

    // send version string and wait for response over ibus
    uint8_t data[7] = {IBUS_MSG_OPENBM_FROM, current_bootldrinfo.app_version >> 8, current_bootldrinfo.app_version & 0xFF, 0, 0, 0, 0};
    ibus_sendMessage(IBUS_DEV_BMBT, IBUS_DEV_LOC, data, 7, 1);

    // enable LED to indicate start of the bootloader
    led_red_immediate_set(1);

    // wait for one secound to enter bootloader mode
    bootloaderMode = WAIT;
    while(tick_get() < TICKS_PER_SECOND() && bootloaderMode == WAIT)
    {
        if (tick_event())
            tick();

        ibus_tick();
    }

    led_red_immediate_set(0);

    // if not match or timeout, then start application
    if (bootloaderMode != MATCHED) while(1) app_start();


	//if (fat1216_init() == 0)
	{
		//for (i=0; i<512; i++)
		{
		//	startcluster = fat1216_readRootDirEntry(i);

		//	if (startcluster == 0xFFFF)
		//		continue;

		//	check_file();
		}

		//if (updatecluster)
		//	flash_update();
	}

	uint16_t flash_crc = 0xFFFF;

	uint16_t adr;

	for (adr=0; adr<FLASHEND - BOOTLDRSIZE + 1; adr++)
		flash_crc = _crc_ccitt_update(flash_crc, pgm_read_byte(adr));

	if (flash_crc == 0)
	{
		//Led off
		#ifdef USE_FLASH_LED
		FLASH_LED_DDR = 0x00;
		#endif
		app_start();
	}


    // start main application
    app_start();
}