/* 
 * File:   main.c
 * Author: tevs
 *
 * Created on November 15, 2010, 10:15 PM
 */
#include <avr/io.h>
#include <avr/boot.h>
#include <avr/pgmspace.h>
#include <util/crc16.h>
#include <stdlib.h>
#include <avr/wdt.h>
#include <util/crc16.h>

// From OpenBM
#include <base.h>
#include <ibus.h>
#include <xtea.h>
#include <config.h>

#define USE_LED

#ifdef USE_LED
#include <leds.h>
#endif


// ----------------------------------------------------------------------------
#define WAIT                    (1 << 0)
#define ACTIVE_IDLE             (1 << 1)
#define NOT_MATCHED             (1 << 2)
#define MATCHED                 (1 << 3)
#define RECEIVE                 (1 << 4)
#define RECEIVE_INIT            (RECEIVE | (1 << 5))
#define FAILED                  (1 << 6)
#define SUCCESS                 (1 << 7)

uint8_t bootloaderMode = WAIT;


// ----------------------------------------------------------------------------
static uint16_t flashpage = 0;
static uint8_t page_buffer_pos = 0;
static uint8_t page_buffer[SPM_PAGESIZE];
static uint8_t seed = 0;
static uint8_t last_fill = 0;
static xtea_key_t encKey;
static uint16_t crc16 = 0;
static uint8_t cipherBlockPerPage = SPM_PAGESIZE >> 3;
static bootldrinfo_t current_bootldrinfo;

ticks_t g_tickNumber = 0;

// ----------------------------------------------------------------------------
#define ERROR_OK                   0
#define ERROR_WRONG_INIT_CHUNK     0x0101
#define ERROR_WRONG_CHUNK_IND      0x0102
#define ERROR_PASSWORD_MISMATCH    0x0201

#if 0
// ----------------------------------------------------------------------------
/* Make sure the watchdog is disabled as soon as possible    */
/* Copy this code to your bootloader if you use one and your */
/* MCU doesn't disable the WDT after reset!                  */
void get_mcusr(void) 
      __attribute__((naked)) 
      __attribute__((section(".init3")));
//     BOOTLOADER_SECTION;
void get_mcusr(void)
{
  MCUSR = 0;
  wdt_disable();
}
#endif

// ----------------------------------------------------------------------------
/**
 * \brief	starts the application program
 */
//void app_start(void) BOOTLOADER_SECTION;
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

// ----------------------------------------------------------------------------
/**
 * \brief	write a complete page to the flash memorey
 *
 * \param	page	page which should be written
 * \param	*buf	Pointer to the buffer with the data
 *
 * \see		avr-libc Documentation > Modules > Bootloader Support Utilities
 */
//void boot_program_page(uint16_t page, uint8_t *buf) BOOTLOADER_SECTION;
void boot_program_page(uint16_t page, uint8_t *buf)
{
    uint32_t adr = page * SPM_PAGESIZE;

    //eeprom_busy_wait();
    
    boot_page_erase(adr);
    boot_spm_busy_wait();	  // Wait until the memory is erased.

    for (uint16_t i=0; i < SPM_PAGESIZE; i+=2)
    {
        // Set up little-endian word.
        uint16_t w = *buf++;
        w += (*buf++) << 8;

        boot_page_fill(adr + i, w);
    }

    boot_page_write(adr);		// Store buffer in flash page.
    boot_spm_busy_wait();		// Wait until the memory is written.

    // Reenable RWW-section again. We need this if we want to jump back
    // to the application after bootloading.
    boot_rww_enable();
}

// ----------------------------------------------------------------------------
/**
 * \brief	Send an error code message over ibus
 */
//void sendErrorCode(uint16_t code) BOOTLOADER_SECTION;
void sendErrorCode(uint16_t code)
{
    uint8_t data[3] = {IBUS_MSG_OPENBM_FROM, (code >> 8), code};
    ibus_sendMessage(IBUS_DEV_BMBT, IBUS_DEV_LOC, data, 3, 3);
}

// ----------------------------------------------------------------------------
/**
 * \brief	Send acknowledge message over ibus
 */
//void sendAck(uint8_t datasum) BOOTLOADER_SECTION;
void sendAck(uint8_t datasum)
{
    uint8_t data[2] = {IBUS_MSG_OPENBM_FROM, seed^datasum};
    ibus_sendMessage(IBUS_DEV_BMBT, IBUS_DEV_LOC, data, 2, 3);
}

//------------------------------------------------------------------------------
/**
 * React on messages received over ibus
 **/
//void onibusMsg(uint8_t src, uint8_t dst, uint8_t* msg, uint8_t msglen) BOOTLOADER_SECTION;
void onibusMsg(uint8_t src, uint8_t dst, uint8_t* msg, uint8_t msglen)
{
    if (dst != IBUS_DEV_BMBT || msg[0] != IBUS_MSG_OPENBM_TO || msglen < 4
            || msg[1] != IBUS_MSG_OPENBM_SPECIAL_REQ) return;

    BEGIN_ATOMAR;

    // we are in the state to wait for password
    if (bootloaderMode == WAIT && msglen == 6)
    {
        // read password contained in the message
        memcpy(&encKey[2], current_bootldrinfo.dev_key, 12);
        encKey[0] = msg[2];
        encKey[1] = msg[3];
        encKey[14] = msg[4];
        encKey[15] = msg[5];
        
        // compute xor sum of the sent password and confirm it
        uint8_t sum = msg[2];
        sum ^= msg[3];
        sum ^= msg[4];
        sum ^= msg[5];
        bootloaderMode = MATCHED;

        // confirm the given password and prepare for upload
        sendErrorCode(sum);

    // we are in the state to receive first frame
    }else if (bootloaderMode & RECEIVE_INIT)
    {
        if (msglen != 7)
        {
            sendErrorCode(ERROR_WRONG_INIT_CHUNK);
            bootloaderMode = FAILED;
            goto stop;
        }

        HILO(current_bootldrinfo.app_version, msg[2], msg[3]);
        HILO(current_bootldrinfo.crc16, msg[4], msg[5]);
        seed = msg[6];

        sendAck(0);

        bootloaderMode = RECEIVE;

    // every other chunk is our data chunk
    }else if (bootloaderMode & RECEIVE)
    {
        uint8_t cnd = msg[2];

        // only if do not want to repeat the last chunk, we proceed further
        if (cnd != 0)
        {
            // if page already full, then write it to the flash
            if (page_buffer_pos >= SPM_PAGESIZE || cnd == 0xFF)
            {
                // encode received chunk into a temporary buffer
                uint8_t dec_buffer[SPM_PAGESIZE];
                uint8_t i;
                for (i=0; i < cipherBlockPerPage; i++)
                    xtea_dec(&dec_buffer[i<<3], &page_buffer[i<<3], encKey);

                // compute crc16-checksum of the encoded firmware
                uint16_t crc = crc16;
                for (i=0; i < SPM_PAGESIZE; i++)
                {
                    crc = _crc16_update(crc16, dec_buffer[i]);
                    crc16 = crc;
                }

                // write encoded chunk to memory
                boot_program_page(flashpage, dec_buffer);
                flashpage++;
                page_buffer_pos = 0;
                memset(page_buffer, 0, SPM_PAGESIZE);
            }else
                page_buffer_pos += last_fill;

        }

        // if last frame then do nothing from here, because it was already uploaded
        if (cnd == 0xFF)
        {
            bootloaderMode = SUCCESS;
            goto stop;
        }

        // get size of data in the chunk and free space in our flash page
        uint8_t msgsize = msglen-4;
        uint8_t free = SPM_PAGESIZE - page_buffer_pos;
        last_fill = free > msgsize ? msgsize : free;

        memcpy(&page_buffer[page_buffer_pos], &msg[4], last_fill);

        uint8_t msgsum = 0; uint8_t i;
        for (i=4; i < msglen; i++)
            msgsum ^= msg[i];

        sendAck(msgsum);
        tick_init();
    }

stop:
    END_ATOMAR;
}

void blink(void)
{
    led_radio_immediate_set(1);
    _delay_ms(500);
    led_radio_immediate_set(0);
    _delay_ms(500);
}

//------------------------------------------------------------------------------
int main(void) __attribute__((naked, OS_main)); //BOOTLOADER_SECTION;
//__attribute__ ((noreturn));
/*
 * 
 */
int main(void)
{
    uint8_t temp;
    temp = MCUCR;
    MCUCR = temp|(1<<IVCE);
    MCUCR = temp|(1<<IVSEL);

    // disable not needed-hardware
    PRR = 0;
    PRR |= (1 << PRUSART1);   // disable USART-1
    PRR |= (1 << PRSPI);      // disable SPI

begin:
    // first we init only those hardware, which is required for proper bootloader handling
    tick_init();
    ibus_init();
    #ifdef USE_LED
    led_init();
    #endif

    sei();
    ibus_setMessageCallback(onibusMsg);

    // get current flash information. This indicates current flashed version and other usefull things
    memcpy_P(&current_bootldrinfo, (PGM_VOID_P)(BOOTLOADERSTARTADR - SPM_PAGESIZE), sizeof(bootldrinfo_t));
    if (current_bootldrinfo.app_version == 0xFFFF || current_bootldrinfo.app_version == 0)
    {
        current_bootldrinfo.app_version  = (uint16_t)VERSION_MAJOR << 8;
        current_bootldrinfo.app_version |= (uint16_t)VERSION_MINOR;

        current_bootldrinfo.prognum = 0;

        // load device ID from preprocessor definitions into the array
        #define CONCATx(a,b) a##b
        #define CONCAT(a,b) CONCATx(a,b)
        #define  DID(i) CONCAT(0x,CONCAT(DEVID_,i))
        current_bootldrinfo.dev_key[0] = DID(1);
        current_bootldrinfo.dev_key[1] = DID(2);
        current_bootldrinfo.dev_key[2] = DID(3);
        current_bootldrinfo.dev_key[3] = DID(4);
        current_bootldrinfo.dev_key[4] = DID(5);
        current_bootldrinfo.dev_key[5] = DID(6);
        current_bootldrinfo.dev_key[6] = DID(7);
        current_bootldrinfo.dev_key[7] = DID(8);
        current_bootldrinfo.dev_key[8] = DID(9);
        current_bootldrinfo.dev_key[9] = DID(10);
        current_bootldrinfo.dev_key[10] = DID(11);
        current_bootldrinfo.dev_key[11] = DID(12);
    }

    //----------------------------- Wait For Password --------------------------
    // send version string and wait for response over ibus
    {
        uint8_t data[] = {IBUS_MSG_OPENBM_FROM, IBUS_MSG_OPENBM_SPECIAL_REQ, current_bootldrinfo.app_version >> 8, current_bootldrinfo.app_version & 0xFF,
                            SPM_PAGESIZE,
                            0,0,0,0};

        ibus_sendMessage(IBUS_DEV_BMBT, IBUS_DEV_LOC, data, 9, 10);
    }

    ticks_t nextTick;
    
    // wait for one second until either we continue with bootloader or go to main app
    #ifdef USE_LED
    led_red_immediate_set(1);
    #endif
    bootloaderMode = WAIT;
    g_tickNumber = 0;
    nextTick = g_tickNumber + (ticks_t)TICKS_PER_SECOND() * (ticks_t)2;
    do{
        if (tick_event()) tick();
        ibus_tick();
    }while((bootloaderMode == WAIT) && (tick_get() < nextTick));
    #ifdef USE_LED
    led_red_immediate_set(0);
    #endif

    // if not match or timeout, then start application
    if (bootloaderMode != MATCHED) goto start;

    //------------------------------ Receive Frames ----------------------------
    // go into frame receive mode
    #ifdef USE_LED
    led_yellow_immediate_set(1);
    #endif
    bootloaderMode = RECEIVE_INIT;
    nextTick = g_tickNumber + (ticks_t)TICKS_PER_SECOND() * (ticks_t)10;
    do{
        if (tick_event()) tick();
        ibus_tick();
    }while((bootloaderMode & RECEIVE) && (tick_get() < nextTick));
    #ifdef USE_LED
    led_yellow_immediate_set(0);
    #endif

    // compare checksum and restart bootloader if wrong
    if (crc16 != current_bootldrinfo.crc16) goto begin;

    // write new bootloader info into the flash
    current_bootldrinfo.prognum++;
    memset(&page_buffer[0], 0xFF, SPM_PAGESIZE);
    memcpy(&page_buffer[0], &current_bootldrinfo, sizeof(bootldrinfo_t));
    boot_program_page(LAST_PAGE, page_buffer);

start:
    // start main application
    app_start();
}
