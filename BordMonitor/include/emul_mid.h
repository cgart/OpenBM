/* 
 * File:   emul_mid.h
 * Author: tevs
 *
 * Emulation of MID interface
 * 
 * Created on June 4, 2010, 12:12 AM
 */

#ifndef _EMUL_MID_H
#define	_EMUL_MID_H

#ifdef	__cplusplus
extern "C"
{
#endif


/**
 * Initialize emulation of the MID device
 **/
extern void mid_init(void);

/**
 * Run MID emulation task. This will check for buttons and execute corresponding actions
 **/
extern void mid_tick(void);

/**
 * React on messages
 **/
extern void mid_on_bus_msg(uint8_t src, uint8_t dst, uint8_t* msg, uint8_t msglen);

extern void mid_stop(void);
extern void mid_resume(void);


#ifdef	__cplusplus
}
#endif

#endif	/* _EMUL_MID_H */

