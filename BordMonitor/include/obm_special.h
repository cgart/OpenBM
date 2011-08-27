/* 
 * File:   obm_special.h
 * Author: tevs
 *
 * Emulation of SPecial features:
 *  - automaticd central lock and unlock
 * 
 * Created on August 13, 2011, 12:12 AM
 */

#ifndef _OBM_SPECIAL_H
#define	_OBM_SPECIAL_H

#ifdef	__cplusplus
extern "C"
{
#endif

extern void obms_init(void);
extern void obms_tick(void);
extern void obms_on_bus_msg(uint8_t src, uint8_t dst, uint8_t* msg, uint8_t msglen);

#ifdef	__cplusplus
}
#endif

#endif	/* _EMUL_MID_H */

