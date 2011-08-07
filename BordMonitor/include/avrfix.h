/****************************************************************
 *                                                              *
 *          __ ____  _________                                  *
 *         /_ \\\  \/   /|    \______                           *
 *        //   \\\     /|| D  /_    /.                          *
 *       //     \\\_  /.||    \ ___/.                           *
 *      /___/\___\\__/. |__|\__\__.___  ___                     *
 *       ....  .......   ...||  _/_\  \////.                    *
 *                          || |.| |\  ///.                     *
 *                          |__|.|_|///  \                      *
 *                           .... ./__/\__\                     *
 *                                  ........                    *
 * Fixed Point Library                                          *
 * according to                                                 *
 * ISO/IEC DTR 18037                                            *
 *                                                              *
 * Version 1.0.1                                                *
 * Maximilan Rosenblattl, Andreas Wolf 2007-02-07               *
 ****************************************************************/

#ifndef _AVRFIX_H
#define _AVRFIX_H

#ifndef TEST_ON_PC
#include <avr/io.h>
#include <avr/interrupt.h>
#include <avr/pgmspace.h>
#endif

/* Only two datatypes are used from the ISO/IEC standard:
 * short _aAccum with s7.8 bit format
 *       _aAccum with s15.16 bit format
 * long  _aAccum with  s7.24 bit format
 */

typedef signed short _sAccum;
typedef signed long  _aAccum;
typedef signed long  _lAccum;

/* Pragmas for defining overflow behaviour */

#define DEFAULT    0
#define SAT        1

#ifndef FX_ACCUM_OVERFLOW
#define FX_ACCUM_OVERFLOW DEFAULT
#endif

/* Pragmas for internal use */

#define SACCUM_IBIT 7
#define SACCUM_FBIT 8
#define ACCUM_IBIT 15
#define ACCUM_FBIT 16
#define LACCUM_IBIT 7
#define LACCUM_FBIT 24

#define SACCUM_MIN -32767
#define SACCUM_MAX  32767
#define ACCUM_MIN -2147483647L
#define ACCUM_MAX  2147483647L
#define LACCUM_MIN -2147483647L
#define LACCUM_MAX  2147483647L

#define SACCUM_FACTOR ((short)1 << SACCUM_FBIT)
#define ACCUM_FACTOR ((long)1 << ACCUM_FBIT)
#define LACCUM_FACTOR ((long)1 << LACCUM_FBIT)

/* Mathematical constants */

#define PIsk 804
#define PIk 205887
#define PIlk 52707179

#define LOG2k  45426
#define LOG2lk 11629080

#define LOG10k 150902
#define LOG10lk 38630967

#ifndef NULL
#define NULL ((void*)0)
#endif

/* conversion Functions */

#define itosk(i) ((_sAccum)(i) << SACCUM_FBIT)
#define itok(i)  ((_aAccum)(i)  << ACCUM_FBIT)
#define itolk(i) ((_lAccum)(i) << LACCUM_FBIT)

#define sktoi(k) ((int8_t)((k) >> SACCUM_FBIT))
#define ktoi(k)  ((signed short)((k) >> ACCUM_FBIT))
#define lktoi(k) ((int8_t)((k) >> LACCUM_FBIT))

#define sktok(sk)  ( (_aAccum)(sk) << (ACCUM_FBIT-SACCUM_FBIT))
#define ktosk(k)   ((_sAccum)((k) >> (ACCUM_FBIT-SACCUM_FBIT)))

#define sktolk(sk) ((_lAccum)(sk) << (LACCUM_FBIT-SACCUM_FBIT))
#define lktosk(lk) ((_sAccum)((lk) >> (LACCUM_FBIT-SACCUM_FBIT)))

#define ktolk(k)   ((_aAccum)(k) << (LACCUM_FBIT-ACCUM_FBIT))
#define lktok(lk)  ((_lAccum)(lk) >> (LACCUM_FBIT-ACCUM_FBIT))

#define ftosk(f)  ((_aAccum)(f)  * (1 << SACCUM_FBIT))
#define ftok(f)   ((_aAccum)(f)  * (1 << ACCUM_FBIT))
#define ftolk(f)  ((_lAccum)(f) * (1 << LACCUM_FBIT))

#define sktof(sk) ((float)((_sAccum)(sk) / (1 << SACCUM_FBIT)))
#define ktod(k)   ((double)((_sAccum)(k) / (1 << SACCUM_FBIT)))
#define lktod(lk) ((double)((_sAccum)(lk) / (1 << SACCUM_FBIT)))

/* Main Functions */

extern _sAccum smulskD(_sAccum, _sAccum);
extern _aAccum mulkD(_aAccum, _aAccum);
extern _lAccum lmullkD(_lAccum, _lAccum);

extern _sAccum sdivskD(_sAccum, _sAccum);
extern _aAccum divkD(_aAccum, _aAccum);
extern _lAccum ldivlkD(_lAccum, _lAccum);

extern _sAccum smulskS(_sAccum, _sAccum);
extern _aAccum mulkS(_aAccum, _aAccum);
extern _lAccum lmullkS(_lAccum, _lAccum);

extern _sAccum sdivskS(_sAccum, _sAccum);
extern _aAccum divkS(_aAccum, _aAccum);
extern _lAccum ldivlkS(_lAccum, _lAccum);

#if FX_ACCUM_OVERFLOW == DEFAULT
  #define smulsk(a,b) smulskD((a),(b))
  #define mulk(a,b) mulkD((a),(b))
  #define lmullk(a,b) lmullkD((a), (b))
  #define sdivsk(a,b) sdivskD((a), (b))
  #define divk(a,b) divkD((a), (b))
  #define ldivlk(a,b) ldivlkD((a), (b))
#elif FX_ACCUM_OVERFLOW == SAT
  #define smulsk(a,b) smulskS((a),(b))
  #define mulk(a,b) mulkS((a),(b))
  #define lmullk(a,b) lmullkS((a), (b))
  #define sdivsk(a,b) sdivskS((a), (b))
  #define divk(a,b) divkS((a), (b))
  #define ldivlk(a,b) ldivlkS((a), (b))
#endif

/* Support Functions */

#define mulikD(i,k) ktoi((i) * (k))
#define mulilkD(i,lk) lktoi((i) * (lk))

#define divikD(i,k) ktoi(divkD(itok(i),(k)))
#define divilkD(i,lk) lktoi(ldivlkD(itolk(i),(lk)))

#define kdiviD(a,b) divkD(itok(a),itok(b))
#define lkdiviD(a,b) ldivlkD(itolk(a),itolk(b))

#define idivkD(a,b) ktoi(divkD((a),(b)))
#define idivlkD(a,b) lktoi(ldivlkD((a),(b)))

#define mulikS(i,k) ktoi(mulkS(itok(i),(k)))
#define mulilkS(i,lk) lktoi(lmullkS(itolk(i),(lk)))

#define divikS(i,k) ktoi(divkS(itok(i),(k)))
#define divilkS(i,lk) lktoi(ldivlkS(itolk(i),(lk)))

#define kdiviS(a,b) divkS(itok(a),itok(b))
#define lkdiviS(a,b) ldivlkS(itolk(a),itolk(b))

#define idivkS(a,b) ktoi(divkS((a),(b)))
#define idivlkS(a,b) lktoi(ldivlkS((a),(b)))

#if FX_ACCUM_OVERFLOW == DEFAULT
  #define mulik(a,b) mulikD((a),(b))
  #define mulilk(a,b) mulilkD((a),(b))
  #define divik(a,b) divikD((a),(b))
  #define divilk(a,b) divilkD((a),(b))
  #define kdivi(a,b) kdiviD((a),(b))
  #define lkdivi(a,b) lkdiviD((a),(b))
  #define idivk(a,b) idivkD((a),(b))
  #define idivlk(a,b) idivlkD((a),(b))
#elif FX_ACCUM_OVERFLOW == SAT
  #define mulik(a,b) mulikS((a),(b))
  #define mulilk(a,b) mulilkS((a),(b))
  #define divik(a,b) divikS((a),(b))
  #define divilk(a,b) divilkS((a),(b))
  #define kdivi(a,b) kdiviS((a),(b))
  #define lkdivi(a,b) lkdiviS((a),(b))
  #define idivk(a,b) idivkS((a),(b))
  #define idivlk(a,b) idivlkS((a),(b))
#endif

/* Abs Functions */

#define sabssk(f)  ((f) < 0 ? (-(f)) : (f))
#define absk(f)  ((f) < 0 ? (-(f)) : (f))
#define labslk(f) ((f) < 0 ? (-(f)) : (f))

/* Rounding Functions */

extern _sAccum roundskD(_sAccum f, uint8_t n);
extern _aAccum roundkD(_aAccum f, uint8_t n);
extern _lAccum roundlkD(_lAccum f, uint8_t n);

extern _sAccum roundskS(_sAccum f, uint8_t n);
extern _aAccum roundkS(_aAccum f, uint8_t n);
extern _lAccum roundlkS(_lAccum f, uint8_t n);

#if FX_ACCUM_OVERFLOW == DEFAULT
  #define roundsk(f, n) roundskD((f), (n))
  #define roundk(f, n) roundkD((f), (n))
  #define roundlk(f, n) roundlkD((f), (n))
#elif FX_ACCUM_OVERFLOW == SAT
  #define roundsk(f, n) roundskS((f), (n))
  #define roundk(f, n) roundkS((f), (n))
  #define roundlk(f, n) roundlkS((f), (n))
#endif

/* countls Functions */

extern uint8_t countlssk(_sAccum f);
extern uint8_t countlsk(_aAccum f);
#define countlslk(f) countlsk((f))

/* Special Functions */

#define CORDICC_GAIN 10188012
#define CORDICH_GAIN 20258445

extern _aAccum sqrtk_uncorrected(_aAccum,int8_t,uint8_t);

#define sqrtkD(a)   mulkD(sqrtk_uncorrected(a, -8, 17), CORDICH_GAIN/256)
#define lsqrtlkD(a) lmullkD(sqrtk_uncorrected(a, 0, 24), CORDICH_GAIN)

#define sqrtkS(a)   mulkS(sqrtk_uncorrected(a, -8, 17), CORDICH_GAIN/256)
#define lsqrtlkS(a) lmullkS(sqrtk_uncorrected(a, 0, 24), CORDICH_GAIN)

#if FX_ACCUM_OVERFLOW == DEFAULT
  #define sqrtk(a) sqrtkD(a)
  #define lsqrtlk(a) lsqrtlkD(a)
#else
  #define sqrtk(a) sqrtkS(a)
  #define lsqrtlk(a) lsqrtlkS(a)
#endif

extern _aAccum sincosk(_aAccum, _aAccum*);
extern _lAccum lsincoslk(_lAccum, _lAccum*);
extern _lAccum lsincosk(_aAccum, _lAccum*);
extern _sAccum ssincossk(_sAccum, _sAccum*);

#define sink(a)   sincosk((a), NULL)
#define lsinlk(a) lsincoslk((a), NULL)
#define lsink(a)  lsincosk((a), NULL)
#define ssinsk(a) ssincossk((a), NULL)

#define cosk(a)   sink((a) + PIk/2 + 1)
#define lcoslk(a) lsinlk((a) + PIlk/2)
#define lcosk(a)  lsink((a) + PIk/2 + 1)
#define scossk(a) ssinsk((a) + PIsk/2)

extern _aAccum tankD(_aAccum);
extern _lAccum ltanlkD(_lAccum);
extern _lAccum ltankD(_aAccum);

extern _aAccum tankS(_aAccum);
extern _lAccum ltanlkS(_lAccum);
extern _lAccum ltankS(_aAccum);

#if FX_ACCUM_OVERFLOW == DEFAULT
  #define tank(a) tankD((a))
  #define ltanlk(a) ltanlkD((a))
  #define ltank(a) ltankD((a))
#elif FX_ACCUM_OVERFLOW == SAT
  #define tank(a) tankS((a))
  #define ltanlk(a) ltanlkS((a))
  #define ltank(a) ltankS((a))
#endif

extern _aAccum atan2k(_aAccum, _aAccum);
extern _lAccum latan2lk(_lAccum, _lAccum);

#define atank(a) atan2k(itok(1), (a))
#define latanlk(a) latan2lk(itolk(1), (a))

extern _aAccum logk(_aAccum);
extern _lAccum lloglk(_lAccum);

#define log2k(x) (divk(logk((x)), LOG2k))
#define log10k(x) (divk(logk((x)), LOG10k))
#define logak(a, x) (divk(logk((x)), logk((a))))

#define llog2lk(x) (ldivlk(lloglk((x)), LOG2lk))
#define llog10lk(x) (ldivlk(lloglk((x)), LOG10lk))
#define llogalk(a, x) (ldivlk(lloglk((x)), lloglk((a))))

#endif /* _AVRFIX_H */

