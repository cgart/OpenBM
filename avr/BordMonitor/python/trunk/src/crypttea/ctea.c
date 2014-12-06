/* Copyright (C) 2007 Christian Heimes <christian (at) cheimes (dot) de>
   pyrexc optimized version
   License: Public Domain

   Source: http://en.wikipedia.org/wiki/XTEA
           http://www.cix.co.uk/~klockstone/xxtea.pdf
           http://www-users.cs.york.ac.uk/~matthew/TEA/
*/

#include "ctea.h"

/* BTEA decipher
 */

int btea_decipher(ULong *const v, const ULong *const k, const UInt n)
    {
    register UInt loop;
    register long xorkey, key=(6+52/n) * DELTA;
    while (key != 0) {
        xorkey = key>>2 & 3;
	loop = n-1;
	while (loop != 0) {
	    v[loop] -= (((v[loop - 1] << 4) ^ (v[loop - 1] >> 3)) +
            ((v[loop - 1] >> 5) ^ (v[loop - 1] << 2))) ^
            ((k[(loop & 3) ^ xorkey] ^ v[loop - 1]) + (key ^ v[loop - 1]));
	    loop -= 1;
	}
	v[0] -= (((v[n - 1] << 4) ^ (v[n - 1] >> 3)) +
        ((v[n - 1] >> 5) ^ (v[n - 1] << 2))) ^
        ((k[(0 & 3) ^ xorkey] ^ v[n - 1]) + (key ^ v[n - 1]));
        key -= DELTA;
        }
    return 0;
    }

int btea_encipher(ULong *const v, const ULong *const k, const UInt n) {
    return -1;
}

/* TEA decipher / encipher
 * v - array with 2 values
 * k - array with 4 values
 * n - number of rounds
 */

int tea_encipher(ULong *const v, const ULong *const k, const UInt n)
    {
    register ULong v0=v[0], v1=v[1], sum=0, i;
    register ULong k0=k[0], k1=k[1], k2=k[2], k3=k[3];
    for (i=0; i < n; i++) {
        sum += DELTA;
        v0 += ((v1<<4) + k0) ^ (v1 + sum) ^ ((v1>>5) + k1);
        v1 += ((v0<<4) + k2) ^ (v0 + sum) ^ ((v0>>5) + k3);
    }
    v[0]=v0; v[1]=v1;
    return 0;
    }

int tea_decipher(ULong *const v, const ULong *const k, const UInt n) 
    {
    register ULong v0=v[0], v1=v[1], sum=DELTA*n, i;
    register ULong k0=k[0], k1=k[1], k2=k[2], k3=k[3];
    for (i=0; i < n; i++) {
        v1 -= ((v0<<4) + k2) ^ (v0 + sum) ^ ((v0>>5) + k3);
        v0 -= ((v1<<4) + k0) ^ (v1 + sum) ^ ((v1>>5) + k1);
        sum -= DELTA;
    }
    v[0]=v0; v[1]=v1;
    return 0;
    }

/* XTEA decipher / encipher
 * v - array with 2 values
 * k - array with 4 values
 * n - number of rounds
 */

int xtea_decipher(ULong *const v, const ULong *const k, const UInt n)
    {
    register UInt i;
    register ULong sum=DELTA*n;
    for (i=0; i < n; i++) {
        v[1] -= ((v[0]<<4 ^ v[0]>>5) + v[0]) ^ (sum + k[sum>>11 & 3]);
        sum  -= DELTA;
        v[0] -= ((v[1]<<4 ^ v[1]>>5) + v[1]) ^ (sum + k[sum & 3]);
        }
    return 0;
    }

int xtea_encipher(ULong *const v, const ULong *const k, const UInt n)
    {
    register UInt i;
    register ULong sum=0;
    for (i=0; i < n; i++) {
        v[0] += ((v[1]<<4 ^ v[1]>>5) + v[1]) ^ (sum + k[sum & 3]);
        sum  += DELTA;
        v[1] += ((v[0]<<4 ^ v[0]>>5) + v[0]) ^ (sum + k[sum>>11 & 3]);
        }
    return 0;
    }

/* xxTEA or block TEA 2
 * Fixed version w/o underflow issue in z=v[n-1]
 * v - array with n values
 * k - array with 4 values
 * n - rounds and number of values in v
 */

#define MX (((z>>5)^(y<<2))+(((y>>3)^(z<<4))^(sum^y))+(k[(p&3)^e]^z));

int xxtea_encipher(ULong *const v, const ULong *const k, const UInt n)
    {
    register long p, q=6+52/n;
    register ULong z=v[n-1], y=v[0], e, sum=0;
    while (q-- > 0) {
        sum += DELTA;
        e = sum >> 2 & 3;
        for (p=0; p<n-1; p++) {
            y = v[p+1];
            z = v[p] += MX;
            }
        y = v[0];
        z = v[n-1] += MX;
        }
    return 0;
    }

int xxtea_decipher(ULong *const v, const ULong *const k, const UInt n)
    {
    register long p, q=6+52/n;
    register ULong z=v[n-1], y=v[0], e, sum=q*DELTA;
    while (sum != 0) {
        e = sum>>2 & 3;
        for (p=n-1; p>0; p--) {
            z = v[p-1];
            y = v[p] -= MX;
            }
        z = v[n-1];
        y = v[0] -= MX;
        sum -= DELTA;
        }
    return 0;
    }
