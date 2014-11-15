#include "base.h"
#include <avr/delay.h>

int main()
{
DDRD = 0xFF;
while (1)
{
PORTD = ~PIND;
//nop();  // added some nop, to slow the clock a bit
//nop();
//_delay_ms(10);
}


}