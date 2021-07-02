/* ATM90E36 Energy Monitor Functions
*/
#include "D:\jobESP\AC Under Voltage\Firmware\AC UnderV1.0\Inc\M90E32.h"

void Delay_usec(uint32_t count)
{
	for(uint8_t i=0; i<count; i++)
  {
		for(uint8_t j=0; j<20; j++)
		{
			__ASM volatile ("NOP");
			__ASM volatile ("NOP");
			__ASM volatile ("NOP");
			__ASM volatile ("NOP");
			__ASM volatile ("NOP");
			__ASM volatile ("NOP");
			__ASM volatile ("NOP");
			__ASM volatile ("NOP");
			__ASM volatile ("NOP");
		}
	}
}


