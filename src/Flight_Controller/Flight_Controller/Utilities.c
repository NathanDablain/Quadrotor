#include "Utilities.h"
#include <stdio.h>

void Delay(unsigned long long length){
	volatile unsigned long long i = 0;
	while (++i < length);
}

void Xor_Checksum(char *data, unsigned char length, unsigned char start_index, char checksum_hex[3]){
	// checksum_hex must be a null terminated array of 3 characters
	signed char checksum = data[start_index];
	for (unsigned char i=start_index+1; i<length; i++){
		checksum ^= data[i];
	}
	unsigned char converted_length = snprintf(checksum_hex, 3, "%X", checksum);
	if (converted_length == 1){ // Won't add the 0 in automatically if the number is less than 8
		checksum_hex[1] = checksum_hex[0];
		checksum_hex[0] = '0';
	}
}