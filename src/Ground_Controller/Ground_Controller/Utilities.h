#ifndef UTILITIES_H
#define UTILITIES_H


// Delays the MCU for the given length of clock cycles
void Delay(unsigned long long length);
// Given a string of characters, returns a compound xor checksum in character hex format
void Xor_Checksum(char *data, unsigned char length, unsigned char start_index, char checksum_hex[3]);


#endif