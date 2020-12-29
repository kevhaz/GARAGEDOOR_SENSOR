#pragma once
#include <AESLib.h>

#define INPUT_BUFFER_LIMIT (128 + 1) // designed for Arduino UNO, not stress-tested anymore (this works with readBuffer[129])

AESLib aesLib;

// AES Encryption Key (same as in node-js example)
byte aes_key[]  = { 0xB4, 0x18, 0x3C, 0x1F, 0x38, 0x86, 0x2A, 0x40, 0xD3, 0x88, 0xFA, 0x28, 0x6B, 0xF3, 0x8A, 0x33 };

// General initialization vector (same as in node-js example) (you must use your own IV's in production for full security!!!)
byte aes_iv[N_BLOCK]  = { 0x79, 0x77, 0x98, 0x90, 0xB7, 0x34, 0xC3, 0xA1, 0x4D, 0x20, 0xE0, 0x10, 0x43, 0x3F, 0xC1, 0x2F };


// Generate IV (once)
void security_init() {
  aesLib.gen_iv(aes_iv);
}


uint16_t security_encrypt(char * msg, uint16_t msgLen, byte iv[], char* ciphertext) 
{
  Serial.println("Calling encrypt (string)...");
  int cipherlength = aesLib.encrypt((byte*)msg, msgLen, ciphertext, aes_key, sizeof(aes_key), iv);
  Serial.print("Encrypted bytes: "); 
  Serial.println(cipherlength);
  return cipherlength;
}

uint16_t security_decrypt(byte *msg, uint16_t msgLen, byte iv[], char* cleartext) {
  Serial.println("Calling decrypt...; ");
  
  uint16_t dec_bytes = aesLib.decrypt(msg, msgLen, cleartext, aes_key, sizeof(aes_key), iv);
  Serial.print("Decrypted bytes: "); 
  Serial.println(dec_bytes);
  return dec_bytes;
}
