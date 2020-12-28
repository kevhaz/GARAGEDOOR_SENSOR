#pragma once

#include <AESLib.h>
//#define INCLUDE_SECURITY

// Encryption.
// AES Encryption Key
AESLib aesLib;

// salt=25B61CA381BF15AC
byte aes_key[]        = { 0xB4, 0x18, 0x3C, 0x1F, 0x38, 0x86, 0x2A, 0x40, 0xD3, 0x88, 0xFA, 0x28, 0x6B, 0xF3, 0x8A, 0x33 };
byte aes_iv[N_BLOCK]  = { 0x79, 0x77, 0x98, 0x90, 0xB7, 0x34, 0xC3, 0xA1, 0x4D, 0x20, 0xE0, 0x10, 0x43, 0x3F, 0xC1, 0x2F };


// ----------------------------------------------------------------
//
// Encrypt Message
// ----------------------------------------------------------------
String security_encrypt(char * msg, byte iv[]) {
 #ifdef INCLUDE_SECURITY
  int msgLen = strlen(msg);
  char encrypted[4 * msgLen];
  aesLib.encrypt64(msg, msgLen, encrypted, aes_key, sizeof(aes_key), iv); 
  return String(encrypted);
#else
  return String(msg);
#endif
}

// ----------------------------------------------------------------
//
// Decrypt Message
// ----------------------------------------------------------------
String security_decrypt(char * msg, byte iv[]) {
#ifdef INCLUDE_SECURITY
  unsigned long ms = micros();
  int msgLen = strlen(msg);
  char decrypted[msgLen]; // half may be enough
  aesLib.decrypt64(msg, msgLen, decrypted, aes_key, sizeof(aes_key), iv); 
  return String(decrypted);
#else
  return String(msg);
#endif
}

// ----------------------------------------------------------------
//
// Generate IV (once)
// ----------------------------------------------------------------
void security_init() {
#ifdef INCLUDE_SECURITY
  aesLib.gen_iv(aes_iv);
  // workaround for incorrect B64 functionality on first run...
  security_encrypt("HELLO WORLD!", aes_iv);
 #else
  return;
#endif
}
