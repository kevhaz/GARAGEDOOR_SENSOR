#pragma once

#include <LoRa.h>
#include "security.h"

// These constants are defined so we can use LORA
#define LORA_SS   18
#define LORA_RST  14
#define LORA_DIO0 26

//
// I've noticed that a thread can get blocked when entering this function....
//

// ----------------------------------------------------------------
//
// Sends the status message via Lora
// Due to duty cycle restrictions it can't do this continuously
// so limit the number of calls to a timer
// ----------------------------------------------------------------
//void loraSendStatus(const t_DoorStatus& ds)
void loraSendMessage( char* msg )
{
  Serial.println( "gg_lora.h: loraSendMessage entered" );
  byte enc_iv[N_BLOCK] = { 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0 }; // iv_block gets written to, provide own fresh copy...
  char ciphertext[3*INPUT_BUFFER_LIMIT] = {0}; // THIS IS OUTPUT BUFFER (FOR BASE64-ENCODED ENCRYPTED DATA)
  uint16_t msgLen = strlen( msg );
  
  uint16_t encrypted_len = security_encrypt(msg, msgLen, enc_iv, ciphertext);
  Serial.println( msg );
  Serial.println( ciphertext );  

  LoRa.beginPacket();               // start packet
  LoRa.write(msgLen);               // add payload length
  LoRa.print(ciphertext);           // add payload
  LoRa.endPacket();  

  Serial.println( "gg_lora.h: loraSendMessage complete" );
  Serial.flush();
}

void init_Lora()
{
  Serial.println("gg_lora.h: init_Lora called!");
  //setup LoRa transceiver module
  LoRa.setPins(LORA_SS, LORA_RST, LORA_DIO0);
  while (!LoRa.begin(915E6)) {
    Serial.print(".");
    delay(500);
  }
  LoRa.enableCrc();
  LoRa.setSpreadingFactor(10);
  LoRa.setTxPower(20);
  LoRa.setGain(6);
  LoRa.setSyncWord(0xe3);
  
  Serial.println("gg_lora.h: LoRa Initialised OK!");
}

int onReceive(int packetSize, String& incoming, String& rssi, String& snr ) 
{  
  
  if (packetSize == 0) 
    return (0);          // if there's no packet, return

  byte messageLength = LoRa.read();    // incoming msg length
  incoming = "";

  while (LoRa.available()) {
    incoming += (char)LoRa.read();
  }

  // get details about the signal strength
  rssi = String(LoRa.packetRssi());
  snr = String(LoRa.packetSnr());  
    
  return messageLength;
}

/*

  // assume incomming is encrypted
  // Decrypt
  byte dec_iv[N_BLOCK] = { 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0 }; // iv_block gets written to, provide own fresh copy...
  uint16_t decrypted_len = security_decrypt((byte*)incoming.c_str(), incoming.length(), dec_iv, cleartext);  
  Serial.print("Decrypted: ");
  Serial.println( cleartext );

  if (messageLength != decrypted_len) {  // check length for error
    Serial.print("warning: message length does not match length. Trying to truncate");
    Serial.print( messageLength );
    Serial.print(" decrypted len was: "); 
    Serial.println( decrypted_len );
    if (decrypted_len > messageLength) {
      cleartext[messageLength] = '\0';
    }
    //return (false);                             // skip rest of function
  }
 
  // get details about the signal strength
  rssi = String(LoRa.packetRssi());
  snr = String(LoRa.packetSnr());

  // return the decypted text
  incoming = String(cleartext);
  return(true);
}

*/
