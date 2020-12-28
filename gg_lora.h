#pragma once

#include <LoRa.h>
#include "security.h"

// These constants are defined so we can use LORA
#define LORA_SS   18
#define LORA_RST  14
#define LORA_DIO0 26

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
  String encrypted = security_encrypt(msg, enc_iv);
  
  Serial.println( msg );
  Serial.println( encrypted );

  LoRa.beginPacket();                   // start packet
  LoRa.write(strlen(msg));              // add payload length
  LoRa.print(encrypted);                // add payload
  LoRa.endPacket();

  Serial.println( "gg_lora.h: loraSendMessage complete" );
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
  LoRa.setSyncWord(0xe3);
  Serial.println("gg_lora.h: LoRa Initialised OK!");
}

bool onReceive(int packetSize, String& incoming, String& rssi, String& snr ) 
{
  if (packetSize == 0) return (false);          // if there's no packet, return

  byte messageLength = LoRa.read();    // incoming msg length
  incoming = "";

  while (LoRa.available()) {
    incoming += (char)LoRa.read();
  }
  // assume incomming is encrypted
  // Decrypt
  byte dec_iv[N_BLOCK] = { 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0 }; // iv_block gets written to, provide own fresh copy...
  String decrypted = security_decrypt((char*)incoming.c_str(), dec_iv);  
  incoming = decrypted;
  Serial.print("Decrypted: ");
  Serial.println( decrypted );

  if (messageLength != decrypted.length()) {  // check length for error
    Serial.print("warning: message length does not match length: ");
    Serial.print( messageLength );
    Serial.print(" decrypted len was: "); 
    Serial.println( decrypted.length() );
    return (false);                             // skip rest of function
  }
 
  // get details about the signal strength
  rssi = String(LoRa.packetRssi());
  snr = String(LoRa.packetSnr());
  
  return(true);
}


