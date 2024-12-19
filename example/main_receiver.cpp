#include <Arduino.h>
#include "iso-tp-twai/CanIsoTp.hpp"

#ifdef XIAO
uint8_t pinTX = D8;
uint8_t pinRX = D7;
#else
uint8_t pinTX = GPIO_NUM_13;
uint8_t pinRX = GPIO_NUM_12;
#endif
typedef struct {
    uint32_t counter;
} MessageData;

CanIsoTp isoTpReceiver;
MessageData rxData, txData;
pdu_t rxPdu, txPdu;

void setup() {
    Serial.begin(115200);
    if (!isoTpReceiver.begin(500, pinTX, pinRX)) {
        Serial.println("Failed to start TWAI");
        while (1);
    }

    // Setup Rx PDU for incoming data
    rxPdu.rxId = 0x123; 
    rxPdu.txId = 0x456;
    rxPdu.data = (uint8_t*)&rxData;
    rxPdu.len = sizeof(rxData);
    rxPdu.cantpState = CANTP_IDLE;
    rxPdu.blockSize = 0;
    rxPdu.separationTimeMin = 0;

    // Setup Tx PDU for responses
    txPdu.txId = 0x456;
    txPdu.rxId = 0x123;
    txPdu.data = (uint8_t*)&txData;
    txPdu.len = sizeof(txData);
    txPdu.cantpState = CANTP_IDLE;
    txPdu.blockSize = 0;
    txPdu.separationTimeMin = 5;

    Serial.println("Receiver ready.");
}

void loop() {
    // Try to receive incoming data
    int result = isoTpReceiver.receive(&rxPdu);
    if (result == 0 && rxPdu.cantpState == CANTP_IDLE) {
        Serial.print("Receiver: Received counter = ");
        Serial.println(rxData.counter);

        // Prepare response
        txData.counter = rxData.counter + 100; // Just an example modification
        txPdu.data = (uint8_t*)&txData;
        txPdu.len = sizeof(txData);

        // Send response
        if (isoTpReceiver.send(&txPdu) == 0) {
            Serial.print("Receiver: Sent response counter = ");
            Serial.println(txData.counter);
        } else {
            Serial.println("Receiver: Error sending response");
        }
    }
    delay(10);
}
