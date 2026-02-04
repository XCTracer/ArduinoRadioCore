/* 
 * Simple Ping-Pong for a LoRa Radio/Modem
 *
 * This example code is in the public domain.
 */
 
#include <SPI.h>
#include <SXRadio.h>
#include <STM32L0.h>

#define STATE_NONE          0
#define STATE_RECEIVING     1
#define STATE_TRANSMITTING  2

int state = STATE_NONE;

#define SWITCH_TIME         2000 // time to switch between states

uint32_t switch_timer = millis();

Radio::SX1262Radio radio = Radio::SX1262Radio();

uint8_t sync_word[] = { 0xf4, 0x14 };

void setup(void) {

    STM32L0.wdtEnable(5000);

    Serial.begin(115200);
    while (!Serial) { }

    delay(2000);

    Serial.println("LoRa Ping-pong test");

    pinMode(LED_BUILTIN, OUTPUT);
    digitalWrite(LED_BUILTIN, LOW);

    radio.begin();
    Serial.println("Starting...");
    radio.configure_lora(868200, 200, 7, 5, sync_word);
    radio.start_rx();

    state = STATE_RECEIVING;
}

const uint8_t message[] = "LoRa Test Message";

void loop(void) {
    STM32L0.wdtReset();
    switch (state) {
        case STATE_NONE:
            break;

        case STATE_RECEIVING:
            if (radio.has_packet()) {
                radio.start_rx();
                Serial.print("Data received: ");
                uint8_t buffer[64];
                Radio::PacketInfo info;
                radio.read_packet(buffer, sizeof(buffer), info);
                Serial.println(reinterpret_cast<char *>(buffer));
                switch_timer = millis();
                state = STATE_TRANSMITTING;
                digitalWrite(LED_BUILTIN, HIGH);
            } else if (millis() - switch_timer > SWITCH_TIME) {
                Serial.println("Timeout, switch to TX");
                switch_timer = millis();
                state = STATE_TRANSMITTING;
                digitalWrite(LED_BUILTIN, HIGH);
            }
            
            break;

        case STATE_TRANSMITTING:
            radio.start_tx(message, sizeof(message));
            while (!radio.tx_done());
            Serial.print("Packet sent: ");
            Serial.println(reinterpret_cast<const char *>(message));
            radio.start_rx();
            state = STATE_RECEIVING;
            digitalWrite(LED_BUILTIN, LOW);
            break;
    }
}
