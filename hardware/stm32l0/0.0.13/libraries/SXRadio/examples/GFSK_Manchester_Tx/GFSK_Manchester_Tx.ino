/* 
 * Simple GFSK, with Manchester encoding, TX and RX, synchword and preamble from ADS-L V2
 *    
 * This example code is in the public domain.
 */
 
#include <SPI.h>
#include <SXRadio.h>
#include <STM32L0.h>

void manchester_encode(uint8_t* dest_buffer, uint8_t* source_buffer, size_t src_length) {
    for (size_t i = 0; i < src_length; i++) {
        uint8_t d_byte = 0;

        uint8_t m_byte = *source_buffer++;

        d_byte = (d_byte << 2) | ((m_byte & 0x80) ? 0x01 : 0x02);
        m_byte <<= 1;
        d_byte = (d_byte << 2) | ((m_byte & 0x80) ? 0x01 : 0x02);
        m_byte <<= 1;
        d_byte = (d_byte << 2) | ((m_byte & 0x80) ? 0x01 : 0x02);
        m_byte <<= 1;
        d_byte = (d_byte << 2) | ((m_byte & 0x80) ? 0x01 : 0x02);
        *dest_buffer++ = d_byte;

        d_byte = 0;
        m_byte <<= 1;
        d_byte = (d_byte << 2) | ((m_byte & 0x80) ? 0x01 : 0x02);
        m_byte <<= 1;
        d_byte = (d_byte << 2) | ((m_byte & 0x80) ? 0x01 : 0x02);
        m_byte <<= 1;
        d_byte = (d_byte << 2) | ((m_byte & 0x80) ? 0x01 : 0x02);
        m_byte <<= 1;
        d_byte = (d_byte << 2) | ((m_byte & 0x80) ? 0x01 : 0x02);
        *dest_buffer++ = d_byte;
    }
}

Radio::SX1262Radio radio = Radio::SX1262Radio();

// Sync word as described in ADS-L
uint8_t sync_word[] = { 0x99, 0x72, 0x4B };

// Channel parameters for ADS-L (frequencies in [kHz], bitrate in [bps])

// Select band: 0 - M-Band 868.2, 1 - M-Band 868.4, 2 - O-Band LDR, 3 - O-Band HDR
#define TEST_BAND 2

// M-Band frequency = 868.200MHz, bitrate = 100kbps, deviation = 50kHz, BT = 0.5, mod. index = 2*50/100 = 1.0
#if (TEST_BAND == 0)
#define ADSL_FREQUENCY          868200
#define ADSL_DEVIATION          50000
#define ADSL_BITRATE            100000
#undef ADSL_GAUSS_BT

// M-Band frequency = 868.400MHz, bitrate = 100kbps, deviation = 50kHz, BT = 0.5, mod. index = 2*50/100 = 1.0
#elif (TEST_BAND == 1)
#define ADSL_FREQUENCY          868400
#define ADSL_DEVIATION          50000
#define ADSL_BITRATE            100000
#undef ADSL_GAUSS_BT

// O-Band LDR frequency = 869.525MHz, bitrate = 38.4kbps, deviation = 12.5kHz, BT = 1.0, mod. index = 2*12.5/38.4 = 0.65
#elif (TEST_BAND == 2)
#define ADSL_FREQUENCY          869525
#define ADSL_DEVIATION          12500
#define ADSL_BITRATE            38400
#define ADSL_GAUSS_BT           Radio::PulseShape::BT_1_0

// O-Band HDR frequency = 869.525MHz, bitrate = 200kbps,  deviation = 50kHz, BT = 0.5, mod. index = 2*50/200 = 0.5
#elif (TEST_BAND == 3)
#define ADSL_FREQUENCY          869525
#define ADSL_DEVIATION          50000
#define ADSL_BITRATE            200000
#undef ADSL_GAUSS_BT

#else
#error Wrong band index defined
#endif

// Transmit power as described in ADS-L
#define ADSL_TX_POWER           14

void setup(void) {

    STM32L0.wdtEnable(5000);

    Serial.begin(115200);
    while (!Serial) { }

    delay(2000);

    Serial.println("GFSK, Manchester encoding, TX");

    pinMode(LED_BUILTIN, OUTPUT);
    digitalWrite(LED_BUILTIN, LOW);

    radio.begin();
    #if defined(ADSL_GAUSS_BT)
    radio.configure_gfsk(ADSL_FREQUENCY, ADSL_BITRATE, ADSL_DEVIATION, sync_word, sizeof(sync_word), ADSL_GAUSS_BT);
    #else
    radio.configure_gfsk(ADSL_FREQUENCY, ADSL_BITRATE, ADSL_DEVIATION, sync_word, sizeof(sync_word));
    #endif
    radio.set_tx_power(ADSL_TX_POWER);
}

char message[32];
uint8_t man_enc[64];
int counter = 0;

void loop(void) {
    STM32L0.wdtReset();
    snprintf(message, sizeof(message), "GFSK TEST %d", counter);
    manchester_encode(man_enc, (uint8_t *)message, strlen(message)+1);
    radio.start_tx(man_enc, (strlen(message)+1)*2);
    while (!radio.tx_done());
    Serial.print("Packet ");
    Serial.print(counter++);
    Serial.println(" sent.");
    delay(1000);
}
