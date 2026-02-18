#ifndef SRC_SXRADIO
#define SRC_SXRADIO

#include <Arduino.h>
#include <SPI.h>

namespace Radio {

    #define IRQ_TX_DONE                 (1 << 0)
    #define IRQ_RX_DONE                 (1 << 1)
    #define IRQ_PREAMBLE_DETECTED       (1 << 2)
    #define IRQ_SYNC_WORD_VALID         (1 << 3)
    #define IRQ_HEADER_VALID            (1 << 4)
    #define IRQ_HEADER_ERR              (1 << 5)
    #define IRQ_CRC_ERR                 (1 << 6)
    #define IRQ_CAD_DONE                (1 << 7)
    #define IRQ_CAD_DETECTED            (1 << 8)
    #define IRQ_TIMEOUT                 (1 << 9)

    enum class RadioMode {
        MODE_STANDBY,
        MODE_RX,
        MODE_TX
    };

    enum class Modulation {
        LoRa,
        GFSK,
        Unknown
    };

    enum class PulseShape : uint8_t {
        NO_FILTER = 0x00,
        BT_0_3    = 0x08,
        BT_0_5    = 0x09,
        BT_0_7    = 0x0A,
        BT_1_0    = 0x0B
    };

    enum class LoRaBandwidth : uint8_t {
        BW_7_8    = 0x00,  // 7.8 kHz
        BW_10_4   = 0x08,  // 10.4 kHz
        BW_15_6   = 0x01,  // 15.6 kHz
        BW_20_8   = 0x09,  // 20.8 kHz
        BW_31_25  = 0x02,  // 31.25 kHz
        BW_41_7   = 0x0A,  // 41.7 kHz
        BW_62_5   = 0x03,  // 62.5 kHz
        BW_125    = 0x04,  // 125 kHz
        BW_250    = 0x05,  // 250 kHz
        BW_500    = 0x06   // 500 kHz
    };

    enum class GfskBandwidth : uint8_t {
        BW_NONE    = 0x00,  // Uninitialized/Undefined
        BW_4_8     = 0x1F,  // 4.8 kHz
        BW_5_8     = 0x17,  // 5.8 kHz
        BW_7_3     = 0x0F,  // 7.3 kHz
        BW_9_7     = 0x1E,  // 9.7 kHz
        BW_11_7    = 0x16,  // 11.7 kHz
        BW_14_6    = 0x0E,  // 14.6 kHz
        BW_19_5    = 0x1D,  // 19.5 kHz
        BW_23_4    = 0x15,  // 23.4 kHz
        BW_29_3    = 0x0D,  // 29.3 kHz
        BW_39_0    = 0x1C,  // 39.0 kHz
        BW_46_9    = 0x14,  // 46.9 kHz
        BW_58_6    = 0x0C,  // 58.6 kHz
        BW_78_2    = 0x1B,  // 78.2 kHz
        BW_93_8    = 0x13,  // 93.8 kHz
        BW_117_3   = 0x0B,  // 117.3 kHz
        BW_156_2   = 0x1A,  // 156.2 kHz
        BW_187_2   = 0x12,  // 187.2 kHz
        BW_234_3   = 0x0A,  // 234.3 kHz
        BW_312_0   = 0x19,  // 312.0 kHz
        BW_373_6   = 0x11,  // 373.6 kHz
        BW_467_0   = 0x09   // 467.0 kHz
    };

    class PacketInfo {
        public:
            int8_t      rssi;
            int8_t      snr;            // LoRa only
            uint32_t    timestamp_ms;   // Time since PPS
    };

    class SX1262Radio {

        public:
            SX1262Radio();

            // Initialization
            void begin(void);
            void reset(void);

            // Configuration
            void configure_lora(uint32_t freq_khz, uint16_t bw_khz, uint8_t sf, uint8_t cr, uint8_t *sync_word);
            void configure_gfsk(uint32_t freq_khz, uint32_t bitrate, uint32_t fdev, 
                uint8_t *sync_word, size_t sw_len, PulseShape pulse = PulseShape::BT_0_5, GfskBandwidth bw = GfskBandwidth::BW_234_3);
            void set_tx_power(int8_t power_dbm);

            // Mode control
            void standby(int mode);
            void start_rx(void);
            void start_tx(const uint8_t* data, uint8_t len);

            // Data
            uint8_t read_packet(uint8_t* buf, uint8_t max_len, PacketInfo &info);
            void get_packet_info(PacketInfo &info);

            // IRQ handling
            uint16_t get_irq_status(void);
            void clear_irq_status(uint16_t mask);
            void clear_irq(void);  // Clear all IRQ flags

            // Status
            bool has_packet(void);
            bool tx_done(void);
            RadioMode get_mode(void);
            Modulation get_modulation(void);

            // Test mode support
            void set_continuous_wave(uint32_t freq_khz, int8_t power_dbm);
            int16_t get_rssi_inst(void);

        private:

            enum class Command : uint8_t {
                SET_SLEEP               = 0x84,
                SET_STANDBY             = 0x80,
                SET_FS                  = 0xC1,
                SET_TX                  = 0x83,
                SET_RX                  = 0x82,
                STOP_TIMER_ON_PREAMBLE  = 0x9F,
                SET_CAD                 = 0xC5,
                SET_TX_CW               = 0xD1,
                SET_TX_INFINITE_PREAMBLE = 0xD2,
                SET_REGULATOR_MODE      = 0x96,
                CALIBRATE               = 0x89,
                CALIBRATE_IMAGE         = 0x98,
                SET_RX_GAIN             = 0x96,
                SET_PA_CONFIG           = 0x95,
                SET_RX_TX_FALLBACK_MODE = 0x93,
                WRITE_REGISTER          = 0x0D,
                READ_REGISTER           = 0x1D,
                WRITE_BUFFER            = 0x0E,
                READ_BUFFER             = 0x1E,
                SET_DIO_IRQ_PARAMS      = 0x08,
                GET_IRQ_STATUS          = 0x12,
                CLR_IRQ_STATUS          = 0x02,
                SET_DIO2_AS_RF_SWITCH   = 0x9D,
                SET_DIO3_AS_TCXO_CTRL   = 0x97,
                SET_RF_FREQUENCY        = 0x86,
                SET_PKT_TYPE            = 0x8A,
                GET_PKT_TYPE            = 0x11,
                SET_TX_PARAMS           = 0x8E,
                SET_MODULATION_PARAMS   = 0x8B,
                SET_PACKET_PARAMS       = 0x8C,
                SET_CAD_PARAMS          = 0x88,
                SET_BUFFER_BASE_ADDR    = 0x8F,
                SET_LORA_SYMB_TIMEOUT   = 0xA0,
                GET_STATUS              = 0xC0,
                GET_RSSI_INST           = 0x15,
                GET_RX_BUFFER_STATUS    = 0x13,
                GET_PACKET_STATUS       = 0x14,
                GET_DEVICE_ERRORS       = 0x17,
                CLR_DEVICE_ERRORS       = 0x07
            };

            SPIClass &_spi;
            uint32_t _nss_pin;
            uint32_t _busy_pin;
            uint32_t _reset_pin;
            uint32_t _vctl_pin;
            uint32_t _irq_pin;

            Modulation modulation;
            uint32_t frequency_khz;
            
            // LoRa specific
            uint16_t lora_bw_khz;    // 125, 250, 500
            uint8_t lora_sf;        // 5-12
            uint8_t lora_cr;        // 5-8 (4/5 to 4/8)
            
            // GFSK specific
            uint32_t gfsk_bitrate;
            uint32_t gfsk_fdev;      // Frequency deviation
            GfskBandwidth gfsk_bw;        // Bandwidth index
            PulseShape gfsk_pulse;
            
            // Common
            int8_t tx_power_dbm;   // -9 to +22
            uint16_t preamble_len;
            uint8_t sync_word[8];
            uint8_t sync_word_len;

            RadioMode sMode = RadioMode::MODE_STANDBY;

            // Pin management
            void spi_select(void);
            void spi_deselect(void);
            void radio_reset(void);
            bool is_busy(void);
            void wait_until_not_busy(void);

            // SPI transfer, register and buffer transfer
            void spi_transfer(const uint8_t* tx, uint8_t* rx, uint8_t len);
            void write_command(Command cmd, const uint8_t* data, uint8_t len);
            void read_command(Command cmd, uint8_t* data, uint8_t len);
            void write_register(uint16_t addr, uint8_t value);
            void write_registers_bulk(uint16_t addr, const uint8_t* values, uint8_t len);
            uint8_t read_register(uint16_t addr);
            void read_registers_bulk(uint16_t addr, uint8_t* values, uint8_t len);
            void write_buffer(uint8_t offset, const uint8_t* data, uint8_t len);
            void read_buffer(uint8_t offset, uint8_t* data, uint8_t len);

            void set_sync_word(const uint8_t* sync, uint8_t len);
            void set_sync_word_lora(const uint8_t* sync);

            uint16_t read_errors(void);
            uint16_t clear_errors(void);
            void switch_rxtx(RadioMode mode);
    };
}

#endif // SRC_SXRADIO
