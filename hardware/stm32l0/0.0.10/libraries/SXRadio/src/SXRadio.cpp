#include "SXRadio.h"

namespace Radio {

    #define PKT_TYPE_GFSK               0x00
    #define PKT_TYPE_LORA               0x01

    SX1262Radio::SX1262Radio() : 
        _spi(SPI), 
        _nss_pin(PIN_SX1262_NSS), 
        _busy_pin(PIN_SX1262_BUSY), 
        _reset_pin(PIN_SX1262_RESET), 
        _vctl_pin(PIN_SX1262_VCTL1), 
        _irq_pin(PIN_SX1262_IRQ) {
    }

    void SX1262Radio::begin(void) {
        pinMode(_busy_pin, INPUT);
        pinMode(_irq_pin, INPUT);

        digitalWrite(_reset_pin, HIGH);
        pinMode(_reset_pin, OUTPUT);
        digitalWrite(_nss_pin, HIGH);
        pinMode(_nss_pin, OUTPUT);
        digitalWrite(_vctl_pin, HIGH);
        pinMode(_vctl_pin, OUTPUT);

        _spi.begin();
        
        modulation = Modulation::Unknown;
        tx_power_dbm = -128;

        // Reset radio
        reset();
        clear_errors();

        // Set DIO2 as RF switch control, initial mode is Rx
        switch_rxtx(RadioMode::MODE_RX);

        // Set regulator mode (DC-DC)
        uint8_t reg_mode = 0x01;
        write_command(Command::SET_REGULATOR_MODE, &reg_mode, 1);
        clear_errors();

        // Set DIO3 as TCXO control
        uint8_t dio3_mode[] = {0x02, 0x00, 1, 44};
        write_command(Command::SET_DIO3_AS_TCXO_CTRL, dio3_mode, 4);
        delay(10);

        // Clear errors
        clear_errors();

        // Calibrate
        uint8_t cal_mask = 0x7E;
        write_command(Command::CALIBRATE, &cal_mask, 1);
        delay(10);
        
        // Clear errors
        clear_errors();

        // Set clamping (workaround fromn datasheet)
        uint8_t v = read_register(0x08d8);
        v |= 0x1E;
        write_register(0x08d8, v);
        delay(10);

        // Clear errors
        clear_errors();

        // Set buffer addresses
        uint8_t buf_addr[2] = {0x00, 0x00};  // TX base, RX base
        write_command(Command::SET_BUFFER_BASE_ADDR, buf_addr, 2);

        sMode = RadioMode::MODE_STANDBY;
    }

    void SX1262Radio::reset(void) {
        digitalWrite(_reset_pin, LOW);
        delay(1);
        digitalWrite(_reset_pin, HIGH);
        delay(2);
    }

    void SX1262Radio::configure_lora(uint32_t freq_khz, uint16_t bw_khz, uint8_t sf, uint8_t cr, uint8_t *sync_word) {
        // Standby to XOSC for consistent state
        standby(1);

        bool reconfigured = false;

        // Track modulation type for packet info parsing
        if (modulation != Modulation::LoRa) {
            uint8_t pkt_type = PKT_TYPE_LORA;
            write_command(Command::SET_PKT_TYPE, &pkt_type, 1);
        
            modulation = Modulation::LoRa;
            reconfigured = true;

            frequency_khz = 0;
            lora_bw_khz = 0;
            lora_cr = 0;
            lora_sf = 0;
        }

        // Set frequency
        if (frequency_khz != freq_khz) {
            uint32_t freq = (uint64_t)freq_khz * 1048576 / 1000;
            uint8_t freq_buf[4] = {
                (uint8_t)(freq >> 24),
                (uint8_t)(freq >> 16),
                (uint8_t)(freq >> 8),
                (uint8_t)(freq)
            };
            write_command(Command::SET_RF_FREQUENCY, freq_buf, 4);

            frequency_khz = freq_khz;
            reconfigured = true;
        }
        
        if ((lora_bw_khz != bw_khz) || (lora_cr != cr) || (lora_sf != sf)) {
            // Set modulation params: SF, BW, CR, LDRO
            uint8_t bw_param = 0;
            if (bw_khz <= 125) bw_param = 0x04;
            else if (bw_khz <= 250) bw_param = 0x05;
            else bw_param = 0x06;
            
            uint8_t mod_params[4] = {sf, bw_param, (uint8_t)(cr - 4), 0x00};
            write_command(Command::SET_MODULATION_PARAMS, mod_params, 4);

            // Set packet params
            uint8_t pkt_params[6] = {0x00, 0x0C, 0x00, 0xFF, 0x01, 0x00};
            write_command(Command::SET_PACKET_PARAMS, pkt_params, 6);

            // Load sync word
            if (sync_word) {
                set_sync_word_lora(sync_word);
            }

            lora_bw_khz = bw_khz;
            lora_cr = cr;
            lora_sf = sf;
            reconfigured = true;

            // Set IRQ
            // uint8_t irq_params[8] = {0x02, 0x03, 0x02, 0x03, 0x00, 0x00, 0x00, 0x00};
            // radio_write_command(CMD_SET_DIO_IRQ_PARAMS, irq_params, 8);
        }

        if (reconfigured) {
            // Set IRQ
            const uint8_t irq_params[8] = {0x02, 0x07, 0x02, 0x07, 0x00, 0x00, 0x00, 0x00};
            write_command(Command::SET_DIO_IRQ_PARAMS, irq_params, 8);

            start_rx();
        }
    }

    void SX1262Radio::configure_gfsk(uint32_t freq_khz, uint32_t bitrate,
                                     uint32_t fdev, uint8_t* sync_word,
                                     size_t sw_len, PulseShape pulse,
                                     GfskBandwidth bw) {
        // Standby to XOSC
        standby(1);

        bool reconfigured = false;

        // Track modulation type for packet info parsing
        if (modulation != Modulation::GFSK) {
            // Set packet type to GFSK
            uint8_t pkt_type = PKT_TYPE_GFSK;
            write_command(Command::SET_PKT_TYPE, &pkt_type, 1);

            modulation = Modulation::GFSK;
            reconfigured = true;

            frequency_khz = 0;
            gfsk_bitrate = 0;
            gfsk_bw = GfskBandwidth::BW_NONE;
            gfsk_fdev = 0;
        }

        if (frequency_khz != freq_khz) {

            // **CRITICAL: Image calibration for the frequency band**
            // Without this, RX sensitivity is severely degraded!
            // 868 MHz band: 863-870 MHz -> freq1=0xD7, freq2=0xDB
            // 915 MHz band: 902-928 MHz -> freq1=0xE1, freq2=0xE9

            int old_band = 0;
            if (frequency_khz >= 902000 && frequency_khz <= 928000) {
                old_band = 1;
            }
            int new_band = 0;
            if (freq_khz >= 902000 && freq_khz <= 928000) {
                new_band = 1;
            }

            if (old_band != new_band) {
                uint8_t cal_freq[2];
                if (new_band) {
                    cal_freq[0] = 0xE1;  // 902 MHz
                    cal_freq[1] = 0xE9;  // 928 MHz
                } else {
                    cal_freq[0] = 0xD7;  // 863 MHz
                    cal_freq[1] = 0xDB;  // 870 MHz
                }
                write_command(Command::CALIBRATE_IMAGE, cal_freq, 2);
                delay(10);
            }
            
            // Set frequency
            uint32_t freq = (uint64_t)freq_khz * 1048576 / 1000;
            uint8_t freq_buf[4] = {
                (uint8_t)(freq >> 24),
                (uint8_t)(freq >> 16),
                (uint8_t)(freq >> 8),
                (uint8_t)(freq)
            };
            write_command(Command::SET_RF_FREQUENCY, freq_buf, 4);

            frequency_khz = freq_khz;
            reconfigured = true;
        }

        if ((gfsk_bitrate != bitrate) || (gfsk_fdev != fdev) || (gfsk_bw != bw) || (gfsk_pulse != pulse)) {
            // Set modulation params
            uint32_t br = (32UL*32000000UL) / bitrate;
            uint32_t fd = (uint32_t)((float)fdev / 0.95367431640625f);
            
            uint8_t mod_params[8] = {
                (uint8_t)(br >> 16), (uint8_t)(br >> 8), (uint8_t)br,
                (uint8_t)pulse,
                (uint8_t)(bw != GfskBandwidth::BW_NONE ? bw : gfsk_bw),
                (uint8_t)(fd >> 16), (uint8_t)(fd >> 8), (uint8_t)fd
            };
            write_command(Command::SET_MODULATION_PARAMS, mod_params, 8);

            const uint8_t sGfskSyncWordLenBits = sw_len * 8; // = 16 bits

            // Set packet params (from original radio firmware)
            const uint8_t pkt_params[9] = {
                0x00, 24,               // Preamble length: 24 bits (3 bytes)
                0x04,                   // Preamble detector: 8 bits
                sGfskSyncWordLenBits,   // Sync word length in bits
                0x00,                   // Address filtering: disabled
                0x00,                   // Packet type: static length
                0x40,                   // Payload length: 64 bytes
                0x01,                   // CRC: off
                0x00                    // Whitening: disabled
            };
            write_command(Command::SET_PACKET_PARAMS, pkt_params, 9);

            // Load sync word
            set_sync_word(sync_word, sw_len);

            gfsk_bitrate = bitrate;
            gfsk_fdev = fdev;
            gfsk_bw = bw != GfskBandwidth::BW_NONE ? bw : gfsk_bw;
            gfsk_pulse = pulse;
            reconfigured = true;
        }

        if (reconfigured) {
            // Set IRQ
            const uint8_t irq_params[8] = {0x02, 0x07, 0x02, 0x07, 0x00, 0x00, 0x00, 0x00};
            write_command(Command::SET_DIO_IRQ_PARAMS, irq_params, 8);

            start_rx();
        }
    }

    void SX1262Radio::set_tx_power(int8_t power_dbm) {
        // Clamp power
        if (power_dbm > 22) power_dbm = 22;
        if (power_dbm < -9) power_dbm = -9;
        
        // Set PA config for high power
        uint8_t pa_config[4] = {0x04, 0x07, 0x00, 0x01};
        write_command(Command::SET_PA_CONFIG, pa_config, 4);
        
        // Set TX params
        uint8_t tx_params[2] = {(uint8_t)power_dbm, 0x04};  // Power, ramp time
        write_command(Command::SET_TX_PARAMS, tx_params, 2);

        tx_power_dbm = power_dbm;
    }

    void SX1262Radio::set_sync_word(const uint8_t* sync, uint8_t len) {
        // Write to sync word registers (0x06C0)
        write_registers_bulk(0x06C0, sync, len);
    }

    void SX1262Radio::set_sync_word_lora(const uint8_t* sync) {
        // Write to sync word registers (0x0740)
        write_registers_bulk(0x0740, sync, 2);
    }

    void SX1262Radio::standby(int mode) {
        uint8_t standby_mode = mode != 0 ? 1 : 0;
        write_command(Command::SET_STANDBY, &standby_mode, 1);
        sMode = RadioMode::MODE_STANDBY;
    }

    void SX1262Radio::start_rx(bool rx_boost_enabled) {
        switch_rxtx(RadioMode::MODE_RX);
        standby(1);

        // Set buffer addresses
        uint8_t buf_addr[2] = {0x00, 0x00};  // TX base, RX base
        write_command(Command::SET_BUFFER_BASE_ADDR, buf_addr, 2);

        // Set RX gain to boosted mode for better sensitivity
        // Register 0x08AC: 0x96 = boosted gain, 0x94 = power saving
        write_register(0x08AC, rx_boost_enabled ? 0x96 : 0x94);

        // Set RX with timeout 0 (continuous)
        uint8_t timeout[3] = {0xFF, 0xFF, 0xFF};
        write_command(Command::SET_RX, timeout, 3);

        sMode = RadioMode::MODE_RX;
    }

    void SX1262Radio::start_tx(const uint8_t* data, uint8_t len) {
        if (!data) return;
        if (len == 0) return;

        // Clear any pending IRQ
        clear_irq_status(IRQ_TX_DONE | IRQ_RX_DONE);
        
        switch_rxtx(RadioMode::MODE_TX);
        standby(1);

        // Set buffer addresses
        uint8_t buf_addr[2] = {0x00, 0x00};  // TX base, RX base
        write_command(Command::SET_BUFFER_BASE_ADDR, buf_addr, 2);

        // Write data to buffer
        write_buffer(0, data, len);
        
        // Set TX with timeout
        uint8_t timeout[3] = {0x00, 0x00, 0x00};  // No timeout
        write_command(Command::SET_TX, timeout, 3);
        sMode = RadioMode::MODE_TX;
    }

    uint8_t SX1262Radio::read_packet(uint8_t* buf, uint8_t max_len, PacketInfo& info) {
        if (!buf || max_len == 0) return 0;
        
        // Get RX buffer status
        uint8_t status[3];
        read_command(Command::GET_RX_BUFFER_STATUS, status, 3);
        
        uint8_t len = status[0];
        uint8_t offset = status[1];

        len = max_len;

        // Clamp to buffer size
        if (len > max_len) {
            len = max_len;
        }
        
        // Read packet data
        read_buffer(offset, buf, len);
        
        // Get packet status
        get_packet_info(info);
        
        // Clear IRQ
        // clear_irq_status(IRQ_RX_DONE | IRQ_CRC_ERR | IRQ_HEADER_ERR);
        clear_irq();
        
        return len;
    }

    void SX1262Radio::get_packet_info(PacketInfo& info) {
        uint8_t status[4];
        read_command(Command::GET_PACKET_STATUS, status, 4);
        
        // Response format differs between LoRa and GFSK:
        // LoRa: [RssiPkt, SnrPkt, SignalRssiPkt, ...]
        // GFSK: [RxStatus, RssiSync, RssiAvg, ...]
        
        if (modulation == Modulation::LoRa) {
            // LoRa: RSSI in dBm = -RssiPkt/2
            info.rssi = -(int16_t)status[0] / 2;
            // SNR in dB = SnrPkt/4
            info.snr = (int8_t)status[1] / 4;
        } else {
            // GFSK: RSSI at sync word detection, in dBm = -RssiSync/2
            info.rssi = -(int16_t)status[1] / 2;
            info.snr = 0;
        }
    }

    uint16_t SX1262Radio::get_irq_status(void) {
        uint8_t status[2];
        read_command(Command::GET_IRQ_STATUS, status, 2);
        return ((uint16_t)status[0] << 8) | status[1];
    }

    void SX1262Radio::clear_irq_status(uint16_t mask) {
        uint8_t clear[2] = {(uint8_t)(mask >> 8), (uint8_t)mask};
        write_command(Command::CLR_IRQ_STATUS, clear, 2);
    }

    void SX1262Radio::clear_irq(void) {
        clear_irq_status(0xFFFF);
    }

    bool SX1262Radio::has_packet(void) {
        uint16_t irq = get_irq_status();
        return (irq & IRQ_RX_DONE) != 0;
    }

    bool SX1262Radio::tx_done(void) {
        uint16_t irq = get_irq_status();
        return (irq & IRQ_TX_DONE) != 0;
    }

    RadioMode SX1262Radio::get_mode(void) { 
        return sMode; 
    }

    Modulation SX1262Radio::get_modulation(void) { 
        return modulation;
    }

    void SX1262Radio::set_continuous_wave(uint32_t freq_khz, int8_t power_dbm) {
        // Set frequency
        uint32_t freq = (uint64_t)freq_khz * 1048576 / 1000;
        uint8_t freq_buf[4] = {
            (uint8_t)(freq >> 24),
            (uint8_t)(freq >> 16),
            (uint8_t)(freq >> 8),
            (uint8_t)(freq)
        };
        write_command(Command::SET_RF_FREQUENCY, freq_buf, 4);

        // Set TX power
        set_tx_power(power_dbm);

        // Set CW mode
        write_command(Command::SET_TX_CW, NULL, 0);
    }

    int16_t SX1262Radio::get_rssi_inst(void) { 
        uint8_t data[2];
        read_command(Command::GET_RSSI_INST, data, 2);
        // data[0] = status, data[1] = rssiInst
        // RSSI in dBm = -rssiInst/2
        return -(int16_t)data[1] / 2;
     }

    void SX1262Radio::spi_select(void) {
        digitalWrite(_nss_pin, LOW);
    }
    void SX1262Radio::spi_deselect(void) {
        digitalWrite(_nss_pin, HIGH);
    }
    void SX1262Radio::radio_reset(void) {
        digitalWrite(_reset_pin, LOW);
        delay(1);
        digitalWrite(_reset_pin, HIGH);
        delay(2);
    }
    bool SX1262Radio::is_busy(void) { 
        return digitalRead(_busy_pin) != LOW ? true : false;
    }
    void SX1262Radio::wait_until_not_busy(void) {
      while (is_busy());
    }

    void SX1262Radio::spi_transfer(const uint8_t* tx, uint8_t* rx, uint8_t len) {
        wait_until_not_busy();
        spi_select();
        for (uint8_t i = 0; i < len; i++) {
            uint8_t b = _spi.transfer(tx ? tx[i] : 0);
            if (rx) rx[i] = b;
        }
        spi_deselect();
    }

    void SX1262Radio::write_command(Command cmd, const uint8_t* data, uint8_t len) {
        wait_until_not_busy();
        spi_select();
        _spi.transfer(static_cast<uint8_t>(cmd));
        for (uint8_t i = 0; i < len; i++) {
            _spi.transfer(data[i]);
        }
        spi_deselect();
    }

    void SX1262Radio::read_command(Command cmd, uint8_t* data, uint8_t len) {
        wait_until_not_busy();
        spi_select();
        _spi.transfer(static_cast<uint8_t>(cmd));
        _spi.transfer(0);  // NOP
        for (uint8_t i = 0; i < len; i++) {
            data[i] = _spi.transfer(0);
        }
        spi_deselect();
    }

    void SX1262Radio::write_register(uint16_t addr, uint8_t value) {
        wait_until_not_busy();
        spi_select();
        _spi.transfer(static_cast<uint8_t>(Command::WRITE_REGISTER));
        _spi.transfer((addr >> 8) & 0xFF);
        _spi.transfer(addr & 0xFF);
        _spi.transfer(value);
        spi_deselect();
    }

    void SX1262Radio::write_registers_bulk(uint16_t addr, const uint8_t *values, uint8_t len) {
        wait_until_not_busy();
        spi_select();
        _spi.transfer(static_cast<uint8_t>(Command::WRITE_REGISTER));
        _spi.transfer((addr >> 8) & 0xFF);
        _spi.transfer(addr & 0xFF);
        for (uint8_t i = 0; i < len; i++) {
            _spi.transfer(values[i]);
        }
        spi_deselect();
    }

    uint8_t SX1262Radio::read_register(uint16_t addr) {
        wait_until_not_busy();
        spi_select();
        _spi.transfer(static_cast<uint8_t>(Command::READ_REGISTER));
        _spi.transfer((addr >> 8) & 0xFF);
        _spi.transfer(addr & 0xFF);
        _spi.transfer(0);  // NOP
        uint8_t value = _spi.transfer(0);
        spi_deselect();
        return value;
    }

    void SX1262Radio::read_registers_bulk(uint16_t addr, uint8_t *values, uint8_t len) {
        wait_until_not_busy();
        spi_select();
        _spi.transfer(static_cast<uint8_t>(Command::READ_REGISTER));
        _spi.transfer((addr >> 8) & 0xFF);
        _spi.transfer(addr & 0xFF);
        _spi.transfer(0);  // NOP
        for (uint8_t i = 0; i < len; i++) {
            values[i] = _spi.transfer(0);
        }
        spi_deselect();
    }

    void SX1262Radio::write_buffer(uint8_t offset, const uint8_t* data, uint8_t len) {
        wait_until_not_busy();
        spi_select();
        _spi.transfer(static_cast<uint8_t>(Command::WRITE_BUFFER));
        _spi.transfer(offset);
        for (uint8_t i = 0; i < len; i++) {
            _spi.transfer(data[i]);
        }
        spi_deselect();
    }

    void SX1262Radio::read_buffer(uint8_t offset, uint8_t* data, uint8_t len) {
        wait_until_not_busy();
        spi_select();
        _spi.transfer(static_cast<uint8_t>(Command::READ_BUFFER));
        _spi.transfer(offset);
        _spi.transfer(0);  // NOP
        for (uint8_t i = 0; i < len; i++) {
            data[i] = _spi.transfer(0);
        }
        spi_deselect();
    }

    uint16_t SX1262Radio::read_errors(void) {
        uint16_t errors = 0;
        read_command(Command::GET_DEVICE_ERRORS, reinterpret_cast<uint8_t *>(&errors), 2);
        return errors;
    }

    uint16_t SX1262Radio::clear_errors(void) {
        uint16_t errors = 0;
        write_command(Command::CLR_DEVICE_ERRORS, reinterpret_cast<const uint8_t *>(&errors), 2);
        return errors;
    }

    void SX1262Radio::switch_rxtx(RadioMode mode) {
        // Control external RF switch via GPIO
        digitalWrite(_vctl_pin, (mode == RadioMode::MODE_TX ? LOW : HIGH));
        
        // Enable DIO2 as RF switch control for BOTH TX and RX
        // When enabled, SX1262 automatically controls DIO2:
        // - DIO2 HIGH during TX
        // - DIO2 LOW during RX/Standby
        uint8_t dio2_mode = (mode == RadioMode::MODE_TX ? 1 : 0);  // Always enable RF switch control
        write_command(Command::SET_DIO2_AS_RF_SWITCH, &dio2_mode, 1);
    }

}  // namespace SX1262
