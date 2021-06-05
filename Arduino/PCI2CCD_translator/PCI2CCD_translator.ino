/*
 * PCI2CCD_translator.ino (https://github.com/laszlodaniel/CCDPCIBusTransceiver)
 * Copyright (C) 2021, Daniel Laszlo
 *
 * This program is free software: you can redistribute it and/or modify
 * it under the terms of the GNU General Public License as published by
 * the Free Software Foundation, either version 3 of the License, or
 * (at your option) any later version.
 *
 * This program is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE. See the
 * GNU General Public License for more details.
 *
 * You should have received a copy of the GNU General Public License
 * along with this program. If not, see <http://www.gnu.org/licenses/>.
 * 
 * Message                                      PCI-bus     CCD-bus
 * -----------------------------------------------------------------------------------
 * Engine speed, MAP sensor value               10          E4
 * Vehicle speed                                10          24
 * TPS volts, Speed control                     1A          42
 * Status bits                                  35          -
 * Distance pulse                               5D          B4
 * Check engine lamp status                     B0          A4
 * Battery voltage, Charging voltage            C0          D4
 * Coolant temperature, Ambient temperature     C0          8C
 * Limp status                                  D0/D1       EC
 * A/C high side pressure                       D2          75
 * Intake air temperature                       D2          8C
 * Accumulated mileage                          DF          CC
 * Vehicle information                          ED          AC
 * VIN ASCII character                          F0          6D
 * -                                            14          -
 * -                                            16          -
 * -                                            AF          -
 * -                                            CD          -
 */

#include <J1850VPWCore.h> // https://github.com/laszlodaniel/J1850VPWCore
#include <CCDLibrary.h> // https://github.com/laszlodaniel/CCDLibrary

J1850VPWCore VPW;

#define J1850_VPW_RX (10)
#define J1850_VPW_TX (11)
#define SLEEP        (5)
#define _4XLOOP      (6)
#define TBEN         (4) // CCDPCIBusTransceiver has programmable CCD-bus termination and bias (TBEN pin instead of jumpers)

#define to_uint16(hb, lb) (uint16_t)(((uint8_t)hb << 8) | (uint8_t)lb)
#define to_uint32(msb, hb, lb, lsb) (uint32_t)(((uint32_t)msb << 24) | ((uint32_t)hb << 16) | ((uint32_t)lb << 8) | (uint32_t)lsb)

uint8_t pci_rx_buf[12];
uint8_t pci_id_byte = 0;
uint8_t last_pci_msg_length = 0;

uint8_t ccd_rx_buf[16];
uint8_t ccd_id_byte = 0;
uint8_t last_ccd_msg_length = 0;

uint8_t vin[17] = { 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0 };
uint8_t vin_char_count = 0;
bool broadcast_vin_ccd = false;
uint8_t current_vin_character = 1;
uint32_t current_millis = 0;
uint32_t last_vin_character_transmit_millis = 0;
uint16_t vin_character_transmit_interval = 3000; // 3 seconds

bool broadcast_custom_ccd_messages = false;
uint8_t custom_ccd_message_count = 9;
uint8_t custom_ccd_message_01[4] = { 0x0D, 0xFF, 0xFF, 0x0B };
uint8_t custom_ccd_message_02[4] = { 0x11, 0xFF, 0xFF, 0x0F };
uint8_t custom_ccd_message_03[4] = { 0x36, 0xFF, 0xFF, 0x34 };
uint8_t custom_ccd_message_04[4] = { 0x4D, 0xFF, 0xFF, 0x4B };
uint8_t custom_ccd_message_05[4] = { 0x76, 0xFF, 0xFF, 0x74 };
uint8_t custom_ccd_message_06[4] = { 0x8D, 0xFF, 0xFF, 0x8B };
uint8_t custom_ccd_message_07[4] = { 0xB6, 0xFF, 0xFF, 0xB4 };
uint8_t custom_ccd_message_08[4] = { 0xCD, 0xFF, 0xFF, 0xCB };
uint8_t custom_ccd_message_09[4] = { 0xF6, 0xFF, 0xFF, 0xF4 };

bool pci_to_ccd = true;
bool ccd_to_pci = false; // for future use

void translate_pci_to_ccd()
{
    ccd_to_pci = false;

    if (VPW.accept(pci_rx_buf))
    {
        last_pci_msg_length = VPW.rx_nbyte;
        pci_id_byte = pci_rx_buf[0];

        switch (pci_id_byte)
        {
            case 0x10: // Engine speed, Vehicle speed, MAP sensor value
            {
                if (last_pci_msg_length > 6)
                {
                    float engine_speed = (float)to_uint16(pci_rx_buf[1], pci_rx_buf[2]) * 0.25;
                    float map_value = (float)pci_rx_buf[5];
                    uint8_t ccd_rpm_map[4] = { 0xE4, (uint8_t)(engine_speed / 32.0), (uint8_t)(map_value / 0.412), 0x00 };
                    CCD.write(ccd_rpm_map, sizeof(ccd_rpm_map));

                    float vehicle_speed_mph = (float)to_uint16(pci_rx_buf[3], pci_rx_buf[4]) * 0.0049;
                    float vehicle_speed_kmh = vehicle_speed_mph * 1.609344;
                    uint8_t ccd_vehicle_speed[4] = { 0x24, (uint8_t)vehicle_speed_mph, (uint8_t)vehicle_speed_kmh, 0x00 };
                    CCD.write(ccd_vehicle_speed, sizeof(ccd_vehicle_speed));
                }
                break;
            }
            case 0x1A: // TPS volts, Speed control
            {
                // CCD-bus message contains pedal depression percentage, not volts.
                // CCD-bus message contains set speed, instead of speed control engaged flag.

                // Send empty CCD-bus message.
                uint8_t ccd_tps_cruise[4] = { 0x42, 0x00, 0x00, 0x00 };
                CCD.write(ccd_tps_cruise, sizeof(ccd_tps_cruise));
                break;
            }
            case 0x35: // Status bits
            {
                // Empty.
                break;
            }
            case 0x5D: // Distance pulse
            {
                // Not sure what portion of the PCI-bus message to copy.

                // Send empty CCD-bus message.
                uint8_t ccd_distance_pulse[4] = { 0xB4, 0xFF, 0xFF, 0x00 };
                CCD.write(ccd_distance_pulse, sizeof(ccd_distance_pulse));
                break;
            }
            case 0xB0: // Check engine lamp status
            {
                // Not sure if lamp bits can be copied like this.
                
                if (last_pci_msg_length > 6)
                {
                    uint8_t ccd_mic_lamp_status[4] = { 0xA4, pci_rx_buf[1], 0x00, 0x00 };
                    CCD.write(ccd_mic_lamp_status, sizeof(ccd_mic_lamp_status));
                }
                break;
            }
            case 0xC0: // Battery voltage, Charging voltage, Coolant temperature, Ambient temperature
            { 
                if (last_pci_msg_length > 5)
                {
                    float battery_voltage = (float)pci_rx_buf[1] * 0.0625;
                    float charging_voltage = (float)pci_rx_buf[2] * 0.0625;
                    uint8_t ccd_battery_voltage[4] = { 0xD4, (uint8_t)(battery_voltage * 16.89189), (uint8_t)(charging_voltage * 16.89189), 0x00 };
                    CCD.write(ccd_battery_voltage, sizeof(ccd_battery_voltage));

                    int16_t coolant_temperature = pci_rx_buf[3] - 40; // °C
                    int16_t ambient_temperature = pci_rx_buf[4] - 40; // °C
                    uint8_t ccd_coolant_ambient_temperature[4] = { 0x8C, (uint8_t)(coolant_temperature + 128), (uint8_t)(ambient_temperature + 128), 0x00 };
                    CCD.write(ccd_coolant_ambient_temperature, sizeof(ccd_coolant_ambient_temperature));
                }
                break;
            }
            case 0xD0: // Limp status
            {
                // Not sure if lamp bits can be copied like this.
                
                if (last_pci_msg_length > 6)
                {
                    uint8_t ccd_limp_status[4] = { 0xEC, pci_rx_buf[1], 0x00, 0x00 };
                    CCD.write(ccd_limp_status, sizeof(ccd_limp_status));
                }
                break;
            }
            case 0xD1: // Limp status
            {
                // Not sure if lamp bits can be copied like this.
                
                if (last_pci_msg_length > 6)
                {
                    uint8_t ccd_limp_status[4] = { 0xEC, pci_rx_buf[1], 0x00, 0x00 };
                    CCD.write(ccd_limp_status, sizeof(ccd_limp_status));
                }
                break;
            }
            case 0xD2: // A/C high side pressure
            { 
                if (last_pci_msg_length > 5)
                {
                    float ac_high_side_pressure = (float)pci_rx_buf[1] * 2.03; // psi
                    uint8_t ccd_ac_high_side_pressure[4] = { 0x75, (uint8_t)(ac_high_side_pressure / 1.961), 0x00, 0x00 };
                    CCD.write(ccd_ac_high_side_pressure, sizeof(ccd_ac_high_side_pressure));
                }
                break;
            }
            case 0xDF: // Accumulated mileage
            {
                // Not sure if this is right.
                // PCI-bus value is 8-bit, CCD-bus value is supposed to be 16-bit.

                if (last_pci_msg_length > 2)
                {
                    uint8_t accumulated_mileage = pci_rx_buf[1];
                    uint8_t ccd_accumulated_mileage[4] = { 0xCC, 0x00, accumulated_mileage, 0x00 };
                    CCD.write(ccd_accumulated_mileage, sizeof(ccd_accumulated_mileage));
                }
                break;
            }
            case 0xED: // Vehicle information
            {
                // Not sure if vehicle information can be copied like this.
                
                if (last_pci_msg_length > 6)
                {
                    uint8_t ccd_vehicle_information[4] = { 0xAC, pci_rx_buf[1], pci_rx_buf[2], 0x00 };
                    CCD.write(ccd_vehicle_information, sizeof(ccd_vehicle_information));
                }
                break;
            }
            case 0xF0: // VIN ASCII character
            {
                if ((pci_rx_buf[1] == 0x01) && (last_pci_msg_length > 3))
                {
                    vin[0] = pci_rx_buf[2]; // first VIN character is coming alone
                    if (vin_char_count != 17) vin_char_count++;
                }
                else if ((pci_rx_buf[1] == 0x02) && (last_pci_msg_length > 6))
                {
                    vin[1] = pci_rx_buf[2];
                    vin[2] = pci_rx_buf[3];
                    vin[3] = pci_rx_buf[4];
                    vin[4] = pci_rx_buf[5];
                    if (vin_char_count != 17) vin_char_count += 4;
                }
                else if ((pci_rx_buf[1] == 0x06) && (last_pci_msg_length > 6))
                {
                    vin[5] = pci_rx_buf[2];
                    vin[6] = pci_rx_buf[3];
                    vin[7] = pci_rx_buf[4];
                    vin[8] = pci_rx_buf[5];
                    if (vin_char_count != 17) vin_char_count += 4;
                }
                else if ((pci_rx_buf[1] == 0x0A) && (last_pci_msg_length > 6))
                {
                    vin[9] = pci_rx_buf[2];
                    vin[10] = pci_rx_buf[3];
                    vin[11] = pci_rx_buf[4];
                    vin[12] = pci_rx_buf[5];
                    if (vin_char_count != 17) vin_char_count += 4;
                }
                else if ((pci_rx_buf[1] == 0x0E) && (last_pci_msg_length > 6))
                {
                    vin[13] = pci_rx_buf[2];
                    vin[14] = pci_rx_buf[3];
                    vin[15] = pci_rx_buf[4];
                    vin[16] = pci_rx_buf[5];
                    if (vin_char_count != 17) vin_char_count += 4;
                }

                if (vin_char_count == 17) broadcast_vin_ccd = true;
                break;
            }
            default: // unknown PCI-bus message
            {
                break;
            }
        }
    }

    if (broadcast_vin_ccd)
    {
        current_millis = millis();

        if ((uint32_t)(current_millis - last_vin_character_transmit_millis) >= vin_character_transmit_interval)
        {
            last_vin_character_transmit_millis = current_millis;

            uint8_t ccd_vin_character[4] = { 0x6D, current_vin_character, vin[current_vin_character - 1], 0x00 };
            CCD.write(ccd_vin_character, sizeof(ccd_vin_character));

            current_vin_character++;
            if (current_vin_character > 17) current_vin_character = 1; // restart at the beginning
        }
    }

    if (broadcast_custom_ccd_messages)
    {
        broadcast_custom_ccd_messages = false; // trigger only once
        
        CCD.write(custom_ccd_message_01, sizeof(custom_ccd_message_01));
        delay(50);
        CCD.write(custom_ccd_message_02, sizeof(custom_ccd_message_02));
        delay(50);
        CCD.write(custom_ccd_message_03, sizeof(custom_ccd_message_03));
        delay(50);
        CCD.write(custom_ccd_message_04, sizeof(custom_ccd_message_04));
        delay(50);
        CCD.write(custom_ccd_message_05, sizeof(custom_ccd_message_05));
        delay(50);
        CCD.write(custom_ccd_message_06, sizeof(custom_ccd_message_06));
        delay(50);
        CCD.write(custom_ccd_message_07, sizeof(custom_ccd_message_07));
        delay(50);
        CCD.write(custom_ccd_message_08, sizeof(custom_ccd_message_08));
        delay(50);
        CCD.write(custom_ccd_message_09, sizeof(custom_ccd_message_09));
    }
}

void translate_ccd_to_pci()
{
    pci_to_ccd = false;

    if (CCD.available())
    {
        last_ccd_msg_length = CCD.read(ccd_rx_buf);
        ccd_id_byte = ccd_rx_buf[0];

        switch (ccd_id_byte)
        {
            case 0x00:
            {
                break;
            }
            default: // unknown CCD-bus message
            {
                break;
            }
        }
    }
}

void setup()
{
    Serial.begin(250000);

    // PCI-bus settings (MC33390).
    pinMode(SLEEP, OUTPUT);
    digitalWrite(SLEEP, HIGH); // enable PCI-bus transmitter (receiver is always enabled), HIGH: enable, LOW: disable
    pinMode(_4XLOOP, OUTPUT);
    digitalWrite(_4XLOOP, LOW); // PCI-bus waveshaping - LOW: enabled, HIGH: disabled

    VPW.init(J1850_VPW_RX, J1850_VPW_TX, ACTIVE_HIGH, DEBUG_ON);

    // CCD-bus settings (custom transceiver).
    pinMode(TBEN, OUTPUT);
    digitalWrite(TBEN, LOW); // LOW: enable, HIGH: disable CCD-bus termination and bias

    CCD.begin(CCD_DEFAULT_SPEED, CUSTOM_TRANSCEIVER, IDLE_BITS_10, ENABLE_RX_CHECKSUM, ENABLE_TX_CHECKSUM);
}

void loop()
{
    if (pci_to_ccd) translate_pci_to_ccd();
    if (ccd_to_pci) translate_ccd_to_pci();
}
