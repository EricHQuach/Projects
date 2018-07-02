/*
 * inverter_driver.c
 *
 *  Created on: Sep 29, 2017
 *      Author: Eric Quach
 */

#include "can_node.h"
#include "can_message_id.h"
#include <stdint.h>

struct data{
    //Refer to the user manual for additional information: http://www.tritium.com.au/source/TRI74.021v5-Users-Manual.pdf starting at page 34
    //Identification information
    uint32_t serial_number;
    char id[4];

    //Status information
    uint8_t receive_error_count;
    uint8_t transmit_error_count;
    uint16_t active_motor;

    //Error flags
    bool igbt_desat;
    bool uvlo_15V;
    bool config_read;
    bool watchdog_reset;
    bool motor_position_hall_sequence;
    bool dc_bus_overvolt;
    bool sw_over_current;
    bool hw_over_current;

    //Limit flags
    bool motor_temp_limit;
    bool bus_volt_lower;
    bool bus_volt_upper;
    bool bus_current_limit;
    bool velocity;
    bool motor_current;
    bool output_volt_pwm;

    //Various measurements
    float bus_current;
    float bus_voltage;
    float vehicle_velocity;
    float motor_velocity;
    float phase_c_current;
    float phase_b_current;
    float voltage_real;
    float voltage_imaginary;
    float current_real;
    float current_imaginary;
    float backEMF_measurement;
    float backEMF_prediction;
    float voltage_rail_15V;
    float voltage_rail_3V3;
    float voltage_rail_1V9;
    float ipm_a_temp;
    float motor_temp;
    float ipm_b_temp;
    float dsp_temp;
    float ipm_c_temp;
    float dc_bus_amphours;
    float odometer;
    float slip_speed;
} inverter_status;

struct cmd{
    //Drive commands
    float motor_current;
    float motor_velocity;

    //Power commands
    float bus_current;
} inverter_command;

void float2bytes(float val,uint8_t *bytes_array){
  // Create union of shared memory space
  union {
    float float_variable;
    uint8_t temp_array[4];
  } u;
  // Overwrite bytes of union with float variable
  u.float_variable = val;
  // Assign bytes to input array
  memcpy(bytes_array, u.temp_array, 4);
}

void inverter_send_commands(float motor_current, float motor_velocity, float bus_current){
    //Note: m_current and b_current represent percentages and should range from 0.0 to 1.0. Do not go all the way to 100.

    uint8_t drive_cmd_data[8];  //byte array for motor drive commands
    uint8_t power_cmd_data[8];  //byte array for motor power commands

    inverter_command.motor_current = motor_current;
    inverter_command.motor_velocity = motor_velocity;
    inverter_command.bus_current = bus_current;

    //converts the float values into byte arrays
    float2bytes(inverter_command.motor_current, &drive_cmd_data[4]);
    float2bytes(inverter_command.motor_velocity, &drive_cmd_data[0]);
    float2bytes(inverter_command.bus_current, &power_cmd_data[4]);
    for(int i = 0; i < 4; i++){
        power_cmd_data[i] = 0;
    }

    //sends data in byte arrays to the inverter
    can_send(CAN_INV_DRIVE_CMD, &drive_cmd_data[0], 8);
    can_send(CAN_INV_POWER_CMD, &power_cmd_data[0], 8);
}

static void inverter_info(uint32_t msgId, uint8_t *data, uint8_t len){
    uint32_t first_bytes = data[7] << 24 | data[6] << 16 | data[5] << 8 | data[4];
    uint32_t last_bytes = data[3] << 24 | data[2] << 16 | data[1] << 8 | data[0];

    switch(msgId){
        case CAN_INV_IDENT:
            inverter_status.serial_number = data[7] << 24 | data[6] << 16 | data[5] << 8 | data[4];
            inverter_status.id[0] = data[0];
            inverter_status.id[1] = data[1];
            inverter_status.id[2] = data[2];
            inverter_status.id[3] = data[3];
            break;

        case CAN_INV_STATUS:
            inverter_status.receive_error_count = data[7];
            inverter_status.transmit_error_count = data[6];
            inverter_status.active_motor = data[5]<<8 | data[4];

            inverter_status.igbt_desat = (data[2] & 0x80) >> 7;
            inverter_status.uvlo_15V = (data[2] & 0x40) >> 6;
            inverter_status.config_read = (data[2] & 0x20) >> 5;
            inverter_status.watchdog_reset = (data[2] & 0x10) >> 4;
            inverter_status.motor_position_hall_sequence = (data[2] & 0x08) >> 3;
            inverter_status.dc_bus_overvolt = (data[2] & 0x04) >> 2;
            inverter_status.sw_over_current = (data[2] & 0x02) >> 1;
            inverter_status.hw_over_current = (data[2] & 0x01);

            inverter_status.motor_temp_limit = (data[0] & 0x40) >> 6;
            inverter_status.bus_volt_lower = (data[0] & 0x20) >> 5;
            inverter_status.bus_volt_upper = (data[0] & 0x10) >> 4;
            inverter_status.bus_current_limit = (data[0] & 0x08) >> 3;
            inverter_status.velocity = (data[0] & 0x04) >> 2;
            inverter_status.motor_current = (data[0] & 0x02) >> 1;
            inverter_status.output_volt_pwm = data[0] & 0x01;
            break;

        case CAN_INV_BUS:
            memcpy(&inverter_status.bus_current, &first_bytes, sizeof inverter_status.bus_current);
            memcpy(&inverter_status.bus_voltage, &last_bytes, sizeof inverter_status.bus_voltage);
            break;

        case CAN_INV_VELOCITY:
            memcpy(&inverter_status.vehicle_velocity, &first_bytes, sizeof inverter_status.vehicle_velocity);
            memcpy(&inverter_status.motor_velocity, &last_bytes, sizeof inverter_status.motor_velocity);
            break;

        case CAN_INV_PHASE_CURRENT:
            memcpy(&inverter_status.phase_c_current, &first_bytes, sizeof inverter_status.phase_c_current);
            memcpy(&inverter_status.phase_b_current, &last_bytes, sizeof inverter_status.phase_b_current);
            break;

        case CAN_INV_MOTOR_VOLTAGE:
            memcpy(&inverter_status.voltage_real, &first_bytes, sizeof inverter_status.voltage_real);
            memcpy(&inverter_status.voltage_imaginary, &last_bytes, sizeof inverter_status.voltage_imaginary);
            break;

        case CAN_INV_MOTOR_CURRENT:
            memcpy(&inverter_status.current_real, &first_bytes, sizeof inverter_status.current_real);
            memcpy(&inverter_status.current_imaginary, &last_bytes, sizeof inverter_status.current_imaginary);
            break;

        case CAN_INV_MOTOR_BEMF:
            memcpy(&inverter_status.backEMF_measurement, &first_bytes, sizeof inverter_status.backEMF_measurement);
            memcpy(&inverter_status.backEMF_prediction, &last_bytes, sizeof inverter_status.backEMF_prediction);
            break;

        case CAN_INV_15V_RAIL:
            memcpy(&inverter_status.voltage_rail_15V, &first_bytes, sizeof inverter_status.voltage_rail_15V);
            break;

        case CAN_INV_3V3_1V9_RAIL:
            memcpy(&inverter_status.voltage_rail_3V3, &first_bytes, sizeof inverter_status.voltage_rail_3V3);
            memcpy(&inverter_status.voltage_rail_1V9, &last_bytes, sizeof inverter_status.voltage_rail_1V9);
            break;

        case CAN_INV_TEMPERATURE_A:
            memcpy(&inverter_status.ipm_a_temp, &first_bytes, sizeof inverter_status.ipm_a_temp);
            memcpy(&inverter_status.motor_temp, &last_bytes, sizeof inverter_status.motor_temp);
            break;

        case CAN_INV_TEMPERATURE_B:
            memcpy(&inverter_status.ipm_b_temp, &first_bytes, sizeof inverter_status.ipm_b_temp);
            memcpy(&inverter_status.dsp_temp, &last_bytes, sizeof inverter_status.dsp_temp);
            break;

        case CAN_INV_TEMPERATURE_C:
            memcpy(&inverter_status.ipm_c_temp, &first_bytes, sizeof inverter_status.ipm_c_temp);
            break;

        case CAN_INV_ODOMETER:
            memcpy(&inverter_status.dc_bus_amphours, &first_bytes, sizeof inverter_status.dc_bus_amphours);
            memcpy(&inverter_status.odometer, &last_bytes, sizeof inverter_status.odometer);
            break;

        case CAN_INV_SLIP_SPEED:
            memcpy(&inverter_status.bus_voltage, &first_bytes, sizeof inverter_status.bus_voltage);
            break;
    }

}

void inverter_driver_init(){
    //uint32_t sysClockFreq;
    //sysClockFreq = SysCtlClockGet();
    //can_framework_init(sysClockFreq, BUS_BITRATE_250KBPS); //init can at 250kpbs
    can_add_callback(CAN_INV_IDENT, 0xffffff00, inverter_info);
}
