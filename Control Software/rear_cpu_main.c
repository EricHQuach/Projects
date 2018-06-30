#include <common/can_node.h>
#include <stdint.h>
#include <stdbool.h>
#include "driverlib/sysctl.h"
#include "driverlib/can.h"
#include "driverlib/gpio.h"
#include "driverlib/pin_map.h"
#include "inc/hw_memmap.h"
#include "inc/hw_can.h"
#include "inc/tm4c123gh6pm.h"

#include "common/inverter_driver.h"
#include "common/util.h"
#include "common/profiler.h"
#include "common/config.h"
#include "common/can_message_id.h"
#include "common/state_machine.h"
#include "common/can_bootloader_control.h"
#include "common/shutdown_relay.h"
#include "common/bms_states.h"

#include "power.h"
#include "DAC.h"
#include "fault_manager_rcpu.h"
#include "dashboard_receiver.h"
#include "pedals.h"
#include "feature_buzzer.h"

        //float dummy = read_accelerator();

uint8_t system_state = STATE_DISCONNECT;
uint8_t last_system_state = STATE_DISCONNECT;

bool stop_button_debug = false;
bool drive_button_debug = false;

MSG_CAN_RCPU_STATUS status_packet = {0};
MSG_CAN_RCPU_PEDAL pedal_packet = {0};
MSG_CAN_RCPU_FAULTS fault_packet = {0};
MSG_CAN_RCPU_DATA_1 data1_packet = {0};
MSG_CAN_RCPU_CAN_DIAGNOSTICS can_diag_packet = {0};
MSG_CAN_PDM_COMMAND pdm_packet = {0};
MSG_CAN_BMS_COMMAND bms_packet = {0};
static int MAX_RPM = 20000; //lol this number is random, don't actually use it for real

extern MSG_CAN_RCPU_FAULTS fault_msg;
extern uint64_t bms_receiver_period;

uint64_t control_cycle_time;

int main(void) {

    /* set the system clock to the full 80 MHz */
    SysCtlClockSet(SYSCTL_SYSDIV_2_5 | SYSCTL_USE_PLL | SYSCTL_XTAL_16MHZ | SYSCTL_OSC_MAIN);
    uint32_t sysClockFreq = SysCtlClockGet();

    /* initialize subsystems */
    util_init();
    profiler_init();
    can_framework_init(sysClockFreq, BUS_BITRATE_250KBPS);
    can_bootloader_control_register_node(CAN_NODE_REAR_CPU);
    config_init();
    power_init();
   inverter_driver_init();
    dashboard_receiver_init();
    bms_receiver_init();
    shutdown_relay_init();
    init_buzzer();
    //can_add_callback(CAN_DASH_PEDAL_RAW, 0xffffffff, pedals_set_values); //there were two callbacks looking for CAN_DASH_PEDAL_RAW, don't use this one
//    can_add_callback(0x00030001, 0xffffffff, config_broadcast_handler);
    
    SysCtlPeripheralEnable(SYSCTL_PERIPH_GPIOE);
    while(!SysCtlPeripheralReady(SYSCTL_PERIPH_GPIOE)){}
    GPIOPinTypeGPIOOutput(GPIO_PORTE_BASE, GPIO_PIN_0);

    /* Main 50Hz loop (runs every 20 ms) */
    int cycle = 0;

    while (1) {
        uint64_t start = util_clock_us();
        cycle++;

        /* Start main loop code here */
    stop_button_debug = dashboard_receiver_get_stop_button();
    drive_button_debug = dashboard_receiver_get_drive_button();

    do_both_pedal_check();

        /* Blink LEDs so it's obvious that the board is alive */
        int sec = cycle / 25;
        GPIOPinWrite(GPIO_PORTE_BASE, GPIO_PIN_0, (sec % 2 == 0) ? GPIO_PIN_0 : 0);
        /* Main state machine */

        /* Get the fault system up to date */
        compile_faults();
       // bool ts_power_detected = !dashboard_receiver_is_stale() && dashboard_receiver_get_ts_switch();

        /* If there is a hard fault, or we are DEAD or DISCONNECTED, keep the shutdown circuit open */
        shutdown_relay_set(!(is_hard_faulted() || system_state == STATE_DEAD || system_state == STATE_DISCONNECT));

        bool bms_receiver_stale = bms_receiver_is_stale();

        if(system_state == STATE_DISCONNECT) {
            clear_soft_faults();
            if(!dashboard_receiver_is_stale() && !bms_receiver_is_stale() && !inverter_is_stale() && util_clock_us() > 1000000) {
                system_state = STATE_SLEEP;
            }
        } else if (system_state == STATE_SLEEP) {
            /* no TS power .... :( */
            clear_soft_faults();

            /* for now, we will use a switch to fake detection of the TS being enabled... */
            if(!bms_receiver_stale && !dashboard_receiver_is_stale() && bms_state == BMS_STATE_PRECHARGE) system_state = STATE_PRECHARGE;
            if(!bms_receiver_stale && !dashboard_receiver_is_stale() && bms_state == BMS_STATE_ACTIVE) system_state = STATE_IDLE;

        } else if (system_state == STATE_PRECHARGE) {
            /* battery is precharging... */
            if(is_hard_faulted()) system_state = STATE_DEAD;
            if(!bms_receiver_is_stale() && bms_state == BMS_STATE_ACTIVE) system_state = STATE_IDLE;
            if(!bms_receiver_is_stale() && bms_state == BMS_STATE_IDLE) system_state = STATE_SLEEP;
            if(!bms_receiver_is_stale() && bms_state == BMS_STATE_FAULT) system_state = STATE_SLEEP;

            inverter_send_commands(0, MAX_RPM, 1.0);

        } else if (system_state == STATE_IDLE) {

            clear_soft_faults();

            /* TS enabled, not in "ready-to-drive mode" - waiting for button press... */
            if(read_brake() > 0.2 && dashboard_receiver_get_drive_button()) system_state = STATE_DRIVE; //pressing brake pedal/green dashboard button while IDLE should enter drive mode

            if(is_hard_faulted()) system_state = STATE_DEAD;
            if(!bms_receiver_is_stale() && bms_state == BMS_STATE_PRECHARGE) system_state = STATE_PRECHARGE;
            if(!bms_receiver_is_stale() && bms_state == BMS_STATE_FAULT) system_state = STATE_SLEEP;
            if(!bms_receiver_is_stale() && bms_state == BMS_STATE_IDLE) system_state = STATE_SLEEP;

            inverter_send_commands(0, MAX_RPM, 1.0);

        } else if (system_state == STATE_DRIVE) {
            /* TS enabled, drive, yay */
            if (last_system_state != STATE_DRIVE) delay_buzzer();

            //if(is_hard_faulted()) system_state = STATE_DEAD; //shouldn't be able to go straight from DRIVE to DEAD
            bool softy = is_soft_faulted();
            bool invy = inverter_is_stale();
            bool bmsy = bms_state != BMS_STATE_ACTIVE;
            MSG_CAN_RCPU_FAULTS faulty = fault_msg;

            if(is_soft_faulted() || dashboard_receiver_get_stop_button() || inverter_is_stale()) system_state = STATE_IDLE;
            if(bms_receiver_is_stale() || bms_state != BMS_STATE_ACTIVE) system_state = STATE_SLEEP;
            inverter_send_commands(read_throttle(), MAX_RPM, 1.0);

            //float dummy = read_accelerator();
        } else if (system_state == STATE_DEAD) {
            /* TS enabled, system hard faulted ... */
            shutdown_relay_set(false);
            inverter_send_commands(0, MAX_RPM, 1.0);
        }


        /* Send overall status packet */
        status_packet.state = system_state;

        can_send(CAN_RCPU_STATUS, &status_packet, sizeof(status_packet));

        /* Send pedal data packet */

        pedal_packet.brake = read_brake()*65535;
        pedal_packet.apps_1 = read_apps1()*65535;
        pedal_packet.apps_2 = read_apps2()*65535;
        pedal_packet.average_pedal = read_accelerator()*65535;
        can_send(CAN_RCPU_PEDAL, &pedal_packet, sizeof(pedal_packet));

        /* Send car data packet */
        data1_packet.torque_command = 0xEEEE;
        data1_packet.vehicle_speed = 0x0001;
        can_send(CAN_RCPU_DATA_1, &data1_packet, sizeof(data1_packet));

        /*Send PDM command packet*/
        //pdm_packet.brake_light = (dashboard_receiver_get_status_packet().switch_data >> 8) & 0x01; //control via dash switch
        //dash switches are numbered from 8(right silver) to 11(left silver), 12(blue), and 15(green)
        //uint16_t raw_brakes = dashboard_receiver_get_brakes();
        //float scaled_brakes = brake_scale(raw_brakes);
        //if(dashboard_receiver_get_brakes() > 16370){
        if(read_brake() > 0.2){
            pdm_packet.brake_light = 1;
        }
        else{
            pdm_packet.brake_light = 0;
        }
        pdm_packet.pump = (dashboard_receiver_get_status_packet().switch_data >> 9) & 0x01;
        pdm_packet.fan_1 = (dashboard_receiver_get_status_packet().switch_data >> 10) & 0x01;
        pdm_packet.fan_2 = (dashboard_receiver_get_status_packet().switch_data >> 10) & 0x01;
        can_send(CAN_PDM_COMMAND, &pdm_packet, sizeof(pdm_packet));

        /* Send BMS command packet*/
        bms_packet.battery_fans = (dashboard_receiver_get_status_packet().switch_data >> 11) & 0x01;
        can_send(CAN_BMS_COMMAND, &bms_packet, sizeof(bms_packet));

        /* Send CAN diagnostics packet */
        can_diag_packet.canerr_reg = CAN1_ERR_R;
        can_diag_packet.cansts_reg = CAN1_STS_R;
        can_send(CAN_RCPU_CAN_DIAGNOSTICS, &can_diag_packet, sizeof(can_diag_packet));

        last_system_state = system_state;

        /* End main loop code */
        profiler_enter_idle();

        control_cycle_time = util_clock_us() - start;

        if ((util_clock_us() - start) > 20000) {
            profiler_record_cycle_exceedance();
        }

        /* busywait for 20k uS, or 20 ms (50 Hz) */
        while ((util_clock_us() - start) < (uint32_t) 20000);
        profiler_leave_idle();
        
    }
}
