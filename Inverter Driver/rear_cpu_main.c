#include <common/can_node.h>
#include <stdint.h>
#include <stdbool.h>
#include "driverlib/sysctl.h"
#include "driverlib/can.h"
#include "driverlib/gpio.h"
#include "driverlib/pin_map.h"
#include "inc/hw_memmap.h"
#include <common/inverter_driver.h>
#include "common/util.h"
#include "common/profiler.h"
#include "common/shutdown_relay.h"

#include "common/can_message_id.h"

#include "power.h"
#include "DAC.h"

/* Global variables for output states */
volatile bool cooling_on = false;
volatile bool brake_light_on = false;
float accelerator_data = 0;
float motor_velocity = 1000;
float brake_data = 0;
uint8_t can_message_timer = 50;

struct bms_bus_measurement {
   uint16_t bus_current;
   uint16_t stack_voltage;
   uint16_t soc;
   uint16_t max_temp;
} __attribute__((packed));

/* CAN callback function for the CAN_DASH_STATUS packet */
static void dashboard_status_receive(uint32_t msgId, uint8_t *data, uint8_t len) {
    cooling_on = (data[2] >> 0) & 1;
    brake_light_on = (data[2] >> 1) & 1;
}

static void pedal_data_receive(uint32_t msgId, uint8_t *data, uint8_t len){
    accelerator_data = (data[7] << 8 | data[6]) / 65535;
    brake_data = (data[1] << 8 | data[0]) / 65535;
    can_message_timer = 50;
}

int main(void) {

    /* set the system clock to the full 80 MHz */
    SysCtlClockSet(SYSCTL_SYSDIV_2_5 | SYSCTL_USE_PLL | SYSCTL_XTAL_16MHZ | SYSCTL_OSC_MAIN);

    uint32_t sysClockFreq = SysCtlClockGet();

    util_init();
   //profiler_init();
    shutdown_relay_init();

    can_framework_init(sysClockFreq, BUS_BITRATE_250KBPS);
    can_add_callback(CAN_DASH_PEDAL, 0xffffffff, pedal_data_receive);
    //can_add_callback(CAN_DASH_STATUS, 0xffffffff, dashboard_status_receive);

    power_init();
    inverter_driver_init();

    can_add_callback(REAR_CPU_UPDATE, 0xffffffff, JumpToBootLoader);

    DAC_init();
    
    
    /* Main 50Hz loop (runs every 20 ms) */
    while (1) {
        uint32_t start = util_clock_us();

        power_set_cooling(cooling_on);
        power_set_brake_light(brake_light_on);

        struct bms_bus_measurement current = {0,0,0,45};
        can_send(0x00000001, &current, 8);

        can_message_timer--;
        if(can_message_timer == 0){
            inverter_send_commands(0,0,0);
            can_message_timer = 1;
        }
        else{
            inverter_send_commands(0.5,accelerator_data * motor_velocity, 0.5);
        }

        if(brake_data > 0){
            brake_light_on = true;
        }
        else{
            brake_light_on = false;
        }

        if(brake_light_on) {
            DAC_set(255);
        } else {
            DAC_set(0);
        }
      //  power_set_brake_light(1);
        profiler_enter_idle();
        /* busywait for 20k uS, or 20 ms (50 Hz) */
        while ((util_clock_us() - start) < (uint32_t) 20000);
        profiler_leave_idle();
        
    }
}
