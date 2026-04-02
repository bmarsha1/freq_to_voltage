#include <stdio.h>
#include "pico/stdlib.h"
#include "pico/time.h"
#include "hardware/pio.h"
#include "hardware/sync.h"
#include <math.h>
#include "dev_mcp4728.h"
#include "sys_i2c.h"


#include "gate.pio.h"
#include "counter.pio.h"
#include "ref.pio.h"
#include "square.pio.h"

//Input pin, gate pin, signal_counter_pin
//15, 14, 13
#define IN_PIN 27
#define GATE_PIN 26
#define CTR_PIN 25
// #define IN_PIN 16
// #define GATE_PIN 15
// #define CTR_PIN 14
#define GATE_SM 0
#define CTR_SM 1
#define REF_SM 2
#define SQUARE_SM 3
#define GATE_TIME 100000
#define ETH_WINDOW_LEN 100
#define ETH_WINDOW_INVALID 151
#define ETH_WINDOW_UNINITIALIZED 152
#define ETH_WINDOW_ERROR_THRESHOLD 0.25F
//#define INCLUDE_SQUARE // Comment out to remove test square generator
#define SQUARE_FREQ_DIVIDER 6200.0F //Generates 63Hz wave
#define I2CERR //I2C error checking

#if PICO_RP2350
#define CLOCK_FREQ 150000000
#else
#define CLOCK_FREQ 125000000
#endif

PIO pio = pio0;
uint32_t irq = PIO0_IRQ_0;

uint32_t time_of_last_measurement = 0;

uint32_t eth_measurement_window[ETH_WINDOW_LEN];
uint32_t eth_measurement_window_cur = 0;

//Adds a frequency to the window
void put_eth_window(uint32_t freq) {
    if(freq < 50 || freq > 150) {
        freq = ETH_WINDOW_INVALID;
    } else {
        //frequency is ethanol content + 50
        freq -= 50;
    }
    if (eth_measurement_window_cur >= ETH_WINDOW_LEN) {
        eth_measurement_window_cur = 0;
    }
    eth_measurement_window[eth_measurement_window_cur++] = freq;
}

//Calculates the average of the window based on how many measurements are in it
//Returns zero if there are no measurements
float avg_eth_window() {
    float sum = 0.0F;
    uint32_t valid = 0;
    // Pause interrupts so the ISR doesn't change data while we are reading it
    uint32_t status = save_and_disable_interrupts(); 
    for(uint32_t i = 0; i < ETH_WINDOW_LEN; i++) {
        uint32_t eth = eth_measurement_window[i];
        if (eth != ETH_WINDOW_INVALID && eth != ETH_WINDOW_UNINITIALIZED) {
            sum += eth;
            valid++;
        }
    }
    restore_interrupts(status);
    return valid > 0 ? sum / valid : 0;
}

//Calculates the error rate of the window
float eth_window_error_rate() {
    float errors = 0.0F;
    uint32_t total = 0;
    // Pause interrupts so the ISR doesn't change data while we are reading it
    uint32_t status = save_and_disable_interrupts(); 
    for(uint32_t i = 0; i < ETH_WINDOW_LEN; i++) {
        uint32_t eth = eth_measurement_window[i];
        if (eth != ETH_WINDOW_UNINITIALIZED) {
            errors += eth == ETH_WINDOW_INVALID;
            total++;
        }
    }
    restore_interrupts(status);
    //Return 0% error rate if we have less than 10 values
    if (total < 10)
        return 0.0f;
    return total > 0 ? errors / total : 0;
}

void handle_isr() {
    //Only handle irq0
    if(pio_interrupt_get(pio, 0)) {
        //Get the data from the SMs
        //Need to subtract from max value since they count down
        uint32_t input_count = 0xffffffff - pio_sm_get_blocking(pio, CTR_SM);
        //Loop takes 2 cycles
        uint32_t ref_count = 2 * (0xffffffff - pio_sm_get_blocking(pio, REF_SM));
        //This should never happen
        if (ref_count == 0) {
            pio_interrupt_clear(pio, 0);
            return;
        }
    
        //Calculate the frequency
        //Use 64 bit integer to avoid overflow
        uint32_t freq = (uint32_t)(((uint64_t)input_count * CLOCK_FREQ) / ref_count);
        put_eth_window(freq);
        time_of_last_measurement = to_ms_since_boot(get_absolute_time());

        //Clear interrupt
        pio_interrupt_clear(pio, 0);
    }
}

void init_reciprocal_ctr_sm() {
    pio_gpio_init(pio, GATE_PIN);
    pio_gpio_init(pio, CTR_PIN);
    #ifndef INCLUDE_SQUARE
    gpio_init(IN_PIN);
    gpio_set_dir(IN_PIN, false);
    #endif
    pio_sm_set_consecutive_pindirs(pio, GATE_SM, CTR_PIN, 2, true);

    //Configure the gate
    uint32_t gate_offset = pio_add_program(pio, &gate_program);
    pio_sm_config gate_config = gate_program_get_default_config(gate_offset);
    sm_config_set_in_pin_base(&gate_config, IN_PIN);
    //sm_config_set_in_pin_count(&gate_config, 1);
    sm_config_set_sideset_pin_base(&gate_config, GATE_PIN);
    sm_config_set_clkdiv(&gate_config, 1.0f);

    //Configure the input counter
    uint32_t ctr_offset = pio_add_program(pio, &counter_program);
    pio_sm_config ctr_config = counter_program_get_default_config(ctr_offset);
    sm_config_set_in_pin_base(&ctr_config, GATE_PIN);
    //sm_config_set_in_pin_count(&ctr_config, 2);
    sm_config_set_jmp_pin(&ctr_config, GATE_PIN);
    sm_config_set_sideset_pin_base(&ctr_config, CTR_PIN);
    sm_config_set_clkdiv(&ctr_config, 1.0f);

    //Configure the ref clock
    uint32_t ref_offset = pio_add_program(pio, &ref_program);
    pio_sm_config ref_config = ref_program_get_default_config(ref_offset);
    sm_config_set_in_pin_base(&ref_config, CTR_PIN);
    //sm_config_set_in_pin_count(&ref_config, 1);
    sm_config_set_jmp_pin(&ref_config, CTR_PIN);
    sm_config_set_clkdiv(&ref_config, 1.0f);

    //Enable the interrupt
    pio_set_irq0_source_enabled(pio, pis_interrupt0, true);
    irq_set_exclusive_handler(irq, handle_isr);
    irq_set_enabled(irq, true);

    //Init all of the state machines
    pio_sm_init(pio, GATE_SM, gate_offset, &gate_config);
    pio_sm_init(pio, CTR_SM, ctr_offset, &ctr_config);
    pio_sm_init(pio, REF_SM, ref_offset, &ref_config);
    
    //Start the SMs
    pio_sm_set_enabled(pio, CTR_SM, true);
    pio_sm_set_enabled(pio, REF_SM, true);
    pio_sm_set_enabled(pio, GATE_SM, true);

    //Set up all of the values
    pio_sm_put(pio, CTR_SM, 0xfffffffe); // Accounts for waiting for one extra rising edge after gate goes high
    pio_sm_put(pio, REF_SM, 0xffffffff);
    pio_sm_put(pio, GATE_SM, GATE_TIME);
}

#ifdef INCLUDE_SQUARE
void init_square_generator(uint32_t pin) {
    PIO pio = pio1;
    pio_gpio_init(pio, pin);
    pio_sm_set_consecutive_pindirs(pio, SQUARE_SM, pin, 1, true);

    //Configure the square test
    uint32_t square_offset = pio_add_program(pio, &square_program);
    pio_sm_config square_config = square_program_get_default_config(square_offset);
    sm_config_set_sideset_pins(&square_config, pin);
    sm_config_set_clkdiv(&square_config, CLOCK_FREQ / SQUARE_FREQ_DIVIDER);
    pio_sm_init(pio, SQUARE_SM, square_offset, &square_config);
    pio_sm_set_enabled(pio, SQUARE_SM, true);
}
#endif

//Use this counter to cut down on the print statements
uint32_t counter = 0;

bool init_dac(){
    //Turn all channels off except for A
    dev_mcp4728_pd(MCP4728_CHA, MCP4728_PD_OFF);
    dev_mcp4728_pd(MCP4728_CHB, MCP4728_PD_100);
    dev_mcp4728_pd(MCP4728_CHC, MCP4728_PD_100);
    dev_mcp4728_pd(MCP4728_CHD, MCP4728_PD_100);
    //Set external VREF for CHA
    dev_mcp4728_vref(MCP4728_CHA, MCP4728_VREF_VDD);
    //Write the voltage
    return dev_mcp4728_set(i2c0, MCP4728_CHA, 0) &&
    dev_mcp4728_set(i2c0, MCP4728_CHB, 0) &&
    dev_mcp4728_set(i2c0, MCP4728_CHC, 0) &&
    dev_mcp4728_set(i2c0, MCP4728_CHD, 0);
}

bool update_voltage() {
    if (eth_window_error_rate() > ETH_WINDOW_ERROR_THRESHOLD) {
        //write a 0
        printf("Error: Too many invalid sensor readings\n");
        return dev_mcp4728_set(i2c0, MCP4728_CHA, 0);
    } else if(to_ms_since_boot(get_absolute_time()) - time_of_last_measurement > 2000)
    {
        printf("Error: No signal from sensor\n");
        return dev_mcp4728_set(i2c0, MCP4728_CHA, 0);
    }
    else {
        float ethanol_percentage = avg_eth_window();
        //0% ethanol is .5V, 100% ethanol is 4.5V
        // Make sure ethanol percentage is within correct range
        if (ethanol_percentage < 0.0f) ethanol_percentage = 0.0f;
        if (ethanol_percentage > 100.0f) ethanol_percentage = 100.0f;
        float voltage = ethanol_percentage * 4.0F / 100.0F + 0.5F;
        uint16_t dac_val = (uint16_t) (voltage * 4096.0F / 5.0F);
        //If divisible by 32 (last 5 digits are 0)
        if ((counter & 0x1F) == 0)
            printf("eth: %f, voltage: %f, dac: %d\n", ethanol_percentage, voltage, dac_val);
        return dev_mcp4728_set(i2c0, MCP4728_CHA, dac_val);
    }
}

int main() {
    //Initialize freq window
    for (uint32_t i = 0; i < ETH_WINDOW_LEN; i++) {
        eth_measurement_window[i] = ETH_WINDOW_UNINITIALIZED;
    }
    time_of_last_measurement = to_ms_since_boot(get_absolute_time());
    sys_i2c_init(i2c0, SYS_SDA0, SYS_SCL0, 100000, true);
    bool dac = init_dac();
    stdio_init_all();
    init_reciprocal_ctr_sm();
#ifdef INCLUDE_SQUARE
    init_square_generator(IN_PIN);
#endif
    while (true) {
        #ifdef I2CERR
        if (!dac && (counter & 0x1F) == 0) {
            printf("dac init failed\n");
        } else if (!update_voltage() && (counter & 0x1F) == 0) {
            printf("update dac voltage failed\n");
        }
        #else
        update_voltage();
        #endif
        counter++;
        sleep_ms(100);
    }
}
