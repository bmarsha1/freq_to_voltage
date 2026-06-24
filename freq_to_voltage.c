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
#define WINDOW_LEN 600
#define WINDOW_INVALID 151
#define WINDOW_UNINITIALIZED 152
#define WINDOW_ERROR_THRESHOLD 0.20F
//This cannot be set smaller than 2
#define GATE_SAMPLES 5
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

//Measurement vars
//Keep a rolling avg of measurements, use it to throw out bad readings
//measurement_freq stores the most recent valid reading
volatile uint32_t time_of_last_measurement = 0;
uint8_t measurement_window[WINDOW_LEN];
uint32_t measurement_window_cur = 0;
uint32_t window_readings = 0;
uint32_t window_total = 0;
volatile uint32_t measured_freq = 50;
uint32_t valid_freq_floor = 50;
uint32_t valid_freq_ceil = 150;
volatile uint32_t num_errors = 0;


//Adds a frequency to the window
void put_window(uint64_t input_freq) {
    //Don't waste window space on error state
    if(input_freq < 50 || input_freq > 150) {
        num_errors++;
        return;
    }
    if (measurement_window_cur >= WINDOW_LEN) {
        measurement_window_cur = 0;
    }
    uint32_t idx = measurement_window_cur;
    measurement_window_cur = (measurement_window_cur + 1) % WINDOW_LEN;
    //Remove the current value if it is valid
    if(measurement_window[idx] != WINDOW_UNINITIALIZED) {
        window_total -= measurement_window[idx];
    }
    measurement_window[idx] = (uint8_t)input_freq;
    window_total += measurement_window[idx];
    //Update total measurements
    if(window_readings < WINDOW_LEN)
        window_readings++;
}

//Calculates the average of the window based on how many measurements are in it
//Returns 50 if there are no measurements
float avg_window() {
    //No measurements, return 50
    if(window_readings == 0)
        return 50.0F;
    uint32_t status = save_and_disable_interrupts();
    float result = (float) window_total / window_readings;
    restore_interrupts(status);
    return result;
}

//Updates the valid floor and ceiling
void update_limits() {
    float average = avg_window();
    float delta = average * WINDOW_ERROR_THRESHOLD;
    float floor_float = average - delta;
    // Pause interrupts so the ISR doesn't change data while we are reading it
    uint32_t status = save_and_disable_interrupts(); 
    valid_freq_floor = floor_float > 0 ? (uint32_t) floor_float : 0;
    valid_freq_ceil = (uint32_t) (average + delta);
    restore_interrupts(status);
}

void handle_isr() {
    //Only handle irq0
    if(pio_interrupt_get(pio, 0)) {
        //Get the data from the SMs
        //Need to subtract from max value since they count down
        uint64_t input_count = 0xffffffff - pio_sm_get_blocking(pio, CTR_SM);
        //Loop takes 2 cycles
        uint64_t ref_count = 2 * (0xffffffff - (uint64_t)pio_sm_get_blocking(pio, REF_SM));
        //This should never happen
        if (ref_count == 0) {
            pio_interrupt_clear(pio, 0);
            return;
        }
    
        //Calculate the frequency
        //Always write to the window, trust averaging to account for wild readings
        //Use 64 bit integer to avoid overflow
        uint64_t freq = ((input_count * CLOCK_FREQ) / ref_count);
        put_window(freq);

        //Only send the value if we decide that the measurement is valid
        if (freq > valid_freq_floor && freq < valid_freq_ceil) {
            time_of_last_measurement = to_ms_since_boot(get_absolute_time());
            measured_freq = freq;
        } else {
            //Treat invalid reading as error - if the reading really was valid it will repeat and the average will stabilize
            num_errors++;
        }

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

    //Calculate the gate time by samples
    uint32_t gate_time = CLOCK_FREQ * (GATE_SAMPLES - 1) / 50;
    pio_sm_put(pio, GATE_SM, gate_time);
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
    float ethanol_percentage;
    //Get number of errors and clear them
    uint32_t status = save_and_disable_interrupts();
    uint32_t local_errors = num_errors;
    uint32_t local_measured_freq = measured_freq;
    uint32_t local_time_of_last_measurement = time_of_last_measurement;
    num_errors = 0;
    restore_interrupts(status);
    if (local_errors > 0) {
        printf("Error: Invalid readings detected\n");
        //In this case we probably shouldn't trust the most recent reading. Send the average and then clear the errors
        ethanol_percentage = avg_window() - 50;
    } else if(to_ms_since_boot(get_absolute_time()) - local_time_of_last_measurement > 2000)
    {
        //Send a zero if we lose the sensor entirely
        printf("Error: No signal from sensor\n");
        ethanol_percentage = 0;
    }
    else {
        //All is good, use most recent value
        ethanol_percentage = local_measured_freq - 50;
    }
    //0% ethanol is .5V, 100% ethanol is 4.5V
    // Make sure ethanol percentage is within correct range
    if (ethanol_percentage < 0.0f) ethanol_percentage = 0.0f;
    if (ethanol_percentage > 100.0f) ethanol_percentage = 100.0f;
    float voltage = ethanol_percentage * 4.0F / 100.0F + 0.5F;
    uint16_t dac_val = (uint16_t) (voltage * 4095.0F / 5.0F);
    //If divisible by 32 (last 5 digits are 0)
    if ((counter & 0x1F) == 0)
        printf("eth: %f, voltage: %f, dac: %d\n", ethanol_percentage, voltage, dac_val);
    return dev_mcp4728_set(i2c0, MCP4728_CHA, dac_val);
}

int main() {
    //Initialize freq window
    for (uint32_t i = 0; i < WINDOW_LEN; i++) {
        measurement_window[i] = WINDOW_UNINITIALIZED;
    }
    time_of_last_measurement = to_ms_since_boot(get_absolute_time());
    sys_i2c_init(i2c0, SYS_SDA0, SYS_SCL0, 100000, true);
    stdio_init_all();
    bool dac = init_dac();
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
        update_limits();
        counter++;
        sleep_ms(100);
    }
}
