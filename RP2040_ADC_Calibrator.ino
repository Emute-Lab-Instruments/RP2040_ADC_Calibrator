// adc_calibration_sd.ino
// RP2040 ADC Calibration using PIO Sigma-Delta DAC
// 
// Hardware setup:
//   Pin 2 (DAC) -> Sallen-Key filter (20kHz cutoff) -> ADC

#include <LittleFS.h>
#include "hardware/pio.h"
#include "hardware/adc.h"

// -----------------------------------------------------------------------------
// Configuration
// -----------------------------------------------------------------------------
#define DAC_PIN             3
#define ADC_PIN             29
#define ADC_CHAN            3

#define SAMPLES_PER_LEVEL   c      // Samples at each DAC level
#define NUM_SWEEPS          4      // Up/down sweeps (averages hysteresis)
#define SETTLE_US           1000     // Settling time after level change

// -----------------------------------------------------------------------------
// Calibration data structures
// -----------------------------------------------------------------------------
typedef struct {
    uint32_t magic;             // 0xCA11B8ED to validate
    int16_t correction[4096];   // correction[raw_code] = offset to add
    uint16_t adc_min;           // ADC reading at 0V
    uint16_t adc_max;           // ADC reading at full scale
} CalibrationData;

static CalibrationData cal_data;
static uint32_t histogram[4096];

// -----------------------------------------------------------------------------
// PIO Sigma-Delta DAC
// -----------------------------------------------------------------------------
static PIO sd_pio;
static uint sd_sm;
static uint32_t sd_accumulator = 0;
static uint32_t sd_target = 0;

// PIO program: just output bits from FIFO
// Assembly: .wrap_target
//           out pins, 1
//           .wrap
static const uint16_t sd_program_instr[] = { 0x6001 };
static const struct pio_program sd_program = {
    .instructions = sd_program_instr,
    .length = 1,
    .origin = -1
};

bool dac_init(uint pin) {
    sd_pio = pio0;
    
    int sm = pio_claim_unused_sm(sd_pio, false);
    if (sm < 0) {
        sd_pio = pio1;
        sm = pio_claim_unused_sm(sd_pio, false);
    }
    if (sm < 0) {
        Serial.printf("ERROR: No PIO state machine available\n");
        return false;
    }
    sd_sm = (uint)sm;
    
    uint offset = pio_add_program(sd_pio, &sd_program);
    
    pio_sm_config c = pio_get_default_sm_config();
    sm_config_set_wrap(&c, offset, offset);
    sm_config_set_out_pins(&c, pin, 1);
    sm_config_set_out_shift(&c, true, true, 32);  // Right shift, autopull at 32
    
    // 488 kHz bit rate - well above 20kHz filter, CPU can keep up
    sm_config_set_clkdiv(&c, 256.0f);
    
    pio_gpio_init(sd_pio, pin);
    pio_sm_set_consecutive_pindirs(sd_pio, sd_sm, pin, 1, true);
    
    pio_sm_init(sd_pio, sd_sm, offset, &c);
    pio_sm_set_enabled(sd_pio, sd_sm, true);
    
    return true;
}

void dac_set_level(uint16_t level_12bit) {
    sd_target = ((uint32_t)(level_12bit & 0xFFF)) << 20;
}

// Generate and push one 32-bit word of sigma-delta data
bool dac_feed_one() {
    if (pio_sm_is_tx_fifo_full(sd_pio, sd_sm)) {
        return false;
    }
    
    uint32_t word = 0;
    uint32_t target = sd_target;
    uint32_t acc = sd_accumulator;
    
    for (int i = 0; i < 32; i++) {
        uint32_t old_acc = acc;
        acc += target;
        if (acc < old_acc) {  // Overflow detected
            word |= (1u << i);
        }
    }
    
    sd_accumulator = acc;
    pio_sm_put(sd_pio, sd_sm, word);
    return true;
}

// Feed FIFO until full
void dac_feed() {
    for (int i = 0; i < 8; i++) {
        if (!dac_feed_one()) break;
    }
}

// Set level and wait for settling (keeps feeding DAC)
void dac_set_and_settle(uint16_t level, uint32_t settle_us) {
    dac_set_level(level);
    uint32_t start = time_us_32();
    while (time_us_32() - start < settle_us) {
        dac_feed_one();
    }
}

// -----------------------------------------------------------------------------
// ADC functions
// -----------------------------------------------------------------------------
void adc_cal_init() {
    adc_init();
    adc_gpio_init(ADC_PIN);
    adc_select_input(ADC_CHAN);
}

uint16_t adc_read_averaged(uint32_t num_samples) {
    uint32_t sum = 0;
    for (uint32_t i = 0; i < num_samples; i++) {
        dac_feed_one();  // Keep DAC running
        sum += adc_read();
    }
    return (sum + num_samples / 2) / num_samples;
}

// -----------------------------------------------------------------------------
// Histogram collection
// -----------------------------------------------------------------------------
void collect_histogram_sweep(bool ascending) {
    Serial.printf("  Sweep %s: ", ascending ? "UP  " : "DOWN");
    
    int start = ascending ? 0 : 4095;
    int end = ascending ? 4095 : 0;
    int step = ascending ? 1 : -1;
    
    int progress_step = 512;
    
    for (int level = start; ascending ? (level <= end) : (level >= end); level += step) {
        dac_set_and_settle(level, SETTLE_US);
        
        // Collect samples at this level
        for (int s = 0; s < SAMPLES_PER_LEVEL; s++) {
            dac_feed_one();
            uint16_t code = adc_read();
            if (code < 4096) {
                histogram[code]++;
            }
        }
        
        // Progress indicator
        if (level % progress_step == 0) {
            Serial.printf("%d ", level);
        }
    }
    Serial.println();
}

void collect_histogram() {
    memset(histogram, 0, sizeof(histogram));
    
    Serial.printf("Collecting histogram (%d sweeps, %d samples/level)...\n", 
                  NUM_SWEEPS, SAMPLES_PER_LEVEL);
    
    for (int sweep = 0; sweep < NUM_SWEEPS; sweep++) {
        collect_histogram_sweep(sweep % 2 == 0);  // Alternate up/down
    }
    
    dac_set_level(0);
    Serial.printf("Histogram collection complete\n\n");
}

// -----------------------------------------------------------------------------
// Build correction LUT from histogram
// -----------------------------------------------------------------------------
void build_correction_lut() {
    Serial.printf("Building correction LUT...\n");
    
    // Find actual ADC range used
    uint16_t code_min = 0, code_max = 4095;
    while (code_min < 4095 && histogram[code_min] == 0) code_min++;
    while (code_max > 0 && histogram[code_max] == 0) code_max--;
    
    cal_data.adc_min = code_min;
    cal_data.adc_max = code_max;
    
    Serial.printf("  ADC range: %d to %d (%d codes)\n", code_min, code_max, code_max - code_min + 1);
    
    // Calculate total samples in valid range
    uint64_t total_samples = 0;
    for (int i = code_min; i <= code_max; i++) {
        total_samples += histogram[i];
    }
    
    double expected_per_code = (double)total_samples / (code_max - code_min + 1);
    Serial.printf("  Total samples: %llu, expected/code: %.1f\n", total_samples, expected_per_code);
    
    // Build cumulative distribution and derive correction
    double cumulative = 0;
    
    for (int i = 0; i < 4096; i++) {
        if (i < code_min) {
            cal_data.correction[i] = 0;
        } else if (i > code_max) {
            cal_data.correction[i] = cal_data.correction[code_max];
        } else {
            // Add half this bin's count (center of bin)
            cumulative += histogram[i] / 2.0;
            
            // Ideal code based on cumulative distribution
            double ideal_code = code_min + (cumulative / expected_per_code);
            
            // Correction is the difference
            cal_data.correction[i] = (int16_t)round(ideal_code) - i;
            
            // Add second half of bin for next iteration
            cumulative += histogram[i] / 2.0;
        }
    }
    
    // Statistics
    int16_t max_corr = 0, min_corr = 0;
    for (int i = code_min; i <= code_max; i++) {
        if (cal_data.correction[i] > max_corr) max_corr = cal_data.correction[i];
        if (cal_data.correction[i] < min_corr) min_corr = cal_data.correction[i];
    }
    Serial.printf("  Correction range: %d to %+d LSBs\n\n", min_corr, max_corr);
    
    cal_data.magic = 0xCA11B8ED;
}

// -----------------------------------------------------------------------------
// Apply correction
// -----------------------------------------------------------------------------
static inline uint16_t adc_corrected(uint16_t raw) {
    int32_t corrected = raw + cal_data.correction[raw];
    if (corrected < 0) corrected = 0;
    if (corrected > 4095) corrected = 4095;
    return corrected;
}

uint16_t adc_read_calibrated(uint32_t oversample) {
    uint32_t sum = 0;
    for (uint32_t i = 0; i < oversample; i++) {
        dac_feed_one();
        uint16_t raw = adc_read();
        sum += adc_corrected(raw);
    }
    return sum / oversample;
}

// -----------------------------------------------------------------------------
// File storage (LittleFS)
// -----------------------------------------------------------------------------
void save_calibration() {
    Serial.printf("Saving calibration to LittleFS...\n");
    
    if (!LittleFS.begin()) {
        Serial.printf("  Formatting LittleFS...\n");
        LittleFS.format();
        if (!LittleFS.begin()) {
            Serial.printf("  ERROR: LittleFS failed!\n");
            return;
        }
    }
    
    // Save CSV for analysis
    File f = LittleFS.open("/adc_cal.csv", "w");
    if (f) {
        f.println("code,count,correction");
        for (int i = 0; i < 4096; i++) {
            f.printf("%d,%lu,%d\n", i, histogram[i], cal_data.correction[i]);
        }
        f.close();
        Serial.printf("  Saved adc_cal.csv\n");
    }
    
    // Save binary for fast loading
    f = LittleFS.open("/adc_cal.bin", "w");
    if (f) {
        f.write((uint8_t*)&cal_data, sizeof(cal_data));
        f.close();
        Serial.printf("  Saved adc_cal.bin (%d bytes)\n", sizeof(cal_data));
    }
    
    Serial.println();
}

bool load_calibration() {
    if (!LittleFS.begin()) {
        return false;
    }
    
    File f = LittleFS.open("/adc_cal.bin", "r");
    if (!f) {
        return false;
    }
    
    size_t bytes = f.read((uint8_t*)&cal_data, sizeof(cal_data));
    f.close();
    
    if (bytes == sizeof(cal_data) && cal_data.magic == 0xCA11B8ED) {
        Serial.printf("Calibration loaded from LittleFS (range %d-%d)\n", 
                      cal_data.adc_min, cal_data.adc_max);
        return true;
    }
    
    return false;
}

// Dump CSV to serial for Jupyter/Python analysis
void dump_csv_to_serial() {
    Serial.println("\n=== CSV Data (copy to file) ===");
    Serial.println("CSV_START");
    Serial.println("code,count,correction");
    for (int i = 0; i < 4096; i++) {
        Serial.printf("%d,%lu,%d\n", i, histogram[i], cal_data.correction[i]);
    }
    Serial.println("CSV_END");
    Serial.flush();
}

// -----------------------------------------------------------------------------
// Verification
// -----------------------------------------------------------------------------
void verify_calibration() {
    Serial.printf("=== Verification ===\n");
    Serial.printf("  %%   | Expected | Raw      | Corrected\n");
    Serial.printf("------+----------+----------+----------\n");
    
    const int test_points[] = {5, 10, 25, 50, 75, 90, 95};
    
    for (int i = 0; i < 7; i++) {
        uint32_t dac_level = (test_points[i] * 4095) / 100;
        uint32_t expected = dac_level;
        
        dac_set_and_settle(dac_level, 5000);  // Extra settling for verification
        
        // Read raw and corrected
        uint32_t raw_sum = 0, corr_sum = 0;
        for (int s = 0; s < 256; s++) {
            dac_feed_one();
            uint16_t raw = adc_read();
            raw_sum += raw;
            corr_sum += adc_corrected(raw);
        }
        uint16_t raw_avg = raw_sum / 256;
        uint16_t corr_avg = corr_sum / 256;
        
        int32_t raw_error = (int32_t)raw_avg - expected;
        int32_t corr_error = (int32_t)corr_avg - expected;
        
        Serial.printf(" %2d%%  | %4d     | %4d (%+3d) | %4d (%+3d)\n",
                      test_points[i], expected, raw_avg, raw_error, corr_avg, corr_error);
    }
    
    dac_set_level(0);
    Serial.println();
}

// -----------------------------------------------------------------------------
// Quick DAC/ADC test
// -----------------------------------------------------------------------------
void quick_test() {
    Serial.printf("=== Quick DAC/ADC Test ===\n");
    
    uint16_t levels[] = {0, 1024, 2048, 3072, 4095};
    
    for (int i = 0; i < 5; i++) {
        dac_set_and_settle(levels[i], 5000);
        uint16_t adc_val = adc_read_averaged(256);
        int error = (int)adc_val - (int)levels[i];
        Serial.printf("  DAC=%4d -> ADC=%4d (error %+d)\n", levels[i], adc_val, error);
    }
    
    dac_set_level(0);
    Serial.println();
}

void setDAC(int level) {
    Serial.printf("=== DAC: %d ===\n", level);
    
    dac_set_and_settle(level, 5000);
    
    uint16_t adc_val = adc_read_averaged(256);
    int error = (int)adc_val - (int)level;
    Serial.printf("  DAC=%4d -> ADC=%4d (error %+d)\n", level, adc_val, error);
    
    Serial.println();
}

// -----------------------------------------------------------------------------
// Full calibration routine
// -----------------------------------------------------------------------------
void run_calibration() {
    Serial.printf("\n");
    Serial.printf("╔════════════════════════════════════════╗\n");
    Serial.printf("║     RP2040 ADC Calibration             ║\n");
    Serial.printf("║     Sigma-Delta DAC Method             ║\n");
    Serial.printf("╚════════════════════════════════════════╝\n\n");
    
    Serial.printf("Hardware: Pin %d (DAC) -> Filter -> Pin %d (ADC)\n\n", DAC_PIN, ADC_PIN);
    
    quick_test();
    
    collect_histogram();
    build_correction_lut();
    save_calibration();
    
    verify_calibration();
    
    Serial.printf("=== Calibration Complete ===\n\n");
}

// -----------------------------------------------------------------------------
// Main
// -----------------------------------------------------------------------------
void setup() {
    Serial.begin(115200);
    while (!Serial) { delay(10); }
    
    Serial.printf("\n\nInitializing...\n");
    
    // Initialize DAC
    if (!dac_init(DAC_PIN)) {
        Serial.printf("DAC init failed!\n");
        while(1) delay(1000);
    }
    Serial.printf("  DAC ready (PIO sigma-delta on pin %d)\n", DAC_PIN);
    
    // Prime DAC FIFO
    dac_set_level(0);
    for (int i = 0; i < 100; i++) {
        dac_feed();
        delayMicroseconds(100);
    }
    
    // Initialize ADC
    adc_cal_init();
    Serial.printf("  ADC ready (pin %d, channel %d)\n\n", ADC_PIN, ADC_CHAN);
    
    // Check for existing calibration
    if (load_calibration()) {
        Serial.printf("Using existing calibration. Send 'c' to recalibrate.\n\n");
        verify_calibration();
    } else {
        Serial.printf("No calibration found. Running calibration...\n");
        run_calibration();
    }
}

void loop() {
    // Keep DAC running
    dac_feed();
    
    // Check for serial commands
    if (Serial.available()) {
        char cmd = Serial.read();
        switch (cmd) {
            case 'c':
            case 'C':
                run_calibration();
                break;
            case 't':
            case 'T':
                quick_test();
                break;
            case 'v':
            case 'V':
                verify_calibration();
                break;
            case 'd':
            case 'D':
                dump_csv_to_serial();
                break;
            case 'h':
            case 'H':
            case '?':
                Serial.printf("\nCommands:\n");
                Serial.printf("  c - Run calibration\n");
                Serial.printf("  t - Quick test\n");
                Serial.printf("  v - Verify calibration\n");
                Serial.printf("  d - Dump CSV to serial\n");
                Serial.printf("  h - This help\n\n");
                break;
            case 'n':
            case 'N':
                setDAC(0);
                break;
            case 'm':
            case 'M':
                setDAC(4095);
                break;
        }
    }
    
    delay(1);
}
