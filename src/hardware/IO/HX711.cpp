

#include "Hx711.h"

//#define SHIFTIN_WITH_SPEED_SUPPORT(data,clock,order) shiftInSlow(data,clock,order)

#define HIGH true
#define LOW false

HX711::HX711() {
    iolib_init();
    iolib_setdir(2, 10, BBBIO_DIR_OUT);
    iolib_setdir(2, 8, BBBIO_DIR_IN);
    iolib_setdir(2, 6, BBBIO_DIR_IN);
}

HX711::~HX711() {

}

//half
void HX711::begin(std::string dout, std::string pd_sck, int gain) {
    set_gain(gain);
}

void HX711::updateInput() {
    force = read();
}

char HX711::shiftInSlow(std::string dataPin, std::string clockPin) {
    uint8_t value = 0;
    uint8_t value2 = 0;

    for (int i = 0; i < 8; ++i) {
        clock_digitalWrite(HIGH);
        value |= digitalRead(6) << (7 - i);
        value2 |= digitalRead(8) << (7 - i);

        clock_digitalWrite(LOW);
    }
    spdlog::info("{}, {}", value, value2);    
    return value;
}

void HX711::clock_digitalWrite (bool value) {

    if (value) {
        pin_high(2,10);
    }
    else {
        pin_low(2, 10);
    }    
}

uint8_t HX711::digitalRead(int pin) {
    if (is_high(2,pin)){
        printf("1");
        return true;
    } else {
        printf("0");

        return false;
    }
}

//no
bool HX711::is_ready() {
    return digitalRead(8) == LOW;
}

//go
void HX711::set_gain(uint8_t gain) {
    switch (gain) {
        case 128:        // channel A, gain factor 128
            GAIN = 1;
            break;
        case 64:        // channel A, gain factor 64
            GAIN = 3;
            break;
        case 32:        // channel B, gain factor 32
            GAIN = 2;
            break;
    }

}

long HX711::read() {
    spdlog::info("HX711::read()");

    typedef unsigned char uint8_t;
    // Wait for the chip to become ready.
    clock_digitalWrite(LOW);
    wait_ready(10);

    // Define structures for reading data into.
    unsigned long value = 0;
    uint8_t data[4] = { 0 };
    // Pulse the clock pin 24 times to read the data.
    data[2] = shiftInSlow(DOUT, PD_SCK);
    data[1] = shiftInSlow(DOUT, PD_SCK);
    data[0] = shiftInSlow(DOUT, PD_SCK);

    // Set the channel and the gain factor for the next reading using the clock pin.
    for (int i = 0; i < GAIN; i++) {
        clock_digitalWrite(HIGH);
        usleep(2);
        clock_digitalWrite(LOW);
    }

    // Replicate the most significant bit to pad out a 32-bit signed integer
    if (data[2] & 0x80) {
        data[3] = 0xFF;
    } else {
        data[3] = 0x00;
    }

    // Construct a 32-bit signed integer
    value = ( static_cast<unsigned long>(data[3]) << 24
            | static_cast<unsigned long>(data[2]) << 16
            | static_cast<unsigned long>(data[1]) << 8
            | static_cast<unsigned long>(data[0]) );


    spdlog::info("Reading: {0:x}", value);
    return static_cast<long>(value);
}

//go
void HX711::wait_ready(unsigned long delay_ms) {
    while (!is_ready()) {
        usleep(delay_ms*1000);
    }
}

//go
bool HX711::wait_ready_retry(int retries, unsigned long delay_ms) {
    int count = 0;
    while (count < retries) {
        printf("wait_ready_retry\n");

        if (is_ready()) {
            return true;
        }
        usleep(delay_ms*1000);
        count++;
    }
    return false;
}

//go
bool HX711::wait_ready_timeout(unsigned long timeout, unsigned long delay_ms) {

    timespec startTime;
    clock_gettime(CLOCK_MONOTONIC, &startTime);

    timespec currTime;
    clock_gettime(CLOCK_MONOTONIC, &currTime);

    double elapsedSec = currTime.tv_sec - startTime.tv_sec + (currTime.tv_nsec - startTime.tv_nsec) / 1e6;

    while (elapsedSec < timeout) {
        if (is_ready()) {
            return true;
        }
        usleep(delay_ms*1000);
        elapsedSec = currTime.tv_sec - startTime.tv_sec + (currTime.tv_nsec - startTime.tv_nsec) / 1e6;
    }
    return false;
}

//go
long HX711::read_average(int times) {
    long sum = 0;
    for (int i = 0; i < times; i++) {
        sum += read();
    }
    return sum / times;
}

//go
double HX711::get_value(int times) {
    return read_average(times) - OFFSET;
}

//go
float HX711::get_units(int times) {
    return get_value(times) / SCALE;
}

//go
void HX711::tare(int times) {
    double sum = read_average(times);
    set_offset(sum);
}

//go
void HX711::set_scale(float scale) {
    SCALE = scale;
}

//go
float HX711::get_scale() {
    return SCALE;
}

//go    
void HX711::set_offset(long offset) {
    OFFSET = offset;
}

//go
long HX711::get_offset() {
    return OFFSET;
}

//no
void HX711::power_down() {
    clock_digitalWrite(LOW);
    clock_digitalWrite(HIGH);
}

//no
void HX711::power_up() {
    clock_digitalWrite(LOW);
}
