

#include "HX711.h"


#define HIGH true
#define LOW false

HX711::HX711(Eigen::Matrix<int, Eigen::Dynamic, 2> inputPins, Eigen::Vector2i clockPin) {
    iolib_init();
    iolib_setdir(clockPin[0], clockPin[1], BBBIO_DIR_OUT);
    spdlog::info("Clock Pin P{}.{}", clockPin[0], clockPin[1]);

    GAIN = 0;                                      // amplification factor
    OFFSET = Eigen::VectorXi::Zero(inputPins.rows());  //= 0;     // used for tare weight
    SCALE = Eigen::VectorXd::Zero(inputPins.rows());   //= 1;     // used to return weight in grams, kg, ounces, whateverc
    force = Eigen::VectorXd::Zero(inputPins.rows());
    rawData = Eigen::VectorXi::Zero(inputPins.rows());

    for (int i = 0; i < inputPins.rows(); i++) {
        iolib_setdir(inputPins(i,0), inputPins(i,1), BBBIO_DIR_IN);
        SCALE(i) = 1;
        OFFSET(i) = 0;
        spdlog::info("Input Pin P{}.{}", inputPins(i, 0), inputPins(i, 1));
    }

    this->inputPins = inputPins;
    this->clockPin = clockPin;
}

HX711::~HX711() {
}

//half
void HX711::begin(int gain) {
    set_gain(gain);
}

void HX711::updateInput() {
    spdlog::trace("HX711::updateInput()");

    // Wait for the chip to become ready.
    clock_digitalWrite(LOW);
    wait_ready(10);

    // Define structures for reading data into.
    unsigned long data[inputPins.rows()] = {0};

    timespec startTime;
    timespec currTime;
    for (int i = 0; i < 24; ++i) {

        clock_digitalWrite(HIGH);
        clock_gettime(CLOCK_MONOTONIC, &startTime);
        for (int j = 0; j < inputPins.rows(); j++)
            data[j] |= digitalRead(j) << (24 - i);

        clock_digitalWrite(LOW);
        clock_gettime(CLOCK_MONOTONIC, &currTime);
        //spdlog::info("{}",(currTime.tv_sec - startTime.tv_sec)*1e6 + (currTime.tv_nsec - startTime.tv_nsec) / 1e3);
    }

    // Set the channel and the gain factor for the next reading using the clock pin.
    for (int i = 0; i < GAIN; i++) {
        clock_digitalWrite(HIGH);
        usleep(1);
        clock_digitalWrite(LOW);
    }

    // Replicate the most significant bit to pad out a 32-bit signed integer
    for (int j = 0; j < inputPins.rows(); j++) {
        if (data[j] & 0x800000) {
            data[j] |= 0xFF000000;
        }

        // Sometimes errorneous data comes in. Usually 0xFFFFFFFE or 0xFFFFE. Ignore if this is the case
        // TODO: Better comparison might be if all of them are equal
        if((data[j] & 0xFFFE) != 0xFFFE){
            rawData(j) = static_cast<INTEGER32>(data[j]);
            force(j) = (rawData(j) - OFFSET(j))*SCALE(j);
        }
    }
}



void HX711::clock_digitalWrite(bool value) {
    if (value) {
        pin_high(clockPin[0], clockPin[1]);
    } else {
        pin_low(clockPin[0], clockPin[1]);
    }
}

uint8_t HX711::digitalRead(int sensorNum) {
    
    if (is_high(inputPins(sensorNum, 0), inputPins(sensorNum, 1))) {
        //printf("1");
        return true;
    } else {
        //printf("0");
        return false;
    }
}

//no
bool HX711::is_ready() {
    bool returnValue = true;

    for (int j =0; j< inputPins.rows(); j++){
        returnValue &= (digitalRead(j)==LOW);
    }

    return returnValue;
}

//go
void HX711::set_gain(uint8_t gain) {
    switch (gain) {
        case 128:  // channel A, gain factor 128
            GAIN = 1;
            break;
        case 64:  // channel A, gain factor 64
            GAIN = 3;
            break;
        case 32:  // channel B, gain factor 32
            GAIN = 2;
            break;
    }
}


//go
void HX711::wait_ready(unsigned long delay_ms) {
    while (!is_ready()) {
        usleep(delay_ms * 1000);
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
        usleep(delay_ms * 1000);
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
        usleep(delay_ms * 1000);
        elapsedSec = currTime.tv_sec - startTime.tv_sec + (currTime.tv_nsec - startTime.tv_nsec) / 1e6;
    }
    return false;
}

//go
Eigen::VectorXi HX711::getAllRawData() {
    return rawData;
}

//go
double HX711::getRawData(int sensorNum) {
    return rawData(sensorNum);
}

Eigen::VectorXd& HX711::getAllForces() {
    return force;
}

//go
double HX711::getForce(int sensorNum) {
    return force(sensorNum);
}

//go
void HX711::set_scale(int sensorNum,double scale) {
    SCALE[sensorNum] = scale;
}

//go
double HX711::get_scale(int sensorNum) {
    return SCALE(sensorNum);
}

//go
void HX711::set_offset(int sensorNum, INTEGER32 offset) {
    OFFSET(sensorNum) = offset;
    spdlog::debug("OffsetSet {}, {}", sensorNum, OFFSET(sensorNum));
}

//go
INTEGER32 HX711::get_offset(int sensorNum) {
    return OFFSET(sensorNum);
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
