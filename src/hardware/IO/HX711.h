
#ifndef HX711_h
#define HX711_h
#include <time.h>
#include <unistd.h>

#include <Eigen/Dense>
#include <fstream>
#include <iostream>
#include <string>

#include "InputDevice.h"
#include "iobb.h"

class HX711 : public InputDevice {
   private:
    int GAIN;      // amplification factor. Can only have one for all sensors
    Eigen::VectorXi OFFSET;  //= 0;     // used for tare weight
    Eigen::VectorXd SCALE;   //= 1;     // used to return weight in grams, kg, ounces, whatever

    Eigen::Matrix<int, Eigen::Dynamic, 2> inputPins;
    Eigen::Vector2i clockPin;

    Eigen::VectorXd force;  //= 0;
    Eigen::VectorXi rawData;  //= 0;

    // Write a value to the clock pin
    void clock_digitalWrite(bool value);

    // Read a value from a data pin
    uint8_t digitalRead(int sensorNum);

   public:
    HX711(Eigen::Matrix<int, Eigen::Dynamic, 2> inputPins, Eigen::Vector2i clockPin);
    ~HX711();

    void updateInput();
    bool configureMasterPDOs() { return true; };

    // Initialize library with data output pin, clock input pin and gain factor.
    // Channel selection is made by passing the appropriate gain:
    // - With a gain factor of 64 or 128, channel A is selected
    // - With a gain factor of 32, channel B is selected
    // The library default is "128" (Channel A).
    void begin(int gain = 128);

    // Check if HX711 is ready
    // from the datasheet: When output data is not ready for retrieval, digital output pin DOUT is high. Serial clock
    // input PD_SCK should be low. When DOUT goes to low, it indicates data is ready for retrieval.
    bool is_ready();

    // Wait for the HX711 to become ready
    void wait_ready(unsigned long delay_ns = 0);

    // set the gain factor; takes effect only after a call to read()
    // channel A can be set for a 128 or 64 gain; channel B has a fixed 32 gain
    // depending on the parameter, the channel is also set to either A or B
    void set_gain(uint8_t gain = 128);

    // Get the latest Raw Data (INTEGER32) from all sensors
    Eigen::VectorXi getAllRawData();

    // Get the latest Raw Data (INTEGER32) from a single sensor
    double getRawData(int sensorNum);

    // Get the latest force measurement from all sensors
    Eigen::VectorXd& getAllForces();

    // Get the latest force measurement from a single
    double getForce(int sensorNum);

    // set the SCALE value; this value is used to convert the raw data to "human readable" data (measure units)
    void set_scale(int sensorNum,double scale = 1);

    // get the current SCALE
    double get_scale(int sensorNum);

    // set OFFSET, the value that's subtracted from the actual reading (tare weight)
    void set_offset(int sensorNum,INTEGER32 offset = 0);

    // get the current OFFSET
    INTEGER32 get_offset(int sensorNum);

    // puts the chip into power down mode
    void power_down();

    // wakes up the chip after power down mode
    void power_up();
};

#endif /* HX711_h */
