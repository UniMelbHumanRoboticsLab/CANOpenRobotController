#include "M1ForceSensor.h"


double M1ForceSensor::sensorValueToNewton(int sensorValue) {
//    std::cout << "Sensor readings : " << sensorValue << "\n";
    if(sensorValue > 1000)
        return (sensorValue-1500.0)*0.1;
    else
        return sensorValue*0.1; // /4.0 todo: change after sensor experimentation
//    return sensorValue*4.0; // /4.0 todo: change after sensor experimentation

}


