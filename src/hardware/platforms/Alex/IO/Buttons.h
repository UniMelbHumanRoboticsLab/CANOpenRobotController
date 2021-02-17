/**
 * \file Buttons.h
 * \author John Laidlaw
 * \brief  The <code>Button</code> class is an implementation of the abstract <class>Input</class>class for the red E-stop button.
 * \version 0.1
 * \date 2020-07-21
 * 
 * \copyright Copyright (c) 2020
 * 
 */

#include <unistd.h>
// #include <string>

#include "InputDevice.h"

class Buttons : public InputDevice {
    private:
     bool errorButton;

     std::string errorButtonPath = "/sys/class/gpio/gpio68/value";

     /**
      * \brief check the value of the e-stop button
      * 
      */
     bool checkButton(std::string path);
    
    public:
     /**
      * \brief construct and a new Button object, deconstructor yet to be implemented
      * 
      */
     Buttons();
     ~Buttons(); // Destructor for the Buttons object

     /**
      * \brief definition of <class>Input<class> pure virtual function. Updates the button input status to check if it 
      * has been pressed or not.
      * 
      */
     void updateInput();
     bool getErrorButton();

     bool configureMasterPDOs(){return true;};
};