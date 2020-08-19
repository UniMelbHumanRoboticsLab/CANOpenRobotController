/**
 * 
 * \file ExoTestMachine.h
 * \author William Campbell 
 * \version 0.1
 * \date 2019-09-24
 * \copyright Copyright (c) 2020
 * 
 * \breif The <code>Keyboard</code> class is an implementation of the abstract <class>Input</class>class for a computer keyboard.
 *    The device has key states which maintain the current values for a given programs update frame or refresh rate. 
 *
 *
 * Version 0.1
 * Date: 07/04/2020
 */

#ifndef KEYBOARD_H_INCLUDED
#define KEYBOARD_H_INCLUDED

#include <unistd.h>

#include "InputDevice.h"
#include "termios.h"
#define NB_DISABLE 0
#define NB_ENABLE 1

/**
 * \brief Struct listing the Keys which exist on a Keyboard.
 * 
 */
typedef struct keys {
    bool a;
    bool s;
    bool d;
    bool w;
    bool x;
    bool q;
} key_states;

/**
 * \brief Example InputDevice which takes input in from a keyboard. Useful for testing without any other input devices
 * 
 */
class Keyboard : public InputDevice {
   private:
    key_states currentKeyStates = {false, false, false, false, false, false};
    int keyboardActive;

   public:
    /**
         * \brief Construct a new keyboard object
         * 
         * * Note: Designer must implement specifc keyboard keys they wish to use.
         *      Add an entry to key_states struct, last and current key State object and edit
         *       set Keys and creat a getter method.
         */
    Keyboard();
    ~Keyboard();
    /**
 * \brief getter method for key_states.
 * 
 */
    key_states getStates();
    /**
 * \brief reads one character from stdin and updates coresponding key state (if one occured)
 * 
 */
    void setKeys();
    /**
    * \brief defintion of <class>Input</class> pure virtual function. Updates the keyboard input devices
    * memory states from implemented keyboard input from user. E.g. A key has been pressed or not.
    * 
    */
    void updateInput();
    /**
 * \brief clear the current key state variables
 * 
 */
    void clearCurrentStates();
    /**
 * \brief Helper method, prints current keys registered as pressed
 * 
 */
    void printPressed();
    // Returns true if the key is pressed.
    /**
 * \brief Getter method for private A key state
 * 
 */
    bool getA();
    /**
 * \brief Getter method for private S key state
 * 
 */
    bool getS();
    /**
 * \brief Getter method for private D key state
 * 
 */
    bool getD();
    /**
 * \brief Getter method for private W key state
 * 
 */
    bool getW();
    /**
 * \brief Getter method for private X key state
 * 
 */
    bool getX();
    /**
 * \brief Getter method for private Q key state
 * 
 */
    bool getQ();
    /**
 * \brief Termios structs for turning on and off terminal echo
 * 
 */
    struct termios original, noecho;
    char ch;
    /**
 * \brief Check if keyboard has been hit - is stdin active.
 * 
 */
    int kbhit();
    /**
 * \brief Configure stdin to be nonblocking to rest of program
 * 
 */
    void nonblock(int state);
    /**
 * \brief getter method for keyboard active flag, set by kbhit function.
 * 
 */
    int getKeyboardActive();
    /**
 * \brief setter method for keyboard active flag, set by kbhit function.
 * 
 */
    void setKeyboardActive(int value);
};
#endif