
#include "Keyboard.h"

Keyboard::Keyboard() {
    keyboardActive = NB_DISABLE;
    nonblock(NB_ENABLE);
    /* obtain the current terminal configuration */
    tcgetattr(STDIN_FILENO, &original);
    /* duplicate it */
    noecho = original;
    /* turn off full duplex */
    noecho.c_lflag = noecho.c_lflag ^ ECHO;
    /* set the terminal */
    tcsetattr(STDIN_FILENO, TCSANOW, &noecho);

    for(int i=0; i<10; i++) {
        currentKeyStates.Nb.push_back(false);
    }
    clearCurrentStates();

    spdlog::debug("Keyboard object created, echo disabled");
}
Keyboard::~Keyboard() {
    /* restore the terminal settings */
    tcsetattr(STDIN_FILENO, TCSANOW, &original);
    spdlog::debug("Keyboard object deleted, echo enabled");
};
void Keyboard::updateInput() {
    clearCurrentStates();
    setKeyboardActive(kbhit());
    if (getKeyboardActive() != 0) {
        setKeys();
        printPressed();
    }
}
void Keyboard::setKeys() {
    /// set last Key states
    char ch = fgetc(stdin);
    /* Set States, limited to one key Press at a time*/

    switch (ch) {
        // TO do add this case when timeout of read occurs
        //case ERR:
        //     //do nothing, just capture and keep runnin
        //     printf("No input from keyboard\n");
        case 'a':
        case 'A':
            currentKeyStates.a = true;
            break;
        case 's':
        case 'S':
            currentKeyStates.s = true;
            break;
        case 'd':
        case 'D':
            currentKeyStates.d = true;
            break;
        case 'w':
        case 'W':
            currentKeyStates.w = true;
            break;
        case 'x':
        case 'X':
            currentKeyStates.x = true;
            break;
        case 'q':
        case 'Q':
            currentKeyStates.q = true;
            break;
        default:
            keyboardActive = 0;
    }
    //Number keys
    if(ch>=48 && ch<48+currentKeyStates.Nb.size()){
        currentKeyStates.Nb[ch-48] = true;
        keyboardActive = 1;
    }
    //Any key
    currentKeyStates.key_code = ch;
}


void Keyboard::printPressed() {
    if (getNb()>=0) {
        spdlog::info("PRESSED #{}", getNb());
    }
    if (getKeyUC()>0) {
        spdlog::info("PRESSED {}", (char)getKeyUC());
    }
}
void Keyboard::clearCurrentStates() {
    currentKeyStates.a = false;
    currentKeyStates.s = false;
    currentKeyStates.d = false;
    currentKeyStates.w = false;
    currentKeyStates.x = false;
    currentKeyStates.q = false;
    for(unsigned int i=0; i<10; i++)
        currentKeyStates.Nb[i] = false;
    currentKeyStates.key_code = -1;
}

bool Keyboard::getA() {
    return currentKeyStates.a;
}
bool Keyboard::getS() {
    return currentKeyStates.s;
}
bool Keyboard::getD() {
    return currentKeyStates.d;
}
bool Keyboard::getW() {
    return currentKeyStates.w;
}
bool Keyboard::getX() {
    return currentKeyStates.x;
}
bool Keyboard::getQ() {
    return currentKeyStates.q;
}
int Keyboard::getNb() {
    for(unsigned int i=0; i<currentKeyStates.Nb.size(); i++) {
        if(currentKeyStates.Nb[i]){
            return i;
        }
    }
    return -1;
}
int Keyboard::getKeyCode() {
    return currentKeyStates.key_code;
}
int Keyboard::getKeyUC() {
    //Lower-case
    if(currentKeyStates.key_code>=97 && currentKeyStates.key_code<=122) {
        return (char) (currentKeyStates.key_code-32);
    }
    //Upper-case
    if(currentKeyStates.key_code>=65 && currentKeyStates.key_code<=90) {
        return (char) (currentKeyStates.key_code);
    }
    return -1;
}

int Keyboard::kbhit() {
    struct timeval tv;
    fd_set fds;
    tv.tv_sec = 0;
    tv.tv_usec = 0;
    FD_ZERO(&fds);
    FD_SET(STDIN_FILENO, &fds);  //STDIN_FILENO is 0
    select(STDIN_FILENO + 1, &fds, NULL, NULL, &tv);
    return FD_ISSET(STDIN_FILENO, &fds);
}
void Keyboard::nonblock(int state) {
    struct termios ttystate;

    //get the terminal state
    tcgetattr(STDIN_FILENO, &ttystate);

    if (state == NB_ENABLE) {
        //turn off canonical mode
        ttystate.c_lflag &= ~ICANON;
        //minimum of number input read.
        ttystate.c_cc[VMIN] = 1;
    } else if (state == NB_DISABLE) {
        //turn on canonical mode
        ttystate.c_lflag |= ICANON;
    }
    //set the terminal attributes.
    tcsetattr(STDIN_FILENO, TCSANOW, &ttystate);
}
int Keyboard::getKeyboardActive() {
    return keyboardActive;
};
void Keyboard::setKeyboardActive(int value) {
    keyboardActive = value;
};
