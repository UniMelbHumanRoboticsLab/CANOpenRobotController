////////// STATE ////////////////////
//-------  Standing ------------/////
////////////////////////////////////
#include "Standing.h"

void Standing::entry(void) {
    std::cout
        << "======================" << endl
        << " HIT W -> Sit DOWN" << endl

        << "======================" << endl;
}

void Standing::during(void) {
}

void Standing::exit(void) {
    std::cout
        << "Standing State Exited" << endl;
}
