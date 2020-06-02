////////// STATE ////////////////////
//-------  Standing ------------/////
////////////////////////////////////
#include "Standing.h"

void Standing::entry(void) {
    std::cout
        << "======================" << std::endl
        << " HIT W -> Sit DOWN" << std::endl
        << "======================" << std::endl;
}

void Standing::during(void) {
}

void Standing::exit(void) {
    std::cout
        << "Standing State Exited" << std::endl;
}
