////////// STATE ////////////////////
//-------  Standing ------------/////
////////////////////////////////////
#include "Standing.h"

void Standing::entry(void) {
    spdlog::info("Standing State Entered");
    std::cout
        << "======================" << std::endl
        << " HIT W -> Sit DOWN" << std::endl
        << "======================" << std::endl;
}

void Standing::during(void) {
}

void Standing::exit(void) {
    spdlog::info("Standing State Exited");
}
