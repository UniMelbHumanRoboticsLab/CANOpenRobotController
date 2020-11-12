////////// STATE ////////////////////
//-------  Sitting ------------/////
////////////////////////////////////
#include "Sitting.h"
void Sitting::entry() {
    spdlog::info("Sitting State Entered");
    //READ TIME OF MAIN
    std::cout << "=======================" << std::endl
              << " HIT W to begin standing up" << std::endl
              << "=======================" << std::endl
              << std::endl;
}
void Sitting::during() {
}
void Sitting::exit() {
    spdlog::info("Sitting State Exited");
}