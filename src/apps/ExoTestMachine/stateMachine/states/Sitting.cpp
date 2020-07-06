////////// STATE ////////////////////
//-------  Sitting ------------/////
////////////////////////////////////
#include "Sitting.h"
void Sitting::entry() {
    //READ TIME OF MAIN
    std::cout << "Sitting State Entered at Time:" << std::endl
              << "=======================" << std::endl
              << " HIT W to begin standing up" << std::endl
              << "=======================" << std::endl
              << std::endl;
}
void Sitting::during() {
}
void Sitting::exit() {
    std::cout
        << "Sitting State Exited" << std::endl;
}