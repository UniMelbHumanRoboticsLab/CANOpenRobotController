/**
 * /file LogHelper.h
 * /author Emek Baris Kucuktabak
 * /brief Helper Class to easy use of SPDLog
 * /version 0.1
 * /date 2020-09-23
 *
 * @copyright Copyright (c) 2020
 *
 */

#ifndef SRC_LOGHELPER_H
#define SRC_LOGHELPER_H

#include <string>
#include <iostream>

class LogElementBase{
public:
    virtual void printValue(){};

};

template <typename ValueType_>
class LogElement : public LogElementBase{

private:
    ValueType_* ptr;
    std::string name;
    ValueType_ val;
public:
    LogElement(ValueType_* ptr, std::string name):
    ptr(ptr),
    name(name)
    {}

    void printValue(){
        std::cout<<"Value of "<<name<<": "<<*((ValueType_*)ptr)<<std::endl;
    }
};


class LogHelper {

public:

    std::vector<std::shared_ptr<LogElementBase>> vectorOfLogElements;

    template<typename ValueType_>
    void add(ValueType_ &var, const std::string & name) {
        LogElement<ValueType_>* logElement = new LogElement<ValueType_>(&var, name);
        vectorOfLogElements.push_back(std::shared_ptr<LogElementBase>(logElement));

    }

    void printValue(){
//        std::cout<< *((vectorOfTypes[0] *) vectorOfPtrs[0]) <<std::endl;
        vectorOfLogElements[0]->printValue();
    }

};



#endif //SRC_LOGHELPER_H
