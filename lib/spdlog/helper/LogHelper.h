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

#include <Eigen/Dense>
#include "spdlog/spdlog.h"
#include "spdlog/async.h"
#include "spdlog/sinks/basic_file_sink.h"
#include "spdlog/fmt/bin_to_hex.h"
#include "spdlog/sinks/rotating_file_sink.h"

enum LogFormat {
    CSV = 0,
    //todo: add BINARY
};

class LogElementBase{
public:
    virtual std::string getName() {};
    virtual void printValue() {};

};

template <typename ValueType_>
class LogElement : public LogElementBase{

public:
    const ValueType_* ptr_;
    const std::string name_;
    ValueType_ val_;

    LogElement(ValueType_* ptr, std::string name):
    ptr_(ptr),
    name_(name)
    {}

    std::string getName(){
        return getNameImplementation<ValueType_>();
    }

private:
    // if scalar just returns the name_
    template <typename T>
    typename std::enable_if<std::is_scalar<T>::value, std::string>::type getNameImplementation(){

        return name_;
    }

    template <typename T>
    typename std::enable_if<!std::is_scalar<T>::value, std::string>::type getNameImplementation(){
        std::string name = "";
        int sizeOfVar = (*((ValueType_*)ptr_)).size();
        for(int i=0; i<sizeOfVar; i++){
            name += name_ + "_" + std::to_string(i+1);

            if(i!= sizeOfVar-1) name += ", ";

        }

        return name;
    }
};

/**
 * \brief Helper class that allows easier use of spdlog.
 *
 *
 */

class LogHelper {

private:
    std::string loggerName_;
    bool isInitialized_ = false;
    bool isStarted_ = false;
    LogFormat logFormat_;

public:
    std::vector<std::shared_ptr<LogElementBase>> vectorOfLogElements;

    /**
       * \brief Templated member to add different typed variables to the logger
       *
       * \return bool success of adding the variable
      */
    template<typename ValueType_>
    bool add(ValueType_ &var, const std::string & name) {
        if(isStarted_){
            spdlog::error("Logger already started. Can't add further variables.");
            return false;
        }
        LogElement<ValueType_>* logElement = new LogElement<ValueType_>(&var, name);
        vectorOfLogElements.push_back(std::shared_ptr<LogElement<ValueType_>>(logElement));
        return true;
    }

    /**
    * \brief Initialize the logger
    *
    * \param loggerName name of the logger
    * \param fileName file name to create the log document
    * \param logFormat format of the log (csv, binary etc.)
    * \return bool success of homing
    */
    bool initLogger(std::string loggerName, std::string fileName, LogFormat logFormat){
        loggerName_ = loggerName;
        logFormat_ = logFormat;

        if(logFormat != LogFormat::CSV){
            spdlog::error("UNHANDLED MESSAGE TYPE!");
            isInitialized_ = false;
        }
        else if(logFormat == LogFormat::CSV){
            auto logger = spdlog::basic_logger_mt<spdlog::async_factory>(loggerName, fileName);
            logger->set_pattern("%v");
            isInitialized_ = true;
        }
        return isInitialized_;
    }


    bool startLogger(){
        if(!isInitialized_){
            spdlog::error("Can't start the Logger without initializing first");
            return false;
        }

        else{
            std::string headerMsg = "";
            for(int i=0; i < vectorOfLogElements.size(); i++){ // iterating through each variable to get their names

                headerMsg += vectorOfLogElements[i]->getName();

                // either a coma or new line comes
                if(i != vectorOfLogElements.size()-1){
                    headerMsg += ", ";
                }else headerMsg += "\n";

            }
            std::cout<<"HEADER MSG: "<<headerMsg<<std::endl;
            return true;
        }


//        spdlog::get(loggerName_)->info(vectorOfLogElements[0]->getValue());
//        spdlog::get("test_logger")->info(vectorOfLogElements[0]->getValue());



    }

    void printValue(){
//        std::cout<< *((vectorOfTypes[0] *) vectorOfPtrs[0]) <<std::endl;
//        vectorOfLogElements[0]->printValue();
//        vectorOfLogElements[1]->printValue();
    }

};



#endif //SRC_LOGHELPER_H
