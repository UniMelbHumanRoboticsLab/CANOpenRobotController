/**
 * /file LogHelper.h
 * /author Emek Baris Kucuktabak
 * /brief Helper Classes to easy use of spdLog
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

/**
 * \brief pure virtual base class of LogElement
 *
 */
class LogElementBase{
public:
    virtual std::string getName() = 0;
    virtual std::string getValue() = 0;

};

/**
 * \brief Templated logElement class to access values and names of different typed variables
 * \param ptr pointer to the variable
 * \param name name of the variable
 *
 */
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

    std::string getValue(){
        return getValueImplementation<ValueType_>();
    }

private:
    /**
     * \brief If the variable is scalar, just returns the variable name
     *
     */
    template <typename T>
    typename std::enable_if<std::is_scalar<T>::value, std::string>::type getNameImplementation(){

        return name_;
    }

    /**
     * \brief If the variable is not scalar, returns the name such that name_1, name_2, name_3...
     *
     */
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

    /**
     * \brief If the variable is scalar, just returns the variable value
     *
     */
    template <typename T>
    typename std::enable_if<std::is_scalar<T>::value, std::string>::type getValueImplementation(){

        return std::to_string(*((ValueType_*)ptr_));
    }

    /**
     * \brief If the variable is not scalar, returns the value such that value_1, value_2, value_3...
     *
     */
    template <typename T>
    typename std::enable_if<!std::is_scalar<T>::value, std::string>::type getValueImplementation(){
        std::string valueStr = "";

        int sizeOfVar = (*((ValueType_*)ptr_)).size();

        for(int i=0; i<sizeOfVar; i++){
            valueStr += std::to_string((*((ValueType_*)ptr_))[i]);

            if(i!= sizeOfVar-1) valueStr += ", ";
        }
        return valueStr;
    }
};

/**
 * \brief Helper class that allows easier use of spdlog.
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
    * \brief Initialize an asynchronous logger
    *
    * \param loggerName name of the logger
    * \param fileName file name to create the log document
    * \param logFormat format of the log (csv, binary etc.)
    * \param truncate boolean flag to clear the file at the beginning
    * \return bool success of homing
    */
    bool initLogger(std::string loggerName, std::string fileName, LogFormat logFormat, bool truncate){
        loggerName_ = loggerName;
        logFormat_ = logFormat;

        if(logFormat != LogFormat::CSV){
            spdlog::error("UNHANDLED MESSAGE TYPE!");
            isInitialized_ = false;
        }
        else if(logFormat == LogFormat::CSV){
            auto logger = spdlog::basic_logger_mt<spdlog::async_factory>(loggerName, fileName, truncate);
            logger->set_pattern("%v");
            isInitialized_ = true;
        }
        return isInitialized_;
    }

    /**
    * \brief Is initialised?
    *
    * \return true if logger is initialised, false otherwise
    */
    bool isInitialised() {
        return isInitialized_;
    }

    /**
    * \brief Is started?
    *
    * \return true if logger is started, false otherwise
    */
    bool isStarted() {
        return isStarted_;
    }

    /**
     * \brief Start the logger. Generates the header based on the added variables
     *
     */
    bool startLogger(){
        if(!isInitialized_){
            spdlog::error("Can't start the Logger without initializing first");
            return false;
        }
        else{
            if(vectorOfLogElements.size()>0){
                std::string headerMsg = "";
                for(unsigned int i=0; i < vectorOfLogElements.size(); i++){ // iterating through each variable to get their names
                    headerMsg += vectorOfLogElements[i]->getName();

                    // either a coma or new line comes
                    if(i != vectorOfLogElements.size()-1){
                        headerMsg += ", ";
                    }
                }
                spdlog::info("Starting logger {} ({})", loggerName_, headerMsg);
                spdlog::get(loggerName_)->info(headerMsg);
                isStarted_ = true;
                return true;
            }
            else {
                isStarted_ = false;
                return false;
            }
        }
    }

    /**
     * \brief Records the values of the added variables at the instant the function is called.
     *
     */
    bool recordLogData(){
        if(!isStarted_){
            spdlog::error("Can't collect data without starting the logger first");
            return false;
        }
        else{
            std::string valueMsg = "";
            for(unsigned int i=0; i < vectorOfLogElements.size(); i++){ // iterating through each variable to get their values

                valueMsg += vectorOfLogElements[i]->getValue();

                // either a coma or new line comes
                if(i != vectorOfLogElements.size()-1){
                    valueMsg += ", ";
                }
            }
            spdlog::get(loggerName_)->info(valueMsg);
            return true;
        }
    }
    void endLog(){
        spdlog::drop(loggerName_);
    }
};

#endif //SRC_LOGHELPER_H
