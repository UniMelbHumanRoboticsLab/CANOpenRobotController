/**
 * \file logging.h
 * \author Vincent Crocher
 * \brief Main logging function used to log trace, debug and error outputs
 * \version 0.3
 * \date 2020-10-30
 * \copyright Copyright (c) 2020
 *
 */
#ifndef LOGGING_H_INCLUDED
#define LOGGING_H_INCLUDED

//Desired logging level can be defined within CMakeLists.txt

//For file logging (and console output) use either the following functions:
//spdlog::trace("Test trace {}", my_value);
//spdlog::debug("Test debug {}", my_value);
//...
//spdlog::critical("Test debug {}", my_value);

//Or the following macros
//SPDLOG_LOGGER_TRACE(main_logger, "Test trace");
//SPDLOG_LOGGER_DEBUG(main_logger, "Test debug");
//SPDLOG_LOGGER_INFO(main_logger, "Test info");
//SPDLOG_LOGGER_WARN(main_logger, "Test warning");
//SPDLOG_LOGGER_ERROR(main_logger, "Test error");
//SPDLOG_LOGGER_CRITICAL(main_logger, "Test critical");

#include "spdlog/spdlog.h"
#include "spdlog/sinks/rotating_file_sink.h" // support for rotating file logging
#include "spdlog/sinks/stdout_color_sinks.h"

void init_logging(const char * filename = "logs/CORC.log");

//Development use to log loop timing
//#define TIMING_LOG

#endif
