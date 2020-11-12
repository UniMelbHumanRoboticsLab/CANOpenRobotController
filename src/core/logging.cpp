/**
 * \file logging.cpp
 * \author Vincent Crocher
 * \brief Main logging function used to log trace, debug and error outputs
 * \version 0.3
 * \date 2020-10-30
 * \copyright Copyright (c) 2020
 *
 */

#include "logging.h"


void init_logging(const char * filename)
{
    //Needed on top of the MACRO definition to be effective
    switch(SPDLOG_ACTIVE_LEVEL)
    {
        case SPDLOG_LEVEL_TRACE:
            spdlog::set_level(spdlog::level::trace);
            break;

        case SPDLOG_LEVEL_DEBUG:
            spdlog::set_level(spdlog::level::debug);
            break;

         case SPDLOG_LEVEL_INFO:
            spdlog::set_level(spdlog::level::info);
            break;

        case SPDLOG_LEVEL_WARN:
            spdlog::set_level(spdlog::level::warn);
            break;

        case SPDLOG_LEVEL_ERROR:
            spdlog::set_level(spdlog::level::err);
            break;

        case SPDLOG_LEVEL_CRITICAL:
            spdlog::set_level(spdlog::level::critical);
            break;

        case SPDLOG_LEVEL_OFF:
            spdlog::set_level(spdlog::level::off);
            break;

        default:
            spdlog::set_level(spdlog::level::off);
    }
    //Create two logging sinks: one file (rotating, max 50M0), one console (with colors)
    std::vector<spdlog::sink_ptr> sinks;
    sinks.push_back(std::make_shared<spdlog::sinks::stdout_color_sink_mt>());
    sinks.push_back(std::make_shared<spdlog::sinks::rotating_file_sink_mt>(filename, 1024 * 1024 * 50, 1));
    //Register this two sinks logger as main logger
    std::shared_ptr<spdlog::logger> main_logger = std::make_shared<spdlog::logger>("CORC", begin(sinks), end(sinks));
    spdlog::initialize_logger(main_logger);
    spdlog::set_default_logger(main_logger);


    spdlog::info("===============================================");
    spdlog::info("============ Start logging session ============");
}
