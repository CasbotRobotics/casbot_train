#pragma once
#include <filesystem>
#include <chrono>
#include <map>

#include <spdlog/version.h>
#include <spdlog/cfg/env.h>
#include <spdlog/async.h>
#include <spdlog/async_logger.h>
#include <spdlog/sinks/stdout_color_sinks.h>
#include <spdlog/sinks/basic_file_sink.h>
#include <spdlog/sinks/rotating_file_sink.h>
#include <spdlog/spdlog.h>

#include <iostream>
#include <algorithm>

#ifndef MOTION_NODE_NAME
#define MOTION_NODE_NAME "motion"
#endif

namespace motion
{

    namespace __logger
    {
        class LogStartup
        {

        public:
            ~LogStartup()
            {
                std::lock_guard<std::mutex> lock(mLoggerListMutex);
                for (auto itor = mLoggerList.begin(); itor != mLoggerList.end(); itor++)
                {
                    spdlog::drop(*itor);
                }
            }
            LogStartup(const LogStartup &) = delete;
            LogStartup &operator=(const LogStartup &) = delete;
            static LogStartup &instance()
            {
                static LogStartup instance;
                return instance;
            }

            void appendLogger(const std::string &logger)
            {
                std::lock_guard<std::mutex> lock(mLoggerListMutex);
                mLoggerList.push_back(logger);
            }

            void dropLogger(const std::string &logger)
            {
                std::lock_guard<std::mutex> lock(mLoggerListMutex);
                spdlog::drop(logger);
                auto itor = std::find(mLoggerList.begin(), mLoggerList.end(), logger);
                if (itor != mLoggerList.end())
                {
                    mLoggerList.erase(itor);
                }
            }

            LogStartup()
            {
                spdlog::init_thread_pool(8192, 1);
            }

        private:
            std::mutex mLoggerListMutex;
            std::vector<std::string> mLoggerList;
        };
    }

    class Log
    {
        // static_assert(SPDLOG_TO_VERSION == 11500, "spdlog version must be 1.15.0");
    public:
        explicit Log(const std::string &name) : __m_logger(nullptr)
        {
            __logger::LogStartup::instance();

            std::shared_ptr<spdlog::sinks::sink> console_sink;
            bool isConsoleLog = true;
            if (getenv("MOTION_DISABLE_CONSOLE_LOG"))
            {
                auto disableConsoleLog = std::string(getenv("MOTION_DISABLE_CONSOLE_LOG"));
                std::transform(disableConsoleLog.begin(), disableConsoleLog.end(), disableConsoleLog.begin(), ::tolower);
                if (disableConsoleLog != "true")
                {
                    isConsoleLog = true;
                }
                else
                {
                    isConsoleLog = false;
                }
            }
            if (isConsoleLog)
            {
                // 初始化命令窗口的 sink
                console_sink = std::make_shared<spdlog::sinks::stdout_color_sink_mt>();
                console_sink->set_level(spdlog::level::trace);
            }

            // 计算日志目录，如果当前目录为 build 目录则使用 build/logs，否则使用 logs
            std::string logPath = "/workspace/.motion/logs";
            if (getenv("MOTION_LOG_PATH"))
            {
                logPath = std::string(getenv("MOTION_LOG_PATH"));
            }
            else if (getenv("HOME"))
            {
                logPath = std::string(getenv("HOME")) + "/.motion/logs";
            }
            // Format the time as a string
            auto const now = std::chrono::system_clock::now();
            std::time_t currentTimeT = std::chrono::system_clock::to_time_t(now);
            std::tm *currentTimeTm = std::localtime(&currentTimeT);
            char formattedTime[20];
            std::strftime(formattedTime, sizeof(formattedTime), "%Y_%m_%d", currentTimeTm);

            logPath = logPath;
            if (std::filesystem::exists(logPath))
            {
                std::filesystem::create_directories(logPath);
            }
            std::string logFile = name + "-" + std::string(formattedTime) + ".log";

            // 初始化文件 sink
            std::size_t max_size = 1024 * 1024 * 5;
            std::size_t max_files = 1024;
            auto file_sink = std::make_shared<spdlog::sinks::rotating_file_sink_mt>(logPath + "/" + logFile, max_size, max_files);
            file_sink->set_level(spdlog::level::trace);

            // 注册到 logger 里
            std::vector<spdlog::sink_ptr> sinks;
            if (console_sink)
            {
                sinks.push_back(console_sink);
            }
            sinks.push_back(file_sink);

            __m_logger = std::make_shared<spdlog::async_logger>(name, sinks.begin(), sinks.end(), spdlog::thread_pool(), spdlog::async_overflow_policy::block);

            spdlog::register_logger(__m_logger);
            __logger::LogStartup::instance().appendLogger(name);

            __m_logger->set_level(spdlog::level::trace);
        }
        ~Log()
        {
            __logger::LogStartup::instance().dropLogger(__m_logger->name());
        }

        Log &operator=(const Log &) = delete;
        Log(const Log &) = delete;

        void setLevel(spdlog::level::level_enum level)
        {
            __m_logger->set_level(level);
        }

        template <typename T, typename... Args>
        void trace(const T &fmt, const Args &...args)
        {
            __m_logger->trace(fmt, args...);
            __m_logger->flush();
        }
        template <typename T, typename... Args>
        void debug(const T &fmt, const Args &...args)
        {
            __m_logger->debug(fmt, args...);
            __m_logger->flush();
        }
        template <typename T, typename... Args>
        void info(const T &fmt, const Args &...args)
        {
            __m_logger->info(fmt, args...);
            __m_logger->flush();
        }
        template <typename T, typename... Args>
        void warn(const T &fmt, const Args &...args)
        {
            __m_logger->warn(fmt, args...);
            __m_logger->flush();
        }
        template <typename T, typename... Args>
        void error(const T &fmt, const Args &...args)
        {
            __m_logger->error(fmt, args...);
            __m_logger->flush();
        }
        template <typename T, typename... Args>
        void critical(const T &fmt, const Args &...args)
        {
            __m_logger->critical(fmt, args...);
            __m_logger->flush();
        }

        inline std::shared_ptr<spdlog::async_logger> logger()
        {
            return __m_logger;
        }

    private:
        std::shared_ptr<spdlog::async_logger> __m_logger;
    };

    /**
     * @brief 实现带有状态的日志
     *
     */
    template <typename ActionT>
    class ActionLog
    {
    public:
        explicit ActionLog(const ActionT &act)
            : __m_currentActionStatus(act)
        {
        }
        ActionLog(Log &log, const ActionT &act)
            : __m_logger(log.logger()), __m_currentActionStatus(act)
        {
        }
        ActionLog(const std::string &name, Log &log, const ActionT &act)
            : __m_logger(log.logger()), __m_currentActionStatus(act), __m_name(name)
        {
        }
        virtual ~ActionLog()
        {
        }

        /**
         * @brief Set the Logger object
         *
         * @param log
         */
        void setLogger(Log &log)
        {
            __m_logger = log.logger();
        }

        /**
         * @brief Set the Name of Log
         *
         * @param name
         */
        void setName(const std::string &name)
        {
            __m_name = name;
        }

        /**
         * @brief 这是一个基于动作的 log
         *
         * @tparam T
         * @tparam Args
         * @param act
         * @param fmt
         * @param args
         */
        template <typename T, typename... Args>
        void log(const ActionT &act, const T &fmt, const Args &...args)
        {
            if (act == __m_currentActionStatus)
            {
                return;
            }
            auto level = spdlog::level::info;
            switch (act.level())
            {
            case 0:
                level = spdlog::level::info;
                break;
            case 1:
                level = spdlog::level::warn;
                break;
            case 2:
                level = spdlog::level::err;
                break;
            default:
                level = spdlog::level::critical;
                break;
            }

            if (__m_logger)
            {
                __m_logger->log(level, "[{}] {} - {}", __m_name, act, fmt::format(fmt, args...));
                __m_logger->flush();
            }
            __m_currentActionStatus = act;
        }

        void log(const ActionT &act)
        {
            if (act == __m_currentActionStatus)
            {
                return;
            }
            auto level = spdlog::level::info;
            switch (act.level())
            {
            case 0:
                level = spdlog::level::info;
                break;
            case 1:
                level = spdlog::level::warn;
                break;
            case 2:
                level = spdlog::level::err;
                break;
            default:
                level = spdlog::level::critical;
                break;
            }

            if (__m_logger)
            {
                __m_logger->log(level, "[{}] {}", __m_name, act);
                __m_logger->flush();
            }
            __m_currentActionStatus = act;
        }

        template <typename T, typename... Args>
        void log(int level, const ActionT &act, const T &fmt, const Args &...args)
        {
            auto lev = spdlog::level::info;
            switch (level)
            {
            case 0:
                lev = spdlog::level::info;
                break;
            case 1:
                lev = spdlog::level::warn;
                break;
            case 2:
                lev = spdlog::level::err;
                break;
            default:
                lev = spdlog::level::critical;
                break;
            }

            if (__m_logger)
            {
                __m_logger->log(lev, "[{}] {} - {}", __m_name, act, fmt::format(fmt, args...));
                __m_logger->flush();
            }
            __m_currentActionStatus = act;
        }

    private:
        std::shared_ptr<spdlog::async_logger> __m_logger;
        ActionT __m_currentActionStatus;
        std::string __m_name;
    };

    /**
     * @brief 实现单例日志
     *
     */
    class Logger
    {
    public:
        static Logger &instance()
        {
            static Logger instance;
            return instance;
        }
        template <typename T, typename... Args>
        void info(const T &fmt, const Args &...args)
        {
            __m_logger->info("<{}> ->{}", MOTION_NODE_NAME, fmt::format(fmt, args...));
        }
        template <typename T, typename... Args>
        void debug(const T &fmt, const Args &...args)
        {
            __m_logger->debug("<{}> ->{}", MOTION_NODE_NAME, fmt::format(fmt, args...));
        }
        template <typename T, typename... Args>
        void error(const T &fmt, const Args &...args)
        {
            __m_logger->error("<{}> ->{}", MOTION_NODE_NAME, fmt::format(fmt, args...));
        }
        template <typename T, typename... Args>
        void warn(const T &fmt, const Args &...args)
        {
            __m_logger->warn("<{}> ->{}", MOTION_NODE_NAME, fmt::format(fmt, args...));
        }

        void setLevel(spdlog::level::level_enum level)
        {
            __m_logger->setLevel(level);
        }

    private:
        Logger()
        {
            __m_logger = std::make_shared<motion::Log>("motion");
        }
        ~Logger() = default;
        Logger(const Logger &) = delete;
        Logger &operator=(const Logger &) = delete;

    private:
        std::shared_ptr<motion::Log> __m_logger;
    };

} // namespace motion
