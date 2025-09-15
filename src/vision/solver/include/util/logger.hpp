#pragma once
#include <atomic>
#include <chrono>
#include <condition_variable>
#include <cstdio>
#include <cstring>
#include <ctime>
#include <filesystem>
#include <format>
#include <fstream>
#include <iostream>
#include <mutex>
#include <optional>
#include <queue>
#include <string_view>
#include <thread>
#include <vector>

#ifdef _WIN32
#include <io.h>
#define ISATTY _isatty
#define FILENO _fileno
#else
#include <unistd.h>
#define ISATTY isatty
#define FILENO fileno
#endif

namespace core::logger
{

enum class Level : uint8_t { trace, debug, info, warn, error, fatal };

inline constexpr std::string_view level_name(Level lv) {
    switch (lv) {
        case Level::trace:
            return "TRACE";
        case Level::debug:
            return "DEBUG";
        case Level::info:
            return "INFO";
        case Level::warn:
            return "WARN";
        case Level::error:
            return "ERROR";
        case Level::fatal:
            return "FATAL";
    }
    return "UNK";
}

inline constexpr std::string_view level_color(Level lv) {
    switch (lv) {
        case Level::trace:
            return "\033[37m";
        case Level::debug:
            return "\033[36m";
        case Level::info:
            return "\033[32m";
        case Level::warn:
            return "\033[33m";
        case Level::error:
            return "\033[31m";
        case Level::fatal:
            return "\033[41m";
    }
    return "\033[0m";
}

// 日志消息体
struct LogMsg {
    std::chrono::system_clock::time_point tp;
    Level lv;
    std::string file, func, msg;
    int line;
};

class Logger final {
public:
    static Logger& instance() {
        static Logger inst;
        return inst;
    }

    // 配置
    void set_level(Level lv) { min_level_.store(lv, std::memory_order_relaxed); }
    void enable_color(bool yes) { use_color_.store(yes && is_tty_, std::memory_order_relaxed); }
    void enable_console(bool yes) { console_on_.store(yes, std::memory_order_relaxed); }
    void set_file(
        const std::filesystem::path& p, std::size_t max_bytes = 10 * 1024 * 1024, int backups = 3) {
        std::lock_guard lk(file_mtx_);
        log_path_ = p;
        max_size_ = max_bytes;
        backup_count_ = backups;
        reopen();
    }

    // 日志入队
    template <class... Args>
    void log(
        Level lv, std::string_view file, int line, std::string_view func,
        std::format_string<Args...> fmt, Args&&... args) {
        if (lv < min_level_.load(std::memory_order_relaxed)) return;
        LogMsg m;
        m.tp = std::chrono::system_clock::now();
        m.lv = lv;
        m.file = file;
        m.line = line;
        m.func = func;
        m.msg = std::format(fmt, std::forward<Args>(args)...);

        {
            std::lock_guard lk(q_mtx_);
            queue_.emplace(std::move(m));
        }
        q_cv_.notify_one();
    }

private:
    Logger()
        : is_tty_(ISATTY(FILENO(stdout)) != 0)
        , use_color_(is_tty_)
        , console_on_(true)
        , min_level_(Level::debug)
        , max_size_(0)
        , cur_size_(0)
        , backup_count_(3)
        , stop_flag_(false) {
        log_thread_ = std::thread([this] { this->thread_run(); });
    }
    ~Logger() {
        stop_flag_.store(true, std::memory_order_relaxed);
        q_cv_.notify_one();
        if (log_thread_.joinable()) log_thread_.join();
        if (ofs_) ofs_->flush();
    }

    // 日志线程主循环
    void thread_run() {
        std::vector<LogMsg> batch;
        while (!stop_flag_.load(std::memory_order_relaxed)) {
            {
                std::unique_lock lk(q_mtx_);
                q_cv_.wait_for(lk, std::chrono::milliseconds(200), [this] {
                    return !queue_.empty() || stop_flag_;
                });
                while (!queue_.empty()) {
                    batch.push_back(std::move(queue_.front()));
                    queue_.pop();
                }
            }
            if (batch.empty()) continue;
            std::lock_guard lk(file_mtx_);
            for (auto& m : batch) do_output(m);
            batch.clear();
            if (ofs_) ofs_->flush();
        }
        // 清空剩余
        std::lock_guard lk(file_mtx_);
        while (true) {
            LogMsg m;
            {
                std::lock_guard lk2(q_mtx_);
                if (queue_.empty()) break;
                m = std::move(queue_.front());
                queue_.pop();
            }
            do_output(m);
        }
        if (ofs_) ofs_->flush();
    }

    void do_output(const LogMsg& m) {
        std::string display_file = std::filesystem::path(m.file).filename().string();

        auto now = m.tp;
        auto zt = std::chrono::zoned_time{std::chrono::current_zone(), now};

        // 截断到秒
        auto local_sec = std::chrono::floor<std::chrono::seconds>(zt.get_local_time());
        auto zt_sec = std::chrono::zoned_time{zt.get_time_zone(), local_sec};

        std::string line_buf = std::format(
            "{:%Y-%m-%d %H:%M:%S} [{}] {}:{} {} | {}\n", zt_sec, level_name(m.lv), display_file,
            m.line, m.func, m.msg);

        if (console_on_.load(std::memory_order_relaxed)) {
            if (use_color_.load(std::memory_order_relaxed))
                std::fwrite(level_color(m.lv).data(), 1, level_color(m.lv).size(), stdout);
            std::fwrite(line_buf.data(), 1, line_buf.size(), stdout);
            if (use_color_.load(std::memory_order_relaxed)) std::fwrite("\033[0m", 1, 4, stdout);
            std::fflush(stdout);
        }
        if (ofs_) {
            ofs_->write(line_buf.data(), static_cast<std::streamsize>(line_buf.size()));
            cur_size_ += line_buf.size();
            if (cur_size_ >= max_size_) rotate();
        }
    }

    // 文件轮转
    void reopen() {
        ofs_.reset();
        if (log_path_.empty()) return;
        ofs_.emplace(log_path_, std::ios::app | std::ios::binary);
        cur_size_ = std::filesystem::exists(log_path_) ? std::filesystem::file_size(log_path_) : 0;
    }
    void rotate() {
        ofs_.reset();
        for (int i = backup_count_ - 1; i >= 0; --i) {
            std::filesystem::path src =
                (i == 0) ? log_path_
                         : std::filesystem::path{log_path_.string() + "." + std::to_string(i)};
            if (std::filesystem::exists(src)) {
                std::filesystem::path dst =
                    std::filesystem::path{log_path_.string() + "." + std::to_string(i + 1)};
                std::filesystem::rename(src, dst);
            }
        }
        reopen();
    }

    // --- 数据成员 ---
    // 配置
    bool is_tty_;
    std::atomic_bool use_color_;
    std::atomic_bool console_on_;
    std::atomic<Level> min_level_;
    std::filesystem::path log_path_;
    std::optional<std::ofstream> ofs_;
    std::size_t max_size_;
    std::size_t cur_size_;
    int backup_count_;
    std::mutex file_mtx_;

    // 队列
    std::queue<LogMsg> queue_;
    std::mutex q_mtx_;
    std::condition_variable q_cv_;

    std::thread log_thread_;
    std::atomic_bool stop_flag_;
};

template <typename... Args>
inline void trace(
    const char* file, int line, const char* func, std::format_string<Args...> fmt, Args&&... args) {
    Logger::instance().log(Level::trace, file, line, func, fmt, std::forward<Args>(args)...);
}
template <typename... Args>
inline void debug(
    const char* file, int line, const char* func, std::format_string<Args...> fmt, Args&&... args) {
    Logger::instance().log(Level::debug, file, line, func, fmt, std::forward<Args>(args)...);
}
template <typename... Args>
inline void info(
    const char* file, int line, const char* func, std::format_string<Args...> fmt, Args&&... args) {
    Logger::instance().log(Level::info, file, line, func, fmt, std::forward<Args>(args)...);
}
template <typename... Args>
inline void warn(
    const char* file, int line, const char* func, std::format_string<Args...> fmt, Args&&... args) {
    Logger::instance().log(Level::warn, file, line, func, fmt, std::forward<Args>(args)...);
}
template <typename... Args>
inline void error(
    const char* file, int line, const char* func, std::format_string<Args...> fmt, Args&&... args) {
    Logger::instance().log(Level::error, file, line, func, fmt, std::forward<Args>(args)...);
}
template <typename... Args>
inline void fatal(
    const char* file, int line, const char* func, std::format_string<Args...> fmt, Args&&... args) {
    Logger::instance().log(Level::fatal, file, line, func, fmt, std::forward<Args>(args)...);
}

}  // namespace atcs::logger
using ATLogger = core::logger::Logger;

// 宏封装
#define ATLOG_TRACE(...) ::core::logger::trace(__FILE__, __LINE__, __PRETTY_FUNCTION__, __VA_ARGS__)
#define ATLOG_DEBUG(...) ::core::logger::debug(__FILE__, __LINE__, __PRETTY_FUNCTION__, __VA_ARGS__)
#define ATLOG_INFO(...) ::core::logger::info(__FILE__, __LINE__, __func__, __VA_ARGS__)
#define ATLOG_WARN(...) ::core::logger::warn(__FILE__, __LINE__, __func__, __VA_ARGS__)
#define ATLOG_ERROR(...) ::core::logger::error(__FILE__, __LINE__, __PRETTY_FUNCTION__, __VA_ARGS__)
#define ATLOG_FATAL(...) ::core::logger::fatal(__FILE__, __LINE__, __PRETTY_FUNCTION__, __VA_ARGS__)