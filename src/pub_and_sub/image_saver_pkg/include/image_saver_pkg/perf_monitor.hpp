#pragma once

#include <fstream>
#include <string>
#include <sstream>
#include <unistd.h>
#include <sys/resource.h>
#include <time.h>

class PerfMonitor {
public:
    PerfMonitor() {
        last_cpu_time_ = get_cpu_time();
        last_wall_time_ = now_seconds();
    }

    // 采样一次性能数据，返回字符串
    std::string sample() {
        double cpu_percent = get_cpu_percent();
        double mem_mb = get_process_memory();
        int threads = get_thread_count();

        std::ostringstream oss;
        oss << "CPU: " << cpu_percent << "% | "
            << "Mem: " << mem_mb << " MB | "
            << "Threads: " << threads;
        return oss.str();
    }

private:
    double last_cpu_time_;
    double last_wall_time_;

    // 获取累计 CPU 时间（秒）
    double get_cpu_time() {
        struct rusage usage;
        getrusage(RUSAGE_SELF, &usage);
        double utime = usage.ru_utime.tv_sec + usage.ru_utime.tv_usec / 1e6;
        double stime = usage.ru_stime.tv_sec + usage.ru_stime.tv_usec / 1e6;
        return utime + stime;
    }

    // 获取当前时间（秒）
    double now_seconds() {
        struct timespec ts;
        clock_gettime(CLOCK_MONOTONIC, &ts);
        return ts.tv_sec + ts.tv_nsec / 1e9;
    }

    // 计算 CPU 使用率（百分比）
    double get_cpu_percent() {
        double current_cpu = get_cpu_time();
        double current_wall = now_seconds();

        double cpu_diff = current_cpu - last_cpu_time_;
        double wall_diff = current_wall - last_wall_time_;

        last_cpu_time_ = current_cpu;
        last_wall_time_ = current_wall;

        return (cpu_diff / wall_diff) * 100.0;
    }

    // 获取进程内存占用（MB）
    double get_process_memory() {
        std::ifstream statm("/proc/self/statm");
        long size, resident;
        statm >> size >> resident;
        long page_size = sysconf(_SC_PAGESIZE);
        return resident * page_size / (1024.0 * 1024.0);
    }

    // 获取线程数
    int get_thread_count() {
        std::ifstream status("/proc/self/status");
        std::string line;
        while (std::getline(status, line)) {
            if (line.find("Threads:") == 0) {
                return std::stoi(line.substr(8));
            }
        }
        return 0;
    }
};
