#pragma once
#include <vector>
#include <mutex>
#include <optional>

template<typename T>
class RingBuffer {
public:
    static RingBuffer& instance(size_t capacity = 1024) {
        static RingBuffer rb(capacity);
        return rb;
    }

    void push(const T& value) {
        std::lock_guard<std::mutex> lock(mutex_);
        buffer_[write_index_] = value;
        write_index_ = (write_index_ + 1) % capacity_;
        if (size_ < capacity_) size_++;
        else read_index_ = (read_index_ + 1) % capacity_;
    }

    std::optional<T> pop() {
        std::lock_guard<std::mutex> lock(mutex_);
        if (size_ == 0) return std::nullopt;
        T val = buffer_[read_index_];
        read_index_ = (read_index_ + 1) % capacity_;
        size_--;
        return val;
    }

    void resize(size_t new_capacity) {
        std::lock_guard<std::mutex> lock(mutex_);
        buffer_.assign(new_capacity, T{});
        capacity_ = new_capacity;
        write_index_ = 0;
        read_index_ = 0;
        size_ = 0;
    }

    size_t capacity() const { return capacity_; }
    size_t size() const { return size_; }

private:
    RingBuffer(size_t capacity) : capacity_(capacity), buffer_(capacity) {}

    size_t capacity_;
    std::vector<T> buffer_;
    size_t write_index_ = 0;
    size_t read_index_ = 0;
    size_t size_ = 0;
    std::mutex mutex_;
};
