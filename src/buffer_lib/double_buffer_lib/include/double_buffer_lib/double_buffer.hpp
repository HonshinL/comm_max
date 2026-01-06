#pragma once
#include <mutex>
#include <atomic>
#include <memory>
#include <vector>
#include <iostream>

namespace dblbuf {

// 简单示例数据类型，可替换为你的结构
struct Data {
  std::vector<uint8_t> bytes;
};

// 双缓冲：front 供读，back 供写；swap 原子切换指针
class DoubleBuffer {
public:
  DoubleBuffer() : front_(std::make_shared<Data>()), back_(std::make_shared<Data>()) {}

  // 写入 back 缓冲
  void write(const Data &d) {
    std::lock_guard<std::mutex> lock(mtx_);
    *back_ = d;  // 拷贝到 back
    dirty_ = true;
  }

  // 读取 front 缓冲（无锁，读到的是稳定数据的共享指针）
  std::shared_ptr<const Data> read() const {
    return front_;
  }

  // 在安全点交换 front/back；返回是否发生交换
  bool swap() {
    std::lock_guard<std::mutex> lock(mtx_);
    if (!dirty_) return false;

    front_.swap(back_);
    dirty_ = false;

    // 输出提示
    std::cout << "[DoubleBuffer] front/back 已切换，最新数据已生效" << std::endl;

    return true;
  }

private:
  std::shared_ptr<Data> front_;
  std::shared_ptr<Data> back_;
  mutable std::mutex mtx_;
  bool dirty_{false};
};

// 进程内共享实例提供者（容器内各组件拿到同一个实例）
class DoubleBufferProvider {
public:
  static std::shared_ptr<DoubleBuffer> get() {
    static std::shared_ptr<DoubleBuffer> instance = std::make_shared<DoubleBuffer>();
    return instance;
  }
};

} // namespace dblbuf
