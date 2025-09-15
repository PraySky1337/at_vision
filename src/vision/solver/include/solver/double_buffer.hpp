// AtomicDoubleBuffer.hpp
#pragma once
#include <atomic>
#include <type_traits>
#include <utility>

template <typename T>
class AtomicDoubleBuffer {
    static_assert(std::is_copy_constructible<T>::value,
                  "T must be copy constructible for safe snapshot read");

public:
    AtomicDoubleBuffer() : front_(0) {
        // 默认构造两个缓冲
        new (&buf_[0]) T();
        new (&buf_[1]) T();
    }

    explicit AtomicDoubleBuffer(const T& init) : front_(0) {
        new (&buf_[0]) T(init);
        new (&buf_[1]) T(init);
    }

    ~AtomicDoubleBuffer() {
        buf_[0].~T();
        buf_[1].~T();
    }

    AtomicDoubleBuffer(const AtomicDoubleBuffer&) = delete;
    AtomicDoubleBuffer& operator=(const AtomicDoubleBuffer&) = delete;

    // 写入：提供一个 lambda/functor 对背面缓冲进行就地修改，然后一次性发布
    template <typename Fn>
    void write(Fn&& fn) {
        // 读到当前正面索引
        int front = front_.load(std::memory_order_acquire);
        int back  = 1 - front;

        // 在背面缓冲上编辑
        fn(buf_[back]);

        // 发布：将背面设为正面
        front_.store(back, std::memory_order_release);
    }

    // 写入：用一份新数据直接覆盖背面，然后发布
    void write_copy(const T& v) {
        int front = front_.load(std::memory_order_acquire);
        int back  = 1 - front;
        buf_[back] = v;
        front_.store(back, std::memory_order_release);
    }

    void write_move(T&& v) {
        int front = front_.load(std::memory_order_acquire);
        int back  = 1 - front;
        buf_[back] = std::move(v);
        front_.store(back, std::memory_order_release);
    }

    // 读取快照：拷贝当前正面缓冲，保证一致性
    T read() const {
        int idx = front_.load(std::memory_order_acquire);
        return buf_[idx]; // 拷贝一份快照
    }

    // 将快照拷贝到外部对象，减少临时对象
    void read_into(T& out) const {
        int idx = front_.load(std::memory_order_acquire);
        out = buf_[idx];
    }

private:
    alignas(64) T buf_[2];              // 避免与原子共享 cache line（简单对齐）
    alignas(64) std::atomic<int> front_; // 0 或 1，表示当前“正面”缓冲
};
