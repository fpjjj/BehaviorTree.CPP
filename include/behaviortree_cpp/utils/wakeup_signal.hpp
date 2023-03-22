#ifndef BEHAVIORTREECORE_WAKEUP_SIGNAL_HPP
#define BEHAVIORTREECORE_WAKEUP_SIGNAL_HPP

#include <chrono>
#include <mutex>
#include <condition_variable>

//这是一个简单的线程同步工具，名为WakeUpSignal，用于在多线程环境下等待信号的到来

namespace BT
{

class WakeUpSignal
{
public:
    //等待信号的到来，最长等待时间为tm，返回值为true表示信号已到达，返回值为false表示等待超时
    /// Return true if the
    bool waitFor(std::chrono::system_clock::duration tm)
    {
        std::unique_lock<std::mutex> lk(mutex_);
        auto res = cv_.wait_for(lk, tm, [this]{
          return ready_;
        });
        ready_ = false;
        return res;
    }

    //发送一个信号
    void emitSignal()
    {
       {
           std::lock_guard<std::mutex> lk(mutex_);
           ready_ = true;
       }
       cv_.notify_all();
    }

private:

    std::mutex mutex_;  //互斥锁，用于保护条件变量和标志位的访问
    std::condition_variable cv_;    //条件变量，用于等待信号的到来
    bool ready_ = false;    //标志位，用于表示信号是否已到达
};

}

#endif // BEHAVIORTREECORE_WAKEUP_SIGNAL_HPP
