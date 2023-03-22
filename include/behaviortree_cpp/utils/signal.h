#ifndef SIMPLE_SIGNAL_H
#define SIMPLE_SIGNAL_H

#include <memory>
#include <functional>
#include <vector>

/*
此代码实现了一个简单的信号/槽模式，也叫做“观察者模式”。
一个信号可以有多个槽，当信号被触发时，所有槽都会被调用。
槽是一个函数指针，它可以是任何可调用对象，如函数、Lambda表达式、成员函数指针等。
Subscriber是一个智能指针，用于管理槽的生命周期。
当Subscriber失效时，它将自动从信号中删除。
*/

namespace BT
{
/**
 * Super simple Signal/Slop implementation, AKA "Observable pattern".
 * The subscriber is active until it goes out of scope or Subscriber::reset() is called.
 */
template <typename... CallableArgs>
class Signal
{
public:
  using CallableFunction = std::function<void(CallableArgs...)>;
  using Subscriber = std::shared_ptr<CallableFunction>;

  //notify函数用于通知所有订阅者。它使用一个循环遍历所有订阅者。如果订阅者仍然有效，则调用它的槽函数，否则将其从订阅者列表中删除
  void notify(CallableArgs... args)
  {
    for (size_t i = 0; i < subscribers_.size();)
    {
      if (auto sub = subscribers_[i].lock())
      {
        (*sub)(args...);
        i++;
      }
      else
      {
        subscribers_.erase(subscribers_.begin() + i);
      }
    }
  }

  //subscribe函数用于订阅一个新的槽
  //它接受一个可调用对象作为参数，并返回一个Subscriber
  //Subscriber是一个智能指针，它指向这个槽，并负责管理它的生命周期
  //订阅者将被添加到subscribers_中
  Subscriber subscribe(CallableFunction func)
  {
    Subscriber sub = std::make_shared<CallableFunction>(std::move(func));
    subscribers_.emplace_back(sub);
    return sub;
  }

private:
  //subscribers_是一个保存Subscriber的vector
  //每个Subscriber都是一个指向槽的智能指针，使用weak_ptr来防止循环引用，从而避免内存泄漏
  std::vector<std::weak_ptr<CallableFunction>> subscribers_;
};
}   // namespace BT

#endif   // SIMPLE_SIGNAL_H
