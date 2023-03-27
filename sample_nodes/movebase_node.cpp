#include "movebase_node.h"
#include "behaviortree_cpp/bt_factory.h"

// 必须在.cpp文件中实现此函数，才能创建可在运行时加载的插件
// This function must be implemented in the .cpp file to create
// a plugin that can be loaded at run-time
BT_REGISTER_NODES(factory)
{
    factory.registerNodeType<MoveBaseAction>("MoveBase");
}

BT::NodeStatus MoveBaseAction::onStart()
{
  if ( !getInput<Pose2D>("goal", _goal))
  {
    throw BT::RuntimeError("missing required input [goal]");
  }
  printf("[ MoveBase: SEND REQUEST ]. goal: x=%.1f y=%.1f theta=%.1f\n",
         _goal.x, _goal.y, _goal.theta);

  // 我们使用此计数器来模拟需要一定时间才能完成的操作（220毫秒）
  // We use this counter to simulate an action that takes a certain
  // amount of time to be completed (220 ms)
  _completion_time = chr::system_clock::now() + chr::milliseconds(220);

  return BT::NodeStatus::RUNNING;
}

BT::NodeStatus MoveBaseAction::onRunning()
{
  // 假设我们正在检查是否已收到回复，您不想在该功能内阻止太多时间
  // Pretend that we are checking if the reply has been received
  // you don't want to block inside this function too much time.
  std::this_thread::sleep_for(chr::milliseconds(10));

  // 假设经过一定时间后，我们已经完成了操作
  // Pretend that, after a certain amount of time,
  // we have completed the operation
  if(chr::system_clock::now() >= _completion_time)
  {
    std::cout << "[ MoveBase: FINISHED ]" << std::endl;
    return BT::NodeStatus::SUCCESS;
  }
  return BT::NodeStatus::RUNNING;
}

void MoveBaseAction::onHalted()
{
  printf("[ MoveBase: ABORTED ]");
}
