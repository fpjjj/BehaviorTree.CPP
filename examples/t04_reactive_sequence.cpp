#include "behaviortree_cpp/bt_factory.h"

#include "dummy_nodes.h"
#include "movebase_node.h"

using namespace BT;

/**本教程将教您：
*-序列和反应序列之间的差异
*-如何创建异步ActionNode。
*/
/** This tutorial will teach you:
 *
 *  - The difference between Sequence and ReactiveSequence
 *
 *  - How to create an asynchronous ActionNode.
*/

// clang-format off

static const char* xml_text_sequence = R"(

 <root BTCPP_format="4" >

     <BehaviorTree ID="MainTree">
        <Sequence name="root">
            <BatteryOK/>
            <SaySomething   message="mission started..." />
            <MoveBase       goal="1;2;3"/>
            <SaySomething   message="mission completed!" />
        </Sequence>
     </BehaviorTree>

 </root>
 )";

static const char* xml_text_reactive = R"(

 <root BTCPP_format="4" >

     <BehaviorTree ID="MainTree">
        <ReactiveSequence name="root">
            <BatteryOK/>
            <Sequence>
                <SaySomething   message="mission started..." />
                <MoveBase       goal="1;2;3"/>
                <SaySomething   message="mission completed!" />
            </Sequence>
        </ReactiveSequence>
     </BehaviorTree>

 </root>
 )";

// clang-format on

using namespace DummyNodes;

int main()
{
  BehaviorTreeFactory factory;

  factory.registerSimpleCondition("BatteryOK", std::bind(CheckBattery));
  factory.registerNodeType<MoveBaseAction>("MoveBase");
  factory.registerNodeType<SaySomething>("SaySomething");

  // 使用xml_text_sequence和xml_text-sequence_star比较状态转换和消息
  // Compare the state transitions and messages using either
  // xml_text_sequence and xml_text_sequence_star

  //您应该注意的主要区别是：
  //1）使用序列时，BatteryOK在__each__tick（）执行
  //2）当使用SequenceStar时，这些ConditionNodes只执行__一次__
  // The main difference that you should notice is:
  //  1) When Sequence is used, BatteryOK is executed at __each__ tick()
  //  2) When SequenceStar is used, those ConditionNodes are executed only __once__.

  for (auto& xml_text : {xml_text_sequence, xml_text_reactive})
  {
    std::cout << "\n------------ BUILDING A NEW TREE ------------\n\n";

    auto tree = factory.createTreeFromText(xml_text);

    NodeStatus status = NodeStatus::IDLE;
#if 0
    // Tick the root until we receive either SUCCESS or RUNNING
    // same as: tree.tickRoot(Tree::WHILE_RUNNING)
    std::cout << "--- ticking\n";
    status = tree.tickWhileRunning();
    std::cout << "--- status: " << toStr(status) << "\n\n";
#else
    // If we need to run code between one tick() and the next,
    // we can implement our own while loop
    while (status != NodeStatus::SUCCESS)
    {
      std::cout << "--- ticking\n";
      status = tree.tickOnce();
      std::cout << "--- status: " << toStr(status) << "\n\n";

      // 如果仍在运行，请增加一些等待时间
      // if still running, add some wait time
      if (status == NodeStatus::RUNNING)
      {
        tree.sleep(std::chrono::milliseconds(100));
      }
    }
#endif
  }
  return 0;
}
