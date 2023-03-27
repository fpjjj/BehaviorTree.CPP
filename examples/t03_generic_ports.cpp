#include "behaviortree_cpp/bt_factory.h"

using namespace BT;

// 本教程将教您如何在端口类型不是std:：string时处理端口
/* This tutorial will teach you how to deal with ports when its
 *  type is not std::string.
*/

// 我们希望能够使用此自定义类型
// We want to be able to use this custom type
struct Position2D
{
  double x, y;
};

//建议（或在某些情况下，强制）定义convertFromString的模板专用化，将字符串转换为Position2D
// It is recommended (or, in some cases, mandatory) to define a template
// specialization of convertFromString that converts a string to Position2D.
namespace BT
{
template <>
inline Position2D convertFromString(StringView str)
{
  printf("Converting string: \"%s\"\n", str.data());

  // real numbers separated by semicolons
  auto parts = splitString(str, ';');
  if (parts.size() != 2)
  {
    throw RuntimeError("invalid input)");
  }
  else
  {
    Position2D output;
    output.x = convertFromString<double>(parts[0]);
    output.y = convertFromString<double>(parts[1]);
    return output;
  }
}
}   // end namespace BT

class CalculateGoal : public SyncActionNode
{
public:
  CalculateGoal(const std::string& name, const NodeConfig& config) :
    SyncActionNode(name, config)
  {}

  NodeStatus tick() override
  {
    Position2D mygoal = {1.1, 2.3};
    setOutput("goal", mygoal);
    return NodeStatus::SUCCESS;
  }
  static PortsList providedPorts()
  {
    return {OutputPort<Position2D>("goal")};
  }
};

class PrintTarget : public SyncActionNode
{
public:
  PrintTarget(const std::string& name, const NodeConfig& config) :
    SyncActionNode(name, config)
  {}

  NodeStatus tick() override
  {
    auto res = getInput<Position2D>("target");
    if (!res)
    {
      throw RuntimeError("error reading port [target]:", res.error());
    }
    Position2D goal = res.value();
    printf("Target positions: [ %.1f, %.1f ]\n", goal.x, goal.y);
    return NodeStatus::SUCCESS;
  }

  static PortsList providedPorts()
  {
    // Optionally, a port can have a human readable description
    const char* description = "Simply print the target on console...";
    return {InputPort<Position2D>("target", description)};
  }
};

//----------------------------------------------------------------

/**该树是一个由4个动作组成的序列
*1）使用CalculateGoal操作将Position2D的值存储在条目“目标位置”中。
*2）调用PrintTarget。输入“target”将从黑板条目“GoalPosition”中读取。
*3）使用内置的动作脚本来编写键“OtherGoal”。将在发动机罩下进行从字符串到位置2D的转换。
*4）调用PrintTarget。输入的“目标”将从黑板条目“OtherGoal”中读取。
*/
/** The tree is a Sequence of 4 actions

*  1) Store a value of Position2D in the entry "GoalPosition"
*     using the action CalculateGoal.
*
*  2) Call PrintTarget. The input "target" will be read from the Blackboard
*     entry "GoalPosition".
*
*  3) Use the built-in action Script to write the key "OtherGoal".
*     A conversion from string to Position2D will be done under the hood.
*
*  4) Call PrintTarget. The input "goal" will be read from the Blackboard
*     entry "OtherGoal".
*/

// clang-format off
static const char* xml_text = R"(

 <root BTCPP_format="4" >
     <BehaviorTree ID="MainTree">
        <Sequence name="root">
            <CalculateGoal   goal="{GoalPosition}" />
            <PrintTarget     target="{GoalPosition}" />
            <Script          code="OtherGoal='-1;3'" />
            <PrintTarget     target="{OtherGoal}" />
        </Sequence>
     </BehaviorTree>
 </root>
 )";

// clang-format on

int main()
{
  using namespace BT;

  BehaviorTreeFactory factory;
  factory.registerNodeType<CalculateGoal>("CalculateGoal");
  factory.registerNodeType<PrintTarget>("PrintTarget");

  auto tree = factory.createTreeFromText(xml_text);

  tree.tickWhileRunning();

  /* Expected output:
 *
    Target positions: [ 1.1, 2.3 ]
    Converting string: "-1;3"
    Target positions: [ -1.0, 3.0 ]
*/
  return 0;
}
