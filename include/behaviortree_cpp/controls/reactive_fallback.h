/* Copyright (C) 2020-2022 Davide Faconti, Eurecat -  All Rights Reserved
*
*   Permission is hereby granted, free of charge, to any person obtaining a copy of this software and associated documentation files (the "Software"),
*   to deal in the Software without restriction, including without limitation the rights to use, copy, modify, merge, publish, distribute, sublicense,
*   and/or sell copies of the Software, and to permit persons to whom the Software is furnished to do so, subject to the following conditions:
*   The above copyright notice and this permission notice shall be included in all copies or substantial portions of the Software.
*
*   THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY,
*   FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT. IN NO EVENT SHALL THE AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER LIABILITY,
*   WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING FROM, OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN THE SOFTWARE.
*/

#pragma once

#include "behaviortree_cpp/control_node.h"

namespace BT
{
/**
  *@brief ReactiveFallback类似于ParallelNode。
  *所有的孩子都从第一个到最后打勾：
  *-如果一个孩子返回RUNNING，则继续到下一个兄弟姐妹。
  *-如果一个孩子返回FAILURE，则继续下一个兄弟姐妹。
  *-如果子级返回SUCCESS，则停止并返回SUCCESS。
  *如果所有子节点都失败，则此节点返回FAILURE。
  *重要提示：为了正常工作，这个节点不应该有超过一个异步子节点。
*/

/**
 * @brief The ReactiveFallback is similar to a ParallelNode.
 * All the children are ticked from first to last:
 *
 * - If a child returns RUNNING, continue to the next sibling.
 * - If a child returns FAILURE, continue to the next sibling.
 * - If a child returns SUCCESS, stop and return SUCCESS.
 *
 * If all the children fail, than this node returns FAILURE.
 *
 * IMPORTANT: to work properly, this node should not have more than
 *            a single asynchronous child.
 *
 */
class ReactiveFallback : public ControlNode
{
public:
  ReactiveFallback(const std::string& name) : ControlNode(name, {})
  {}

private:
  virtual BT::NodeStatus tick() override;
};

}   // namespace BT
