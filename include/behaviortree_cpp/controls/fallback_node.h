/* Copyright (C) 2015-2018 Michele Colledanchise -  All Rights Reserved
 * Copyright (C) 2018-2020 Davide Faconti, Eurecat -  All Rights Reserved
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

/*
FallbackNode
FallbackNode是一个控制节点，用于尝试不同的策略，直到其中一个成功。
如果任何子节点返回RUNNING，则不会再次触发前面的子节点。
如果所有子节点都返回FAILURE，则此节点返回FAILURE。
如果子节点返回RUNNING，则此节点返回RUNNING。
如果子节点返回SUCCESS，则停止循环并返回SUCCESS。
*/
namespace BT
{
/**
 * @brief The FallbackNode is used to try different strategies,
 * until one succeeds.
 * If any child returns RUNNING, previous children will NOT be ticked again.
 *
 * - If all the children return FAILURE, this node returns FAILURE.
 *
 * - If a child returns RUNNING, this node returns RUNNING.
 *
 * - If a child returns SUCCESS, stop the loop and return SUCCESS.
 *
 */
class FallbackNode : public ControlNode
{
public:
  FallbackNode(const std::string& name);

  virtual ~FallbackNode() override = default;

  virtual void halt() override;

private:
  //当前执行的字节点index
  size_t current_child_idx_;
  //是否所有的节点都是跳过的
  bool all_skipped_;

  virtual BT::NodeStatus tick() override;
};

}   // namespace BT
