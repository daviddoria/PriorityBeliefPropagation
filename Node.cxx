/*
Copyright (C) 2011 David Doria, daviddoria@gmail.com

This program is free software: you can redistribute it and/or modify
it under the terms of the GNU General Public License as published by
the Free Software Foundation, either version 3 of the License, or
(at your option) any later version.

This program is distributed in the hope that it will be useful,
but WITHOUT ANY WARRANTY; without even the implied warranty of
MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
GNU General Public License for more details.

You should have received a copy of the GNU General Public License
along with this program.  If not, see <http://www.gnu.org/licenses/>.
*/

#include "Node.h"

Node::Node()
{
  this->Committed = false;
  this->Priority = 1.0;
}

void Node::SetLabelSet(std::set<int> labelSet)
{
  this->LabelSet = labelSet;
}

std::set<int>& Node::GetLabelSet()
{
  return this->LabelSet;
}

void Node::RemoveLabel(int label)
{
  std::set<int>::iterator it;
  it = this->LabelSet.find(label);
   if(it != this->LabelSet.end())
    {
    this->LabelSet.erase(it);
    }
}

void Node::SetOutgoingMessages(std::vector<MessageVector> messages)
{
  this->OutgoingMessages = messages;
}

unsigned int Node::GetNumberOfNeighbors() const
{
  return this->OutgoingMessages.size();
}

void Node::SetCommitted(const bool committed)
{
  this->Committed = committed;
}

bool Node::IsCommitted() const
{
  return this->Committed;
}

std::vector<MessageVector>& Node::GetOutgoingMessages()
{
  return this->OutgoingMessages;
}

void Node::SetPriority(float priority)
{
  this->Priority = priority;
}

float Node::GetPriority() const
{
  return this->Priority;
}


void Node::SetGridIndex(itk::Index<2> index)
{
  this->GridIndex = index;
}

itk::Index<2> Node::GetGridIndex() const
{
  return this->GridIndex;
}