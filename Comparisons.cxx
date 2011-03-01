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

#include "Comparisons.h"

bool operator<(const itk::Index<2> &index1, const itk::Index<2> &index2)
{
  if((index1[0] < index2[0]))
  {
    return true;
  }

  if(index1[0] == index2[0])
    {
    if(index1[1] < index2[1])
      {
      return true;
      }
    }

  return false;
}

bool IndexComparison::operator()(const itk::Index<2>& index1, const itk::Index<2>& index2) const
{
  return index1 < index2;
}

bool MessageVectorComparison::operator()(const MessageVector& mv1, const MessageVector& mv2) const
{
  if((mv1.SourceNode < mv2.SourceNode))
  {
    return true;
  }

  if(mv1.SourceNode == mv2.SourceNode)
    {
    if(mv1.DesintationNode < mv2.DesintationNode)
      {
      return true;
      }
    }

  return false;
}

bool NodeComparison::operator()(const Node& node1, const Node& node2) const
{
  if(node1.GetPriority() < node2.GetPriority())
  {
    return true;
  }

  return false;
}


bool NodePointerComparison::operator()(const Node* node1, const Node* node2) const
{
  if(node1->GetPriority() < node2->GetPriority())
  {
    return true;
  }

  return false;
}
