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

#ifndef MESSAGEVECTORCOMPARISON_H
#define MESSAGEVECTORCOMPARISON_H

#include "itkIndex.h"

#include "MessageVector.h"
#include "Node.h"

bool operator<(const itk::Index<2> &index1, const itk::Index<2> &index2);

struct MessageVectorComparison
{
  bool operator()(const MessageVector& mv1, const MessageVector& mv2) const;
};

struct IndexComparison
{
  bool operator()(const itk::Index<2>& index1, const itk::Index<2>& index2) const;
};

struct NodeComparison
{
  bool operator()(const Node& node1, const Node& node2) const;
};

struct NodePointerComparison
{
  bool operator()(const Node* node1, const Node* node2) const;
};

#endif