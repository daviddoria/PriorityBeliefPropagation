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

#ifndef LoopyBP_H
#define LoopyBP_H

#include "Types.h"
#include "MessageVector.h"
#include "Comparisons.h"

#include <queue>
#include <set>

template <typename TImage>
class LoopyBP
{
public:

  LoopyBP();

  bool Iterate();

  IntImageType::Pointer GetResult();

  void SetOriginalLabelSet(std::set<int>);

  void SetDefaultMessageValue(const float value);

  void Initialize();
  void CreateAndInitializeMessages(const float defaultMessageValue);

  void OutputMessagesBetweenNodes(const itk::Index<2> FromNode, const itk::Index<2> ToNode);

  bool SumProductMessageUpdate(MessageVector& messageVector);

  void OutputBeliefImage();
  void OutputMessageImage();
  void WriteBeliefImage(std::string filename);

  IntImageType::Pointer GetImage();

  void SetBeliefThreshold(float threshold);

protected:

  float DefaultMessageValue;

  std::priority_queue<Node*, std::vector<Node*>, NodePointerComparison> Priorities;
  std::queue<Node*> VisitedOrder;
  //std::priority_queue<itk::Index<2> > Priorities; // Can't use < operator directory for some reason?

  float ComputePriority(Node*);
  float BeliefThreshold;
  unsigned int ComputeSizeOfConfusionSet();
  void PrioritizeNodes();
  Node* GetHighestPriorityNode();

  virtual float UnaryCost(const int label, const itk::Index<2> pixel) = 0;
  virtual float BinaryCost(const int label1, const int label2) = 0;

  float Belief(Node* node, const int label);
  std::vector<float> ComputeAllBeliefs();
  int FindLabelWithLowestBelief(Node* node);
  int FindLabelWithHighestBelief(Node* node);

  IntImageType::Pointer Image;

  NodeImageType::Pointer NodeImage;

  std::set<int> OriginalLabelSet;

  // Get a specific message
  Message& GetMessage(Node* fromNode, Node* toNode, const int label);

  std::vector<Message*> GetMessages(Node* node);

  MessageVector& GetMessages(Node* fromNode, Node* toNode);

  float SumOfMessages(const std::vector<Message*> messages);

  std::vector<Message*> GetIncomingMessagesWithLabel(Node* node, const int label);



};

#include "LoopyBP.txx"

#endif
