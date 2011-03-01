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

#include "LoopyBP.h"

#include "Helpers.h"

#include "itkImageRegionIterator.h"
#include "itkIntensityWindowingImageFilter.h"

#include <vector>

template <typename T>
LoopyBP<T>::LoopyBP()
{
  this->Image = NULL;
  this->NodeImage = NULL;

  this->DefaultMessageValue = 1.0;
}

template <typename T>
IntImageType::Pointer LoopyBP<T>::GetImage()
{
  return this->Image;
}


template <typename T>
void LoopyBP<T>::WriteBeliefImage(std::string filename)
{
  FloatImageType::Pointer beliefImage = FloatImageType::New();
  beliefImage->SetRegions(this->NodeImage->GetLargestPossibleRegion());
  beliefImage->Allocate();

  itk::Size<2> imageSize = beliefImage->GetLargestPossibleRegion().GetSize();

  for(unsigned int i = 0; i < imageSize[0]; i++)
    {
    for(unsigned int j = 0; j < imageSize[1]; j++)
      {
      std::vector<float> beliefs(this->LabelSet.size());
      itk::Index<2> index;
      index[0] = i;
      index[1] = j;
      for(unsigned int l = 0; l < this->LabelSet.size(); l++)
        {
        float belief = Belief(index, l);
        beliefs[l] = belief;
        }
      Helpers::NormalizeVector(beliefs);
      beliefImage->SetPixel(index, beliefs[1]); // the probability of "white pixel"
      }// end for j
    }// end for i

  typedef itk::IntensityWindowingImageFilter <FloatImageType, IntImageType> IntensityWindowingImageFilterType;

  IntensityWindowingImageFilterType::Pointer filter = IntensityWindowingImageFilterType::New();
  filter->SetInput(beliefImage);
  filter->SetWindowMinimum(0);
  filter->SetWindowMaximum(1);
  filter->SetOutputMinimum(0);
  filter->SetOutputMaximum(255);
  filter->Update();

  Helpers::WriteScaledImage<IntImageType>(filter->GetOutput(), filename);
}

template <typename T>
void LoopyBP<T>::SetOriginalLabelSet(std::set<int> labelSet)
{
  this->OriginalLabelSet = labelSet;

  this->BeliefThreshold = 1./this->OriginalLabelSet.size(); // !!! What should this be?
}

template <typename T>
void LoopyBP<T>::Initialize()
{
  CreateAndInitializeMessages(this->DefaultMessageValue);
  PrioritizeNodes();
}

template <typename T>
void LoopyBP<T>::SetDefaultMessageValue(const float value)
{
  this->DefaultMessageValue = value;
}

template <typename T>
void LoopyBP<T>::CreateAndInitializeMessages(const float defaultMessageValue)
{
  // For each pixel, create |L| messages to each neighboring pixel
  if(!this->NodeImage)
    {
    this->NodeImage = NodeImageType::New();
    }
  this->NodeImage->SetRegions(this->Image->GetLargestPossibleRegion());
  this->NodeImage->Allocate();

  itk::ImageRegionIterator<NodeImageType> imageIterator(this->NodeImage, this->NodeImage->GetLargestPossibleRegion());

  while(!imageIterator.IsAtEnd())
    {
    std::vector<Node* > neighbors = Helpers::Get4Neighbors<NodeImageType>(this->NodeImage, imageIterator.GetIndex());
    std::vector<MessageVector> messageVectors;

    for(unsigned int i = 0; i < neighbors.size(); i++) // add a message to each neighbor
      {
      MessageVector messageVector;
      messageVector.SourceNode = imageIterator.Get();
      messageVector.DesintationNode = neighbors[i];

      //std::cout << "Created a message from " << messageVector.FromNode << " to " << messageVector.ToNode << std::endl;

      if(messageVector.SourceNode == messageVector.DesintationNode)
        {
        std::cerr << "Cannot create a message from " << messageVector.SourceNode << " to " << messageVector.DesintationNode << std::endl;
        exit(-1);
        }
      for(unsigned int l = 0; l < this->OriginalLabelSet.size(); l++) // add a message for each label
        {
        Message m;
        m.Label = l;
        m.Value = defaultMessageValue;
        messageVector.AddMessage(m);
        }
      messageVectors.push_back(messageVector);
      }
    Node* node = new Node;
    node->SetOutgoingMessages(messageVectors);
    imageIterator.Set(node);

    ++imageIterator;
    }

}

template <typename T>
Node* LoopyBP<T>::GetHighestPriorityNode()
{
  return this->Priorities.top();
  this->Priorities.pop();
}

template <typename T>
unsigned int LoopyBP<T>::ComputeSizeOfConfusionSet()
{
  std::vector<float> beliefs = ComputeAllBeliefs();

  unsigned int confusionSetSize = 0;
  for(unsigned int i = 0; i < beliefs.size(); i++)
    {
    if(beliefs[i] > this->BeliefThreshold)
      {
      confusionSetSize++;
      }
    }

  return confusionSetSize;
}

template <typename T>
void LoopyBP<T>::PrioritizeNodes()
{
  itk::ImageRegionIteratorWithIndex<NodeImageType> imageIterator(this->NodeImage, this->NodeImage->GetLargestPossibleRegion());

  while(!imageIterator.IsAtEnd())
    {
    float priority = ComputePriority(imageIterator.Get());
    imageIterator.Value()->SetPriority(priority);
    this->Priorities.push(imageIterator.Get());
    ++imageIterator;
    }

}

template <typename T>
std::vector<float> LoopyBP<T>::ComputeAllBeliefs()
{
  /*
  std::vector<float> beliefs;

  for(unsigned int l = 0; l < this->LabelSet.size(); l++)
    {
    float belief = Belief(node, l);
    beliefs[l] = belief;
    }

  Helpers::NormalizeVector(beliefs);
  return beliefs;
  */
}

template <typename T>
float LoopyBP<T>::ComputePriority(Node* node)
{
  unsigned int confusionSetSize = ComputeSizeOfConfusionSet();

  return 1.0/static_cast<float>(confusionSetSize);
}

template <typename T>
void LoopyBP<T>::SetBeliefThreshold(float threshold)
{
  this->BeliefThreshold = threshold;
}

template <typename T>
bool LoopyBP<T>::Iterate()
{
  // Determine which message to update (following the schedule)
  Node* node = GetHighestPriorityNode();

  // Don't process nodes that are already committed.
  if(node->IsCommitted())
    {
    return Iterate();
    }

  this->VisitedOrder.push(node);
//  node->PruneLabels();
  node->SetCommitted(true);

  for(unsigned int i = 0; i < node->GetNumberOfNeighbors(); i++)
    {
    MessageVector& messageVector = node->GetOutgoingMessages()[i];

    // only send messages to uncommitted neighbors
    if(messageVector.DesintationNode->IsCommitted())
      {
      continue;
      }

    SumProductMessageUpdate(messageVector);

    }

  return false; // Not yet converged
}

template <typename T>
bool LoopyBP<T>::SumProductMessageUpdate(MessageVector& messageVector)
{
  // This function returns true if nothing changed (converged) (Convergence checking is not yet implemented, so it always returns false at the moment)

  // Message update equation
  // m_{ij}(l) = \sum_{p \in L} \left[ B(L(p), L(l)) U(L(p)) \prod_{k=N(i)\backslash j} m_{ki}(L(p)) \right]

  std::set<int> labelSet = messageVector.SourceNode->GetLabelSet();

  for(unsigned int l = 0; l < messageVector.GetNumberOfMessages(); l++)
    {
    float sum = 0.0;
    for(std::set<int>::iterator p = labelSet.begin(); p != labelSet.end(); p++)
      {
      float unary = this->UnaryCost(*p, messageVector.SourceNode->GetGridIndex());
      float binary = this->BinaryCost(*p, messageVector.GetMessage(l).Label);

      float product = 1.0;
      //std::cout << "Getting messages with label (" << m.FromNode << " , " << p << ")" << std::endl;
      std::vector<Message*> messages = GetIncomingMessagesWithLabel(messageVector.SourceNode, *p);
      for(unsigned int k = 0; k < messages.size(); k++)
        {
        product *= messages[k]->Value;
        }
      sum += exp(-unary) * exp(-binary) * product;
      }
    //std::cout << "Setting message to " << sum << std::endl;
    messageVector.GetMessage(l).Value = sum;
    }

  //std::cout << "Messages between node " << messageVector.FromNode << " and " << messageVector.ToNode << " before normalization: " << std::endl;
  //OutputMessagesBetweenNodes(messageVector.FromNode, messageVector.ToNode);

  messageVector.Normalize();

  //std::cout << "Normalized message value: " << std::endl;
  //OutputMessagesBetweenNodes(messageVector.FromNode, messageVector.ToNode);

  //std::cout << std::endl;

  return false; // Not yet converged
}



template <typename T>
float LoopyBP<T>::SumOfMessages(std::vector<Message*> messages)
{
  float sum = 0.0;
  for(unsigned int i = 0; i < messages.size(); i++)
    {
    sum += messages[i]->Value;
    }
  return sum;
}

template <typename T>
float LoopyBP<T>::Belief(Node* node, const int label)
{
  float unary = UnaryCost(label, node->GetGridIndex());
  //std::cout << "UnaryCost: " << unary << std::endl;

  float product = 1.0;

  std::vector<Message*> incomingMessages = GetIncomingMessagesWithLabel(node, label);

  for(unsigned int i = 0; i < incomingMessages.size(); i++)
    {
    product *= incomingMessages[i]->Value;
    }

  //std::cout << "product: " << product << std::endl;

  float belief = exp(-unary) * product;
  return belief;
}

template <typename T>
IntImageType::Pointer LoopyBP<T>::GetResult()
{
  itk::ImageRegionConstIterator<NodeImageType> nodeIterator(this->NodeImage, this->NodeImage->GetLargestPossibleRegion());
  itk::ImageRegionIterator<IntImageType> outputIterator(this->Image, this->Image->GetLargestPossibleRegion());

  while(!nodeIterator.IsAtEnd())
    {
    int label = FindLabelWithHighestBelief(nodeIterator.Get());
    //std::cout << "Highest belief for " << nodeIterator.Get() << " is " << label << std::endl;
    outputIterator.Set(label);
    ++nodeIterator;
    ++outputIterator;
    }

  return this->Image;
}


template <typename T>
int LoopyBP<T>::FindLabelWithHighestBelief(Node* node)
{
  float bestBelief = 0;
  int bestLabel = 0;

  std::set<int> labelSet = node->GetLabelSet();

  for(std::set<int>::iterator l = labelSet.begin(); l != labelSet.end(); l++)
    {
    float belief = Belief(node, *l);
    //std::cout << "Pixel: " << node << " Label: " << l << " belief: " << belief << std::endl;
    if(belief > bestBelief)
      {
      bestBelief = belief;
      bestLabel = *l;
      }
    }
  //std::cout << "Best label is " << bestLabel << " with belief " << bestBelief << std::endl;

  return bestLabel;
}


template <typename T>
int LoopyBP<T>::FindLabelWithLowestBelief(Node* node)
{
  // Find the label with the lowest belief

  float bestBelief = itk::NumericTraits<float>::max();
  int bestLabel = 0;

  std::set<int> labelSet = node->GetLabelSet();

  for(std::set<int>::iterator l = labelSet.begin(); l != labelSet.end(); l++)
    {
    float belief = Belief(node, *l);
    //std::cout << "Pixel: " << node << " Label: " << l << " belief: " << belief << std::endl;
    if(belief < bestBelief)
      {
      bestBelief = belief;
      bestLabel = *l;
      }
    }
  //std::cout << "Best label is " << bestLabel << " with belief " << bestBelief << std::endl;

  return bestLabel;
}

template <typename T>
Message& LoopyBP<T>::GetMessage(Node* fromNode, Node* toNode, const int label)
{
  std::vector<MessageVector>& messageVectors = fromNode->GetOutgoingMessages();
  for(unsigned int v = 0; v < messageVectors.size(); v++)
    {
    if(messageVectors[v].DesintationNode == toNode)
      {
      for(unsigned int m = 0; m < messageVectors[v].GetNumberOfMessages(); m++)
        {
        if(messageVectors[v].GetMessage(m).Label == label)
          {
          return messageVectors[v].GetMessage(m);
          }
        }
      }
    }
  std::cerr << "No message exists with FromNode = " << fromNode << " ToNode = " << toNode << " Label = " << label << "(" << std::endl;
  exit(-1);
}


template <typename T>
std::vector<Message*> LoopyBP<T>::GetMessages(Node* node)
{
  std::vector<Message>& outgoingMessages = node->GetOutgoingMessages();
  std::vector<Message*> messages;

  for(unsigned int i = 0; i < outgoingMessages.size(); i++)
    {
    messages.push_back(&(outgoingMessages[i]));
    }
  return messages;
}

template <typename T>
MessageVector& LoopyBP<T>::GetMessages(Node* fromNode, Node* toNode)
{
  unsigned int numberOfMessageVectors = fromNode->GetNumberOfNeighbors();

  for(unsigned int i = 0; i < numberOfMessageVectors; i++)
    {
    if(fromNode->GetOutgoingMessages()[i].DesintationNode == toNode)
      {
      return fromNode->GetOutgoingMessages()[i];
      }
    }
  std::cerr << "No MessageVector from " << fromNode << " to " << toNode << " exists!" << std::endl;
  exit(-1);
}


template <typename T>
std::vector<Message*> LoopyBP<T>::GetIncomingMessagesWithLabel(Node* node, const int label)
{
  std::vector<Message*> incomingMessages;
  std::vector<Node*> neighbors = Helpers::Get4Neighbors<NodeImageType>(this->NodeImage, node->GetGridIndex());
  for(unsigned int i = 0; i < neighbors.size(); i++)
    {
    MessageVector& messageVector = GetMessages(neighbors[i], node); // Get incoming messages
    for(unsigned int m = 0; m < messageVector.GetNumberOfMessages(); m++)
      {
      if(messageVector.GetMessage(m).Label == label)
        {
        incomingMessages.push_back(&(messageVector.GetMessage(m)));
        }
      }
    }
  return incomingMessages;
}

template <typename T>
void LoopyBP<T>::OutputMessagesBetweenNodes(const itk::Index<2> fromNode, const itk::Index<2> toNode)
{
  MessageVector& messageVector = GetMessages(fromNode, toNode);
  std::cout << "Messages from " << fromNode << " to " << toNode << std::endl << messageVector << std::endl;
}