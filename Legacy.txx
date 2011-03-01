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

template <typename T>
std::vector<Message*> LoopyBP<T>::GetMessagesWithLabel(itk::Index<2> fromNode, int label)
{
  std::vector<Message*> messagesWithLabel;
  std::vector<Message>& allMessages = this->OutgoingMessageImage->GetPixel(fromNode);
  for(unsigned int i = 0; i < allMessages.size(); i++)
    {
    if(allMessages[i].Label == label)
      {
      messagesWithLabel.push_back(&(allMessages[i]));
      }
    }

  if(messagesWithLabel.size() == 0)
    {
    std::cerr << "No message exists with FromNode = " << fromNode << " Label = " << label << std::endl;
    exit(-1);
    }

  return messagesWithLabel;
}