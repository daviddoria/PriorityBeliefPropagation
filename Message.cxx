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

#include "Message.h"


Message::Message()
{
  this->Label = 0;
  this->Value = 1.0;
}

std::ostream& operator<<(std::ostream& output, const Message &m)
{
  output << "Message: " << std::endl
         << "Label: " << m.Label << std::endl
         << "Value: " << m.Value << std::endl << std::endl;
  return output;
}


