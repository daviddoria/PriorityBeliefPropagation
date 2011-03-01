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

#ifndef HELPERS_H
#define HELPERS_H

#include "Types.h"

namespace Helpers
{

void AddNoise(IntImageType::Pointer image, float percentToFlip);

template<typename T>
void DeepCopy(typename T::Pointer inputImage, typename T::Pointer outputImage);

template<typename T>
void WriteScaledImage(typename T::Pointer image, std::string filename);

template<typename T>
void WriteCastedImage(typename T::Pointer image, std::string filename);


template<typename T>
void WriteBinaryImage(typename T::Pointer image, std::string filename);

template<typename T>
void ConvertToBinaryImage(typename T::Pointer image, IntImageType::Pointer binary);

template<typename T>
std::vector<itk::Index<2> > Get8Neighbors(typename T::Pointer image, itk::Index<2> pixel);

template<typename T>
std::vector<typename T::PixelType> Get4Neighbors(typename T::Pointer image, itk::Index<2> pixel);

template<typename T>
std::vector<itk::Index<2> > Get4NeighborIndices(typename T::Pointer image, itk::Index<2> pixel);

template<typename T>
void OutputVector(std::vector<T> vec);

void NormalizeVector(std::vector<float> &vec);

}

#include "Helpers.txx"

#endif
