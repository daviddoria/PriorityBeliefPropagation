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

#include "itkImage.h"
#include "itkImageFileReader.h"
#include "itkImageFileWriter.h"



#include "Types.h"
#include "MRF.h"
#include "Helpers.h"

int main(int argc, char *argv[])
{
  if(argc < 2)
    {
    std::cerr << "Required: filename" << std::endl;
    return EXIT_FAILURE;
    }

  std::string filename = argv[1];

  typedef itk::ImageFileReader<BoolImageType> ReaderType;

  ReaderType::Pointer reader = ReaderType::New();
  reader->SetFileName(filename);
  reader->Update();

  BoolImageType::Pointer observations = reader->GetOutput();
  observations->DisconnectPipeline();
  Helpers::AddNoise(observations, .1);
  Helpers::WriteImage(observations, "noisy.png");

  MRF mrf;
  mrf.SetObservations(observations);

  for(unsigned int i = 0; i < 10; i++)
    {
    mrf.Iterate();
    }

  Helpers::WriteImage(mrf.GetResult(), "result.png");

  return EXIT_SUCCESS;
}
