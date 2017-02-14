/*=========================================================================
 *
 *  Copyright RTK Consortium
 *
 *  Licensed under the Apache License, Version 2.0 (the "License");
 *  you may not use this file except in compliance with the License.
 *  You may obtain a copy of the License at
 *
 *         http://www.apache.org/licenses/LICENSE-2.0.txt
 *
 *  Unless required by applicable law or agreed to in writing, software
 *  distributed under the License is distributed on an "AS IS" BASIS,
 *  WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
 *  See the License for the specific language governing permissions and
 *  limitations under the License.
 *
 *=========================================================================*/

#include "rtkgeometrydistance_ggo.h"
#include "rtkGgoFunctions.h"

#include "rtkThreeDCircularProjectionGeometryXMLFile.h"
#include "rtkGeometryDistanceImageFilter.h"

#include <itkImageFileReader.h>
#include <itkImageFileWriter.h>

int main(int argc, char * argv[])
{
  GGO(rtkgeometrydistance, args_info);

  typedef float OutputPixelType;
  const unsigned int Dimension = 3;

  typedef itk::Image< OutputPixelType, Dimension > OutputImageType;

  // Geometry
  if(args_info.verbose_flag)
    std::cout << "Reading geometry information from "
              << args_info.geometry1_arg
              << " and "
              << args_info.geometry2_arg
              << "..."
              << std::endl;
  rtk::ThreeDCircularProjectionGeometryXMLFileReader::Pointer geometryReader1, geometryReader2;
  geometryReader1 = rtk::ThreeDCircularProjectionGeometryXMLFileReader::New();
  geometryReader1->SetFilename(args_info.geometry1_arg);
  TRY_AND_EXIT_ON_ITK_EXCEPTION( geometryReader1->GenerateOutputInformation() )
  geometryReader2 = rtk::ThreeDCircularProjectionGeometryXMLFileReader::New();
  geometryReader2->SetFilename(args_info.geometry2_arg);
  TRY_AND_EXIT_ON_ITK_EXCEPTION( geometryReader2->GenerateOutputInformation() )

  // Create a stack of empty projection images
  typedef rtk::ConstantImageSource< OutputImageType > ConstantImageSourceType;
  ConstantImageSourceType::Pointer constantImageSource = ConstantImageSourceType::New();
  rtk::SetConstantImageSourceFromGgo<ConstantImageSourceType, args_info_rtkgeometrydistance>(constantImageSource, args_info);

  // Adjust size according to geometry
  ConstantImageSourceType::SizeType sizeOutput;
  sizeOutput[0] = constantImageSource->GetSize()[0];
  sizeOutput[1] = constantImageSource->GetSize()[1];
  sizeOutput[2] = geometryReader1->GetOutputObject()->GetGantryAngles().size();
  constantImageSource->SetSize( sizeOutput );

  // Create projection image filter
  typedef rtk::GeometryDistanceImageFilter<OutputImageType, OutputImageType> GDType;
  GDType::Pointer gd = GDType::New();
  gd->SetInput( constantImageSource->GetOutput() );
  gd->SetGeometry1( geometryReader1->GetOutputObject() );
  gd->SetGeometry2( geometryReader2->GetOutputObject() );
  TRY_AND_EXIT_ON_ITK_EXCEPTION( gd->Update() )

  // Write
  typedef itk::ImageFileWriter<  OutputImageType > WriterType;
  WriterType::Pointer writer = WriterType::New();
  writer->SetFileName( args_info.output_arg );
  writer->SetInput( gd->GetOutput() );
  if(args_info.verbose_flag)
    std::cout << "Computing and writing... " << std::flush;
  TRY_AND_EXIT_ON_ITK_EXCEPTION( writer->Update() )

  return EXIT_SUCCESS;
}
