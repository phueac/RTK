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

#ifndef rtkGeometryDistanceImageFilter_hxx
#define rtkGeometryDistanceImageFilter_hxx

#include <itkImageRegionConstIterator.h>
#include <itkImageRegionIteratorWithIndex.h>

#include "rtkHomogeneousMatrix.h"
#include "rtkProjectionsRegionConstIteratorRayBased.h"
#include "rtkFieldOfViewImageFilter.h"
#include "rtkRayQuadricIntersectionFunction.h"

namespace rtk
{

template <class TInputImage, class TOutputImage>
GeometryDistanceImageFilter<TInputImage,TOutputImage>
::GeometryDistanceImageFilter():
  m_Geometry1(ITK_NULLPTR),
  m_Geometry2(ITK_NULLPTR)
{
}

template <class TInputImage, class TOutputImage>
void
GeometryDistanceImageFilter<TInputImage,TOutputImage>
::BeforeThreadedGenerateData()
{
  if(this->GetGeometry1()->GetGantryAngles().size() !=
     this->GetGeometry2()->GetGantryAngles().size())
    {
    itkExceptionMacro(<<"Number of projections in the two geometries differ.")
    }

  if(this->GetGeometry1()->GetGantryAngles().size() !=
     this->GetOutput()->GetLargestPossibleRegion().GetSize()[2])
    {
    itkExceptionMacro(<<"Number of projections in the input stack and the geometry objects differ.")
    }

  m_Geometry[0] = m_Geometry1;
  m_Geometry[1] = m_Geometry2;

  // Compute the two FOV objects
  for(int i=0; i<2; i++)
    {
    double x, z, r;
    typedef typename rtk::FieldOfViewImageFilter<OutputImageType, OutputImageType> FOVFilterType;
    typename FOVFilterType::Pointer fieldofview = FOVFilterType::New();
    typename Superclass::InputImagePointer inputPtr  = const_cast< TInputImage * >( this->GetInput() );
    fieldofview->SetProjectionsStack( inputPtr.GetPointer() );
    fieldofview->SetGeometry( m_Geometry[i] );
    bool hasOverlap = fieldofview->ComputeFOVRadius(FOVFilterType::RADIUSBOTH, x, z, r);
    if(!hasOverlap)
      {
      itkGenericExceptionMacro(<< "FOV is empty for geometry " << i+1);
      }
    EQPFunctionType::VectorType semiprincipalaxis(0.);
    EQPFunctionType::VectorType center(0.);
    semiprincipalaxis[0] = r;
    semiprincipalaxis[2] = r;
    center[0] = x;
    center[2] = z;
    m_FOV[i] = EQPFunctionType::New();
    m_FOV[i]->SetFigure("Ellipsoid");
    m_FOV[i]->Translate(semiprincipalaxis);
    m_FOV[i]->Rotate(0, center);
    }
}

template <class TInputImage, class TOutputImage>
void
GeometryDistanceImageFilter<TInputImage,TOutputImage>
::ThreadedGenerateData(const OutputImageRegionType& outputRegionForThread,
                       ThreadIdType threadId )
{
  // Quadric intersection functors
  typedef RayQuadricIntersectionFunction<double, 3> RQIFunctionType;
  RQIFunctionType::Pointer rqi[2];
  for(int i=0; i<2; i++)
    {
    rqi[i] = RQIFunctionType::New();
    rqi[i]->SetA( m_FOV[i]->GetA() );
    rqi[i]->SetB( m_FOV[i]->GetB() );
    rqi[i]->SetC( m_FOV[i]->GetC() );
    rqi[i]->SetD( m_FOV[i]->GetD() );
    rqi[i]->SetE( m_FOV[i]->GetE() );
    rqi[i]->SetF( m_FOV[i]->GetF() );
    rqi[i]->SetG( m_FOV[i]->GetG() );
    rqi[i]->SetH( m_FOV[i]->GetH() );
    rqi[i]->SetI( m_FOV[i]->GetI() );
    rqi[i]->SetJ( m_FOV[i]->GetJ() );
    }

  // Iterators on input and output
  typedef ProjectionsRegionConstIteratorRayBased<TInputImage> InputRegionIterator;
  InputRegionIterator *itIn[2];
  for(int i=0; i<2; i++)
    {
    itIn[i] = InputRegionIterator::New(this->GetInput(),
                                       outputRegionForThread,
                                       m_Geometry[i]);
    }
  typedef itk::ImageRegionIteratorWithIndex<TOutputImage> OutputRegionIterator;
  OutputRegionIterator itOut(this->GetOutput(), outputRegionForThread);

  // Go over each projection
  for(unsigned int pix=0; pix<outputRegionForThread.GetNumberOfPixels(); pix++, itIn[0]->Next(), itIn[1]->Next(), ++itOut)
    {
    // Get source and direction of each ray
    RQIFunctionType::VectorType src[2];
    RQIFunctionType::VectorType dir[2];
    RQIFunctionType::VectorType xnear[2];
    RQIFunctionType::VectorType xfar[2];

    // Compute intersections with FOV
    for(int i=0; i<2; i++)
      {
      src[i] = itIn[i]->GetSourcePosition();
      rqi[i]->SetRayOrigin( src[i] );
      dir[i] = itIn[i]->GetDirection();
      assert( rqi[i]->Evaluate(dir[i]) );
      xnear[i] = src[i]+dir[i]*rqi[i]->GetNearestDistance();
      xfar[i]  = src[i]+dir[i]*rqi[i]->GetFarthestDistance();
      }

    typename TOutputImage::PixelType pixval = -1.;
    for(int i=0; i<2; i++)
      {
      // Project onto other ray
      RQIFunctionType::VectorType xnearproj, xfarproj;
      xnearproj = (xnear[i]-src[(i+1)%2])*dir[(i+1)%2]*dir[(i+1)%2]+src[(i+1)%2];
      xfarproj  = (xfar[i] -src[(i+1)%2])*dir[(i+1)%2]*dir[(i+1)%2]+src[(i+1)%2];
      // Take max of the difference norm
      xnearproj -= xnear[i];
      xfarproj -= xfar[i];
      pixval = std::max(pixval, (typename TOutputImage::PixelType)xnearproj.GetNorm() );
      pixval = std::max(pixval, (typename TOutputImage::PixelType)xfarproj.GetNorm() );
      }
    itOut.Set(pixval);
    }

  delete itIn[0];
  delete itIn[1];
}

} // end namespace rtk

#endif
