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

#ifndef rtkGeometryDistanceImageFilter_h
#define rtkGeometryDistanceImageFilter_h

#include <itkInPlaceImageFilter.h>
#include "rtkConfiguration.h"
#include "rtkThreeDCircularProjectionGeometry.h"
#include "rtkConvertEllipsoidToQuadricParametersFunction.h"
#include "rtkRayQuadricIntersectionFunction.h"

namespace rtk
{

/** \class GeometryDistanceImageFilter
 * \brief Computes the distance between rays for two geometries 
 *
 * \author Simon Rit
 *
 * \ingroup InPlaceImageFilter
 */
template <class TInputImage, class TOutputImage>
class ITK_EXPORT GeometryDistanceImageFilter :
  public itk::InPlaceImageFilter<TInputImage,TOutputImage>
{
public:
  /** Standard class typedefs. */
  typedef GeometryDistanceImageFilter                       Self;
  typedef itk::InPlaceImageFilter<TInputImage,TOutputImage> Superclass;
  typedef itk::SmartPointer<Self>                           Pointer;
  typedef itk::SmartPointer<const Self>                     ConstPointer;

  typedef TOutputImage                                    OutputImageType;
  typedef typename TOutputImage::RegionType               OutputImageRegionType;
  typedef typename TOutputImage::Superclass::ConstPointer OutputImageBaseConstPointer;
  typedef rtk::ThreeDCircularProjectionGeometry           GeometryType;
  typedef typename GeometryType::Pointer                  GeometryPointer;
  typedef ConvertEllipsoidToQuadricParametersFunction     EQPFunctionType;
  typedef RayQuadricIntersectionFunction<double, 3>       RQIFunctionType;

  /** Method for creation through the object factory. */
  itkNewMacro(Self);

  /** Run-time type information (and related methods). */
  itkTypeMacro(GeometryDistanceImageFilter, itk::ImageToImageFilter);

  /** Get / Set the object pointer to projection geometry */
  itkGetMacro(Geometry1, GeometryPointer);
  itkSetMacro(Geometry1, GeometryPointer);

  /** Get / Set the object pointer to projection geometry */
  itkGetMacro(Geometry2, GeometryPointer);
  itkSetMacro(Geometry2, GeometryPointer);

protected:
  GeometryDistanceImageFilter();
  ~GeometryDistanceImageFilter() ITK_OVERRIDE {};

  void BeforeThreadedGenerateData() ITK_OVERRIDE;

  /** Apply changes to the input image requested region. */
  void ThreadedGenerateData( const OutputImageRegionType& outputRegionForThread,
                                     ThreadIdType threadId ) ITK_OVERRIDE;

  /** The two inputs should not be in the same space so there is nothing
   * to verify. */
  void VerifyInputInformation() ITK_OVERRIDE {}

private:
  GeometryDistanceImageFilter(const Self&); //purposely not implemented
  void operator=(const Self&);            //purposely not implemented

  /** RTK geometry objects */
  GeometryPointer m_Geometry1;
  GeometryPointer m_Geometry2;

  /** Copy of m_Geometry1 and m_Geometry2 in a table */
  GeometryPointer m_Geometry[2];

  /** FOV parameters for each geometry */
  EQPFunctionType::Pointer m_FOV[2];
};

} // end namespace rtk

#ifndef ITK_MANUAL_INSTANTIATION
#include "rtkGeometryDistanceImageFilter.hxx"
#endif

#endif
