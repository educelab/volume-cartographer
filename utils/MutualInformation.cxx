#include "itkImage.h"
#include "itkImageFileReader.h"
#include "itkJoinImageFilter.h"
#include "itkImageToHistogramFilter.h"

int main( int argc, char * argv [] )
{

  if( argc < 3 )
    {
    std::cerr << "Missing command line arguments" << std::endl;
    std::cerr << "Usage :  ImageMutualInformation1  inputImage1 inputImage2 " << std::endl;
    return -1;
    }

  typedef unsigned char                                 PixelComponentType;
  const unsigned int                                    Dimension = 2;

  typedef itk::Image< PixelComponentType, Dimension >   ImageType;

  typedef itk::ImageFileReader< ImageType >             ReaderType;

  ReaderType::Pointer reader1 = ReaderType::New();
  ReaderType::Pointer reader2 = ReaderType::New();

  reader1->SetFileName( argv[1] );
  reader2->SetFileName( argv[2] );

  typedef itk::JoinImageFilter< ImageType, ImageType >  JoinFilterType;

  JoinFilterType::Pointer joinFilter = JoinFilterType::New();

  joinFilter->SetInput1( reader1->GetOutput() );
  joinFilter->SetInput2( reader2->GetOutput() );

  try
    {
    joinFilter->Update();
    }
  catch( itk::ExceptionObject & excp )
    {
    std::cerr << excp << std::endl;
    return -1;
    }

  typedef JoinFilterType::OutputImageType               VectorImageType;

  typedef itk::Statistics::ImageToHistogramFilter<
                                       VectorImageType >  HistogramFilterType;

  HistogramFilterType::Pointer histogramFilter = HistogramFilterType::New();

  histogramFilter->SetInput(  joinFilter->GetOutput()  );

  histogramFilter->SetMarginalScale( 10.0 );

  typedef HistogramFilterType::HistogramSizeType   HistogramSizeType;

  HistogramSizeType size( 2 );

  size[0] = 255;  // number of bins for the first  channel
  size[1] = 255;  // number of bins for the second channel

  histogramFilter->SetHistogramSize( size );

  typedef HistogramFilterType::HistogramMeasurementVectorType
    HistogramMeasurementVectorType;

  HistogramMeasurementVectorType binMinimum( 3 );
  HistogramMeasurementVectorType binMaximum( 3 );

  binMinimum[0] = -0.5;
  binMinimum[1] = -0.5;
  binMinimum[2] = -0.5;

  binMaximum[0] = 255.5;
  binMaximum[1] = 255.5;
  binMaximum[2] = 255.5;

  histogramFilter->SetHistogramBinMinimum( binMinimum );
  histogramFilter->SetHistogramBinMaximum( binMaximum );

  histogramFilter->Update();

  typedef HistogramFilterType::HistogramType  HistogramType;

  const HistogramType * histogram = histogramFilter->GetOutput();

  HistogramType::ConstIterator itr = histogram->Begin();
  HistogramType::ConstIterator end = histogram->End();

  const double Sum = histogram->GetTotalFrequency();

  double JointEntropy = 0.0;

  while( itr != end )
    {
    const double count = itr.GetFrequency();
    if( count > 0.0 )
      {
      const double probability = count / Sum;
      JointEntropy += - probability * vcl_log( probability ) / vcl_log( 2.0 );
      }
    ++itr;
    }

  const unsigned int histogramSize = histogram->Size();

  unsigned int bin;
  for( bin=0; bin < histogramSize; bin++ )
    {
    std::cout << bin << " ";
    std::cout << histogram->GetFrequency( bin, 0 ) << std::endl;
    }


  // std::cout << "Joint Entropy      = " << JointEntropy << " bits " << std::endl;

  // size[0] = 255;  // number of bins for the first  channel
  // size[1] =   1;  // number of bins for the second channel

  // histogramFilter->SetHistogramSize( size );
  // histogramFilter->Update();

  // itr = histogram->Begin();
  // end = histogram->End();

  // double Entropy1 = 0.0;

  // while( itr != end )
  //   {
  //   const double count = itr.GetFrequency();
  //   if( count > 0.0 )
  //     {
  //     const double probability = count / Sum;
  //     Entropy1 += - probability * vcl_log( probability ) / vcl_log( 2.0 );
  //     }
  //   ++itr;
  //   }

  // std::cout << "Image1 Entropy   = " << Entropy1 << " bits " << std::endl;

  // size[0] =   1;  // number of bins for the first channel
  // size[1] = 255;  // number of bins for the second channel

  // histogramFilter->SetHistogramSize( size );
  // histogramFilter->Update();

  // itr = histogram->Begin();
  // end = histogram->End();

  // double Entropy2 = 0.0;

  // while( itr != end )
  //   {
  //   const double count = itr.GetFrequency();
  //   if( count > 0.0 )
  //     {
  //     const double probability = count / Sum;
  //     Entropy2 += - probability * vcl_log( probability ) / vcl_log( 2.0 );
  //     }
  //   ++itr;
  //   }

  // std::cout << "Image2 Entropy   = " << Entropy2 << " bits " << std::endl;

  // double MutualInformation = Entropy1 + Entropy2 - JointEntropy;

  // std::cout << "Mutual Information = " << MutualInformation << " bits " << std::endl;

  // double NormalizedMutualInformation1 =
  //                    2.0 * MutualInformation / ( Entropy1 + Entropy2 );

  // std::cout << "Normalized Mutual Information 1 = " << NormalizedMutualInformation1 <<  std::endl;

  // double NormalizedMutualInformation2 = ( Entropy1 + Entropy2 ) / JointEntropy;

  // std::cout << "Normalized Mutual Information 2 = " << NormalizedMutualInformation2 <<  std::endl;

  return 0;
}
