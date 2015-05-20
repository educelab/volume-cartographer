#include "itkScalarImageToHistogramGenerator.h"
#include "itkImage.h"

#include "itkImageFileReader.h"

int main( int argc, char * argv [] )
{

  if( argc < 3 )
    {
    std::cerr << "Missing command line arguments" << std::endl;
    std::cerr << "Usage :  Histogram  inputImageFileName  numberOfHistogramBins" << std::endl;
    return -1;
    }

  typedef unsigned char       PixelType;
  const unsigned int          Dimension = 2;

  typedef itk::Image<PixelType, Dimension > ImageType;

  typedef itk::ImageFileReader< ImageType > ReaderType;

  ReaderType::Pointer reader = ReaderType::New();

  reader->SetFileName( argv[1] );

  try
    {
    reader->Update();
    }
  catch( itk::ExceptionObject & excp )
    {
    std::cerr << "Problem reading image file : " << argv[1] << std::endl;
    std::cerr << excp << std::endl;
    return -1;
    }

  typedef itk::Statistics::ScalarImageToHistogramGenerator<
                                 ImageType >   HistogramGeneratorType;

  HistogramGeneratorType::Pointer histogramGenerator =
                                        HistogramGeneratorType::New();

  histogramGenerator->SetInput(  reader->GetOutput() );

  histogramGenerator->SetNumberOfBins( atoi(argv[2]) );
  histogramGenerator->SetMarginalScale( 10.0 );

  histogramGenerator->SetHistogramMin(  -0.5 );
  histogramGenerator->SetHistogramMax( 255.5 );

  histogramGenerator->Compute();

  typedef HistogramGeneratorType::HistogramType  HistogramType;

  const HistogramType * histogram = histogramGenerator->GetOutput();

  const unsigned int histogramSize = histogram->Size();

  unsigned int bin;
  for( bin=0; bin < histogramSize; bin++ )
    {
    std::cout << bin << " ";
    std::cout << histogram->GetFrequency( bin, 0 ) << std::endl;
    }

  return 0;
}
