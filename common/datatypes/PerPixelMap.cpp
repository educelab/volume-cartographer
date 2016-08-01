//
// Created by Seth Parker on 3/18/16.
//

#include "PerPixelMap.h"

using namespace volcart;

///// Constructors /////
// Empty Map of width x height
PerPixelMap::PerPixelMap( int height, int width ) : _height(height), _width(width) {
  _map = cv::Mat_<cv::Vec6d>( height, width, cv::Vec6d(0,0,0,0,0,0) );
}

// Construct map from file
PerPixelMap::PerPixelMap( boost::filesystem::path path ) {
  read(path);
}

///// Disk IO /////
void PerPixelMap::write( boost::filesystem::path path ) {

  // Ensure proper file extension
  path.replace_extension(".yml.gz");

  // Write to file
  std::cerr << "volcart::PerPixelMap: Writing to file " << path.filename() << std::endl;
  cv::FileStorage fs( path.string(), cv::FileStorage::WRITE );
  fs << "PerPixelMapping" << _map;
  fs.release();

}

void PerPixelMap::read( boost::filesystem::path path ) {

  std::cerr << "volcart::PerPixelMap: Reading from file " << path.filename() << std::endl;
  cv::FileStorage file( path.string(), cv::FileStorage::READ );
  cv::FileNode map = file["PerPixelMapping"];

  // Read the header info
  _height = (int) map["rows"];
  _width  = (int) map["cols"];

  // Fill the _map from the list of doubles
  _map = cv::Mat_<cv::Vec6d>( _height, _width, cv::Vec6d(0,0,0,0,0,0) );
  cv::Vec6d v;
  cv::FileNodeIterator dbl = map["data"].begin();

  // Make sure the size is as expected
  // To-do: Throw exception if they don't match
  bool matches = map["data"].size() == _height * _width * 6;

  for( int y = 0; y < _height; ++y ) {
    for( int x = 0; x < _width; ++x ) {

      // Fill each cv::Vec6d
      for( int n = 0; n < 6; ++n, ++dbl ) {
        v(n) = (double)(*dbl);
      }

      // Assign _map in the correct position
      _map(y,x) = v;

    } // x
  } // y

  file.release();
}
