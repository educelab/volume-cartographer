// splitcloud
// Split input point cloud into smaller chunks

#include <iostream>

#include <boost/filesystem.hpp>

#include <pcl/io/pcd_io.h>
#include <pcl/common/common.h>

pcl::PointCloud<pcl::PointXYZRGB>::Ptr input  (new pcl::PointCloud<pcl::PointXYZRGB>);
pcl::PointCloud<pcl::PointXYZRGB>::Ptr outputCache (new pcl::PointCloud<pcl::PointXYZRGB>);
int caches_written = 0;
void emptyCache();

int main(int argc, char* argv[]) {
  if (argc < 2) {
    std::cerr << "Usage:" << std::endl;
    std::cerr << argv[0] << " [input].pcd" << std::endl;
    return (1);
  }

  // Setup file paths
  std::string input_path = argv[1];
  if(input_path.substr(input_path.find_last_of(".") + 1) != "pcd") {
    std::cerr << "ERROR: Path file does not have expected pcd extension." << std::endl;
    exit(EXIT_FAILURE);
  }

  // prepare point cloud
  pcl::io::loadPCDFile<pcl::PointXYZRGB> (input_path, *input);

  // copy points to cache
  for ( size_t p_id = 0; p_id < input->size(); ++p_id ) {
    outputCache->push_back( input->points[p_id] );
    if ( outputCache->size() == input->width * 100 ) {
      emptyCache();
      p_id = p_id - (input->width * 10);
    }
  }

  // Emptying leftovers in cache
  emptyCache();

}

void emptyCache() {
  std::cout << "Writing cached cloud to file: " << caches_written << std::endl;

  boost::filesystem::path seg(boost::filesystem::current_path().stem().string() + "_" + std::to_string(caches_written));
  boost::filesystem::create_directories(seg);
  std::string outputName = seg.string() + "/cloud.pcd";

  // Write to file
  outputCache->width = input->width;
  outputCache->height = outputCache->size()/outputCache->width;
  outputCache->points.resize(outputCache->height * outputCache->width);
  pcl::io::savePCDFileBinaryCompressed(outputName, *outputCache);

  outputCache->clear();
  ++caches_written;
};