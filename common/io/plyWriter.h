// VC PLY Exporter v1.0
// Created by Media Team on 10/30/15.
//

#ifndef VC_PLYWRITER_H
#define VC_PLYWRITER_H

#include <iostream>
#include <fstream>

#include <opencv2/opencv.hpp>
#include <boost/filesystem.hpp>
#include <boost/lexical_cast.hpp>

#include "../vc_defines.h"
#include "../vc_datatypes.h"

namespace volcart {
    namespace io {
        class plyWriter {
        public:
            plyWriter() {};
            plyWriter( std::string outputPath, VC_MeshType::Pointer mesh );
            plyWriter( std::string outputPath, VC_MeshType::Pointer mesh, volcart::Texture texture);

            void setPath( std::string path ) { _outputPath = path; };
            void setMesh( VC_MeshType::Pointer mesh ) { _mesh = mesh; };
            void setTexture( volcart::Texture texture ) { _texture = texture; };

            bool validate(); // make sure all required output parameters have been set

            int write();

        private:
            boost::filesystem::path _outputPath; // The desired filepath. This should include the .obj extension.
            std::ofstream           _outputMesh;

            VC_MeshType::Pointer _mesh;
            volcart::Texture     _texture;

            int _writeHeader();
            int _writeVertices();
            int _writeFaces();
        };
    }
}

#endif //VC_PLYWRITER_H
