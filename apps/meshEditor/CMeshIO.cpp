// CMeshIO.cpp
// Chao Du Sept. 2014
#include "CMeshIO.h"

#include <iostream>
#include <fstream>
#include <string>

#include <opencv2/opencv.hpp>

// use ply reader/writer libaray
// http://www-sop.inria.fr/members/Thijs.Van-Lankveld/
#include "Ply/io.h"
#include "Ply/example.h" // data structure defined in example

#include "CPoint.h"

namespace PLY {
	const char* Vertex::name = "vertex";
	const Property Vertex::prop_x = Property("x", SCALAR, Float32);
	const Property Vertex::prop_y = Property("y", SCALAR, Float32);
	const Property Vertex::prop_z = Property("z", SCALAR, Float32);
	const char* Face::name = "face";
	const Property Face::prop_ind = Property("vertex_indices", LIST, Int32, Uint8);
} // namespace PLY

namespace ChaoVis {

using namespace std;

// read ply file
bool CMeshIO::ReadPlyFile( const string &nFileName,
								CMesh &nMesh )
{
	// open file
	ifstream aMeshFile( nFileName.c_str() );

	// REVISIT - for debug
//	cout << "File name: " << nFileName << endl;

	if ( !aMeshFile.is_open() ) {
		cerr << "Open file " << nFileName << " failed." << endl;
		return false;
	}

	// parse content
	// REVISIT - quick and dirty read, specific for the file written by write_mesh() in simulation/simulation.cpp
	string aLine;

	// read header
	int aNumVertices, aNumFaces;
	const int NUM_LINES_IN_HEADER = 18;
	const int NUM_VERTEX_POS = 15;
	const int NUM_FACE_POS   = 13;
	for ( int i = 0; i < NUM_LINES_IN_HEADER; ++i ) {
		getline( aMeshFile, aLine );
		if ( aLine.find( "element vertex " ) != string::npos ) {
			aNumVertices = std::atoi( aLine.substr( NUM_VERTEX_POS ).c_str() );
		}
		if ( aLine.find( "element face " ) != string::npos ) {
			aNumFaces = std::atoi( aLine.substr( NUM_FACE_POS ).c_str() );
		}
	}
	// REVISIT - for debug
	cout << "# of vertices to be read: " << aNumVertices << endl;
	cout << "# of faces to be read: " << aNumFaces << endl;

	// read vertices
	double x, y, z, nx, ny, nz, s, t;
	int red, green, blue;
	for ( int i = 0; i < aNumVertices; ++i ) {
		ChaoVis::PointXYZRGBNormal aPoint;
		aMeshFile >> x >> y >> z >> nx >> ny >> nz >> s >> t >> red >> green >> blue;
		aPoint.x = x;
		aPoint.y = y;
		aPoint.z = z;
		aPoint.normal[ 0 ] = nx;
		aPoint.normal[ 1 ] = ny;
		aPoint.normal[ 2 ] = nz;
		aPoint.argb[ 0 ] = 255;
		aPoint.argb[ 1 ] = red;
		aPoint.argb[ 2 ] = green;
		aPoint.argb[ 3 ] = blue;
		nMesh.fPoints.push_back( aPoint );
	}
	
	// read faces
	int aTmpInt, p1, p2, p3;
	for ( int i = 0; i < aNumFaces; ++i ) {
		aMeshFile >> aTmpInt >> p1 >> p2 >> p3;
		nMesh.fFaces.push_back( Vec3< int >( p1, p2, p3 ) );
		// add edges
		// make sure edge vertices are ordered
		nMesh.fEdges.insert( p1 > p2 ? Vec2< int >( p2, p1 ) : Vec2< int >( p1, p2 ) );
		nMesh.fEdges.insert( p2 > p3 ? Vec2< int >( p3, p2 ) : Vec2< int >( p2, p3 ) );
		nMesh.fEdges.insert( p3 > p1 ? Vec2< int >( p1, p3 ) : Vec2< int >( p3, p1 ) );
		// REVISIT - for debug
	//	printf( "p1p2p3: %d %d %d edge #: %d, ", p1, p2, p3, nMesh.fEdges.size() );
	//	char x = getc(stdin);
	}

	aMeshFile.close();
	return true;
}

bool CMeshIO::ReadPlyFileBinary( const std::string &nFileName,
								CMesh &nMesh )
{
	// open file
//	ifstream aMeshFile( nFileName.c_str(), ios::in | ios::binary );

	// REVISIT - for debug
//	cout << "File name: " << nFileName << endl;

//	if ( !aMeshFile.is_open() ) {
//		cerr << "Open file " << nFileName << " failed." << endl;
//		return false;
//	}
/*
	PLY::Header aPlyHeader;
	
	aPlyHeader.stream_type = PLY::BINARY_LE;
	
	PLY::Element aPlyElement1( "vertex" );
	PLY::Property aPlyProperty1( "x", PLY::SCALAR, PLY::Float32 );
	PLY::Property aPlyProperty2( "y", PLY::SCALAR, PLY::Float32 );
	PLY::Property aPlyProperty3( "z", PLY::SCALAR, PLY::Float32 );
	aPlyElement1.add_property( aPlyProperty1 );
	aPlyElement1.add_property( aPlyProperty2 );
	aPlyElement1.add_property( aPlyProperty3 );
	aPlyHeader.add_element( aPlyElement1 );

	PLY::Element aPlyElement2( "face" );
	PLY::Property aPlyProperty4( "uchar int vertex_indices", PLY::LIST, PLY::StartType );
	aPlyElement2.add_property( aPlyProperty4 );
	aPlyHeader.add_element( aPlyElement2 );	

	PLY::Reader aPlyReader( aPlyHeader, aMeshFile );
*/
	
	PLY::Header header;
	header.stream_type = PLY::BINARY_LE;
	PLY::Reader reader( header, nFileName.c_str() );

	
	// Once read, the Objects are put in the Storage.
	// In order to access these more easily, we can
	// implement the way they are stored.
	PLY::Storage store(header);

	// We first need to know which Objects we are
	// looking for.
	// We can describe the Element ourselves, although
	// there is a large chance that this will not match
	// the element in the file header exactly.
	// Luckily, only the names and types of the
	// Properties are important for this, not their
	// storage types.
#if 0
	// An easier way is to either let a pre-implemented
	// Object describe the Element.
	PLY::Element vertex, face;
	PLY::Vertex().make_element(vertex);
	PLY::Face().make_element(face);
#else
	// Or to get the Elements from the Header itself.
	PLY::Element& vertex = *header.find_element(PLY::Vertex::name);
	PLY::Element& face = *header.find_element(PLY::Face::name);

	// In this case, you may want to check if this
	// Element could be stored in the Object.
	bool v_is_v = PLY::Vertex().storage_test(vertex);	// Should be true..
	bool v_is_f = PLY::Vertex().storage_test(face);		// Should be false..
	bool f_is_v = PLY::Face().storage_test(vertex);		// Should be false..
	bool f_is_f = PLY::Face().storage_test(face);		// Should be true..
#endif

	// Then we need to have a place to store them.
	// This can be slight modification of the standard
	// AnyArray that uses the specific objects.
	PLY::VertexArray vertices;//vertices( vertex.num );
	store.set_collection(header, vertex, vertices);

	// Or a more extensive modification that.
	// For example, we can store the objects in an
	// external vector.
	std::vector<PLY::Face> collection;
	PLY::FaceExternal faces(collection);
	store.set_collection(header, face, faces);

	// Read the data in the file into the storage.
	bool ok = reader.read_data(&store);
	reader.close_file();
	if (!ok)
		exit(-1);

	std::cout << "data read.." << std::endl;

	// Now it's easy to access the data.
	if (vertices.size() > 0) {
		vertices.restart();
		for ( int i = 0; i < vertices.size(); ++i ) {
			PLY::Vertex& v = vertices.next<PLY::Vertex>();
	//		std::cout << "example vertex: " << v.x() << " " << v.y() << " " << v.z() << std::endl;
			ChaoVis::PointXYZRGBNormal aPoint;
	
			aPoint.x = v.x();
			aPoint.y = v.y();
			aPoint.z = v.z();
			aPoint.normal[ 0 ] = 0.0;
			aPoint.normal[ 1 ] = 0.0;
			aPoint.normal[ 2 ] = 0.0;
			aPoint.argb[ 0 ] = 255;
			aPoint.argb[ 1 ] = 0;
			aPoint.argb[ 2 ] = 0;
			aPoint.argb[ 3 ] = 0;
			nMesh.fPoints.push_back( aPoint );
		}
	}
	if (collection.size() > 0) {
		for ( int i = 0; i < collection.size(); ++i ) {
			PLY::Face& f = collection[i];
			size_t size = f.size();
			if (size == 0)
				std::cout << "example face: empty" << std::endl;
			else {
			
				int aTmpInt, p1, p2, p3;

//				std::cout << "example face (" << size << "):";
				for (size_t n = 0; n < size; ++n) {
	//				std::cout << " " << f.vertex(n);
	//				aMeshFile >> aTmpInt >> p1 >> p2 >> p3;
					if ( n == 0 ) {
						p1 = f.vertex( n );
					} else if ( n == 1 ) {
						p2 = f.vertex( n );
					} else if ( n == 2 ) {
						p3 = f.vertex( n );
					} else {
						printf( "ERROR: not triangle mesh.\n" );
					}
				}
				nMesh.fFaces.push_back( Vec3< int >( p1, p2, p3 ) );
				// add edges
				// make sure edge vertices are ordered
				nMesh.fEdges.insert( p1 > p2 ? Vec2< int >( p2, p1 ) : Vec2< int >( p1, p2 ) );
				nMesh.fEdges.insert( p2 > p3 ? Vec2< int >( p3, p2 ) : Vec2< int >( p2, p3 ) );
				nMesh.fEdges.insert( p3 > p1 ? Vec2< int >( p1, p3 ) : Vec2< int >( p3, p1 ) );
	//			std::cout << std::endl;
			}
		}
	}

	// parse content
	// REVISIT - quick and dirty read, specific for the file written by write_mesh() in simulation/simulation.cpp
/*	string aLine;

	// read header
	int aNumVertices, aNumFaces;
	const int NUM_LINES_IN_HEADER = 18;
	const int NUM_VERTEX_POS = 15;
	const int NUM_FACE_POS   = 13;
	for ( int i = 0; i < NUM_LINES_IN_HEADER; ++i ) {
		getline( aMeshFile, aLine );
		if ( aLine.find( "element vertex " ) != string::npos ) {
			aNumVertices = std::atoi( aLine.substr( NUM_VERTEX_POS ).c_str() );
		}
		if ( aLine.find( "element face " ) != string::npos ) {
			aNumFaces = std::atoi( aLine.substr( NUM_FACE_POS ).c_str() );
		}
	}
	// REVISIT - for debug
	cout << "# of vertices to be read: " << aNumVertices << endl;
	cout << "# of faces to be read: " << aNumFaces << endl;

	// read vertices
	double x, y, z, nx, ny, nz, s, t;
	int red, green, blue;
	for ( int i = 0; i < aNumVertices; ++i ) {
		ChaoVis::PointXYZRGBNormal aPoint;
		aMeshFile >> x >> y >> z >> nx >> ny >> nz >> s >> t >> red >> green >> blue;
		aPoint.x = x;
		aPoint.y = y;
		aPoint.z = z;
		aPoint.normal[ 0 ] = nx;
		aPoint.normal[ 1 ] = ny;
		aPoint.normal[ 2 ] = nz;
		aPoint.r = red;
		aPoint.g = green;
		aPoint.b = blue;
		nMesh.fPoints.push_back( aPoint );
	}
	
	// read faces
	int aTmpInt, p1, p2, p3;
	for ( int i = 0; i < aNumFaces; ++i ) {
		aMeshFile >> aTmpInt >> p1 >> p2 >> p3;
		nMesh.fFaces.push_back( Vec3< int >( p1, p2, p3 ) );
		// add edges
		// make sure edge vertices are ordered
		nMesh.fEdges.insert( p1 > p2 ? Vec2< int >( p2, p1 ) : Vec2< int >( p1, p2 ) );
		nMesh.fEdges.insert( p2 > p3 ? Vec2< int >( p3, p2 ) : Vec2< int >( p2, p3 ) );
		nMesh.fEdges.insert( p3 > p1 ? Vec2< int >( p1, p3 ) : Vec2< int >( p3, p1 ) );
		// REVISIT - for debug
	//	printf( "p1p2p3: %d %d %d edge #: %d, ", p1, p2, p3, nMesh.fEdges.size() );
	//	char x = getc(stdin);
	}

	aMeshFile.close();
	*/
	return true;
}

// write ply file
bool CMeshIO::WritePlyFile( const std::string &nFileName,
								const CMesh &nMesh )
{
  std::ofstream meshFile;
  meshFile.open( nFileName.c_str() );
  std::cout << "creating mesh file" << std::endl;

  // write header
  meshFile << "ply" << std::endl
           << "format ascii 1.0" << std::endl
           << "comment Created by particle simulation https://github.com/viscenter/registration-toolkit" << std::endl
           << "element vertex " << nMesh.fPoints.size() << std::endl
           << "property float x" << std::endl
           << "property float y" << std::endl
           << "property float z" << std::endl
           << "property float nx" << std::endl
           << "property float ny" << std::endl
           << "property float nz" << std::endl
           << "property float s" << std::endl
           << "property float t" << std::endl
           << "property uchar red" << std::endl
           << "property uchar green" << std::endl
           << "property uchar blue" << std::endl
           << "element face " << nMesh.fFaces.size() << std::endl
           << "property list uchar int vertex_indices" << std::endl
           << "end_header" << std::endl;

  // write vertex information
  for ( size_t i = 0; i < nMesh.fPoints.size(); i++ ) {
	ChaoVis::PointXYZRGBNormal aP = nMesh.fPoints[ i ];
	unsigned char b = ( unsigned char )( ( *reinterpret_cast< unsigned int* >( &( aP.rgb ) ) & 0xFF000000 ) >> 24 );
	unsigned char g = ( unsigned char )( ( *reinterpret_cast< unsigned int* >( &( aP.rgb ) ) & 0x00FF0000 ) >> 16 );
	unsigned char r = ( unsigned char )( ( *reinterpret_cast< unsigned int* >( &( aP.rgb ) ) & 0x0000FF00 ) >> 8 );
	unsigned char a = ( unsigned char )( *reinterpret_cast< unsigned int* >( &( aP.rgb ) ) & 0x000000FF );
    meshFile << aP.x << " "
             << aP.y << " "
             << aP.z << " "
             << aP.normal[ 0 ] << " "
             << aP.normal[ 1 ] << " "
             << aP.normal[ 2 ] << " "
             << 0 << " "
             << 0 << " "
             << ( unsigned int )r << " "
			 << ( unsigned int )g << " "
			 << ( unsigned int )b << std::endl;
  }

  // write face information
  for ( size_t i = 0; i < nMesh.fFaces.size(); i++ ) {
	Vec3< int > aFace = nMesh.fFaces[ i ];
    meshFile << "3 " << aFace[ 0 ] << " " << aFace[ 1 ] << " " << aFace[ 2 ] << std::endl;
  }

  meshFile.close();
	return false;
}



bool CMeshIO::ReadObjFile( const std::string &nFileName,
						 CMesh &nMesh )
{
	// open file
	ifstream aMeshFile( nFileName.c_str( ) );

	if ( !aMeshFile.is_open( ) ) {
		cerr << "Open file " << nFileName << " failed." << endl;
		return false;
	}

	// parse content
	// REVISIT - quick and dirty read, specific for the file written by write_mesh() in simulation/simulation.cpp
	string aLine;

	//int aNumVert = 0;
	//int aNumFace = 0;
	int p1 = 0, p2 = 0, p3 = 0;

	while ( std::getline( aMeshFile, aLine ) ) {

		// REVISIT - parsing a line of string with C++
		// http://stackoverflow.com/questions/14265581/parse-split-a-string-in-c-using-string-delimiter-standard-c

		// skip empty lines
		if ( aLine.length() < 1 ) {
			continue;
		}

		switch ( aLine[ 0 ] ) {
		case 'v':
			if ( aLine[ 1 ] == ' ' ) {
				ChaoVis::PointXYZRGBNormal aPoint;
				parseObjVertexLine( aLine, aPoint.x, aPoint.y, aPoint.z );
				nMesh.fPoints.push_back( aPoint );
//				printf( "INFO: vertex is read.\n" );
//				aNumVert++;
			} else if ( aLine[ 1 ] == 't' ) {
//				printf( "INFO: vertex texture is read.\n" );
			}
			break;
		case 'f':
//			printf( "INFO: face is read.\n" );
//			aNumFace++;
			parseObjFaceLine( aLine, p1, p2, p3 );



			// REVISIT - *.obj file format use 1 as the start index, so, minus one!!!!!!!
			//           put this into "survival handbook"
			p1--;
			p2--;
			p3--;



			nMesh.fFaces.push_back( Vec3< int >( p1, p2, p3 ) );

			// add edges
			// make sure edge vertices are ordered
			nMesh.fEdges.insert( p1 > p2 ? Vec2< int >( p2, p1 ) : Vec2< int >( p1, p2 ) );
			nMesh.fEdges.insert( p2 > p3 ? Vec2< int >( p3, p2 ) : Vec2< int >( p2, p3 ) );
			nMesh.fEdges.insert( p3 > p1 ? Vec2< int >( p1, p3 ) : Vec2< int >( p3, p1 ) );
			break;
		default:
			printf( "ERROR: unkown line\n" );
			break;
		}
	}

	aMeshFile.close( );
	return true;
}

bool CMeshIO::WriteObjFile( const std::string &nFileName,
						  const CMesh &nMesh )
{
	// REVISIT - FILL ME HERE
    return false;
}

void CMeshIO::parseObjVertexLine( const std::string &nLine, 
								  double &x,
								  double &y,
								  double &z )
{
	x = 0.0;
	y = 0.0;
	z = 0.0;

	std::string aLineCpy( nLine );
	std::string delimiter = " ";

	size_t pos = 0;
	std::string token;
	// tokens should be: "v x y z"
	int aCnt = 0;
	while ( ( pos = aLineCpy.find( delimiter ) ) != std::string::npos ) {
		token = aLineCpy.substr( 0, pos );
//		std::cout << token << std::endl;
		if ( aCnt == 0 ) {
			// do nothing
		} else if ( aCnt == 1 ) {
			x = atof( token.c_str() );
		} else if ( aCnt == 2 ) {
			y = atof( token.c_str( ) );
//		} else if ( aCnt == 3 ) {
//			z = atof( token.c_str( ) );
		} else {
			printf( "ERROR: parsing incorrect string.\n" );
		}
		aLineCpy.erase( 0, pos + delimiter.length( ) );
		aCnt++;
	}
	z = atof( aLineCpy.c_str( ) ); // the rest is z
}

void CMeshIO::parseObjFaceLine( const std::string &nLine,
								int &p1,
								int &p2,
								int &p3 )
{
	std::string aLineCpy( nLine );
	std::string delimiter = " ";

	size_t pos = 0;
	std::string token;
	// tokens should be: "f p1/ptex1 p2/ptex2 p3/ptex3"
	int aCnt = 0;
	int aDummyPtex;

	while ( ( pos = aLineCpy.find( delimiter ) ) != std::string::npos ) {
		token = aLineCpy.substr( 0, pos );
		//		std::cout << token << std::endl;
		if ( aCnt == 0 ) {
			// do nothing
		} else if ( aCnt == 1 ) {
			parseObjFaceSubLine( token, p1, aDummyPtex );
		} else if ( aCnt == 2 ) {
			parseObjFaceSubLine( token, p2, aDummyPtex );
		} else {
			printf( "ERROR: parsing incorrect string.\n" );
		}
		aLineCpy.erase( 0, pos + delimiter.length( ) );
		aCnt++;
	}
	parseObjFaceSubLine( aLineCpy, p3, aDummyPtex );
}

void CMeshIO::parseObjFaceSubLine( const std::string &nSubLine,
									int &p,
									int &ptex )
{
	std::string aLineCpy( nSubLine );
	std::string delimiter = "/";

	size_t pos = 0;
	std::string token;
	// tokens should be: "f p1/ptex1 p2/ptex2 p3/ptex3"
	int aCnt = 0;
	while ( ( pos = aLineCpy.find( delimiter ) ) != std::string::npos ) {
		token = aLineCpy.substr( 0, pos );
		//		std::cout << token << std::endl;
		if ( aCnt == 0 ) {
			p = atoi( token.c_str( ) );
		} else {
			printf( "ERROR: parsing incorrect string.\n" );
		}
		aLineCpy.erase( 0, pos + delimiter.length( ) );
		aCnt++;
	}
	// REVISIT - usually format is "f v/vt/vn v/vt/vn v/vt/vn"
	//           but need to handle case like "f v v v"
	if ( aCnt == 0 ) {	// "f v v v"
		p = atoi( aLineCpy.c_str() );
	} else {			// "f v/vt/vn v/vt/vn v/vt/vn"
		ptex = atoi( aLineCpy.c_str( ) ); // the rest is z
	}
}

} // namespace ChaoVis
