//An example application using the C++ ply reader/writer.
//Copyright (C) 2013  INRIA - Sophia Antipolis
//
//This program is free software: you can redistribute it and/or modify
//it under the terms of the GNU General Public License as published by
//the Free Software Foundation, either version 3 of the License, or
//(at your option) any later version.
//
//This program is distributed in the hope that it will be useful,
//but WITHOUT ANY WARRANTY; without even the implied warranty of
//MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
//GNU General Public License for more details.
//
//You should have received a copy of the GNU General Public License
//along with this program.  If not, see <http://www.gnu.org/licenses/>.
//
// Author(s):      Thijs van Lankveld


#include "io.h"
#include "example.h"


namespace PLY {
	const char* Vertex::name = "vertex";
	const Property Vertex::prop_x = Property("x", SCALAR, Float32);
	const Property Vertex::prop_y = Property("y", SCALAR, Float32);
	const Property Vertex::prop_z = Property("z", SCALAR, Float32);
	const char* Face::name = "face";
	const Property Face::prop_ind = Property("vertex_indices", LIST, Uint32, Uint8);
} // namespace PLY


#define TOFILE

void write_ply(const char* name) {
	PLY::Header header;
	// When writing it is easy to reuse the header of
	// a ply file read to reuse all/most of the
	// Elements read.
#ifdef TOFILE
	// Open the file for writing.
	PLY::Writer writer(header);
	if (!writer.open_file(name, PLY::ASCII))
		exit(-1);
#else
	// Write to the standard output.
	PLY::Writer writer(header, std::cout);
#endif

	// The header contains all  information
	// on the stored objects.
	//
	// We can describe the header ourselves,
	// or we can use the convenience method
	// of the Object if it is implemented.
#if 1
	PLY::Element vertex, face;
	PLY::Vertex().make_element(vertex);
	PLY::Face().make_element(face);
	header.add_element(vertex);
	header.add_element(face);
#else
	// Another convenience method for this.
	PLY::Vertex().describe_element(header);
	PLY::Face().describe_element(header);
	PLY::Element& vertex = *header.find_element(PLY::Vertex::name);
	PLY::Element& face = *header.find_element(PLY::Face::name);
#endif

	// Note that it is still possible to change
	// some Properies. For example, you could
	// change the storage type of the Property
	// "x" to Float64.
#if 0
	PLY::Property& x = *vertex.find_property("x");
	x.data_type = PLY::Float64;
#endif

	// Finally, there must be actual data to store.
	std::vector<PLY::Vertex> vert_data;
	vert_data.push_back(PLY::Vertex(1.0f, 2.0f, 3.0f));
	vert_data.push_back(PLY::Vertex(4.5f, 6.7f, 8.9f));
	vert_data.push_back(PLY::Vertex(10.0f, 11.1f, 12.2f));
	vert_data.push_back(PLY::Vertex(1.1f, 1.2f, 1.3f));

	std::vector<PLY::Face> face_data;
	face_data.push_back(PLY::Face(3));
	face_data.back().vertex(0, 0);
	face_data.back().vertex(1, 2);
	face_data.back().vertex(2, 3);

#if defined TOFILE && 1
	// This should be either put into a Storage.
	PLY::Storage store(header);
	PLY::VertexArray vertices;
	store.set_collection(header, vertex, vertices);
	vertices.prepare(vert_data.size());
	for (size_t n = 0; n < vert_data.size(); ++n)
		vertices.next<PLY::Vertex>() = vert_data[n];

	PLY::FaceExternal faces(face_data);
	store.set_collection(header, face, faces);

	// And written to the file in one go.
	bool ok = writer.write_data(&store);
#else
	// Or it can be written one element at a time,
	// although then the header should know how
	// many objects of each element will follow
	// and the header must be written first.
	// Also note that the 'store' values of the
	// Elements and Properties must be set correctly
	// before writing the header (i.e. if you are
	// going to write an Element/Property, the store
	// value should be true, which is default).
	PLY::Element& v_header = *header.find_element(PLY::Vertex::name);
	PLY::Element& f_header = *header.find_element(PLY::Face::name);
	v_header.num = vert_data.size();
	f_header.num = face_data.size();
	if (!writer.write_header())
		exit(-1);

	// The elements must be written in the same
	// order as they appear in the header.
	bool ok = true;
	for (size_t n = 0; n < vert_data.size(); ++n)
		if (!writer.write_object(v_header, &vert_data[n])) ok = false;
	for (size_t n = 0; n < face_data.size(); ++n)
		if (!writer.write_object(f_header, &face_data[n])) ok = false;
#endif
	
#ifdef TOFILE
	writer.close_file();
#endif
	if (!ok)
		exit(-1);

	std::cout << "data written.." << std::endl;
}

void read_ply(const char* name) {
	// Open the file for reading.
	PLY::Header header;
	PLY::Reader reader(header);
	if (!reader.open_file(name))
		exit(-1);
	
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
	PLY::VertexArray vertices;
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
		PLY::Vertex& v = vertices.next<PLY::Vertex>();
		std::cout << "example vertex: " << v.x() << " " << v.y() << " " << v.z() << std::endl;
	}
	if (collection.size() > 0) {
		PLY::Face& f = collection[0];
		size_t size = f.size();
		if (size == 0)
			std::cout << "example face: empty" << std::endl;
		else {
			std::cout << "example face (" << size << "):";
			for (size_t n = 0; n < size; ++n)
				std::cout << " " << f.vertex(n);
			std::cout << std::endl;
		}
	}
}

void copy_ply(const char* name) {
	// Open the file for reading.
	PLY::Header header;
	PLY::Reader reader(header);
	if (!reader.open_file(name))
		exit(-1);
	
	PLY::Storage store;
	
	// Read the data in the file into the mesh.
	bool ok = reader.read_data(&store);
	reader.close_file();
	if (!ok)
		exit(-1);

	std::cout << "data read.." << std::endl;

	// Open the file for writing.
	// Note that we reuse the earlier header,
	// but we set the type to ASCII.
	PLY::Writer writer(header);
	if (!writer.open_file(std::string(name).append("_cpy").c_str(), PLY::ASCII))
		exit(-1);
	
	// Store the data from the mesh.
	ok = writer.write_data(&store);
	writer.close_file();
	if (!ok)
		exit(-1);

	std::cout << "data copied.." << std::endl;
}

int main(int argc, char** argv) {
	if (argc < 2) {
		std::cout << "provide file to write/read.." << std::endl;
		exit(0);
	}

	// Write a file and then read the same file.
	write_ply(argv[1]);
//	read_ply(argv[1]);

	// Copy only the mesh data from one file to the other.
//	copy_ply(argv[1]);
}