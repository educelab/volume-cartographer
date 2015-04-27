/*
A C++ reader/writer of .ply files

Addapted to C++ streams by Thijs van Lankveld.

Header for PLY polygon files.

- Greg Turk

A PLY file contains a single polygonal _object_.

An object is composed of lists of _elements_.  Typical elements are
vertices, faces, edges and materials.

Each type of element for a given object has one or more _properties_
associated with the element type.  For instance, a vertex element may
have as properties three floating-point values x,y,z and three unsigned
chars for red, green and blue.

-----------------------------------------------------------------------

Copyright (c) 1998 Georgia Institute of Technology.  All rights reserved.

Permission to use, copy, modify and distribute this software and its
documentation for any purpose is hereby granted without fee, provided
that the above copyright notice and this permission notice appear in
all copies of this software and that you do not sell the software.

THE SOFTWARE IS PROVIDED "AS IS" AND WITHOUT WARRANTY OF ANY KIND,
EXPRESS, IMPLIED OR OTHERWISE, INCLUDING WITHOUT LIMITATION, ANY
WARRANTY OF MERCHANTABILITY OR FITNESS FOR A PARTICULAR PURPOSE.
*/

#include "io.h"
#include <fstream>
#include <sstream>
#include <cstring>


namespace PLY {
	// Open a file for reading.
	bool Reader::open_file(const char* file_name) {
		// Tack on the extension .ply, if necessary.
		std::string name = file_name;
		if (name.length() < 4 || name.substr(name.length()-4).compare(".ply") != 0)
			name.append(".ply");
		
		// Open the file for reading.
        stream = new std::ifstream(name.c_str(), std::ifstream::in|std::ifstream::binary);
		if (stream == NULL || !(*stream))
			HANDLE_FAULT("Reader::open : invalid stream");
	
		// Read the header of the ply file.
		return read_header();
	}

	// Close the reader.
	void Reader::close_file() {
		if (stream) {
			std::ifstream* fs = dynamic_cast<std::ifstream*>(stream);
			if (fs)
				fs->close();
			delete stream;
			stream = 0;
		}
	}
	
	// Read the ply header from the current stream.
	bool Reader::read_header() {
		// Check the stream.
		if (stream == NULL || !(*stream))
			HANDLE_FAULT("Reader::read_header : invalid stream");
	
		// The first word must be "ply".
		char line[BIG_STRING];
		char word[BIG_STRING];
		Tokenizer tokenizer;
		stream->getline(line, BIG_STRING);
		tokenizer.line = line;
		if (!tokenizer.next(word) || std::strcmp(word, "ply") != 0 || tokenizer.next(word))
			HANDLE_FAULT("Reader::read_header : not a ply stream");

		// The second line must specify the format.
		stream->getline(line, BIG_STRING);
		tokenizer.line = line;
		if (!tokenizer.next(word) || std::strcmp(word, "format") != 0 ||
			!tokenizer.next(word) || !read_stream_type(word, header.stream_type) ||
			!tokenizer.next(word) || !std::sscanf(word, "%f", &header.version))
				HANDLE_FAULT("Reader::open : invalid format");

		// Handle the remaining lines one word at a time.
		Element* elem = NULL;
		stream->getline(line, BIG_STRING);
		tokenizer.line = line;
		while (tokenizer.next(word)) {
			if (std::strcmp(word, "element") == 0) {
				// This line starts a new element.
				header.elements.push_back(Element());
				elem = &header.elements.back();
				if (!tokenizer.next(word))
					HANDLE_FAULT("Reader::read_header : invalid element");
				// Each element has a name.
				elem->name = word;
				// And the number of objects.
				if (!tokenizer.next(word) ||
					!std::sscanf(word, "%ud", &elem->num))
						HANDLE_FAULT("Reader::read_header : invalid element number");
			}
			else if (std::strcmp(word, "property") == 0) {
				// This line describes a property of the current element.
				Property prop;

				// Check the type of property.
				if (!tokenizer.next(word))
					HANDLE_FAULT("Reader::read_header : invalid property");
				if (std::strcmp(word, "list") == 0) {
					prop.type = LIST;
					if (!tokenizer.next(word) || !read_scalar(word, prop.size_type) ||
						!tokenizer.next(word) || !read_scalar(word, prop.data_type) ||
						!tokenizer.next(word))
							HANDLE_FAULT("Reader::read_header : invalid list");
					prop.name = word;
				}
				else if (std::strcmp(word, "string") == 0) {
					prop.type = STRING;
					prop.size_type = Int8;
					prop.data_type = Int8;
					if (!tokenizer.next(word))
						HANDLE_FAULT("Reader::read_header : invalid string");
					prop.name = word;
				}
				else {
					prop.type = SCALAR;
					if (!read_scalar(word, prop.data_type) ||
						!tokenizer.next(word))
							HANDLE_FAULT("Reader::read_header : invalid string");
					prop.name = word;
				}
				elem->props.push_back(prop);
			}
			else if (std::strcmp(word, "comment") == 0)
				header.comments.push_back(line);
			else if (std::strcmp(word, "obj_info") == 0)
				header.obj_info.push_back(line);
			else if (std::strcmp(word, "end_header") == 0)
				break;
			else 
				HANDLE_FAULT("Reader::read_header : unknown word");
				
			// Collect more words.
			stream->getline(line, BIG_STRING);
			tokenizer.line = line;
		}

		return true;
	}
	
	// Read the data from the file.
	bool Reader::read_data(Storage* store) {
		Element* elem;
		Property* prop;
		Array* collect;
		Object* obj;
		Value* value = NULL;
		double dval;
		size_t size;
		char str[BIG_STRING];
		bool store_prop;

		// Prepare the store to receive the objects.
		store->prepare(header);

		// The data is read in the same order as the elements.
		for (size_t e = 0; e < header.elements.size(); ++e) {
			elem = &header.elements[e];
			collect = 0;
			obj = 0;

			if (store && elem->store)
				collect = store->get_collection(header, *elem);
			if (collect)
				collect->prepare(*elem);

			// We read a number of objects of this element.
			for (size_t n = 0; n < elem->num; ++n) {
				if (collect)
					obj = &collect->next_object();
				if (obj)
					obj->prepare(*elem);
				
				// The properties are read in the same order as in the elements.
				for (size_t p = 0; p < elem->props.size(); ++p) {
					prop = &elem->props[p];
					if (obj)
						value = obj->get_value(*elem, *prop);
					store_prop = obj && value && prop->store;

					// Read the property.
					switch (prop->type) {
					case SCALAR:
						// Read one value.
						if (!read_value(prop->data_type, dval)) return false;
						if (store_prop && !value->set_scalar(*prop, dval))
							HANDLE_FAULT("Reader::read_data : invalid object");
						break;
					case STRING:
						if (header.stream_type == ASCII) {
							// Strings are stored differently in ASCII.
							read_ascii_string(str);
							if (store_prop && !value->set_string(*prop, str))
								HANDLE_FAULT("Reader::read_data : invalid object");
						}
						else {
							// Read the size (this includes the '\0').
							if (!read_size(Int8, size)) return false;
							// Read the characters (including the '\0').
							for (size_t n = 0; n < size; ++n) {
								if (!read_value(Int8, dval)) return false;
								str[n] = (char)dval;
							}
							// Store the string.
							if (store_prop && !value->set_string(*prop, str))
								HANDLE_FAULT("Reader::read_data : invalid object");
						}
						break;
					case LIST:
						// Read the size.
						if (!read_size(prop->size_type, size)) return false;
						if (store_prop && !value->set_size(*prop, size))
							HANDLE_FAULT("Reader::read_data : invalid object");
						// Read the items.
						for (size_t n = 0; n < size; ++n) {
							if (!read_value(prop->data_type, dval)) return false;
							if (store_prop && !value->set_item(*prop, n, dval))
								HANDLE_FAULT("Reader::read_data : invalid object");
						}
						break;
					}
				}
			}
		}
		return true;
	}
	
	bool Reader::read_stream_type(const std::string& word, Stream_type& type) {
		if (word.compare("ascii") == 0)
			header.stream_type = ASCII;
		else if (word.compare("binary_big_endian") == 0)
			header.stream_type = BINARY_BE;
		else if (word.compare("binary_little_endian") == 0)
			header.stream_type = BINARY_LE;
		else
			return false;
		return true;
	}

	bool Reader::read_scalar(const std::string& word, Scalar_type& type) {
		for (int t = StartType; t != EndType; ++t) {
			if (word.compare(type_names[t]) == 0) {
				type = Scalar_type(t);
				return true;
			}
		}
		for (int t = StartType; t != EndType; ++t) {
			if (word.compare(old_type_names[t]) == 0) {
				type = Scalar_type(t);
				return true;
			}
		}
		HANDLE_FAULT("Reader::read_scalar : unknown scalar");
		return true;
	}
	
	bool Reader::read_ascii_value(const Scalar_type& type, double& value) {
		switch (type) {
		default:
		case StartType:
		case EndType:
			HANDLE_FAULT("Reader::read_ascii_value : invalid scalar type");
			break;
		case Int8:
		case Int16:
		case Int32:
		case Uint8:
		case Uint16:
		case Uint32:
		case Float32:
		case Float64:
			(*stream) >> value;
			break;
		}
		return true;
	}

	bool Reader::read_binary_value(const Scalar_type& type, double& value) {
		char c[8];
		char* ptr = c;
		
		switch (type) {
		default:
		case StartType:
		case EndType:
			HANDLE_FAULT("Reader::read_binary_value : invalid scalar type");
			break;
		case Int8:
			read(ptr, 1);
			value = *((char*)ptr);
			break;
		case Int16:
			read(ptr, 2);
			value = *((short*)ptr);
			break;
		case Int32:
			read(ptr, 4);
			value = *((int*)ptr);
			break;
		case Uint8:
			read(ptr, 1);
			value = *((unsigned char*)ptr);
			break;
		case Uint16:
			read(ptr, 2);
			value = *((unsigned short*)ptr);
			break;
		case Uint32:
			read(ptr, 4);
			value = *((unsigned int*)ptr);
			break;
		case Float32:
			read(ptr, 4);
			value = *((float*)ptr);
			break;
		case Float64:
			read(ptr, 8);
			value = *((double*)ptr);
			break;
		}
		return true;
	}

	void Reader::read(char* ptr, size_t num) {
		stream->read(ptr, num);
		header.apply_stream_type(ptr, num);
	}

	void Reader::read_ascii_string(char* str) {
		WhiteSpace ignore;
		*str = ' ';
		// Ignore leading white-space.
		while (ignore(*str))
			(*stream) >> *str;

		if (*str == '\"')
			stream->getline(str, BIG_STRING, '\"');
		else {
			// Extend the string to the next white-space.
			int len = 1;
			for (; len < BIG_STRING-1; ++len)
				(*stream) >> str[len];
				if (ignore(str[len]) ||
					str[len] == '\0')
						return;
			str[len] = '\0';
		}
	}
	

	bool Writer::open_file(const char* file_name, const Stream_type& type) {
		// Tack on the extension .ply, if necessary.
		std::string name = file_name;
		if (name.length() < 4 || name.substr(name.length()-4).compare(".ply") != 0)
			name.append(".ply");
		
		// Open the file for reading.
        stream = new std::ofstream(name.c_str(), std::ifstream::out|std::ifstream::binary);
		if (stream == NULL || !(*stream))
			HANDLE_FAULT("Writer::open : invalid stream");

		header.stream_type = type;
		return true;
	}
	
	// Close the writer.
	void Writer::close_file() {
		if (stream) {
			std::ofstream* fs = dynamic_cast<std::ofstream*>(stream);
			if (fs)
				fs->close();
			delete stream;
			stream = 0;
		}
	}

	// Write the ply header to the current stream.
	bool Writer::write_header() {
		// Check the stream.
		if (stream == NULL || !(*stream))
			HANDLE_FAULT("Writer::write_header : invalid stream");

		// The header is always in ASCII.
		// First, write the storage type.
		(*stream) << "ply" << std::endl;
		switch (header.stream_type) {
		case ASCII:
			(*stream) << "format ascii 1.0" << std::endl;
			break;
		case BINARY_BE:
			(*stream) << "format binary_big_endian 1.0" << std::endl;
			break;
		case BINARY_LE:
			(*stream) << "format binary_little_endian 1.0" << std::endl;
			break;
		default:
			HANDLE_FAULT("Writer::write_header : invalid storage type");
			break;
		}

		// Then, write the comments and obj_info.
		for (size_t i = 0; i < header.comments.size(); ++i)
			(*stream) << "comment " << header.comments[i] << std::endl;
		for (size_t i = 0; i < header.obj_info.size(); ++i)
			(*stream) << "obj_info " << header.obj_info[i] << std::endl;

		// Then the elements.
		Element* elem;
		Property* prop;
		for (size_t e = 0; e < header.elements.size(); ++e) {
			elem = &header.elements[e];
			if (!elem->store) continue;
			(*stream) << "element " << elem->name << " " << elem->num << std::endl;

			// With their properties.
			for (size_t p = 0; p < elem->props.size(); ++p) {
				prop = &elem->props[p];
				if (!prop->store) continue;
				switch (prop->type) {
				case LIST:
					(*stream) << "property list " << type_names[prop->size_type] <<
						" " << type_names[prop->data_type] << " " << prop->name << std::endl;
					break;
				case STRING:
					(*stream) << "property string " << prop->name << std::endl;
					break;
				case SCALAR:
					(*stream) << "property " << type_names[prop->data_type] <<
						" " << prop->name << std::endl;
					break;
				default:
					HANDLE_FAULT("Writer::write_header : invalid property type");
					break;
				}
			}
		}

		// Finally, we indicate the end of the header.
		(*stream) << "end_header" << std::endl;
		return true;
	}

	// Write the data to the file.
	bool Writer::write_data(Storage* store) {
		if (store == 0) return false;

		// Update the number of objects.
		Element* elem;
		Array* collect;
		for (size_t e = 0; e < header.elements.size(); ++e) {
			elem = &header.elements[e];
			if (!elem->store) continue;
			collect = store->get_collection(header, *elem);
			if (collect)
				elem->num = collect->size();
			else
				elem->num = 0;
		}

		// Write the header.
		if (!write_header())
			return false;
		
		// The data is written in the same order as the elements.
		for (size_t e = 0; e < header.elements.size(); ++e) {
			elem = &header.elements[e];
			if (!elem->store) continue;
			collect = store->get_collection(header, *elem);
			if (!write_element(*elem, collect))
				return false;
		}
		return true;
	}

	// Write all objects of one element.
	bool Writer::write_element(const Element& elem, Array* collect) {
		if (collect == 0) return false;
		// All objects of the element should be written.
		Object* obj = 0;
		collect->restart();
		for (size_t n = 0; n < elem.num; ++n) {
			obj = &collect->next_object();
			if (!obj || !write_object(elem, obj))
				return false;
		}
		return true;
	}
	
	bool Writer::write_ascii_value(const Scalar_type& type, const double& value) {
		switch (type) {
		default:
		case StartType:
		case EndType:
			return false;
		case Int8:
		case Int16:
		case Int32:
		case Uint8:
		case Uint16:
			(*stream) << (int)value;
			break;
		case Uint32:
			(*stream) << (unsigned int)value;
			break;
		case Float32:
		case Float64:
			(*stream) << value;
			break;
		}
		return true;
	}

	bool Writer::write_binary_value(const Scalar_type& type, const double& value) {
		char			cval;
		short			sval;
		int				ival;
		float			fval;
		unsigned char	ucval;
		unsigned short	usval;
		unsigned int	uival;
		
		switch (type) {
		default:
		case StartType:
		case EndType:
			return false;
		case Int8:
			cval = (char)value;
			write(&cval, 1);
			break;
		case Int16:
			sval = (short)value;
			write((char*)&sval, 2);
			break;
		case Int32:
			ival = (int)value;
			write((char*)&ival, 4);
			break;
		case Uint8:
			ucval = (unsigned char)value;
			write((char*)&ucval, 1);
			break;
		case Uint16:
			usval = (unsigned short)value;
			write((char*)&usval, 2);
			break;
		case Uint32:
			uival = (unsigned int)value;
			write((char*)&uival, 4);
			break;
		case Float32:
			fval = (float)value;
			write((char*)&fval, 4);
			break;
		case Float64:
			write((char*)&value, 8);
			break;
		}
		return true;
	}

	void Writer::write(char* ptr, size_t num) {
		header.apply_stream_type(ptr, num);
		stream->write(ptr, num);
	}

	bool Writer::write_ascii_object(const Element& elem, Object* obj) {
		const Property* prop;
		const Value* value;
		size_t size;
		double dval;
		char str[BIG_STRING];

		// The values are written in the same order as the properties.
		for (size_t p = 0; p < elem.props.size(); ++p) {
			prop = &elem.props[p];
			if (!prop->store) continue;
			value = obj->get_value(elem, *prop);
			if (!value) return false;

			// Write the value.
			if (p > 0)
				(*stream) << " ";
			switch (prop->type) {
			case SCALAR:
				// Write the scalar.
				if (!value->get_scalar(*prop, dval) ||
					!write_ascii_value(prop->data_type, dval))
						HANDLE_FAULT("Writer::write_ascii_object : invalid object" << std::endl);
				break;
			case STRING:
				// Write the string.
				if (!value->get_string(*prop, str))
					HANDLE_FAULT("Writer::write_ascii_object : invalid string");
				(*stream) << "\"" << str << "\"";
				break;
			case LIST:
				// Write the length.
				if (!value->get_size(*prop, size) ||
					!write_ascii_value(prop->size_type, (double)size))
						HANDLE_FAULT("Writer::write_ascii_object : invalid list size");
				// Write the items.
				for (size_t n = 0; n < size; ++n) {
					(*stream) << " ";
					if (!value->get_item(*prop, n, dval) ||
						!write_ascii_value(prop->data_type, dval))
							HANDLE_FAULT("Writer::write_ascii_object : invalid list item");
				}
				break;
			default:
				HANDLE_FAULT("Writer::write_ascii_object : invalid property type");
				break;
			}
		}

		// Each object ends with a newline.
		(*stream) << std::endl;
		return true;
	}

	bool Writer::write_binary_object(const Element& elem, Object* obj) {
		const Property* prop;
		const Value* value;
		size_t size;
		double dval;
		char str[BIG_STRING];

		// The values are written in the same order as the properties.
		for (size_t p = 0; p < elem.props.size(); ++p) {
			prop = &elem.props[p];
			if (!prop->store) continue;
			value = obj->get_value(elem, *prop);
			if (!value) return false;

			// Write the value.
			switch (prop->type) {
			case SCALAR:
				// Write the scalar.
				if (!value->get_scalar(*prop, dval) ||
					!write_binary_value(prop->data_type, dval))
						HANDLE_FAULT("Writer::write_binary_object : invalid object");
				break;
			case STRING:
				// Write the string.
				if (!value->get_string(*prop, str))
					HANDLE_FAULT("Writer::write_ascii_object : invalid string");
				// Write the length (including the '\0').
				size = std::strlen(str) + 1;
				if (!write_binary_value(Int8, (double)size))
					HANDLE_FAULT("Writer::write_binary_object : invalid string size");
				// Write the characters (including the '\0').
				for (size_t n = 0; n < size; ++n) {
					dval = str[n];
					if (!write_binary_value(Int8, dval))
						HANDLE_FAULT("Writer::write_binary_object : invalid string char");
				}
				break;
			case LIST:
				// Write the length.
				if (!value->get_size(*prop, size) ||
					!write_binary_value(prop->size_type, (double)size))
						HANDLE_FAULT("Writer::write_binary_object : invalid list size");
				// Write the items.
				for (size_t n = 0; n < size; ++n) {
					if (!value->get_item(*prop, n, dval) ||
						!write_binary_value(prop->data_type, dval))
							HANDLE_FAULT("Writer::write_binary_object : invalid list item");
				}
				break;
			default:
				HANDLE_FAULT("Writer::write_binary_object : invalid property type");
				break;
			}
		}
		return true;
	}
} // namespace PLY
