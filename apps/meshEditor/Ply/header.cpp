//
// Adapted to C++ streams by Thijs van Lankveld.

/*
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


#include "header.h"
#include <algorithm>

namespace PLY {
	// Find the index of a Property.
	bool Element::find_index(const char* name, size_t& index) const {
		for (index = 0; index < props.size(); ++index)
			if (props[index].name.compare(name) == 0)
				return true;
		return false;
	}
	
	// Find a Property.
	Property* Element::find_property(const char* name) {
		size_t index;
		if (!find_index(name, index)) return 0;
		return &props[index];
	}


	// Determine the binary mode of the system.
	Stream_type Header::is_LE() const {
		char c[8] = {1,0,0,0,0,0,0,0};
		if (*((int*)c) == 1)
			return BINARY_LE;
		return BINARY_BE;
	}
	
	// Find the index of an Element.
	bool Header::find_index(const char* name, size_t& index) const {
		for (index = 0; index < elements.size(); ++index)
			if (elements[index].name.compare(name) == 0)
				return true;
		return false;
	}
	
	// Find the index of an Element.
	Element* Header::find_element(const char* name) {
		size_t index;
		if (!find_index(name, index)) return 0;
		return &elements[index];
	}
	
	// Apply the stream type.
	void Header::apply_stream_type(char* ptr, size_t n) {
		if (stream_type != system_type && n > 1) {
			// Reverse the bytes in the ptr.
			for (size_t i = 0; i < n/2; ++i)
				std::swap(ptr[i], ptr[n-i-1]);
		}
	}

	
	// Checks if a character is white-space.
	bool WhiteSpace::operator()(char c) const {
		return c == ' ' ||
			   c == '\n' ||
			   c == '\t' ||
			   c == '\r';
	}

	
	// Get the next word.
	bool Tokenizer::next(char* word) {
		// Ignore leading white-space.
		WhiteSpace ignore;
		if (line == 0)
			return false;
		while (*line != '\0' && ignore(*line))
			++line;
		
		// Find the end of the word.
		size_t len = 1, q = 0;
		switch (*line) {
		case '\0':
			line = 0;
			return false;
		case '\"':
			// Quotes are handled as one word.
			while (*(line+len) != '\0' && *(line+len) != '\"')
				++len;
			if (*(line+len) != '\"')
				q = 1;
			break;
		default:
			while (*(line+len) != '\0' && !ignore(*(line+len)))
				++len;
			break;
		}

		// Copy the word;
		for (size_t i = 0; i < len; ++i)
			word[i] = line[i];
		word[len] = '\0';

		// Increment the line.
		line += len+q;
		return true;
	}
} // namespace PLY