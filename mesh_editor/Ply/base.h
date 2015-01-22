// A C++ reader/writer of .ply files.
// Base types.
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


#ifndef __PLY_BASE_H__
#define __PLY_BASE_H__


/** \mainpage
 *
 *  \section overview Overview
 *
 *  This project provides a simple, but general,
 *  implementation of a reader and writer for
 *  streams using the ply format. It has the
 *  following features:
 *
 *  - The streams are generally file-streams, but
 *  it is very simple to replace these by any other
 *  stream, such as the standard in/output stream
 *  (i.e. std::cout, std::cin).
 *  - The implementation is general in that it can
 *  read and write any file in ply format,
 *  irrespective of the actual elements used.
 *  The required data is stored by letting the user
 *  construct their own subclasses for Object and
 *  Storage. Examples are provided in the form of
 *  the Vertex, Face and Mesh classes.
 *  - Very easy to use.
 *  - The reader automatically parses the header.
 *  After this, it is very easy to check which of
 *  your own Object subclasses match.
 *  - All the values are passed through the code as
 *  double, making it easier to handle them.
 *  - Possible to easily change the way faults
 *  while reading/writing the files are handled
 *  (e.g. stop processing, or just an error message).
 *  - Correctly handles each type of stream (i.e.
 *  ASCII, little-endian, or big-endian).
 *  - Can process the data very fast and with small
 *  memory usage.
 *  - Platform independent due to restriction to
 *  ANSI C and standard template libraries. Similarly,
 *  both "\n" and "\r\n" line ending are handled.
 *
 * \subsection discl Disclaimer
 *
 *  The code is based upon Greg Turk's code from 1998.
 *  I have written the complete project from scratch
 *  and the interface is very different to Turk's code.
 *  However, the underlying code ended up very similar
 *  to the original, so I feel I should give proper
 *  credit here.
 *
 *  Any parts that are significantly different, such
 *  as the usage of streams, may be used under the
 *  GPL 3.0 license.
 *
 *  Thank you for using this free code. May your data
 *  storage be smooth and safe.
 *
 *  Thijs van Lankveld
 */


#include <stdlib.h>
#include <iostream>

/// All the classes for this project fall within the PLY namespace.
namespace PLY {
	/// The version of the PLY code.
	const char _VERSION[] = "1.2.0";

	/// The storage type of the objects.
	enum Stream_type {ASCII =		1,	///< ASCII PLY file.
					  BINARY_BE =	2,	///< Binary PLY file, big endian.
					  BINARY_LE =	3	///< Binary PLY file, little endian.
	};

	/// Scalar data types supported by PLY format.
	enum Scalar_type {StartType =	0,	///< start marker
					  Int8 =		1,	///< char
					  Int16 =		2,	///< short
					  Int32 =		3,	///< int
					  Uint8 =		4,	///< unsigned char
					  Uint16 =		5,	///< unsigned short
					  Uint32 =		6,	///< unsigned int
					  Float32 =		7,	///< float
					  Float64 =		8,	///< double
					  EndType =		9	///< end marker
	};
	
	/// The names used to describe the data types.
    extern const char* type_names[]; // REVISIT - modified by Chao, 2015 Jan on Linux
	/// Old names used in early versions to describe the data types.
    extern const char* old_type_names[]; // REVISIT - modified by Chao, 2015 Jan on Linux

	/// The sizes (in bytes) of the different data types.
	const size_t ply_type_bytes[] = {0, 1, 2, 4, 1, 2, 4, 4, 8, 0};
	
	/// The maximum number of characters in a line to read.
	const int BIG_STRING = 4096;

	/// Variable types supported by PLY format.
	enum Variable_type {SCALAR =	0,	///< Scalar value
						LIST =  	1,	///< List of scalars
						STRING =	2	///< String of chars
	};

	/// Ways to handle faults.
	enum Verbatim {IGNORE,		///< Ignore faults. Note that this may lead to unexpected results.
				   WARNING,		///< Give a message and ignore.
				   EXCEPTION,	///< Give a message and break.
				   ERROR		///< Give a message and exit.
	};

	/// How faults are handled.
	const Verbatim ON_FAULT = EXCEPTION;

#define HANDLE_FAULT(msg) {\
	switch (ON_FAULT) {\
		case IGNORE:\
			break;\
		case WARNING:\
		case EXCEPTION:\
		case ERROR:\
			std::cerr << msg << std::endl;\
			switch (ON_FAULT) {\
			case WARNING:\
				break;\
			case ERROR:\
				exit(1);\
			case EXCEPTION:\
				return false;\
			}\
		}\
	}
} // namespace PLY


#endif // __PLY_BASE_H__
