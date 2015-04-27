/*
A C++ reader/writer of .ply files.

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


#ifndef __PLY_IO_H__
#define __PLY_IO_H__


#include "object.h"

namespace PLY {
	/// The reader for extracting [Objects](\ref Object) from a ply stream.
	/** \sa Writer.
	 */
	struct Reader {
		Header& header;					///< The Header of the ply file.
		std::istream* stream;			///< The stream to read from.

		/// Base constructor.
		/** In order to be able to share a Header,
		 *  for example with a Writer, it is stored
		 *  as a reference.
		 *  \param [in,out] h the Header to use.
		 */
		Reader(Header& h): header(h), stream(0) {}
		/// Construct from existing stream.
		/** It is assumed that the stream was opened
		 *  correctly. In order to be able to read any
		 *  binary file, the stream must be opened in
		 *  binary mode.
		 *  \param [in,out] h the Header to use.
		 *  \param [in,out] is the stream to use.
		 */
		Reader(Header& h, std::istream& is): header(h), stream(&is) {}
		/// Construct and open a file.
		/** This constructor tries to open a file for reading
		 *  and reads the Header if the file could be opened.
		 *  \param [in,out] h the Header to use.
		 *  \param file_name the name of the file to read.
		 */
		Reader(Header& h, const char* file_name): header(h), stream(0) {if (!open_file(file_name)) close_file();}
		
		/// Open a file for reading.
		/** \param file_name the file to open.
		 *  \return true if the file could be successfully
		 *  opened.
		 */
		bool open_file(const char* file_name);

		/// Close the stream.
		/** Note that this will throw an exception
		 *  when trying to close a standard io stream.
		 */
		void close_file();

		/// Read the Header from the current stream.
		/** \return true if the header could be correctly read.
		 */
		bool read_header();

		/// Read the Object data from the current stream.
		/** \param [out] store where to store the [Objects](\ref Object).
		 *  \return true if all the data could be successfully read.
		 */
		bool read_data(Storage* store);
	
	private:
		// Read storage and scalar types.
		bool read_stream_type(const std::string& word, Stream_type& type);
		bool read_scalar(const std::string& word, Scalar_type& type);

		// Read a value from an ASCII or binary stream.
		bool read_ascii_value(const Scalar_type& type, double& value);
		bool read_binary_value(const Scalar_type& type, double& value);
		void read(char* ptr, size_t n);

		// Read a value.
		inline bool read_value(const Scalar_type& type, double& value) {
			if (header.stream_type == ASCII)
				return read_ascii_value(type, value);
			else
				return read_binary_value(type, value);
		}

		// Read a size.
		inline bool read_size(const Scalar_type& type, size_t& size) {
			double value;
			if (!read_value(type, value))
				return false;
			size = (size_t)value;
			return true;
		}

		// Read a string from an ASCII stream.
		void read_ascii_string(char* str);
	}; // struct Reader

	
	/// The writer for storing [Objects](\ref Object) into a ply stream.
	/** \sa Reader.
	 */
	struct Writer {
		Header& header;					///< The Header to write.
		std::ostream* stream;			///< The stream to write to.

		/// Base constructor.
		/** In order to be able to share a Header,
		 *  for example with a Writer, it is stored
		 *  as a reference.
		 *  \param [in,out] h the Header to use.
		 */
		Writer(Header& h): header(h), stream(0) {}
		/// Construct from existing stream.
		/** It is assumed that the stream was opened
		 *  correctly. In order to be able to write any
		 *  binary file, the stream must be opened in
		 *  binary mode.
		 *  \param [in,out] h the Header to use.
		 *  \param [in,out] os the stream to use.
		 */
		Writer(Header& h, std::ostream& os): header(h), stream(&os) {}
		/// Construct and open a file.
		/** This constructor tries to open a file for writing
		 *  \param [in,out] h the Header to use.
		 *  \param file_name the name of the file to read.
		 *  \param type the storage type to use for the data.
		 */
		Writer(Header& h, const char* file_name, const Stream_type& type = ASCII)
			: header(h), stream(0) {if (!open_file(file_name, type)) close_file();}
		
		/// Open a file for writing.
		/** \param file_name the file to open.
		 *  \param type the storage type to use for the data.
		 *  \return true if the file could be successfully
		 *  opened.
		 */
		bool open_file(const char* file_name, const Stream_type& type = ASCII);

		/// Close the stream.
		/** Note that this will throw an exception
		 *  when trying to close a standard io stream.
		 */
		void close_file();

		/// Write the Header to the current stream.
		/** Note that you should not change the num values
		 *  of the [Elements](\ref Element) after writing
		 *  the Header, as then the header and data no longer match.
		 *  \return true if the header could be correctly written.
		 */
		bool write_header();

		/// Write the Object data to the current stream.
		/** Note that this method writes the Header and all
		 *  the [Objects](\ref Object) to the stream.
		 *  \param store the [Objects](\ref Object) to store.
		 *  \return true if all the data could be successfully written.
		 */
		bool write_data(Storage* store);

		/// Write all [Objects](\ref Object) of one Element.
		/** \param elem the Element of the [Objects](\ref Object).
		 *  \param collect the [Objects](\ref Object) to write.
		 *  \return true if all the data could be successfully written.
		 */
		bool write_element(const Element& elem, Array* collect);
		
		/// Write an Object.
		/** \param elem the Element of the Object to store.
		 *  \param obj the Object to store.
		 *  \return true if the Object could be successfully written.
		 */
		inline bool write_object(const Element& elem, Object* obj) {
			if (header.stream_type == ASCII)
				return write_ascii_object(elem, obj);
			else
				return write_binary_object(elem, obj);
		}

	private:
		// Write a value to an ASCII or binary stream.
		bool write_ascii_value(const Scalar_type& type, const double& value);
		bool write_binary_value(const Scalar_type& type, const double& value);
		void write(char* ptr, size_t n);

		// Write an element to an ASCII or binary stream.
		bool write_ascii_object(const Element& elem, Object* obj);
		bool write_binary_object(const Element& elem, Object* obj);
	}; // class Writer
} // namespace PLY


#endif // __PLY_IO_H__