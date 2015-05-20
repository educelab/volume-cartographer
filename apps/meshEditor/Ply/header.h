// A C++ reader/writer of .ply files.
// The header and its elements.
// These describe how the data is stored in the file/stream.
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


#ifndef __PLY_HEADER_H__
#define __PLY_HEADER_H__


#include "base.h"
#include <iterator>
#include <string>
#include <vector>

namespace PLY {
	/// The description of a Property of an Element.
	/** Each Element contains a number of [Properties](\ref Property).
	 *  These [Properties](\ref Property) should be described in the Header.
	 *  \sa Element and Header.
	 */
	struct Property {
		std::string name;		///< The name of the Property.
		Variable_type type;		///< Whether this Property is a scalar, list, or string.

		Scalar_type data_type;	///< How the data of this Property is stored in the file.
		Scalar_type size_type;	///< How the list size is stored in the file.

		bool store;				///< Whether to store this Property.

		/// The default constructor.
		Property(): type(SCALAR),
			data_type(StartType),
			size_type(StartType),
			store(true) {}
		/// The instantiated constructor.
		/** This constructor is mainly for ease of use
		 *  when describing new Properties.
		 *  \param n the name.
		 *  \param t the type of the property.
		 *  \param dt the type of the data.
		 *  \param st the type of the list size.
		 */
		Property(const char* n, const Variable_type& t,
			const Scalar_type& dt, const Scalar_type& st = StartType)
			: name(n), type(t), data_type(dt), size_type(st), store(true) {}
	}; // struct Property


	/// The description of an Element in the Header.
	/** Each Element contains a number of [Properties](\ref Property).
	 *  These [Properties](\ref Property) should be described in the Header.
	 *
	 *  Note that an Element does not contain any actual data,
	 *  only the description of how this data is organized.
	 *  The data itself is stored in an Object.
	 *  \sa Property, Header and Object.
	 */
	struct Element {
		std::string name;				///< The name of the Element.
		size_t num;						///< Number of [Objects](\ref Object) of the Element.
		std::vector<Property> props;	///< Vector of [Properties](\ref Property) of the Element.

		bool store;						///< Whether to store this Element.

		/// Default constructor.
		Element(): num(0), store(true) {}
		/// Instantiated constructor.
		/** \param n the name of the Element.
		 */
		Element(const char* n): name(n), num(0), store(true) {}

		/// Add a Property to the Element.
		/** \param prop the Property to add.
		 */
		inline void add_property(const Property& prop) { props.push_back(prop); }

		/// Find the index of a Property.
		/** \param name the name of the Property to find.
		 *  \param [out] index the index of the Property.
		 *  \return true if the name refers to a Property of this Element.
		 */
		bool find_index(const char* name, size_t& index) const;

		/// Find a Property.
		/** \param name the name of the Property to find.
		 *  \return the Property, or NULL if the Element
		 *  does not contain the Property.
		 */
		Property* find_property(const char* name);
	}; // struct Element

	
	/// The Header of a ply file.
	/** The Header describes the [Elements](\ref Element) that a file contains.
	 *  This includes a description of all the [Properties](\ref Property)
	 *  of the [Elements](\ref Element).
	 *
	 *  The Header also contains the ply version and modality in which the
	 *  data is stored, as well as any comments on the file.
	 *  \sa Element
	 */
	struct Header {
	private:
		Stream_type system_type;			// The binary mode of the system.

		Stream_type is_LE() const;

	public:
		float version;						///< The PLY version of the file.
		Stream_type stream_type;			///< How the data is stored.

		std::vector<Element> elements;		///< List of data [Elements](\ref Element).
		std::vector<std::string> comments;	///< List of comments.
		std::vector<std::string> obj_info;	///< List of object info descriptions.

		/// Default constructor.
		/** The default stream_type is ASCII.
		 */
		Header(): stream_type(ASCII), version(1.0), system_type(is_LE()) {}

		/// Get the binary storage-mode of the system.
		/** \return the binary mode of the system.
		 */
		Stream_type system() const {return system_type;}

		/// Add an Element to the Header.
		/** \param elem the Element to add.
		 */
		inline void add_element(const Element& elem) {elements.push_back(elem);}
		
		/// Find the index of an Element.
		/** \param name the name of the Element to find.
		 *  \param [out] index the index of the Element.
		 *  \return true if the name refers to a Element of this Header.
		 */
		bool find_index(const char* name, size_t& index) const;

		/// Find an Element.
		/** \param name the name of the Element to find.
		 *  \return the Element, or NULL if the Header
		 *  does not contain the Element.
		 */
		Element* find_element(const char* name);

		/// Apply the stream mode.
		/** This will adjust a byte-array such that it is
		 *  read or stored correctly in binary mode.
		 *
		 *  This depends on the stream_type and binary mode of the system.
		 *  \param [in,out] ptr a pointer to the byte-array.
		 *  \param n the number of bytes to consider.
		 */
		void apply_stream_type(char* ptr, size_t n);
	}; // struct Header


	// Checks if a character is white-space.
	struct WhiteSpace {
		WhiteSpace() {}
		bool operator()(char c) const;
	}; // struct WhiteSpace


	/*  The line Tokenizer extracts words from a line based
	 *  on white-space. Any type of white-space is considered
	 *  to separate different words.
	 */
	struct Tokenizer {
		// The current line.
		char* line;

		// Constructor.
		Tokenizer() {}

		// Get the next word.
		bool next(char* word);
	}; // class Tokenizer
} // namespace PLY


#endif // __PLY_HEADER_H__