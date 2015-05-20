// A C++ reader/writer of .ply files.
// Data objects.
// These contain the actual data.
// Generally, the user will implement subclasses
// for their own object types.
//
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


#ifndef __PLY_OBJECT_H__
#define __PLY_OBJECT_H__


#include "header.h"

namespace PLY {
	/// A Value representing a Property.
	/** Note that a Property describes how the
	 *  Value should be interpreted.
	 */
	struct Value {
		///\name Methods for scalar Values.
		/**\{*/
		/// Get the scalar value.
		/** \param prop the Property describing the Value.
		 *  \param [out] value the value.
		 *  \return true if the Value could be read.
		 */
		virtual bool get_scalar(const Property& prop, double& value) const {return false;}

		/// Set the scalar value.
		/** \param prop the Property describing the Value.
		 *  \param value the value.
		 *  \return true if the Value could be set.
		 */
		virtual bool set_scalar(const Property& prop, const double& value) {return false;}
		/**\}*/

		///\name Methods for list Values.
		/**\{*/
		/// Get the size of the list Value.
		/** \param prop the Property describing the Value.
		 *  \param [out] size the size of the list.
		 *  \return true if the size could be read.
		 */
		virtual bool get_size(const Property& prop, size_t& size) const {return false;}

		/// Get an item of the list Value.
		/** \param prop the Property describing the Value.
		 *  \param num the index of the item.
		 *  \param [out] value the value.
		 *  \return true if the item could be read.
		 */
		virtual bool get_item(const Property& prop, const size_t& num, double& value) const {return false;}

		/// Prepare the list Value to recieve a number of items.
		/** \param prop the Property describing the Value.
		 *  \param size the expected size of the list.
		 *  \return true if the size could be written.
		 */
		virtual bool set_size(const Property& prop, const size_t& size) {return false;}

		/// Set an item of the list Value.
		/** \param prop the Property describing the Value.
		 *  \param num the index of the item.
		 *  \param value the value.
		 *  \return true if the item could be written.
		 */
		virtual bool set_item(const Property& prop, const size_t& num, const double& value) {return false;}
		/**\}*/

		///\name Methods for string Values.
		/**\{*/
		/// Get the string Value.
		/** \param prop the Property describing the Value.
		 *  \param [out] str the string.
		 *  This array should be long enough to contain the string.
		 *  \return true if the string could be read.
		 */
		virtual bool get_string(const Property& prop, char* str) const {return false;}

		/// Set the string Value.
		/** \param prop the Property describing the Value.
		 *  \param str the string value.
		 *  \return true if the string could be written.
		 */
		virtual bool set_string(const Property& prop, const char* str) {return false;}
		/**\}*/

		/// Copy the data to another Value.
		/** Note this only works if both [Values](\ref Value)
		 *  have [Properties](\ref Property) of the same type.
		 *  \param prop the Property describing this Value.
		 *  \param [out] to the Value to copy to.
		 *  \param to_prop the Property of the other Value.
		 *  \return true if the Value could be copied.
		 */
		bool copy(const Property& prop, Value& to, const Property& to_prop) const;
	}; // struct Value


	/// The basis for an Object containing some data.
	/** Note that an Element describes how the
	 *  Object should be interpreted.
	 */
	struct Object {
		/// Prepare the Object to represent an Element.
		/** \param elem the Element to prepare for.
		 */
		virtual void prepare(const Element& elem) {}

		/// Get a Value.
		/** \param elem the Element describing the Object.
		 *  \param prop the Property describing the Value.
		 *  \return the Value or NULL if the Value is not
		 *  contained in this Object.
		 */
		virtual Value* get_value(const Element& elem, const Property& prop) = 0;

		/// Construct an Element describing this Object.
		/** Note that this is only meant for user-constructed
		 *  [Objects](\ref Object) to get the Element easily.
		 *  \param [out] elem the constructed Element.
		 *  \return true if the Element could be constructed.
		 */
		virtual bool make_element(Element& elem) const {return false;}

		/// Add an Element describing this Object to a Header.
		/** \param [in,out] header the header to add the new Element to.
		 *  \return true if the Element could be constructed.
		 */
		bool describe_element(Header& header) const;

		/// Check if the Object matches with an Element.
		/** Note that this method changes the values of the Object.
		 *  \param elem the Element to compare to.
		 *  \return true if the Object could store the Element.
		 */
		bool storage_test(const Element& elem);
	}; // struct Object


	/// A collection of [Objects](\ref Object).
	struct Array {
		/// Get the size of the collection.
		/** \return the size of the collection.
		 */
		virtual size_t size() = 0;
		
		/// Prepare the Array to contain a number of [Objects](\ref Object).
		/** \param size the number of [Objects](\ref Object) to prepare for.
		 */
		virtual void prepare(const size_t& size) = 0;
		
		/// Prepare the Array to contain the [Objects](\ref Object) of an Element.
		/** \param elem the Element to prepare for.
		 */
//		void prepare(const Element& elem) { prepare(elem.props.size()); }
		void prepare(const Element& elem) { prepare(elem.num); }	// REVISIT - use property size is not correct

		/// Remove all [Objects](\ref Object) from the Array.
		virtual void clear() = 0;

		/// Restart from the beginning of the Array.
		virtual void restart() = 0;

		/// Get the next Object.
		/** \return the next Object.
		 */
		virtual Object& next_object() = 0;
		
		/// Get the next Object as a certain type.
		/** \return the Object.
		 */
		template < class T >
		T& next() { return dynamic_cast<T&>(next_object()); }
	}; // struct Array


	/// A container for all [Objects](\ref Object).
	struct Storage {
		Array** collect;					///< The Object collections.
		
		/// Deafult constructor.
		Storage(): collect(0), num(0), unknown(0) {}
		/// Instantiated constructor.
		/** \param header the Header that describes the data.
		 */
		Storage(const Header& header);
		~Storage() { deinit(); }		///< Destructor.
		
		/// Prepare the Storage to store the [Objects](\ref Object).
		/** \param header the Header to prepare for.
		 */
		void prepare(const Header& header);

		/// Get a collection of [Objects](\ref Object).
		/** \param header the Header describing the [Objects](\ref Object).
		 *  \param elem the Element describing the Object.
		 *  \return the Array or NULL if the Element is not
		 *  contained in this Storage.
		 */
		Array* get_collection(const Header& header, const Element& elem);

		/// Set a collection of [Objects](\ref Object).
		/** \param header the Header describing the [Objects](\ref Object).
		 *  \param elem the Element describing the Object.
		 *  \param coll the collection to assign.
		 *  \return true if the collection could be set.
		 */
		bool set_collection(const Header& header, const Element& elem, Array& coll);

	private:
		Storage(const Storage&) {}		// Private copy constructor to protect the collections.
		
		void init(size_t size);			// Initialize the arrays.
		void deinit();					// Deinitialize the arrays.

		size_t num;						// The number of collections.
		bool* unknown;					// The unknown Object collections.
	}; // struct Storage
} // namespace PLY


#endif // __PLY_OBJECT_H__