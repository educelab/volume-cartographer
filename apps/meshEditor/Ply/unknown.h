// A C++ reader/writer of .ply files.
// Undescribed data objects.
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


#ifndef __PLY_UNKNOWN_H__
#define __PLY_UNKNOWN_H__


#include "object.h"

namespace PLY {
	/// A Value representing a generic Property.
	struct AnyValue: public Value {
		char* data;					///< The data value.

		AnyValue(): data(0) {}		///< Constructor.
		~AnyValue() { deinit(); }	///< Destructor.
	
		bool get_scalar(const Property& prop, double& value) const;
		bool set_scalar(const Property& prop, const double& value);
		bool get_size(const Property& prop, size_t& size) const;
		bool get_item(const Property& prop, const size_t& num, double& value) const;
		bool set_size(const Property& prop, const size_t& size);
		bool set_item(const Property& prop, const size_t& num, const double& value);
		bool get_string(const Property& prop, char* str) const;
		bool set_string(const Property& prop, const char* str);

	private:
		AnyValue(const AnyValue&) {} ///< Private copy constructor to protect the data.

		/// Deinitialize the data.
		void deinit() { if (data) { delete[] data; data = 0; } }

		/// Convert a pointer to a value.
		/** \param ptr the pointer.
		 *  \param type the type of value.
		 *  \param [out] value the value.
		 *  \return true if it could be converted.
		 */
		bool to_value(const char* ptr, const Scalar_type& type, double& value) const;

		/// Convert a value to a pointer.
		/** \param value the value.
		 *  \param type the type of value.
		 *  \param [out] ptr the pointer.
		 *  Note that it is assumed ptr can hold enough
		 *  bytes to store the value.
		 *  \return true if it could be converted.
		 */
		bool to_pointer(const double& value, const Scalar_type& type, char* ptr) const;
		
		/// Copy bytes from one array to another.
		void copy(const char* from, char* to, const size_t& num) const;
	}; // struct AnyValue
	

	/// An Object representing a generic Element.
	struct AnyObject: public Object {
		Value** values;					///< The Values of the Object.

		/// Default constructor.
		AnyObject(): values(0) {}
		~AnyObject() { deinit(); }		///< Destructor.

		void prepare(const Element& elem);
		Value* get_value(const Element& elem, const Property& prop);

	private:
		AnyObject(const AnyObject&) {}	// Private copy constructor to protect the values.
		
		void init(size_t size);			// Initialize the objects.
		void deinit();					// Deinitialize the objects.
		
		size_t num;						// The number of values.
	}; // struct AnyObject

	
	/// An Array representing a generic Element.
	struct AnyArray: public Array {
		Object** objects;				///< The [Objects](\ref Object) in the array.
		size_t incr;					///< Indicator for the Object to get.
	
		/// Default constructor.
		AnyArray(): objects(0), incr(0), num(0) {}
		virtual ~AnyArray() { deinit(); }	///< Destructor.
	
		virtual size_t size() { return num; }
		virtual void prepare(const size_t& size);
		virtual void clear();
		virtual void restart() { incr = 0; }
		virtual Object& next_object();

	private:
		AnyArray(const AnyArray&) {}	// Private copy constructor to protect the objects.
		
		void init(size_t size);			// Initialize the objects.
		void deinit();					// Deinitialize the objects.

		size_t num;						// The number of objects.
	}; // struct AnyArray
} // namespace PLY

#endif // __PLY_UNKNOWN_H__