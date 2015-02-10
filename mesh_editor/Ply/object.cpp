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


#include "object.h"
#include "unknown.h"
#include <cstring>

namespace PLY {
	// Copy the data to another Value.
	bool Value::copy(const Property& prop, Value& to, const Property& to_prop) const {
		double value;
		size_t size;
		char str[BIG_STRING];
		switch(prop.type) {
		case SCALAR:
			if (!get_scalar(prop, value)) return false;
			return to.set_scalar(to_prop, value);
		case LIST:
			if (!get_size(prop, size) ||
				!to.set_size(to_prop, size)) return false;
			for (size_t n = 0; n < size; ++n)
				if (!get_item(prop, n, value) ||
					!to.set_item(to_prop, n, value))
						return false;
			return true;
		case STRING:
			if (!get_string(prop, str)) return false;
			return to.set_string(to_prop, str);
		}
		return false;
	}


	// Add an Element describing this Object to a Header.
	bool Object::describe_element(Header& header) const {
		header.elements.push_back(Element());
		if (make_element(header.elements.back()))
			return true;
		header.elements.pop_back();
		return false;
	}

	// Check if the Object matches with an Element.
	bool Object::storage_test(const Element& elem) {
		double D = 2, dval = D;
		size_t S = 1, size = S;
		char str[BIG_STRING] = "str";
		const Property* prop;
		Value* value;

		// The object is matches the element if it
		// has get/set methods for each property.
		for (size_t p = 0; p < elem.props.size(); ++p) {
			prop = &elem.props[p];
			value = get_value(elem, *prop);
			if (!value) return false;

			switch (prop->type) {
			case SCALAR:
				// Check the scalar.
				if (!value->set_scalar(*prop, dval) ||
					!value->get_scalar(*prop, dval) ||
					dval != D)
						return false;
				break;
			case STRING:
				// Check the string.
				if (!value->set_string(*prop, str) ||
					!value->get_string(*prop, str) ||
					std::strcmp(str, "str") != 0)
						return false;
				break;
			case LIST:
				// Check the list.
				if (!value->set_size(*prop, size) ||
					!value->get_size(*prop, size) ||
					size != 1)
						return false;
				for (size_t n = 0; n < size; ++n)
					if (!value->set_item(*prop, n, dval) ||
						!value->get_item(*prop, n, dval) ||
						dval != D)
							return false;
				break;
			}
		}
		return true;
	}


	// Instantiated constructor.
	Storage::Storage(const Header& header): collect(0), num(0), unknown(0) {
		prepare(header);
	}

	// Prepare the Storage to store the Objects.
	void Storage::prepare(const Header& header) {
		// In order to let the user add his own collections,
		// existing collections are kept if there are enough.
		if (collect == 0 || header.elements.size() > num) {
			deinit();
			init(header.elements.size());
		}
		else {
			for (size_t n = 0; n < num; ++n)
				if (collect[n] != 0)
					collect[n]->clear();
		}
	}

	// Get a collection of Objects.
	Array* Storage::get_collection(const Header& header, const Element& elem) {
		size_t index;
		if (!header.find_index(elem.name.c_str(), index)) {};
		if (collect[index] == 0) {
			collect[index] = new AnyArray;
			unknown[index] = true;
		}
		return collect[index];
	}

	// Set a collection of [Objects](\ref Object).
	bool Storage::set_collection(const Header& header, const Element& elem, Array& coll) {
		size_t index;
		if (!header.find_index(elem.name.c_str(), index)) return false;
		if (collect[index] != 0 && unknown[index])
			delete collect[index];
		collect[index] = &coll;
		unknown[index] = false;
		return true;
	}

	// Initialize the objects.
	void Storage::init(size_t size) {
		collect = new Array*[size];
		unknown = new bool[size];
		num = size;
		for (size_t n = 0; n < size; ++n)
			collect[n] = 0;
	}
	
	// Deinitialize the objects.
	void Storage::deinit() {
		if (collect) {
			for (size_t n = 0; n < num; ++n)
				if (collect[n] != 0 && unknown[n])
					delete collect[n];
			delete[] collect;
			collect = 0;
			delete[] unknown;
		}
	}
} // namespace PLY
