//Example elements for the C++ ply reader/writer.
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
//
//
// TODO: doxygen


#ifndef __PLY_EXAMPLES_H__
#define __PLY_EXAMPLES_H__


#include "unknown.h"

namespace PLY {
	// Example object classes.

	/// An example Value: a float.
	struct FloatValue: public Value {
		float val;		///< The data value;

		/// Default constructor.
		FloatValue(): val(0) {}
		/// Instantiated constructor.
		/** \param v the value.
		 */
		FloatValue(const float& v): val(v) {}
	
		// Get the scalar value.
		bool get_scalar(const Property& prop, double& value) const {
			if (prop.type != SCALAR) return false;
			value = val; return true;
		}

		// Set the scalar value.
		bool set_scalar(const Property& prop, const double& value) {
			if (prop.type != SCALAR) return false;
			val = (float)value; return true;
		}
	}; // struct FloatValue


	/// An example Object: a Vertex with x,y,z coordinates.
	/** Note that this Object forgets any [Propeties](\ref Property)
	 *  of the Vertex (e.g. color) except the coordinates.
	 */
	struct Vertex: public Object {
		///\name The coordinates.
		/**\{*/
		FloatValue value_x, value_y, value_z;
		/**\}*/

		/// Default constructor.
		Vertex(): value_x(0), value_y(0), value_z(0) {}
		/// Instantiated constructor.
		/** \param x the value for x
		 *  \param y the value for y
		 *  \param z the value for z
		 */
		Vertex(float x, float y, float z): value_x(x), value_y(y), value_z(z) {}

		// Get a Value.
		Value* get_value(const Element& elem, const Property& prop) {
			if (prop.name.compare(prop_x.name.c_str()) == 0)		return &value_x;
			else if (prop.name.compare(prop_y.name.c_str()) == 0)	return &value_y;
			else if (prop.name.compare(prop_z.name.c_str()) == 0)	return &value_z;
			return 0;
		}

		// Construct an Element describing this Object.
		bool make_element(Element& elem) const {
			elem.name = name;
			elem.props.push_back(prop_x);
			elem.props.push_back(prop_y);
			elem.props.push_back(prop_z);
			return true;
		}
		
		///\name Accessors.
		/**\{*/
		/// Get the x coordinate.
		/** \return the x coordinate. */
		float x() const { double dval; value_x.get_scalar(prop_x, dval); return (float)dval; }

		/// Get the y coordinate.
		/** \return the y coordinate. */
		float y() const { double dval; value_y.get_scalar(prop_y, dval); return (float)dval; }

		/// Get the z coordinate.
		/** \return the z coordinate. */
		float z() const { double dval; value_z.get_scalar(prop_z, dval); return (float)dval; }
		/**\}*/

		///\name Mutators.
		/**\{*/
		/// Set the x coordinate.
		/** \param coord the x coordinate. */
		void x(float coord) { value_x.set_scalar(prop_x, coord); }
		
		/// Set the y coordinate.
		/** \param coord the y coordinate. */
		void y(float coord) { value_y.set_scalar(prop_y, coord); }
		
		/// Set the z coordinate.
		/** \param coord the z coordinate. */
		void z(float coord) { value_z.set_scalar(prop_z, coord); }
		/**\}*/

		/// The identifier
		static const char* name;
		///\name The Properties.
		/**\{*/
		static const Property prop_x;
		static const Property prop_y;
		static const Property prop_z;
		/**\}*/
	}; // struct Vertex


	/// An example Object: a Face with a list of indices.
	struct Face: public Object {
		AnyValue indices;	///< The vertex indices.

		/// Default constructor.
		Face() {}
		/// Instantiated constructor.
		/** \param size the number of vertices the Face has.
		 */
		Face(const size_t& size) { indices.set_size(prop_ind, size); }
		/// Copy constructor.
		/**	This constructor must be explicitly defined,
		 *  to protect the indices.
		 */
		Face(const Face& f) {
			if (f.indices.data != 0) {
				size_t size;
				double val;
				f.indices.get_size(prop_ind, size);
				indices.set_size(prop_ind, size);
				for (size_t n = 0; n < size; ++n) {
					f.indices.get_item(prop_ind, n, val);
					indices.set_item(prop_ind, n, val);
				}
			}
		}

		// Get a Value.
		Value* get_value(const Element& elem, const Property& prop) {
			if (prop.name.compare(prop_ind.name.c_str()) == 0) return &indices;
			return 0;
		}

		// Construct an Element describing this Object.
		bool make_element(Element& elem) const {
			elem.name = name;
			elem.props.push_back(prop_ind);
			return true;
		}

		///\name Accessors.
		/**\{*/
		/// Get the number of vertices.
		/** \return the number of vertices. */
		size_t size() const { size_t s; indices.get_size(prop_ind, s); return s; }

		/// Get the index of a vertex.
		/** \param num which of the vertices of the Face to access.
		 *  \return the index of the vertex.
		 */
		size_t vertex(const size_t& num) const { double index; indices.get_item(prop_ind, num, index); return (size_t)index; }
		/**\}*/

		///\name Mutators.
		/**\{*/
		/// Set the number of vertices.
		/** \param size the number of vertices. */
		void size(const size_t& size) { indices.set_size(prop_ind, size); }

		/// Set the index of a vertex.
		/** \param num which of the vertices of the Face to access.
		 *  \param index the index of the vertex.
		 */
		void vertex(const size_t& num, const size_t& index) { indices.set_item(prop_ind, num, (double)index); }
		/**\}*/
		
		/// The identifier
		static const char* name;
		///\name The Property.
		/**\{*/
		static const Property prop_ind;
		/**\}*/
	}; // struct Face


	/// An example Array: a collection of [Vertices](\ref Vertex).
	struct VertexArray: public AnyArray {
		/// Default constructor.
		VertexArray(): AnyArray() {}

		// REVISIT - Chao - size not initialized
		VertexArray( int nSize ): AnyArray() {
			objects = ( Object ** )( new Vertex[ nSize ] );
			for ( int i = 0; i < nSize; ++i ) {
				objects[ i ] = new Vertex; 
			}
		}
		~VertexArray() { /*if ( objects ) { delete []objects; objects = NULL; }*/ }
	
		// Get the next Object.
		Object& next_object() {
			if (objects[incr] == 0)
				objects[incr] = new Vertex;
			return *objects[incr++];
		}
	}; // struct VertexArray


	/// An example Array: a collection of [Faces](\ref Face).
	struct FaceArray: public AnyArray {
		/// Default constructor.
		FaceArray(): AnyArray() {}
	
		// Get the next Object.
		Object& next_object() {
			if (objects[incr] == 0)
				objects[incr] = new Face;
			return *objects[incr++];
		}
	}; // struct FaceArray
	

	/// An example Array: an collection of [Vertices](\ref Vertex).
	/** These [Vertices](\ref Vertex) are stored extrenally.
	 */
	struct VertexExternal: public Array {
		std::vector<Vertex>& vertices;	///< The [Vertices](\ref Vertex) in the array.
		size_t incr;					///< Indicator for the Object to get.

		/// Constructor.
		/** \param v the external storage.
		 */
		VertexExternal(std::vector<Vertex>& v): Array(), vertices(v), incr(0) {}

		size_t size() { return vertices.size(); }
		void prepare(const size_t& size) { vertices.reserve(size); restart(); }
		void clear() { vertices.clear(); }
		void restart() { incr = 0; }

		// Get the next Object.
		Object& next_object() {
			if (vertices.size() <= incr)
				vertices.resize(incr+1);
			return vertices[incr++];
		}
	}; // struct VertexExternal
	

	/// An example Array: an collection of [Faces](\ref Face).
	/** These [Faces](\ref Face) are stored extrenally.
	 */
	struct FaceExternal: public Array {
		std::vector<Face>& faces;	///< The [Faces](\ref Face) in the array.
		size_t incr;				///< Indicator for the Object to get.

		/// Constructor.
		/** \param f the external storage.
		 */
		FaceExternal(std::vector<Face>& f): Array(), faces(f), incr(0) {}

		size_t size() { return faces.size(); }
		void prepare(const size_t& size) { faces.reserve(size); restart(); }
		void clear() { faces.clear(); }
		void restart() { incr = 0; }

		// Get the next Object.
		Object& next_object() {
			if (faces.size() <= incr)
				faces.resize(incr+1);
			return faces[incr++];
		}
	}; // struct FaceExternal
} // namespace PLY


#endif // __PLY_EXAMPLES_H__