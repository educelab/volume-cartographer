// David Pennington
// 10/7/2014

#include "volumepkg.h"
#include <opencv2/opencv.hpp>
#include <cstdlib>
#include <cmath>

static int     lower_slice_index;
static cv::Mat lower_slice;
static cv::Mat upper_slice;
static cv::Mat output_slice;

struct DoublePoint3D {
	double x;
	double y;
	double z;

	DoublePoint3D(double ix, double iy, double iz) {
		x = ix;
		y = iy;
		z = iz;
	}

	DoublePoint3D(DoublePoint3D p1, DoublePoint3D p2) {
		x = p1.x + p2.x;
		y = p1.y + p2.y;
		z = p1.z + p2.z;
	}
};
struct IntPoint3D {
	int x;
	int y;
	int z;

	IntPoint3D(int ix, int iy, int iz) {
		x = ix;
		y = iy;
		z = iz;
	}

	IntPoint3D(IntPoint3D p1, IntPoint3D p2) {
		x = p1.x + p2.x;
		y = p1.y + p2.y;
		z = p1.z + p2.z;
	}
};

struct DoublePoint2D {
	double x;
	double y;

	DoublePoint2D(double ix, double iy) {
		x = ix;
		y = iy;
	}

	DoublePoint2D(DoublePoint2D p1, DoublePoint2D p2) {
		x = p1.x + p2.x;
		y = p1.y + p2.y;
	}
};

struct IntPoint2D {
	int x;
	int y;

	IntPoint2D(int ix, int iy) {
		x = ix;
		y = iy;
	}

	IntPoint2D(IntPoint2D p1, IntPoint2D p2) {
		x = p1.x + p2.x;
		y = p1.y + p2.y;
	}

	IntPoint2D() {
		x = 0;
		y = 0;
	}
};

class IntPoint2DBinaryTree {
	struct Node {
		IntPoint2D data;
		double score;
		Node* left_child;
		Node* right_child;

		Node(IntPoint2D new_data, double new_score) {
			data = new_data;
			score = new_score;
			left_child = NULL;
			right_child = NULL;
		}
	};

private:
	void insert(IntPoint2D new_data, double new_score, Node* &tree_root) {
		if (tree_root == NULL) {
			tree_root = new Node(new_data, new_score);
		}
		else if (tree_root->score < new_score) {
			insert(new_data, new_score, tree_root->right_child);
		}
		else {
			insert(new_data, new_score, tree_root->left_child);
		}
	}
public:
	Node * root;
	IntPoint2DBinaryTree() {
		root = NULL;
	}

	IntPoint2DBinaryTree(Node * tree_root) {
		root = tree_root;
	}

	void add(IntPoint2D new_data, double new_score) {
		insert(new_data, new_score, root);
	}

	IntPoint2DBinaryTree getLeftChild() {
		return IntPoint2DBinaryTree(root->left_child);
	}
	IntPoint2DBinaryTree getRightChild() {
		return IntPoint2DBinaryTree(root->right_child);
	}
	IntPoint2D getData() {
		return root->data;
	}
};

bool pointInBox(
		// point to check
		DoublePoint3D point,

		// Box Point 1, should be min_x , min_y, min_z, of box
		double b1_x,
		double b1_y,
		double b1_z,

		// Box Point 2, should be max_x , max_y, max_z, of box 
		double b2_x,
		double b2_y,
		double b2_z) {

	return (
		point.x >= b1_x & point.x <= b2_x &
		point.y >= b1_y & point.y <= b2_y &
		point.z >= b1_z & point.z <= b2_z
		);
}

DoublePoint3D calculateDeParameterizedPoint(
		// the parameterized points
		int px,
		int py,

		// midpoint values
		double m_x,
		double m_y,
		double m_z,

		// X_vector, change in vector over the x direction in new mapping
		double xv_x,
		double xv_y,
		double xv_z,

		// Y_vector, change in vector over the y direction in new mapping
		double yv_x,
		double yv_y,
		double yv_z) {
	return DoublePoint3D(
		(px * xv_x + py * yv_x + m_x),
		(px * xv_y + py * yv_y + m_y),
		(px * xv_z + py * yv_z + m_z)
	);
}

double calculateArea(
		double p1x, double p1y, double p1z,
		double p2x, double p2y, double p2z) {
	return std::abs((p2x-p1x) * (p2y-p1y) * (p2z-p1z));
}

void inOrderPerformSampling(
		// file path to ordered slices
		VolumePkg volume,

		// the current tree
		IntPoint2DBinaryTree tree,

		// the parameterized points
		int px, int py,

		// midpoint values
		double m_x, double m_y, double m_z,

		// X_vector, change in vector over the x direction in new mapping
		double xv_x, double xv_y, double xv_z,

		// Y_vector, change in vector over the y direction in new mapping
		double yv_x, double yv_y, double yv_z,

		// Offset for the bounds of the output image
		int min_x, int min_y
		) {

	// peform sampling on values less than current node
	if (tree.root->left_child != NULL)
		inOrderPerformSampling(
			volume,
			tree.getLeftChild(),
			px, py,
			m_x, m_y, m_z,
			xv_x, xv_y, xv_z,
			yv_x, yv_y, yv_z,
			min_x, min_y);


	// get the parameterized points from the root node
	IntPoint2D parameterized_point = tree.root->data;

	// calcluate the deparameterized point
	DoublePoint3D deparameterized_point = 
		calculateDeParameterizedPoint(
			parameterized_point.x, parameterized_point.y,
			m_x, m_y, m_z,
			xv_x, xv_y, xv_z,
			yv_x, yv_y, yv_z);

	//
	// Load the correct slices and point values
	//

	// check that the loaded slices are correct
	int lower_slice_index_check = floor(deparameterized_point.z);
	if (lower_slice_index_check != lower_slice_index) {
		
		// check if we can just make the upper slice the lower slice
		if (lower_slice_index_check == lower_slice_index + 1) {

			// [CHECK] make sure this doesnt just create 2 pointers
			//         to the new upper_slice
			lower_slice = upper_slice;
			upper_slice = volume.getSliceAtIndex(lower_slice_index_check+1);
		}

		// if not just load both required slices
		else {
			lower_slice = volume.getSliceAtIndex(lower_slice_index_check);
			upper_slice = volume.getSliceAtIndex(lower_slice_index_check+1);
		}

		// reset the index
		lower_slice_index = lower_slice_index_check;
	}

	int down = floor(deparameterized_point.z);
	int left = floor(deparameterized_point.x);
	int back = floor(deparameterized_point.y);

	int up      = down + 1;
	int right   = left + 1;
	int forward = back + 1;

	// think of a 2 x 2 rubix cube pointing forwards
	double up_left_back       = upper_slice.at<double>(left, back);
	double up_left_forward    = upper_slice.at<double>(left, forward);
	double up_right_back      = upper_slice.at<double>(right, back);
	double up_right_forward   = upper_slice.at<double>(right, forward);

	double down_left_back     = lower_slice.at<double>(left, back);
	double down_left_forward  = lower_slice.at<double>(left, forward);
	double down_right_back    = lower_slice.at<double>(right, back);
	double down_right_forward = lower_slice.at<double>(right, forward);

	// calculate the areas
	double up_left_back_area       = 
		calculateArea(
			right, forward, down,
			deparameterized_point.x,
			deparameterized_point.y,
			deparameterized_point.z);
	double up_left_forward_area    = 
		calculateArea(
			right, back, down,
			deparameterized_point.x,
			deparameterized_point.y,
			deparameterized_point.z);
	double up_right_back_area      = 
		calculateArea(
			left, forward, down,
			deparameterized_point.x,
			deparameterized_point.y,
			deparameterized_point.z);
	double up_right_forward_area   = 
		calculateArea(
			left, back, down,
			deparameterized_point.x,
			deparameterized_point.y,
			deparameterized_point.z);

	double down_left_back_area     = 
		calculateArea(
			right, forward, up,
			deparameterized_point.x,
			deparameterized_point.y,
			deparameterized_point.z);
	double down_left_forward_area  = 
		calculateArea(
			right, back, up,
			deparameterized_point.x,
			deparameterized_point.y,
			deparameterized_point.z);
	double down_right_back_area    = 
		calculateArea(
			left, forward, up,
			deparameterized_point.x,
			deparameterized_point.y,
			deparameterized_point.z);
	double down_right_forward_area = 
		calculateArea(
			left, back, up,
			deparameterized_point.x,
			deparameterized_point.y,
			deparameterized_point.z);

	// create sampled value
	// place sample value in the output file
	double sampled_value = 
		up_left_back * up_left_back_area +
		up_left_forward * up_left_forward_area +
		up_right_back * up_right_back_area +
		up_right_forward * up_right_forward_area +
		down_left_back * down_left_back_area +
		down_left_forward * down_left_forward_area +
		down_right_back * down_right_back_area +
		down_right_forward * down_right_forward_area;

	output_slice.at<double>(
			parameterized_point.x - min_x,
			parameterized_point.y - min_y) = sampled_value;

	// peform sampling on values greater than current node
	if (tree.root->right_child != NULL)
		inOrderPerformSampling(
			volume,
			tree.getRightChild(),
			px, py,
			m_x, m_y, m_z,
			xv_x, xv_y, xv_z,
			yv_x, yv_y, yv_z,
			min_x, min_y);
}

void resliceGivenXY(
		// file path to ordered slices
		VolumePkg volume,

		// midpoint values
		double m_x,double m_y,double m_z,

		// X_vector, change in vector over the x direction in new mapping
		double xv_x,double xv_y,double xv_z,

		// Y_vector, change in vector over the y direction in new mapping
		double yv_x,double yv_y,double yv_z,

		// Box Point 1, should be min_x , min_y, min_z, of box
		double b1_x,double b1_y,double b1_z,

		// Box Point 2, should be max_x , max_y, max_z, of box 
		double b2_x,double b2_y,double b2_z) {

	// build a list of paramterized points to map sorted on z coordinate
	// before paramterization
	IntPoint2DBinaryTree points_to_calc_list = IntPoint2DBinaryTree();

	//
	// expand in x y z direction getting points to calculate
	// save the max and min values of parameterized x and y
	// so that we can save the mat as small as needed
	//

	//setup vars needed
	int min_px = 0;
	int min_py = 0;

	int max_px = 0;
	int max_py = 0;

	int px = 0;
	int py = 0;

	//check x up
	px = 0;
	while(true) {
		bool points_in_px = false;

		// check y up
		py = 0;
		while (true) {
			// calculate parameterized point
			DoublePoint3D point =
				calculateDeParameterizedPoint(
					px, py,
					m_x, m_y, m_z,
					xv_x, xv_y, xv_z,
					yv_x, yv_y, yv_z);
			if (pointInBox(point, b1_x, b1_y, b1_z, b2_x, b2_y, b2_z)) {
				if (min_px > px) min_px = px;
				if (min_py > py) min_py = py;
				if (max_px < px) max_px = px;
				if (max_py < py) max_py = py;

				points_to_calc_list.add(IntPoint2D(px, py), point.z);

				points_in_px = true;
			}
			else break;
			py += 1;
		}

		// check y down
		py = -1;
		while (true) {
			// calculate parameterized point
			DoublePoint3D point =
				calculateDeParameterizedPoint(
					px, py,
					m_x, m_y, m_z,
					xv_x, xv_y, xv_z,
					yv_x, yv_y, yv_z);
			if (pointInBox(point, b1_x, b1_y, b1_z, b2_x, b2_y, b2_z)) {
				if (min_px > px) min_px = px;
				if (min_py > py) min_py = py;
				if (max_px < px) max_px = px;
				if (max_py < py) max_py = py;

				points_to_calc_list.add(IntPoint2D(px, py), point.z);

				points_in_px = true;
			}
			else break;
			py -= 1;
		}


		//check that there was a value in this x
		if (!points_in_px) {break;}

		// continue searching up
		px += 1;
	}

	// check x down
	px = -1;
	while(true) {
		bool points_in_px = false;

		// check y up
		py = 0;
		while (true) {
			// calculate parameterized point
			DoublePoint3D point =
				calculateDeParameterizedPoint(
					px, py,
					m_x, m_y, m_z,
					xv_x, xv_y, xv_z,
					yv_x, yv_y, yv_z);
			if (pointInBox(point, b1_x, b1_y, b1_z, b2_x, b2_y, b2_z)) {
				if (min_px > px) min_px = px;
				if (min_py > py) min_py = py;
				if (max_px < px) max_px = px;
				if (max_py < py) max_py = py;

				points_to_calc_list.add(IntPoint2D(px, py), point.z);

				points_in_px = true;
			}
			else break;
			py += 1;
		}

		// check y down
		py = -1;
		while (true) {
			// calculate parameterized point
			DoublePoint3D point =
				calculateDeParameterizedPoint(
					px, py,
					m_x, m_y, m_z,
					xv_x, xv_y, xv_z,
					yv_x, yv_y, yv_z);
			if (pointInBox(point, b1_x, b1_y, b1_z, b2_x, b2_y, b2_z)) {
				if (min_px > px) min_px = px;
				if (min_py > py) min_py = py;
				if (max_px < px) max_px = px;
				if (max_py < py) max_py = py;

				points_to_calc_list.add(IntPoint2D(px, py), point.z);

				points_in_px = true;
			}
			else break;
			py -= 1;
		}


		//check that there was a value in this x
		if (!points_in_px) {break;}

		// continue searching up
		px -= 1;
	}

	// initialize the output file
	output_slice = cv::Mat(max_px-min_px,max_py-min_py,cv::DataType<double>::type);

	// perform an inorder traversal of the tree containing parameterized
	// values and return the output

	inOrderPerformSampling(
		volume,
		points_to_calc_list,
		px, py,
		m_x, m_y, m_z,
		xv_x, xv_y, xv_z,
		yv_x, yv_y, yv_z,
		min_px, min_py);
}

int main() {
	VolumePkg vpkg = VolumePkg("/Users/david/Desktop//volumepkg/");

}
