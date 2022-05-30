#include "rayAccelerator.h"
#include "macros.h"

using namespace std;

BVH::BVHNode::BVHNode(void) {}

void BVH::BVHNode::setAABB(AABB& bbox_) { this->bbox = bbox_; }

void BVH::BVHNode::makeLeaf(unsigned int index_, unsigned int n_objs_) {
	this->leaf = true;
	this->index = index_; 
	this->n_objs = n_objs_; 
}

void BVH::BVHNode::makeNode(unsigned int left_index_) {
	this->leaf = false;
	this->index = left_index_; 
			//this->n_objs = n_objs_; 
}


BVH::BVH(void) {}

int BVH::getNumObjects() { return objects.size(); }


void BVH::Build(vector<Object *> &objs) {

		
			BVHNode *root = new BVHNode();

			Vector min = Vector(FLT_MAX, FLT_MAX, FLT_MAX), max = Vector(-FLT_MAX, -FLT_MAX, -FLT_MAX);
			AABB world_bbox = AABB(min, max);

			for (Object* obj : objs) {
				AABB bbox = obj->GetBoundingBox();
				world_bbox.extend(bbox);
				objects.push_back(obj);
			}
			world_bbox.min.x -= EPSILON; world_bbox.min.y -= EPSILON; world_bbox.min.z -= EPSILON;
			world_bbox.max.x += EPSILON; world_bbox.max.y += EPSILON; world_bbox.max.z += EPSILON;
			root->setAABB(world_bbox);
			nodes.push_back(root);
			build_recursive(0, objects.size(), root); // -> root node takes all the 
		}

void BVH::build_recursive(int left_index, int right_index, BVHNode *node) {

		int num_objs = (right_index - left_index);

		if (num_objs <= Threshold) {
			node->makeLeaf(left_index, num_objs);
		}
		else {

			AABB aabb = node->getAABB();

			int dim = -1;

			Vector diff = aabb.max - aabb.min;

			float maxDim = std::max(std::max(diff.x, diff.y), diff.z);

			if (maxDim == diff.x ) {
				dim = 0;
			}
			else if (maxDim == diff.y) {
				dim = 1;
			}
			else {
				dim = 2;
			}

			Comparator cmp;
			cmp.dimension = dim;

			sort(objects.begin() + left_index, objects.begin() + right_index, cmp);

			float mid = (aabb.max.getAxisValue(dim) + aabb.min.getAxisValue(dim)) * 0.5;

			int split_index;

			//Make sure that neither left nor right is completely empty
			if (objects[left_index]->getCentroid().getAxisValue(dim) > mid ||
				objects[right_index - 1]->getCentroid().getAxisValue(dim) <= mid) {
				mid = 0;
				for (split_index = left_index; split_index < right_index; split_index++) {
					mid += objects[split_index]->getCentroid().getAxisValue(dim);
				}
				mid /= num_objs;
			}

			//Split intersectables objects into left and right by finding a split_index
			if (objects[left_index]->getCentroid().getAxisValue(dim) > mid ||
				objects[right_index - 1]->getCentroid().getAxisValue(dim) <= mid) {

				split_index = left_index + Threshold;
			}
			else {
				for (split_index = left_index; split_index < right_index; split_index++) {
					if (objects[split_index]->getCentroid().getAxisValue(dim) > mid) {
						break;
					}
				}
			}

			//Create two new nodes, leftNode and rightNode and assign bounding boxes
			Vector min_right, min_left;
			min_right = Vector(FLT_MAX, FLT_MAX, FLT_MAX);
			min_left = min_right;
			Vector max_right, max_left;
			max_right = Vector(-FLT_MAX, -FLT_MAX, -FLT_MAX);
			max_left = max_right;

			AABB leftBox = AABB(min_left, max_left);
			AABB rightBox = AABB(min_right, max_right);

			for (int left = left_index; left < split_index; left++) {
				leftBox.extend(objects[left]->GetBoundingBox());
			}

			for (int right = split_index; right < right_index; right++) {
				rightBox.extend(objects[right]->GetBoundingBox());
			}


			// Create two new nodes, leftNode and rightNode and assign bounding boxes
			BVHNode* leftNode = new BVHNode();
			BVHNode* rightNode = new BVHNode();


			leftNode->setAABB(leftBox);

			rightNode->setAABB(rightBox);

			//Initiate current node as an interior node with leftNode and rightNode as children: 
			node->makeNode(nodes.size());

			//Push back leftNode and rightNode into nodes vector 
			nodes.push_back(leftNode);
			nodes.push_back(rightNode);

			build_recursive(left_index, split_index, leftNode);
			build_recursive(split_index, right_index, rightNode);

		}	
		
	}

bool BVH::Traverse(Ray& ray, Object** hit_obj, Vector& hit_point) {
			float tmp;
			float tmin = FLT_MAX;  //contains the closest primitive intersection
			bool hit = false;

			BVHNode* currentNode = nodes[0];

			//PUT YOUR CODE HERE
			
			return(false);
	}

bool BVH::Traverse(Ray& ray) {  //shadow ray with length
			float tmp;

			double length = ray.direction.length(); //distance between light and intersection point
			ray.direction.normalize();

			//PUT YOUR CODE HERE

			return(false);
	}		
