#ifndef KDTREE_H
#define KDTREE_H

// node used on the KdTree structure
struct Node{
	std::vector<float> point;
	int id;
	Node* left;
	Node* right;

	Node(std::vector<float> arr, int setId){
		point = arr;
		id = setId;
		left = NULL;
		right = NULL;
	}
};

// structure used to organize point in a 3-dimensional space
struct KdTree{
	Node* root;

	KdTree(){
		root = NULL;
	}

	void insert(std::vector<float> point, int id){
		insertNode(root, 0, point, id);
	}

	// recursive call to insert a 3D point in the tree
	void insertNode(Node *&node, int depth, std::vector<float> point, int id){
		if(node == NULL){
			node = new Node(point, id);
		}else{
			int aux = depth%3;
			if(point[aux] < node->point[aux]){
				insertNode(node->left, depth+1, point, id);
			}else{
				insertNode(node->right, depth+1, point, id);
			}
		}
	}

	// return a list of point ids in the tree that are within distance of target
	std::vector<int> search(std::vector<float> target, float distanceTol){
		std::vector<int> ids;
		searchNodes(target, root, 0, distanceTol, ids);
		return ids;
	}
	
	void searchNodes(std::vector<float> target, Node* node, int depth, float distanceTol, std::vector<int>& ids){
		if(node!=NULL){
			if(node->point[0] >= (target[0]-distanceTol) && node->point[0] <= (target[0]+distanceTol)
				&& node->point[1] >= (target[1]-distanceTol) && node->point[1] <= (target[1]+distanceTol)
				&& node->point[2] >= (target[2]-distanceTol) && node->point[2] <= (target[2]+distanceTol)){
				float distance = sqrt((node->point[0]-target[0])*(node->point[0]-target[0])+(node->point[1]-target[1])*(node->point[1]-target[1])+(node->point[2]-target[2])*(node->point[2]-target[2]));
				if(distance <= distanceTol){
					ids.push_back(node->id);
				}
			}

			int aux = depth%3;
			if((target[aux]-distanceTol) < node->point[aux]){
				searchNodes(target, node->left, depth+1, distanceTol, ids);
			}
			if((target[aux]+distanceTol) > node->point[aux]){
				searchNodes(target, node->right, depth+1, distanceTol, ids);
			}
		}
	}
};

#endif