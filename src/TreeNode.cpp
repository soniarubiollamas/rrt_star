#include <string>
#include <vector>
#include <iostream>
#include <geometry_msgs/PoseStamped.h>
#include "RRT_STAR/TreeNode.h"

TreeNode::TreeNode(){
    std::cout << "Constructor" << std::endl;
	parent = NULL;
    cost = 0.0; // Initialize cost to 0.0
}

TreeNode::TreeNode(std::vector <int> point_, double cost_){
    // std::cout << "Constructor" << std::endl;
    // std::cout << "point_:" << point_[0] << ", " << point_[1] << std::endl;
    point = point_;
    parent = NULL;
    cost = cost_; // Initialize cost to cost_
}

TreeNode::~TreeNode(){
    TreeNode *child = NULL;
    TreeNode *parent = this;
    // std::cout << "Destructor" << std::endl;
    for(int it = 0; it < parent->childrenNumber(); it++){
        child = parent->getChild(it);
        if(child->hasChildren()){
            child->~TreeNode();
        }
        else{
            child->point.clear();
            child->children.clear();
            child->parent = NULL;
            //delete child;
        }
    }    
    this->point.clear();
    this->children.clear();
    this->parent = NULL;    
} 

bool TreeNode::hasChildren()
{
    if(children.size() > 0)
        return true;
    else
        return false;
}

int TreeNode::countNodesRec(TreeNode *root, int& count)
{   
    TreeNode *parent = root;
    TreeNode *child = NULL;

    for(int it = 0; it < parent->childrenNumber(); it++)
    {   
        child = parent->getChild(it);
        count++;
        if(child->childrenNumber() > 0)
        {   
            countNodesRec(child, count);
        } 
    }

    return count;
}

void TreeNode::appendChild(TreeNode *child)
{       
    child->setParent(this);
    children.push_back(child);
}

void TreeNode::setParent(TreeNode *theParent)
{
    parent = theParent;
    // this->parent = theParent; ?? try this if above doesnt work
}

bool TreeNode::hasParent()
{
    if(parent != NULL)
        return true;
    else 
        return false;
}

TreeNode * TreeNode::getParent()
{
    return parent;
}

TreeNode* TreeNode::getChild(int pos)
{   
    if(children.size() < pos)
        return NULL;
    else
        return children[pos];
}

int TreeNode::childrenNumber()
{
    return children.size();
}

std::vector <int> TreeNode::getNode() 
{
    return point;
}
double TreeNode::getCost() const
{
    return cost;
}
void TreeNode::setCost(double new_cost)
{
    cost = new_cost;
}

void TreeNode::printNode() 
{
	std::cout << "Node: (" << point[0] << "," << point[1] << ")." << std::endl;
}

void TreeNode::printTree()
{   
    TreeNode *child = NULL;
    
    std::cout << "Parent node: (" << point[0] << "," << point[1] << ")." << std::endl;
	
    for(int it = 0; it < children.size(); it++)
    {   
        std::cout << "    Child node: (" << children[it]->point[0] << "," << children[it]->point[1] << ")." << std::endl;
    }
    for(int it = 0; it < children.size(); it++)
    {   
        children[it]->printTree();
    }
}

TreeNode* TreeNode::nearNode(TreeNode* node1, TreeNode* node2){
	std::vector <int> pos1 = node1->getNode();	
	std::vector <int> pos2 = node2->getNode();	
	int distance1 = sqrt((pos1[0]-point[0])*(pos1[0]-point[0]) + (pos1[1]-point[1])*(pos1[1]-point[1]));
	int distance2 = sqrt((pos2[0]-point[0])*(pos2[0]-point[0]) + (pos2[1]-point[1])*(pos2[1]-point[1]));
	if (distance1 < distance2)
		return node1;
	else
		return node2;
}

TreeNode* TreeNode::neast(TreeNode *root){

    TreeNode *parent = root;
    TreeNode *child = NULL;
    TreeNode *shortest = parent;

    for(int it = 0; it < parent->childrenNumber(); it++)
    {   
        child = parent->getChild(it);
	    shortest = nearNode(shortest,neast(child));
    }
    return shortest;
}


// Function to find nodes in the tree that are within a certain distance from a given node
std::vector<TreeNode*> TreeNode::near(TreeNode* target_node,TreeNode* start_node , double max_dist) {
    std::vector<TreeNode*> near_nodes;
    std::vector<TreeNode*> all_nodes = start_node->getAllNodes();
    double x1, x2, y1, y2,distance;
    for (int it = 0; it < all_nodes.size(); it++) {
        x1 = all_nodes[it]->point[0];
        x2 = target_node->point[0];
        y1 = all_nodes[it]->point[1];
        y2 = target_node->point[1];        
        double distance = sqrt((x1-x2)*(x1-x2) + (y1-y2)*(y1-y2))*0.032;        
        if (distance <= max_dist) {
            near_nodes.push_back(all_nodes[it]);
                  }
    } 

    return near_nodes;
}

void TreeNode::removeChild(TreeNode* child) {
        auto it = std::remove(children.begin(), children.end(), child);
        children.erase(it, children.end());
    }
    
std::vector<TreeNode*> TreeNode::getAllNodes() {
    std::vector<TreeNode*> all_nodes;

    // Add the current node to the vector
    all_nodes.push_back(this);

    // Recursively add all children's nodes to the vector
    for (int it = 0; it < children.size(); it++) {
        std::vector<TreeNode*> child_nodes = children[it]->getAllNodes();
        all_nodes.insert(all_nodes.end(), child_nodes.begin(), child_nodes.end());
    }

    return all_nodes;
}

std::vector <std::vector <int>> TreeNode::returnSolution(){

	std::vector <std::vector <int>> solution;
	TreeNode *node = this;
//	std::cout << "Start returning the solution" << std::endl;
	while(node->hasParent()){
		solution.push_back(node->getNode());
		if(node->hasParent())
			node = node->getParent();
//		node->printNode();
	};
//	std::cout << "Finish returning the solution" << std::endl;
	return solution;
}


