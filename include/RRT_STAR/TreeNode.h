#ifndef TreeNode_H
#define TreeNode_H

#include <string>
#include <vector>
#include <geometry_msgs/PoseStamped.h>

class TreeNode
{
    private:
        std::vector <int> point;
        TreeNode *parent;
        std::vector<TreeNode *> children;
        double cost; // add cost parameter

        int countNodesRec(TreeNode *root, int& count);

    public:
        TreeNode();
		TreeNode(std::vector <int> point_, double cost_ = 0.0);
		~TreeNode();
		bool hasChildren();
        void appendChild(TreeNode *child);
        void setParent(TreeNode *parent);

        bool hasChildren() const { return children.size() > 0; }
        bool hasParent(); 

        TreeNode* getParent();
        TreeNode* getChild(int pos);

        int childrenNumber(); 

		std::vector <int> getNode();
        double getCost() const; // Add getter for cost  
        void setCost(double new_cost_); // Add setter for cost

		void printNode();
		void printTree();

		TreeNode* nearNode(TreeNode* node1, TreeNode* node2);
		TreeNode* neast(TreeNode *root);

        std::vector <std::vector <int>> returnSolution();
};

#endif
