#pragma once
#include <maya/MObject.h>
#include <vector>
#include <unordered_map>
#include <string>
#include <maya/MPoint.h>
#include <maya/MGlobal.h>
class DagTreeItem;
class JointPair;
class DagTree
{
public:
	std::vector<DagTreeItem*> roots;
	std::unordered_map<std::string, DagTreeItem*> dagMap;
	std::vector<DagTreeItem*> dagVector;
	std::vector<JointPair> jointPair;
	DagTree(){}
	~DagTree();
	DagTree(DagTree&&t);
	void operator=(DagTree&&t);
	void appendItem(const MObject &obj, int index);
	void build();
	void makeJointPairVector();

	void print();
	int getNearestItems(const MPoint &p, DagTreeItem**item, float *dis, int size);
	DagTreeItem* getNearestPairItem(const MPoint &p);
private:
	void printItem(DagTreeItem*, int);
	void makeChildJointPair(std::vector<DagTreeItem*> &items);
};
class DagTreeItem
{
public:
	MObject node;
	DagTreeItem *parent;
	int index;
	MPoint point;

	
	std::vector<DagTreeItem*> children;
	DagTreeItem() { parent = NULL; }
	DagTreeItem(const MObject &obj, int indexV, MPoint pointV, DagTreeItem *parentP = NULL) :node(obj), index(indexV), point(pointV), parent(parentP) {}




};
class JointPair
{
public:
	DagTreeItem *jointA;
	DagTreeItem *jointB;
	MVector vct;
	JointPair():jointA(NULL),jointB(NULL){}
	JointPair(DagTreeItem *a, DagTreeItem *b) :jointA(a), jointB(b)
	{
		vct = b->point - a->point;
	}
};