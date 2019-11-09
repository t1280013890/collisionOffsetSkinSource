#include "computeWeight.h"
#include <maya/MFnDagNode.h>
#include <maya/MGlobal.h>
#include <maya/MDagPath.h>
#include <maya/MMatrix.h>
DagTree::~DagTree()
{
	for (auto&item : dagMap)
		delete item.second;
}
DagTree::DagTree(DagTree &&t)
{
	for (auto &item : dagMap)
		delete item.second;
	dagMap.clear();
	roots = std::move(t.roots);
	dagMap = std::move(t.dagMap);
	dagVector = std::move(t.dagVector);
}
void DagTree::operator=(DagTree&&t)
{
	for (auto &item : dagMap)
		delete item.second;
	dagMap.clear();
	roots = std::move(t.roots);
	dagMap = std::move(t.dagMap);
	dagVector = std::move(t.dagVector);
}
void DagTree::appendItem(const MObject&obj, int index)
{
	MFnDagNode fn(obj);
	std::string name(fn.partialPathName().asChar());
	if (dagMap.find(name) == dagMap.end())
	{
		MDagPath dag;
		fn.getPath(dag);
		MMatrix mat = dag.inclusiveMatrix();
		DagTreeItem *item = new DagTreeItem(obj, index, MPoint(mat[3][0], mat[3][1], mat[3][2]));
		dagMap[name] = item;
	}
}
void DagTree::build()
{
	dagVector.clear();
	for (auto &item : dagMap)
	{
		std::string parentName((MFnDagNode(MFnDagNode(item.second->node).parent(0)).partialPathName().asChar()));
		auto &pos = dagMap.find(parentName);
		if (pos == dagMap.end())
			roots.push_back(item.second);
		else
		{
			pos->second->children.push_back(item.second);
			item.second->parent = pos->second;
		}
		if (dagVector.size() <= item.second->index)
			dagVector.resize(item.second->index + 1, NULL);
		dagVector[item.second->index] = item.second;
	}
	int dagSize = dagVector.size();
	for (int i = 0; i < dagSize; ++i)
	{
		auto &itemPtrI = dagVector[i];
		if (itemPtrI)
		{

			for (int j = 0; j < dagSize; ++j)
			{
				auto &itemPtrJ = dagVector[j];
				if (itemPtrJ)
				{


					float distance = itemPtrI->point.distanceTo(itemPtrJ->point);
					if (distance < 0.01&&distance >= -0.01&&i != j)
					{

						auto parent = itemPtrJ->parent;
						std::vector<DagTreeItem*> *parChilP;
						if (parent)
							parChilP = &parent->children;
						else parChilP = &roots;
						for (auto &it = parChilP->begin(), end = parChilP->end(); it != end; ++it)
						{
							if ((*it)->index == dagVector[j]->index)
							{
								parChilP->erase(it);
								break;
							}
						}
						for (auto &child : itemPtrJ->children)
						{
							parChilP->push_back(child);
							child->parent = parent;
						}
						auto &pos = dagMap.find(MFnDagNode(dagVector[j]->node).partialPathName().asChar());
						delete(pos->second);
						dagMap.erase(pos);
						dagVector[j] = NULL;
					}
				}
			}
		}
	}
	makeJointPairVector();
}
void DagTree::makeChildJointPair(std::vector<DagTreeItem*> &items)
{
	for (auto &item : items)
	{
		if (item->children.empty())
		{
			if (!(item->parent))
				jointPair.push_back(JointPair(item, item));
		}
		else
		{
			for (auto &child : item->children)
				jointPair.push_back(JointPair(item, child));
			makeChildJointPair(item->children);
		}
	}
}
void DagTree::makeJointPairVector()
{
	jointPair.clear();
	makeChildJointPair(roots);
}
int DagTree::getNearestItems(const MPoint &p, DagTreeItem** item, float *dis, int size)
{
	for (int i = 0; i < size; ++i)
	{
		item[i] = NULL;
		dis[i] = FLT_MAX;
	}
	int number = 0;
	for (const auto&ditem : dagVector)
	{
		if (ditem)
		{
			float distance = ditem->point.distanceTo(p);
			for (int i = 0; i < size; ++i)
			{
				if (distance < dis[i])
				{
					for (int j = size - 1; j > i; --j)
					{
						item[j] = item[j - 1];
						dis[j] = dis[j - 1];
					}
					item[i] = ditem;
					dis[i] = distance;
					number += 1;
					break;
				}
			}
		}
	}
	return number;
}

DagTreeItem * DagTree::getNearestPairItem(const MPoint &p)
{
	int dagVectorSize = dagVector.size();
	std::vector<MVector> ray(dagVectorSize);
	std::vector<float> rayLength(dagVectorSize);
	for (int i = 0; i < dagVectorSize; ++i)
	{
		if (dagVector[i])
		{
			ray[i] = dagVector[i]->point - p;
			rayLength[i] = ray[i].length();
		}
	}
	DagTreeItem *minItem;
	float minDis = FLT_MAX;
	for (auto &jp : jointPair)
	{
		if (jp.jointA == jp.jointB)
		{
			float lineDis = jp.jointA->point.distanceTo(p);
			if (lineDis < minDis)
			{
				minDis = lineDis;
				minItem = jp.jointA;
			}
			continue;
		}
		auto &rayA = ray[jp.jointA->index];
		auto &rayB = ray[jp.jointB->index];

		if ((rayA*jp.vct)*(rayB*jp.vct) < 0)
		{
			MVector normal = rayA ^ rayB^jp.vct;
			float normalLength = normal.length();
			if (normalLength < 0.0001)
			{
				if (rayLength[jp.jointA->index] < rayLength[jp.jointB->index])
					minItem = jp.jointA;
				else
					minItem = jp.jointB;
				break;
			}
			float lineDis = abs(normal*rayA / normalLength);
			if (lineDis < minDis)
			{
				minDis = lineDis;
				if (rayLength[jp.jointA->index] < rayLength[jp.jointB->index])
					minItem = jp.jointA;
				else
					minItem = jp.jointB;
			}
		}
		else
		{
			if (rayLength[jp.jointA->index] < rayLength[jp.jointB->index] && rayLength[jp.jointA->index] < minDis)
			{
				minDis = rayLength[jp.jointA->index];
				minItem = jp.jointA;
			}
			else if (rayLength[jp.jointB->index] < rayLength[jp.jointA->index] && rayLength[jp.jointB->index] < minDis)
			{
				minDis = rayLength[jp.jointB->index];
				minItem = jp.jointB;
			}
		}
	}
	return minItem;
}


void DagTree::print()
{
	for (auto &item : dagMap)
	{
		MGlobal::displayInfo(MString(item.first.c_str()) + "   " + item.second->index);
	}
	for (int i = 0; i < dagVector.size(); ++i)
	{
		if (dagVector[i])
			MGlobal::displayInfo(MString() + i + "     " + MFnDagNode(dagVector[i]->node).partialPathName());
	}
	MGlobal::displayInfo("--------------");
	for (auto &item : roots)
	{
		printItem(item, 0);
	}
	for (auto &pair : jointPair)
	{
		MGlobal::displayInfo(MFnDagNode(pair.jointA->node).partialPathName() + "    " + MFnDagNode(pair.jointB->node).partialPathName());
	}
}
void DagTree::printItem(DagTreeItem*item, int n)
{
	MString space;
	for (int i = 0; i < n; ++i)
		space += "     ";
	MGlobal::displayInfo(space + MFnDagNode(item->node).partialPathName() + "    " + item->index);
	for (auto &child : item->children)
		printItem(child, n + 1);
}