#include "skinCmd.h"
#include <maya/MArgDatabase.h>
#include <maya/MGlobal.h>
#include <maya/MSelectionList.h>
#include <maya/MFnDependencyNode.h>
#include <maya/MPlug.h>
#include <maya/MPxNode.h>
#include <maya/MArrayDataBuilder.h>
#include <maya/MFnMatrixData.h>
#include <maya/MItDependencyNodes.h>
#include <maya/MEventMessage.h>
#include <maya/MFnToolContext.h>
#include <maya/MDagPath.h>
#include <maya/MFnDagNode.h>
#include <vector>
#include <maya/MItGeometry.h>
#include <maya/MFnSkinCluster.h>
#include <maya/MFnMesh.h>
#include <maya/MItMeshFaceVertex.h>
#include <maya/MItMeshVertex.h>
#include <maya/MItMeshPolygon.h>
#include <Eigen/Dense>
#include "computeWeight.h"
#include "skin.h"
skinManipContext* collisionOffsetSkinCmd::manip(NULL);
MStatus collisionOffsetSkinCmd::doIt(const MArgList&args)
{

	MSyntax syntax = createSyntax();
	MArgDatabase argparser(syntax, args);
	if (argparser.isFlagSet("refreshManipulators"))
	{
		if (manip)
			manip->refreshManipulators();
		return MS::kSuccess;
	}
	if (argparser.isFlagSet("reBindAll"))
	{
		rebindAll();
		return MS::kSuccess;
	}
	MSelectionList objSlls;
	argparser.getObjects(objSlls);
	if (objSlls.length() < 1)
		return MS::kFailure;
	MObject nodeObj;
	objSlls.getDependNode(0, nodeObj);
	MFnDependencyNode nodeFn(nodeObj);
	if (nodeFn.typeName() != "collisionOffsetSkin")
		return MS::kSuccess;
	if (argparser.isFlagSet("reBind"))
	{
		collisionOffsetSkin *skinNode = (collisionOffsetSkin*)nodeFn.userNode();
		skinNode->needRefresh = true;
	}
	else if (argparser.isFlagSet("alignParent"))
	{
		int index;
		argparser.getFlagArgument("alignParent", 0, index);
		collisionOffsetSkin *skinNode = (collisionOffsetSkin*)nodeFn.userNode();
		MPlug infPlug(MPlug(skinNode->thisMObject(), skinNode->influenceInfoP).elementByLogicalIndex(index));
		infPlug.child(2).setValue(MFnMatrixData().create(MMatrix()));
	}
	else if (argparser.isFlagSet("alignChild"))
	{
		int index;
		argparser.getFlagArgument("alignChild", 0, index);
		collisionOffsetSkin *skinNode = (collisionOffsetSkin*)nodeFn.userNode();
		MPlug infPlug(MPlug(skinNode->thisMObject(), skinNode->influenceInfoP).elementByLogicalIndex(index));
		MMatrix bindMat = MFnMatrixData(infPlug.child(0).asMObject()).matrix();
		MMatrix parentBindMat = MFnMatrixData(infPlug.child(1).asMObject()).matrix();
		infPlug.child(2).setValue(MFnMatrixData().create(bindMat *parentBindMat.inverse()));
	}

	else if (argparser.isFlagSet("initWeight"))
	{
		int smooth;
		if (argparser.isFlagSet("smoothWeight"))
			argparser.getFlagArgument("smoothWeight", 0, smooth);
		else
			smooth = 0;
#ifdef WITHCGAL
		initWeight(nodeObj, smooth);
#endif
	}
	else if (argparser.isFlagSet("smoothWeight"))
	{
		int smooth;
		argparser.getFlagArgument("smoothWeight", 0, smooth);
#ifdef WITHCGAL
		smoothWeight(nodeObj, smooth);
#endif
	}

	return MS::kSuccess;
}
#ifdef WITHCGAL
void collisionOffsetSkinCmd::initWeight(MObject &nodeObj, int smooth)
{


	MFnSkinCluster skinFn(nodeObj);
	MDagPathArray influenceObjs;
	int influNum = skinFn.influenceObjects(influenceObjs);
	MIntArray infIndex;

	DagTree influenceTree;
	for (int i = 0; i < influNum; ++i)
	{
		influenceTree.appendItem(influenceObjs[i].node(), i);
		infIndex.append(i);
	}
	influenceTree.build();
	
	MObjectArray outGeo;
	skinFn.getOutputGeometry(outGeo);
	if (outGeo.length() < 0)
		return;
	MItGeometry outGeoMit(outGeo[0]);
	MDagPath dag;
	MFnDagNode(outGeo[0]).getPath(dag);
	MObject compments;
	MSelectionList compmentSelect;
	int pointNum = outGeoMit.count();
	MDoubleArray weights(pointNum*influNum);
	int startIndex = 0;
	while (!outGeoMit.isDone())
	{
		DagTreeItem* item = influenceTree.getNearestPairItem(outGeoMit.position());
		weights[startIndex + item->index] = 1;
		compmentSelect.add(dag, outGeoMit.currentItem(), true);
		startIndex += influNum;
		outGeoMit.next();
	}
	compmentSelect.getDagPath(0, dag, compments);
	MDoubleArray newWeights = weights;



	Poly poly;
	mayaObjToPoly(outGeo[0], poly);

	std::vector<int> vertexList;
	for (int s = 0; s < smooth; ++s)
	{
		startIndex = 0;
		auto verticesBegin = poly.vertices_begin();
		for (auto itv = poly.vertices_begin(), itve = poly.vertices_end(); itv != itve; ++itv)
		{
			auto halfedgeCircu = itv->vertex_begin();
			auto halfedgeCircuB = itv->vertex_begin();
			vertexList.clear();
			do
			{
				vertexList.push_back(std::distance(verticesBegin, halfedgeCircu->opposite()->vertex()));
			} while (++halfedgeCircu != halfedgeCircuB);
			int l = vertexList.size();
			for (int i = 0; i < influNum; ++i)
			{
				float bufferWeight = newWeights[startIndex + i];
				if (bufferWeight > 0.01)
				{
					float addWeight = bufferWeight / l;
					for (int j = 0; j < l; ++j)
					{
						newWeights[vertexList[j] * influNum + i] += addWeight;
					}
				}
			}

			startIndex += influNum;
		}

		for (int i = 0; i < pointNum; ++i)
		{
			int index = i * influNum;
			float sum = 0;
			for (int j = 0; j < influNum; ++j)
			{
				sum += newWeights[index + j];
			}
			if (sum > 0.01)
			{
				float sum_inv = 1 / sum;
				for (int j = 0; j < influNum; ++j)
				{
					newWeights[index + j] *= sum_inv;
				}
			}
		}
		weights = newWeights;
	}
	skinFn.setWeights(dag, compments, infIndex, weights, false);
}

void collisionOffsetSkinCmd::smoothWeight(MObject &nodeObj, int smooth)
{
	MFnSkinCluster skinFn(nodeObj);
	MDagPathArray influenceObjs;
	int influNum = skinFn.influenceObjects(influenceObjs);
	MIntArray infIndex;

	for (int i = 0; i < influNum; ++i)
		infIndex.append(i);

	MObjectArray outGeo;
	skinFn.getOutputGeometry(outGeo);
	if (outGeo.length() < 0)
		return;
	MItGeometry outGeoMit(outGeo[0]);
	MDagPath dag;
	MFnDagNode(outGeo[0]).getPath(dag);
	MObject compments;
	MSelectionList compmentSelect;
	int pointNum = outGeoMit.count();
	MDoubleArray weights;
	while (!outGeoMit.isDone())
	{
		compmentSelect.add(dag, outGeoMit.currentItem(), true);
		outGeoMit.next();
	}
	compmentSelect.getDagPath(0, dag, compments);
	int startIndex = 0;
	skinFn.getWeights(dag, compments, infIndex, weights);
	MDoubleArray newWeights = weights;
	Poly poly;
	mayaObjToPoly(outGeo[0], poly);
	std::vector<int> vertexList;
	for (int s = 0; s < smooth; ++s)
	{
		startIndex = 0;
		auto verticesBegin = poly.vertices_begin();
		for (auto itv = poly.vertices_begin(), itve = poly.vertices_end(); itv != itve; ++itv)
		{
			auto halfedgeCircu = itv->vertex_begin();
			auto halfedgeCircuB = itv->vertex_begin();
			vertexList.clear();
			do
			{
				vertexList.push_back(std::distance(verticesBegin, halfedgeCircu->opposite()->vertex()));
			} while (++halfedgeCircu != halfedgeCircuB);
			int l = vertexList.size();
			for (int i = 0; i < influNum; ++i)
			{
				float bufferWeight = newWeights[startIndex + i];
				if (bufferWeight > 0.01)
				{
					float addWeight = bufferWeight / l;
					for (int j = 0; j < l; ++j)
					{
						newWeights[vertexList[j] * influNum + i] += addWeight;
					}
				}
			}
			startIndex += influNum;
		}

		for (int i = 0; i < pointNum; ++i)
		{
			int index = i * influNum;
			float sum = 0;
			for (int j = 0; j < influNum; ++j)
			{
				sum += newWeights[index + j];
			}
			if (sum > 0.01)
			{
				float sum_inv = 1 / sum;
				for (int j = 0; j < influNum; ++j)
				{
					newWeights[index + j] *= sum_inv;
				}
			}
		}
		weights = newWeights;
	}
	skinFn.setWeights(dag, compments, infIndex, weights, false);
}
#endif
MSyntax collisionOffsetSkinCmd::createSyntax()
{
	MSyntax syntax;
	syntax.addFlag("reb", "reBind", MSyntax::kBoolean);
	syntax.addFlag("rba", "reBindAll", MSyntax::kBoolean);
	syntax.addFlag("alp", "alignParent", MSyntax::kBoolean);
	syntax.addFlag("alc", "alignChild", MSyntax::kBoolean);
	syntax.addFlag("rfm", "refreshManipulators", MSyntax::kBoolean);
	syntax.addFlag("iwt", "initWeight", MSyntax::kBoolean);
	syntax.addFlag("smt", "smoothWeight", MSyntax::kBoolean);
	syntax.useSelectionAsDefault(true);
	syntax.setObjectType(MSyntax::kSelectionList);
	return syntax;
}
void * collisionOffsetSkinCmd::creator()
{
	return new collisionOffsetSkinCmd();
}
void collisionOffsetSkinCmd::rebindAll()
{
	MItDependencyNodes mitDg;
	while (!mitDg.isDone())
	{
		MFnDependencyNode dgNodeFn(mitDg.thisNode());
		if (dgNodeFn.typeId() == collisionOffsetSkin::id)
		{
			collisionOffsetSkin *skinNode = (collisionOffsetSkin *)dgNodeFn.userNode();
			skinNode->needRefresh = true;
		}
		mitDg.next();
	}
}
MCallbackId collisionOffsetSkinCmd::toolchangeID;
void collisionOffsetSkinCmd::initWeightPaintEvent()
{
	toolchangeID = MEventMessage::addEventCallback("ToolChanged",
		[](void*) {if (MFnToolContext(MGlobal::currentToolContext()).name() == "artAttrSkinContext")collisionOffsetSkinCmd::rebindAll(); });
}
void collisionOffsetSkinCmd::uninitWeightPaintEvent()
{
	MMessage::removeCallback(toolchangeID);
}
#ifdef WITHCGAL
void collisionOffsetSkinCmd::mayaObjToPoly(MObject &obj, Poly &poly)
{
	poly.clear();
	CGAL::Polyhedron_incremental_builder_3<Poly::HalfedgeDS> builder(poly.hds(), false);
	MFnMesh meshfn(obj);
	MPointArray vertexArray;
	meshfn.getPoints(vertexArray);
	int verLength = vertexArray.length();
	builder.begin_surface(verLength, meshfn.numPolygons(), meshfn.numEdges() * 2);
	for (int i = 0; i < verLength; ++i)
	{
		builder.add_vertex(Point(vertexArray[i].x, vertexArray[i].y, vertexArray[i].z));
	}
	MItMeshPolygon mitPolygon(obj);
	while (!mitPolygon.isDone())
	{
		builder.begin_facet();
		MIntArray verticesIndex;
		mitPolygon.getVertices(verticesIndex);
		for (int i = 0,l = verticesIndex.length(); i < l; ++i)
		{
			builder.add_vertex_to_facet(verticesIndex[i]);
		}
		builder.end_facet();
		mitPolygon.next();
	}
	builder.end_surface();
}
#endif