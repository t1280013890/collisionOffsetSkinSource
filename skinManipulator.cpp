#include "skinManipulator.h"
#include <maya/MModelMessage.h>
#include <maya/MGlobal.h>
#include <maya/MMatrix.h>
#include <maya/MFnRotateManip.h>
#include <maya/MEulerRotation.h>
#include <maya/MPlugArray.h>
#include <maya/MSelectionList.h>
#include <maya/MFnMatrixData.h>
#include <maya/MFnGeometryFilter.h>
#include <unordered_map>
#include <string>
#include "skinCmd.h"
MTypeId skinManip::id(0x5001d);

skinManip::skinManip()
{
}
void *skinManip::creator()
{
	return new skinManip();
}
MStatus skinManip::initialize()
{
	MStatus stat;
	stat = MPxManipContainer::initialize();
	return stat;
}
MStatus skinManip::createChildren()
{
	MStatus stat = MStatus::kSuccess;
	rotateManipDagPath = addRotateManip("pointManip", "freePoint");
	return stat;
}
MStatus skinManip::connectPlugArray(MPlugArray &parentOffsetPlugs, MMatrix &parent, MMatrix &globalOff)
{
	MStatus stat;
	rotateManipMat = globalOff * parent;
	parentMat = parent;
	globalOffMat = globalOff;
	parentMatInv = parent.inverse();
	globalOffMatInv = globalOff.inverse();
	MFnRotateManip rotateManip(rotateManipDagPath);
	rotateManip.setRotateMode(MFnRotateManip::kObjectSpace);
	rotateManip.setRotationCenter(parent[3]);
	for (int i = 0; i < parentOffsetPlugs.length(); ++i)
		rotatePlugIndex = addManipToPlugConversionCallback(parentOffsetPlugs[i], (manipToPlugConversionCallback)& skinManip::rotationChangedCallback);
	addPlugToManipConversionCallback(rotateManip.rotationIndex(),(plugToManipConversionCallback)&skinManip::rotateToManipCallback);
	finishAddingManips();
	MMatrix mat;
	getConverterPlugValue(rotatePlugIndex, mat);
	offsetMat = globalOff * mat*parent;
	return stat;
}
void skinManip::drawUI(MHWRender::MUIDrawManager& drawManager, const MHWRender::MFrameContext&frameContext)const
{
	MFnRotateManip rotateManip(rotateManipDagPath);
	float manipScale = rotateManip.manipScale(NULL)*1.4;
	drawManager.beginDrawable();
	drawManager.setDepthPriority(5);
	drawManager.setPointSize(5);
	drawManager.setColor(MColor(1, 1, 0, 1));
	drawManager.cone(rotateManipMat[3], offsetMat[0], 0.1*manipScale, manipScale);
	drawManager.setColor(MColor(0.5, 0.5, 0, 1));
	drawManager.cone(MPoint(rotateManipMat[3]) + -manipScale * MVector(offsetMat[0]), offsetMat[0], 0.1*manipScale, manipScale);
	drawManager.setColor(MColor(0, 0, 0, 1));
	drawManager.line(rotateManipMat[3], MPoint(rotateManipMat[3]) + MVector(offsetMat[1])*manipScale);
	drawManager.endDrawable();
}
MManipData skinManip::rotationChangedCallback(unsigned index)
{
	MFnMatrixData numericData;
	MObject obj = numericData.create();
	MFnRotateManip rotateManip(rotateManipDagPath);
	MEulerRotation manipRotation;
	getConverterManipValue(rotateManip.rotationIndex(), manipRotation);
	offsetMat = manipRotation.asMatrix();
	numericData.set(globalOffMatInv*offsetMat*parentMatInv);
	return MManipData(obj);
}
MManipData skinManip::rotateToManipCallback(unsigned index)
{
	MFnNumericData numericData;
	MObject obj = numericData.create(MFnNumericData::k3Double);
	MMatrix mat;
	getConverterPlugValue(rotatePlugIndex, mat);
	MEulerRotation plugRotation;
	plugRotation = globalOffMat * mat*parentMat;
	numericData.setData(plugRotation.x, plugRotation.y, plugRotation.z);
	return MManipData(obj);
}
skinManipContext::skinManipContext()
{
}
void skinManipContext::toolOnSetup(MEvent &)
{
	updateManipulators(this);
	MStatus status;
	id1 = MModelMessage::addCallback(MModelMessage::kActiveListModified, updateManipulators, this);
	collisionOffsetSkinCmd::manip = this;
}
void skinManipContext::toolOffCleanup()
{
	MModelMessage::removeCallback(id1);
	collisionOffsetSkinCmd::manip = NULL;
	MGlobal::executePythonCommand("skinParameterWidget.deleteWindow()");
	MPxContext::toolOffCleanup();
}
void skinManipContext::updateManipulators(void *data)
{
	skinManipContext*ctxPtr = (skinManipContext*)data;
	ctxPtr->deleteManipulators();
	MSelectionList list;
	MGlobal::getActiveSelectionList(list);
	std::unordered_map<std::string, std::pair<std::string, std::vector<int>>>skinMap;
	for (int j = 0; j < list.length(); ++j)
	{
		MObject dependNode;
		list.getDependNode(j, dependNode);
		MFnTransform jointNodeFn(dependNode);
		if (jointNodeFn.typeName() != "joint" || jointNodeFn.parentCount() < 1)continue;
		MStatus status;
		MPlug lockWeightPlug = jointNodeFn.findPlug("lockInfluenceWeights", &status);
		if (status != MS::kSuccess)continue;
		MPlugArray outPlugArray;
		lockWeightPlug.connectedTo(outPlugArray, false, true);
		if (outPlugArray.length() < 1)continue;
		MPlugArray parentOffsetPlugArray;
		MMatrix globalOffsetMat;
		for (int i = 0; i < outPlugArray.length(); ++i)
		{
			MFnGeometryFilter skinNodeFn(outPlugArray[i].node());
			if (skinNodeFn.typeName() != "collisionOffsetSkin")continue;
			MObjectArray outObjs;
			skinNodeFn.getOutputGeometry(outObjs);
			if (outObjs.length() < 0)continue;
			int index = outPlugArray[i].logicalIndex();
			MPlug influenceInfoPlug = skinNodeFn.findPlug("influenceInfo");
			parentOffsetPlugArray.append(influenceInfoPlug.elementByLogicalIndex(index).child(2));
			if (i == 0)globalOffsetMat = MFnMatrixData(influenceInfoPlug.elementByLogicalIndex(index).child(3).asMObject()).matrix();
			skinMap[std::string(skinNodeFn.name().asChar())].first = MFnDependencyNode(MFnDagNode(outObjs[0]).parent(0)).name().asChar();
			skinMap[std::string(skinNodeFn.name().asChar())].second.push_back(index);
		}
		MMatrix jointMat = MFnMatrixData(jointNodeFn.findPlug("worldMatrix").elementByLogicalIndex(0).asMObject()).matrix();
		MMatrix parentMat = MFnMatrixData(jointNodeFn.findPlug("parentMatrix").elementByLogicalIndex(0).asMObject()).matrix();
		parentMat[3][0] = jointMat[3][0];
		parentMat[3][1] = jointMat[3][1];
		parentMat[3][2] = jointMat[3][2];
		MObject manipObject;
		skinManip* manipulator = (skinManip *)skinManip::newManipulator("skinManip", manipObject);
		ctxPtr->addManipulator(manipObject);
		manipulator->connectPlugArray(parentOffsetPlugArray, parentMat, globalOffsetMat);
	}
	MGlobal::executePythonCommand("skinParameterWidget.deleteWindow()");
	if (!skinMap.empty())
	{
		MString pythonCmd("skinParameterWidget.showWindow([");
		for (auto &item : skinMap)
		{
			pythonCmd += "['";
			pythonCmd += item.first.c_str();
			pythonCmd += "','";
			pythonCmd += item.second.first.c_str();
			pythonCmd += "',[";
			for (auto &i : item.second.second)
			{
				pythonCmd += i;
				pythonCmd += ",";
			}
			pythonCmd += "]],";
		}
		pythonCmd += "])";
		MGlobal::executePythonCommand(pythonCmd);
	}
}
void skinManipContext::refreshManipulators()
{
	deleteManipulators();
	MSelectionList list;
	MGlobal::getActiveSelectionList(list);
	for (int j = 0; j < list.length(); ++j)
	{
		MObject dependNode;
		list.getDependNode(j, dependNode);
		MFnTransform jointNodeFn(dependNode);
		if (jointNodeFn.typeName() != "joint" || jointNodeFn.parentCount() < 1)continue;
		MStatus status;
		MPlug lockWeightPlug = jointNodeFn.findPlug("lockInfluenceWeights", &status);
		if (status != MS::kSuccess)continue;
		MPlugArray outPlugArray;
		lockWeightPlug.connectedTo(outPlugArray, false, true);
		if (outPlugArray.length() < 1)continue;
		MPlugArray parentOffsetPlugArray;
		MMatrix globalOffsetMat;
		for (int i = 0; i < outPlugArray.length(); ++i)
		{
			MFnGeometryFilter skinNodeFn(outPlugArray[i].node()); 
			if (skinNodeFn.typeName() != "collisionOffsetSkin")continue;
			MObjectArray outObjs;
			skinNodeFn.getOutputGeometry(outObjs);
			if (outObjs.length() < 0)continue;
			int index = outPlugArray[i].logicalIndex();
			MPlug influenceInfoPlug = skinNodeFn.findPlug("influenceInfo");
			parentOffsetPlugArray.append(influenceInfoPlug.elementByLogicalIndex(index).child(2));
			if (i == 0)globalOffsetMat = MFnMatrixData(influenceInfoPlug.elementByLogicalIndex(index).child(3).asMObject()).matrix();
		}
		MMatrix jointMat = MFnMatrixData(jointNodeFn.findPlug("worldMatrix").elementByLogicalIndex(0).asMObject()).matrix();
		MMatrix parentMat = MFnMatrixData(jointNodeFn.findPlug("parentMatrix").elementByLogicalIndex(0).asMObject()).matrix();
		parentMat[3][0] = jointMat[3][0];
		parentMat[3][1] = jointMat[3][1];
		parentMat[3][2] = jointMat[3][2];
		MObject manipObject;
		skinManip* manipulator = (skinManip *)skinManip::newManipulator("skinManip", manipObject);
		addManipulator(manipObject);
		manipulator->connectPlugArray(parentOffsetPlugArray, parentMat, globalOffsetMat);
	}
}
MPxContext *skinManipContextCmd::makeObj()
{
	return new skinManipContext();
}
void *skinManipContextCmd::creator()
{
	return new skinManipContextCmd;
}