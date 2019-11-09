#include "skin.h"

#include <maya/MMatrixArray.h>
#include <maya/MPxSkinCluster.h>
#include <maya/MItGeometry.h>
#include <maya/MPoint.h>
#include <maya/MFnMatrixData.h>
#include <maya/MFnCompoundAttribute.h>
#include <maya/MFnNumericAttribute.h>
#include <maya/MFnMatrixAttribute.h>
#include <maya/MArrayDataBuilder.h>
#include <maya/MGlobal.h>
#include <maya/MQuaternion.h>
#include <maya/MPlugArray.h>
#include <maya/MEulerRotation.h>
#include <maya/MFnUnitAttribute.h>
#include <maya/MFnDependencyNode.h>
#include <maya/MFnDoubleArrayData.h>
#include <maya/MAngle.h>
#include <Windows.h>

inline MQuaternion operator *(MQuaternion &q, double v)
{
	return MQuaternion(q.x*v, q.y*v, q.z*v, q.w*v);
}
inline MQuaternion& operator *=(MQuaternion &q, double v)
{
	q.x *= v;
	q.y *= v;
	q.z *= v;
	q.w *= v;
	return q;
}
inline MPoint& operator *=(MPoint&q, MQuaternion &u)
{
	double X = u.w*q.x + u.y*q.z - u.z*q.y;
	double Y = u.w*q.y - u.x*q.z + u.z*q.x;
	double Z = u.w*q.z + u.x*q.y - u.y*q.x;
	double W = -(u.x*q.x + u.y*q.y + u.z*q.z);
	q.x = X * u.w - Y * u.z + Z * u.y - W * u.x;
	q.y = X * u.z + Y * u.w - Z * u.x - W * u.y;
	q.z = Y * u.x + Z * u.w - W * u.z - X * u.y;
	return q;
}
inline double dot(MQuaternion &a, MQuaternion &b)
{
	return  a.x*b.x + a.y*b.y + a.z*b.z + a.w*b.w;
}
double getQuatWeight(MQuaternion &q1, MQuaternion &q2, MQuaternion &q3)
{
#define isSameA(a,b) ((a)*(b)>=0)
#define isSameB(a,b) ((a)*(b)<=0)
	MQuaternion checkQuat;
	double q1v, q2v, q3v;
	double *checkQuatV;
	double maxDistance = 0;
	char maxChar =0;
	double itemDistance;
	itemDistance = abs(q1.w - q2.w);
	if (itemDistance > maxDistance)
	{
		maxDistance = itemDistance;
		maxChar = 'w';
	}
	itemDistance = abs(q1.x - q2.x);
	if (itemDistance > maxDistance)
	{
		maxDistance = itemDistance;
		maxChar = 'x';
	}
	itemDistance = abs(q1.y - q2.y);
	if (itemDistance > maxDistance)
	{
		maxDistance = itemDistance;
		maxChar = 'y';
	}
	itemDistance = abs(q1.z - q2.z);
	if (itemDistance > maxDistance)
	{
		maxChar = 'z';
	}
	switch (maxChar)
	{
	case 'w':
		q1v = q1.w;
		q2v = q2.w;
		q3v = q3.w;
		checkQuatV = &checkQuat.w;
		break;
	case 'x':
		q1v = q1.x;
		q2v = q2.x;
		q3v = q3.x;
		checkQuatV = &checkQuat.x;
		break;
	case 'y':
		q1v = q1.y;
		q2v = q2.y;
		q3v = q3.y;
		checkQuatV = &checkQuat.y;
		break;
	case 'z':
		q1v = q1.z;
		q2v = q2.z;
		q3v = q3.z;
		checkQuatV = &checkQuat.z;
		break;
	default:
		return 0;
	}
	double dotq = dot(q1, q2);
	double powQ3v = q3v * q3v;
	double _buff = (1 - dotq) * 2 * powQ3v;
	double q1v_q2v = q1v - q2v;
	double a = _buff - q1v_q2v * q1v_q2v;
	double b = -_buff - 2 * q2v*q1v_q2v;
	double c = powQ3v - q2v * q2v;
	double B2_4ac = b * b - 4 * a*c;
	if (B2_4ac < 0)
		B2_4ac = 0;
	double sqrtB2_4ac = sqrt(B2_4ac);
	double a2 = 2 * a;
	double result1 = (sqrtB2_4ac - b) / a2;
	auto &checkResult = [&]()
	{
		checkQuat = q1 * result1 + q2 * (1 - result1);
		if (q3v**checkQuatV < 0)
			return isSameB(checkQuat.x, q3.x) && isSameB(checkQuat.y, q3.y) && isSameB(checkQuat.z, q3.z) && isSameB(checkQuat.w, q3.w);
		else return isSameA(checkQuat.x, q3.x) && isSameA(checkQuat.y, q3.y) && isSameA(checkQuat.z, q3.z) && isSameA(checkQuat.w, q3.w);
	};
	if (result1 >= 0 && result1 < 1 && checkResult())
		return result1;
	else return (-sqrtB2_4ac - b) / a2;
}


const MTypeId collisionOffsetSkin::id(0x00088803);
MObject collisionOffsetSkin::influenceInfoP;
MObject collisionOffsetSkin::bindMatP;
MObject collisionOffsetSkin::bindParentMatP;
MObject collisionOffsetSkin::bindParentOffsetP;
MObject collisionOffsetSkin::jointLengthP;
MObject collisionOffsetSkin::parentJointLengthP;
MObject collisionOffsetSkin::twistCutWeight1P;
MObject collisionOffsetSkin::twistCutWeight2P;
MObject collisionOffsetSkin::outsideCutProportionP;
MObject collisionOffsetSkin::collisionDeepP;
MObject collisionOffsetSkin::collisionHeightP;
MObject collisionOffsetSkin::collisionOffsetP;
MObject collisionOffsetSkin::outsideHeightP;
MObject collisionOffsetSkin::startAngleCosP;
MObject collisionOffsetSkin::endAngleCosP;
MObject collisionOffsetSkin::collisionSmooth1P;
MObject collisionOffsetSkin::collisionSmooth2P;
MObject collisionOffsetSkin::sideSmooth1P;
MObject collisionOffsetSkin::sideSmooth2P;
MObject collisionOffsetSkin::outSideExtendWeight1P;
MObject collisionOffsetSkin::outSideExtendWeight2P;
MObject collisionOffsetSkin::isLinearP;
MObject collisionOffsetSkin::globalOffsetMatP;
MObject collisionOffsetSkin::inGeoChangeEventP;

collisionOffsetSkin::collisionOffsetSkin()
{
	SYSTEM_INFO sysInfo;
	GetSystemInfo(&sysInfo);
	threadPool = CreateThreadpool(NULL);
	SetThreadpoolThreadMaximum(threadPool, sysInfo.dwNumberOfProcessors +1);
	SetThreadpoolThreadMinimum(threadPool, 1);
	InitializeThreadpoolEnvironment(&env);
	SetThreadpoolCallbackPool(&env, threadPool);

}
collisionOffsetSkin::~collisionOffsetSkin()
{
	DestroyThreadpoolEnvironment(&env);
	CloseThreadpool(threadPool);
}
void* collisionOffsetSkin::creator()
{
	return new collisionOffsetSkin();
}
void collisionOffsetSkin::postConstructor()
{
	blendWeightsObj = MFnDependencyNode(thisMObject()).findPlug("blendWeights").attribute();
	paintWeightsP = MFnDependencyNode(thisMObject()).findPlug("paintWeights");
}
MStatus collisionOffsetSkin::initialize()
{
	MFnCompoundAttribute cAttr;
	MFnNumericAttribute nAttr;
	MFnMatrixAttribute mAttr;
	MFnUnitAttribute uAttr;

	bindParentOffsetP = mAttr.create("bindParentOffsetMat", "bpo");
	mAttr.setWritable(true);
	mAttr.setReadable(false);
	mAttr.setStorable(true);

	bindParentMatP = mAttr.create("bindParentMat", "bpm");
	mAttr.setWritable(true);
	mAttr.setReadable(false);
	mAttr.setStorable(true);

	bindMatP = mAttr.create("bindMat", "pma");
	mAttr.setWritable(true);
	mAttr.setReadable(false);
	mAttr.setStorable(true);

	globalOffsetMatP = mAttr.create("globalOffsetMat", "gom");
	mAttr.setWritable(true);
	mAttr.setReadable(false);
	mAttr.setStorable(true);

	jointLengthP = nAttr.create("jointLengthC", "jlC", MFnNumericData::kDouble, 10);
	nAttr.setWritable(true);
	nAttr.setReadable(false);
	nAttr.setStorable(true);
	nAttr.setKeyable(false);
	nAttr.setConnectable(false);

	parentJointLengthP = nAttr.create("jointLengthP", "jpC", MFnNumericData::kDouble, 10);
	nAttr.setWritable(true);
	nAttr.setReadable(false);
	nAttr.setStorable(true);

	nAttr.setKeyable(false);
	nAttr.setConnectable(false);

	twistCutWeight1P = nAttr.create("twistCutWeightP", "cwP", MFnNumericData::kDouble, 0.35);
	nAttr.setMin(0);
	nAttr.setMax(1);
	nAttr.setWritable(true);
	nAttr.setReadable(false);
	nAttr.setStorable(true);
	nAttr.setKeyable(false);
	nAttr.setConnectable(false);

	twistCutWeight2P = nAttr.create("twistCutWeightC", "cwC", MFnNumericData::kDouble, 0.35);
	nAttr.setMin(0);
	nAttr.setMax(1);
	nAttr.setWritable(true);
	nAttr.setReadable(false);
	nAttr.setStorable(true);
	nAttr.setKeyable(false);
	nAttr.setConnectable(false);

	outsideCutProportionP = nAttr.create("outsideCutProportion", "ocp", MFnNumericData::kDouble, 1.3);
	nAttr.setMin(0);
	nAttr.setWritable(true);
	nAttr.setReadable(false);
	nAttr.setStorable(true);
	nAttr.setKeyable(false);
	nAttr.setConnectable(false);

	collisionDeepP = nAttr.create("collisionDeep", "cld", MFnNumericData::kDouble, 0);
	nAttr.setMin(0);
	nAttr.setMax(1);
	nAttr.setWritable(true);
	nAttr.setReadable(false);
	nAttr.setStorable(true);
	nAttr.setKeyable(false);
	nAttr.setConnectable(false);

	collisionHeightP = nAttr.create("collisionHeight", "clh", MFnNumericData::kDouble, 0);
	nAttr.setMin(0);
	nAttr.setWritable(true);
	nAttr.setReadable(false);
	nAttr.setStorable(true);
	nAttr.setKeyable(false);
	nAttr.setConnectable(false);

	collisionOffsetP = nAttr.create("collisionOffset", "clo", MFnNumericData::kDouble, 0);
	nAttr.setMin(0);
	nAttr.setMax(1);
	nAttr.setWritable(true);
	nAttr.setReadable(false);
	nAttr.setStorable(true);
	nAttr.setKeyable(false);
	nAttr.setConnectable(false);

	outsideHeightP = nAttr.create("outsideHeight", "osh", MFnNumericData::kDouble, 0);
	nAttr.setWritable(true);
	nAttr.setReadable(false);
	nAttr.setStorable(true);
	nAttr.setKeyable(false);
	nAttr.setConnectable(false);

	startAngleCosP = nAttr.create("startAngleCos", "sac", MFnNumericData::kDouble, 0);
	nAttr.setMin(-1);
	nAttr.setMax(1);
	nAttr.setWritable(true);
	nAttr.setReadable(false);
	nAttr.setStorable(true);
	nAttr.setKeyable(false);
	nAttr.setConnectable(false);

	endAngleCosP = nAttr.create("endAngleCos", "eac", MFnNumericData::kDouble, -1);
	nAttr.setMin(-1);
	nAttr.setMax(1);
	nAttr.setWritable(true);
	nAttr.setReadable(false);
	nAttr.setStorable(true);
	nAttr.setKeyable(false);
	nAttr.setConnectable(false);

	collisionSmooth1P = nAttr.create("collisionSmoothP", "csP", MFnNumericData::kDouble, 0.5);
	nAttr.setMin(0);
	nAttr.setMax(1);
	nAttr.setWritable(true);
	nAttr.setReadable(false);
	nAttr.setStorable(true);
	nAttr.setKeyable(false);
	nAttr.setConnectable(false);

	collisionSmooth2P = nAttr.create("collisionSmoothC", "csC", MFnNumericData::kDouble, 0.5);
	nAttr.setMin(0);
	nAttr.setMax(1);
	nAttr.setWritable(true);
	nAttr.setReadable(false);
	nAttr.setStorable(true);
	nAttr.setKeyable(false);
	nAttr.setConnectable(false);

	sideSmooth1P = nAttr.create("sideSmoothP", "ssP", MFnNumericData::kDouble, 0);
	nAttr.setMin(0);
	nAttr.setMax(1);
	nAttr.setWritable(true);
	nAttr.setReadable(false);
	nAttr.setStorable(true);
	nAttr.setKeyable(false);
	nAttr.setConnectable(false);

	sideSmooth2P = nAttr.create("sideSmoothC", "ssC", MFnNumericData::kDouble, 0);
	nAttr.setMin(0);
	nAttr.setMax(1);
	nAttr.setWritable(true);
	nAttr.setReadable(false);
	nAttr.setStorable(true);
	nAttr.setKeyable(false);
	nAttr.setConnectable(false);

	outSideExtendWeight1P = nAttr.create("outsideExtendWeightP", "owP", MFnNumericData::kDouble, 1);
	nAttr.setMin(0);
	nAttr.setMax(1);
	nAttr.setWritable(true);
	nAttr.setReadable(false);
	nAttr.setStorable(true);
	nAttr.setKeyable(false);
	nAttr.setConnectable(false);

	outSideExtendWeight2P = nAttr.create("outsideExtendWeightC", "owC", MFnNumericData::kDouble, 1);
	nAttr.setMin(0);
	nAttr.setMax(1);
	nAttr.setWritable(true);
	nAttr.setReadable(false);
	nAttr.setStorable(true);
	nAttr.setKeyable(false);
	nAttr.setConnectable(false);

	isLinearP = nAttr.create("isLinear", "iLn", MFnNumericData::kBoolean, false);
	nAttr.setWritable(true);
	nAttr.setReadable(false);
	nAttr.setStorable(true);

	nAttr.setKeyable(false);
	nAttr.setConnectable(false);

	influenceInfoP = cAttr.create("influenceInfo", "ifi");
	cAttr.addChild(bindMatP);
	cAttr.addChild(bindParentMatP);
	cAttr.addChild(bindParentOffsetP);
	cAttr.addChild(globalOffsetMatP);
	cAttr.addChild(parentJointLengthP);
	cAttr.addChild(jointLengthP);
	cAttr.addChild(twistCutWeight1P);
	cAttr.addChild(twistCutWeight2P);
	cAttr.addChild(outsideCutProportionP);
	cAttr.addChild(collisionDeepP);
	cAttr.addChild(collisionHeightP);
	cAttr.addChild(collisionOffsetP);
	cAttr.addChild(outsideHeightP);
	cAttr.addChild(startAngleCosP);
	cAttr.addChild(endAngleCosP);
	cAttr.addChild(collisionSmooth1P);
	cAttr.addChild(collisionSmooth2P);
	cAttr.addChild(sideSmooth1P);
	cAttr.addChild(sideSmooth2P);
	cAttr.addChild(outSideExtendWeight1P);
	cAttr.addChild(outSideExtendWeight2P);
	cAttr.addChild(isLinearP);
	cAttr.setArray(true);

	inGeoChangeEventP = nAttr.create("inGeoChangeEvent", "igc", MFnNumericData::kBoolean);
	nAttr.setHidden(true);

	addAttribute(influenceInfoP);
	addAttribute(inGeoChangeEventP);
	attributeAffects(influenceInfoP, outputGeom);
	attributeAffects(inputGeom, inGeoChangeEventP);
	return MStatus::kSuccess;
}
void collisionOffsetSkin::refreshInfluenceInfo()
{
	MPlug influencePlug(thisMObject(), influenceInfoP);
	int numElement = influencePlug.numElements();
	MPlug endEle = influencePlug.elementByPhysicalIndex(numElement - 1);
	int maxIndex = endEle.logicalIndex();
	influenceInfo.resize(maxIndex + 1);
	for (int i = 0; i < numElement; ++i)
	{
		MPlug ele = influencePlug.elementByPhysicalIndex(i);
		int logicalIndex = ele.logicalIndex();
		influenceInfo[logicalIndex].bindMat = MFnMatrixData(ele.child(bindMatP).asMObject()).matrix();
		influenceInfo[logicalIndex].bindParentOffsetMat = MFnMatrixData(ele.child(bindParentOffsetP).asMObject()).matrix();
		influenceInfo[logicalIndex].bindParentMat = MFnMatrixData(ele.child(bindParentMatP).asMObject()).matrix();
		influenceInfo[logicalIndex].globalOffsetMat = MFnMatrixData(ele.child(globalOffsetMatP).asMObject()).matrix();
		influenceInfo[logicalIndex].twistCutWeight1 = ele.child(twistCutWeight1P).asDouble();
		influenceInfo[logicalIndex].twistCutWeight2 = ele.child(twistCutWeight2P).asDouble();
		influenceInfo[logicalIndex].outSideExtendWeight1 = ele.child(outSideExtendWeight1P).asDouble();
		influenceInfo[logicalIndex].outSideExtendWeight2 = ele.child(outSideExtendWeight2P).asDouble();
		influenceInfo[logicalIndex].outsideCutProportion = ele.child(outsideCutProportionP).asDouble();
		influenceInfo[logicalIndex].collisionDeep = ele.child(collisionDeepP).asDouble();
		influenceInfo[logicalIndex].collisionHeight = ele.child(collisionHeightP).asDouble();
		influenceInfo[logicalIndex].collisionOffset = ele.child(collisionOffsetP).asDouble();
		influenceInfo[logicalIndex].outsideHeight = ele.child(outsideHeightP).asDouble();
		influenceInfo[logicalIndex].startAngleCos = ele.child(startAngleCosP).asDouble();
		influenceInfo[logicalIndex].endAngleCos = ele.child(endAngleCosP).asDouble();
		influenceInfo[logicalIndex].collisionSmooth1 = ele.child(collisionSmooth1P).asDouble();
		influenceInfo[logicalIndex].collisionSmooth2 = ele.child(collisionSmooth2P).asDouble();
		influenceInfo[logicalIndex].sideSmooth1 = ele.child(sideSmooth1P).asDouble();
		influenceInfo[logicalIndex].sideSmooth2 = ele.child(sideSmooth2P).asDouble();
		influenceInfo[logicalIndex].jointLength = ele.child(jointLengthP).asDouble();
		influenceInfo[logicalIndex].parentJointLength = ele.child(parentJointLengthP).asDouble();
		influenceInfo[logicalIndex].isLinear = ele.child(isLinearP).asBool();
		influenceInfo[logicalIndex].itemNeedBind = 1;
	}
	needBind = true;
}
MStatus collisionOffsetSkin::setDependentsDirty(const MPlug&plug, MPlugArray& plugArray)
{

	if (plug == matrix || plug == bindPreMatrix)
		return MPxSkinCluster::setDependentsDirty(plug, plugArray);
	else if (plug == weights)
		weightPlugDirty = true;
	else if (plug == paintWeightsP)
		paintWeightsDirty = true;
	else if (plug == bindMatP)
		dirtyInfPlugEnumArray.push_back(std::make_pair(bindMatE, plug.parent().logicalIndex()));
	else if (plug == bindParentOffsetP)
		dirtyInfPlugEnumArray.push_back(std::make_pair(bindParentOffsetE, plug.parent().logicalIndex()));
	else if (plug == bindParentMatP)
		dirtyInfPlugEnumArray.push_back(std::make_pair(bindParentMatE, plug.parent().logicalIndex()));
	else if (plug == globalOffsetMatP)
		dirtyInfPlugEnumArray.push_back(std::make_pair(globalOffsetMatE, plug.parent().logicalIndex()));
	else if (plug == jointLengthP)
		dirtyInfPlugEnumArray.push_back(std::make_pair(jointLengthE, plug.parent().logicalIndex()));
	else if (plug == parentJointLengthP)
		dirtyInfPlugEnumArray.push_back(std::make_pair(parentJointLengthE, plug.parent().logicalIndex()));
	else if (plug == twistCutWeight1P)
		dirtyInfPlugEnumArray.push_back(std::make_pair(twistCutWeight1E, plug.parent().logicalIndex()));
	else if (plug == twistCutWeight2P)
		dirtyInfPlugEnumArray.push_back(std::make_pair(twistCutWeight2E, plug.parent().logicalIndex()));
	else if (plug == outsideCutProportionP)
		dirtyInfPlugEnumArray.push_back(std::make_pair(outsideCutProportionE, plug.parent().logicalIndex()));
	else if (plug == collisionDeepP)
		dirtyInfPlugEnumArray.push_back(std::make_pair(collisionDeepE, plug.parent().logicalIndex()));
	else if (plug == collisionHeightP)
		dirtyInfPlugEnumArray.push_back(std::make_pair(collisionHeightE, plug.parent().logicalIndex()));
	else if (plug == collisionOffsetP)
		dirtyInfPlugEnumArray.push_back(std::make_pair(collisionOffsetE, plug.parent().logicalIndex()));
	else if (plug == outsideHeightP)
		dirtyInfPlugEnumArray.push_back(std::make_pair(outsideHeightE, plug.parent().logicalIndex()));
	else if (plug == startAngleCosP)
		dirtyInfPlugEnumArray.push_back(std::make_pair(startAngleCosE, plug.parent().logicalIndex()));
	else if (plug == endAngleCosP)
		dirtyInfPlugEnumArray.push_back(std::make_pair(endAngleCosE, plug.parent().logicalIndex()));
	else if (plug == collisionSmooth1P)
		dirtyInfPlugEnumArray.push_back(std::make_pair(collisionSmooth1E, plug.parent().logicalIndex()));
	else if (plug == collisionSmooth2P)
		dirtyInfPlugEnumArray.push_back(std::make_pair(collisionSmooth2E, plug.parent().logicalIndex()));
	else if (plug == sideSmooth1P)
		dirtyInfPlugEnumArray.push_back(std::make_pair(sideSmooth1E, plug.parent().logicalIndex()));
	else if (plug == sideSmooth2P)
		dirtyInfPlugEnumArray.push_back(std::make_pair(sideSmooth2E, plug.parent().logicalIndex()));
	else if (plug == outSideExtendWeight1P)
		dirtyInfPlugEnumArray.push_back(std::make_pair(outSideExtendWeight1E, plug.parent().logicalIndex()));
	else if (plug == outSideExtendWeight2P)
		dirtyInfPlugEnumArray.push_back(std::make_pair(outSideExtendWeight2E, plug.parent().logicalIndex()));
	else if (plug == isLinearP)
		dirtyInfPlugEnumArray.push_back(std::make_pair(isLinearE, plug.parent().logicalIndex()));
	return MPxSkinCluster::setDependentsDirty(plug, plugArray);
}

MStatus collisionOffsetSkin::compute(const MPlug&plug,MDataBlock&datablock)
{
	for (int i = 0, l = dirtyInfPlugEnumArray.size(); i < l; ++i)
	{
		MPlug infPlug(thisMObject(), influenceInfoP);
		const int&index = dirtyInfPlugEnumArray[i].second;
		if (influenceInfo.size() <= index)
			influenceInfo.resize(index + 1);
		switch (dirtyInfPlugEnumArray[i].first)
		{
		case bindMatE:
			influenceInfo[index].bindMat = MFnMatrixData(infPlug.elementByLogicalIndex(index).child(bindMatP).asMObject()).matrix();
			influenceInfo[index].itemNeedBind = 1;
			needBind = true;
			break;
		case bindParentOffsetE:
			influenceInfo[index].bindParentOffsetMat = MFnMatrixData(infPlug.elementByLogicalIndex(index).child(bindParentOffsetP).asMObject()).matrix();
			influenceInfo[index].itemNeedBind = 1;
			needBind = true;
			break;
		case bindParentMatE:
			influenceInfo[index].bindParentMat = MFnMatrixData(infPlug.elementByLogicalIndex(index).child(bindParentMatP).asMObject()).matrix();
			influenceInfo[index].itemNeedBind = 1;
			needBind = true;
			break;
		case globalOffsetMatE:
			influenceInfo[index].globalOffsetMat = MFnMatrixData(infPlug.elementByLogicalIndex(index).child(globalOffsetMatP).asMObject()).matrix();
			influenceInfo[index].itemNeedBind = 1;
			needBind = true;
			break;
		case jointLengthE:
			influenceInfo[index].jointLength = infPlug.elementByLogicalIndex(index).child(jointLengthP).asDouble();
			influenceInfo[index].itemNeedBind = 1;
			needBind = true;
			break;
		case parentJointLengthE:
			influenceInfo[index].parentJointLength = infPlug.elementByLogicalIndex(index).child(parentJointLengthP).asDouble();
			influenceInfo[index].itemNeedBind = 1;
			needBind = true;
			break;
		case twistCutWeight1E:
			influenceInfo[index].twistCutWeight1 = infPlug.elementByLogicalIndex(index).child(twistCutWeight1P).asDouble();
			influenceInfo[index].itemNeedBind = 1;
			needBind = true;
			break;
		case twistCutWeight2E:
			influenceInfo[index].twistCutWeight2 = infPlug.elementByLogicalIndex(index).child(twistCutWeight2P).asDouble();
			influenceInfo[index].itemNeedBind = 1;
			needBind = true;
			break;
		case outsideCutProportionE:
			influenceInfo[index].outsideCutProportion = infPlug.elementByLogicalIndex(index).child(outsideCutProportionP).asDouble();
			influenceInfo[index].itemNeedBind = 1;
			needBind = true;
			break;
		case collisionDeepE:
			influenceInfo[index].collisionDeep = infPlug.elementByLogicalIndex(index).child(collisionDeepP).asDouble();
			break;
		case collisionHeightE:
			influenceInfo[index].collisionHeight = infPlug.elementByLogicalIndex(index).child(collisionHeightP).asDouble();
			break;
		case collisionOffsetE:
			influenceInfo[index].collisionOffset = infPlug.elementByLogicalIndex(index).child(collisionOffsetP).asDouble();
			break;
		case outsideHeightE:
			influenceInfo[index].outsideHeight = infPlug.elementByLogicalIndex(index).child(outsideHeightP).asDouble();
			break;
		case startAngleCosE:
			influenceInfo[index].startAngleCos = infPlug.elementByLogicalIndex(index).child(startAngleCosP).asDouble();
			break;
		case endAngleCosE:
			influenceInfo[index].endAngleCos = infPlug.elementByLogicalIndex(index).child(endAngleCosP).asDouble();
			break;
		case collisionSmooth1E:
			influenceInfo[index].collisionSmooth1 = infPlug.elementByLogicalIndex(index).child(collisionSmooth1P).asDouble();
			break;
		case collisionSmooth2E:
			influenceInfo[index].collisionSmooth2 = infPlug.elementByLogicalIndex(index).child(collisionSmooth2P).asDouble();
			break;
		case sideSmooth1E:
			influenceInfo[index].sideSmooth1 = infPlug.elementByLogicalIndex(index).child(sideSmooth1P).asDouble();
			break;
		case sideSmooth2E:
			influenceInfo[index].sideSmooth2 = infPlug.elementByLogicalIndex(index).child(sideSmooth2P).asDouble();
			break;
		case outSideExtendWeight1E:
			influenceInfo[index].outSideExtendWeight1 = infPlug.elementByLogicalIndex(index).child(outSideExtendWeight1P).asDouble();
			influenceInfo[index].itemNeedBind = 1;
			needBind = true;
			break;
		case outSideExtendWeight2E:
			influenceInfo[index].outSideExtendWeight2 = infPlug.elementByLogicalIndex(index).child(outSideExtendWeight2P).asDouble();
			influenceInfo[index].itemNeedBind = 1;
			needBind = true;
			break;
		case isLinearE:
			influenceInfo[index].isLinear = infPlug.elementByLogicalIndex(index).child(isLinearP).asBool();
			break;
		default:
			break;
		}
	}
	dirtyInfPlugEnumArray.clear();


	auto &result = MPxSkinCluster::compute(plug, datablock);
	if (plug == paintWeightsP)
		paintWeights.copy(MFnDoubleArrayData(paintWeightsP.asMObject()).array());



	
	return result;
}
void collisionOffsetSkin::refreshWeights(MDataBlock&data)
{


	for (auto &item : influenceInfo)
	{
		if (item.itemNeedBind != -1)
		{
			item.weights.clear();
			item.weightIndexes.clear();
			if (!isWeightsLocked)
				item.weights.resize(sourcePoints.length(), 0);
		}
	}
	MArrayDataHandle weightListH = data.inputArrayValue(weightList);
	for (int i = 0, l = weightListH.elementCount(); i < l; ++i, weightListH.next())
	{
		MArrayDataHandle weightsHandle = weightListH.inputValue().child(weights);
		for (int j = 0, len = weightsHandle.elementCount(); j < len; ++j, weightsHandle.next())
		{
			unsigned int index = weightsHandle.elementIndex();
			double weight = weightsHandle.inputValue().asDouble();
			if (weight > 0.00001)
			{
				if (isWeightsLocked)
				{
					influenceInfo[index].weights.push_back(weight);
					influenceInfo[index].weightIndexes.push_back(i);
				}
				else influenceInfo[index].weights[i] = weight;
			}
		}
	}


}
MStatus collisionOffsetSkin::deform(MDataBlock& data, MItGeometry&iter, const MMatrix&, unsigned int)
{
	int numPoints = iter.count();
	if (needRefresh)
	{
		isWeightsLocked = true;
		refreshInfluenceInfo();
		iter.allPositions(sourcePoints);
		refreshWeights(data);
		needRefresh = false;
	}
	if (!data.isClean(inGeoChangeEventP))
	{
		iter.allPositions(sourcePoints);
		for (auto &item : influenceInfo)
			item.itemNeedBind = 1;
		needBind = true;
		data.setClean(inGeoChangeEventP);
	}
	MArrayDataHandle matH = data.inputArrayValue(matrix);
	for (int i = 0, l = matH.elementCount(); i < l; ++i, matH.next())
	{
		int index = matH.elementIndex();
		influenceInfo[index].mat = MFnMatrixData(matH.inputValue().data()).matrix();
	}
	MArrayDataHandle parMatH = data.inputArrayValue(bindPreMatrix);
	for (int i = 0, l = parMatH.elementCount(); i < l; ++i, parMatH.next())
		influenceInfo[parMatH.elementIndex()].parentMat = MFnMatrixData(parMatH.inputValue().data()).matrix();

	int influenceNum = matH.elementCount();
	MArrayDataHandle weightListH = data.inputArrayValue(weightList);
	if (paintWeightsDirty)
	{
		if (isWeightsLocked)
		{
			isWeightsLocked = false;
			refreshWeights(data);
			for (auto &item : influenceInfo)
				item.itemNeedBind = 1;
			needBind = true;
		}
		paintWeightsDirty = false;
		const auto &&paintw = MFnDoubleArrayData(paintWeightsP.asMObject()).array();
		int l = paintw.length();
		paintWeights.setLength(l);
		for (int i = 0; i < l; ++i)
		{
			if (paintWeights[i] != paintw[i])
			{
				weightListH.jumpToElement(i);
				MArrayDataHandle weightsHandle = weightListH.inputValue().child(weights);
				for (unsigned int j = 0, index = weightsHandle.elementIndex(); j < influenceNum; ++j)
				{
					if (j == index)
					{
						influenceInfo[index].weights[i] = weightsHandle.inputValue().asDouble();
						weightsHandle.next();
						index = weightsHandle.elementIndex();
					}
					else
						influenceInfo[j].weights[i] = 0;
				}
				paintWeights[i] = paintw[i];
			}
		}
	}
	if (weightPlugDirty)
	{
		if (isWeightsLocked)
		{
			refreshWeights(data);
			for (auto &item : influenceInfo)
				item.itemNeedBind = 1;
			needBind = true;
		}
		weightPlugDirty = false;
	}

	if(needBind)
	{
		bindParameters.clear();
		for (int i = 0, l = influenceInfo.size(); i < l; ++i)
			if (influenceInfo[i].itemNeedBind == 1)
				bindParameters.push_back(std::make_pair(std::make_pair(this, i), PTP_WORK()));
		for (auto &pa : bindParameters)
		{
			pa.second = CreateThreadpoolWork(bindThread, &pa.first, &env);
			SubmitThreadpoolWork(pa.second);
		}
		for (auto &pa : bindParameters)
			WaitForThreadpoolWorkCallbacks(pa.second, false);

		needBind = false;
	}

	resultPoints.resize(numPoints);
	for (int i = 0; i<numPoints; ++i)
	{
		resultPoints[i].x = 0;
		resultPoints[i].y = 0;
		resultPoints[i].z = 0;
	}

	deformParameters.clear();
	for (auto &item : influenceInfo)
		deformParameters.push_back(std::make_pair(std::make_pair(this, &item), PTP_WORK()));
	for (auto &pa : deformParameters)
	{
		pa.second = CreateThreadpoolWork(calcuThread, &pa.first, &env);
		SubmitThreadpoolWork(pa.second);
	}
	for (auto &pa : deformParameters)
		WaitForThreadpoolWorkCallbacks(pa.second, false);

	MPointArray resultDoublePoints;
	resultDoublePoints.setLength(numPoints);

	for (int i = 0; i < numPoints; ++i)
	{
		resultDoublePoints[i].x = double(resultPoints[i].x)*0.0001;
		resultDoublePoints[i].y = double(resultPoints[i].y)*0.0001;
		resultDoublePoints[i].z = double(resultPoints[i].z)*0.0001;
	}
	iter.setAllPositions(resultDoublePoints);

	return MStatus::kSuccess;
}

void NTAPI collisionOffsetSkin::calcuThread(_Inout_ PTP_CALLBACK_INSTANCE, _Inout_opt_ PVOID parameter, _Inout_ PTP_WORK)
{
	DeformThreadParameter* para = (DeformThreadParameter*)parameter;
	collisionOffsetSkin *skinPtr = para->first;
	auto &resultPoints = skinPtr->resultPoints;
	auto &sourcePoints = skinPtr->sourcePoints;
	auto &item = para->second;
	if(item->itemNeedBind != -1)
	{
		int(*mapping)(int, std::vector<int>*);
		int l;
		if (skinPtr->isWeightsLocked)
		{
			l = item->weightIndexes.size();
			mapping = [](int i, std::vector<int>* weightIndexes) {return weightIndexes->operator[](i); };
		}
		else
		{
			mapping = [](int i, std::vector<int>*) {return i; };
			l = skinPtr->sourcePoints.length();
		}
		if (item->isLinear)
		{
			auto &&transformMat = item->bindMat.inverse()*item->mat;
			for (int id = 0; id < l; ++id)
			{
				int i = mapping(id, &item->weightIndexes);
				if (item->weights[id] < 0.0001)
					continue;
				auto &&calcuPoint = sourcePoints[i] * transformMat*item->weights[id];
				InterlockedExchangeAdd64(&resultPoints[i].x, calcuPoint.x * 10000);
				InterlockedExchangeAdd64(&resultPoints[i].y, calcuPoint.y * 10000);
				InterlockedExchangeAdd64(&resultPoints[i].z, calcuPoint.z * 10000);
			}
		}
		else
		{
			const double &jointLength = item->jointLength;
			const double &parentJointLength = item->parentJointLength;
			const double &twistCutWeight1 = item->twistCutWeight1;
			const double &twistCutWeight2 = item->twistCutWeight2;
			const double &outsideCutProportion = item->outsideCutProportion;
			const double &outSideExtendWeight1 = item->outSideExtendWeight1;
			const double &outSideExtendWeight2 = item->outSideExtendWeight2;
			const auto &influenceVertexes = item->influenceVertexes;
			double collisionDeep = item->collisionDeep;
			double collisionHeight = item->collisionHeight;
			double collisionOffset = item->collisionOffset;
			double outsideHeight = item->outsideHeight;
			const double &startAngleCos = item->startAngleCos;
			const double &endAngleCos = item->endAngleCos;
			const double &collisionSmooth1 = item->collisionSmooth1;
			const double &collisionSmooth2 = item->collisionSmooth2;
			double sideSmooth1 = item->sideSmooth1;
			double sideSmooth2 = item->sideSmooth2;
			const MMatrix &joint2Mat = item->globalOffsetMat*item->mat;
			MMatrix joint1Mat = item->globalOffsetMat*item->bindParentOffsetMat*item->parentMat;
			MPoint centerPoint = MPoint(joint2Mat[3][0], joint2Mat[3][1], joint2Mat[3][2]);
			joint1Mat[3][0] = joint2Mat[3][0];
			joint1Mat[3][1] = joint2Mat[3][1];
			joint1Mat[3][2] = joint2Mat[3][2];
			MVector transPoint2 = MVector(joint2Mat[0][0], joint2Mat[0][1], joint2Mat[0][2]);
			MVector transPoint1 = MVector(-joint1Mat[0][0], -joint1Mat[0][1], -joint1Mat[0][2]);
			MVector currentLogicalV = transPoint2 * joint1Mat.inverse();
			MVector bindLogicalV(item->globalOffsetMat[0][0], item->globalOffsetMat[0][1], item->globalOffsetMat[0][2]);
			bindLogicalV *= item->bindMat;
			bindLogicalV *= (item->globalOffsetMat*item->bindParentOffsetMat*item->bindParentMat).inverse();
			double offCos = currentLogicalV * bindLogicalV;
			if (offCos > startAngleCos)
			{
				collisionDeep = 1;
				outsideHeight = 0;
				collisionHeight = 0;
				sideSmooth1 = 0;
				sideSmooth2 = 0;
				collisionOffset = 0;
			}
			else if (offCos > endAngleCos)
			{
				double fc = (offCos - startAngleCos) / (endAngleCos - startAngleCos);
				collisionDeep = (collisionDeep - 1)*fc + 1;
				outsideHeight *= fc;
				collisionHeight *= fc;
				sideSmooth1 *= fc;
				sideSmooth2 *= fc;
				double _1_fc = 1 - fc;
				collisionOffset *= 1 - _1_fc * _1_fc;
			}
			collisionOffset = 1 - collisionOffset;
			MPoint joint2endPoint = centerPoint + transPoint2 * jointLength;
			MPoint joint2endVector = transPoint2 * jointLength;
			MPoint joint1endPoint = centerPoint + transPoint1 * parentJointLength;
			MVector joint1endVector = transPoint1 * parentJointLength;
			MVector planeUPVector = (transPoint2 + transPoint1) / 2;
			planeUPVector.normalize();
			MVector planeTangentVector;
			MVector planeNormal;
			if (planeUPVector.x<0.001&&planeUPVector.x>-0.001&&planeUPVector.y<0.001&&planeUPVector.y>-0.001&&planeUPVector.z<0.001&&planeUPVector.z>-0.001)
			{
				planeUPVector = MVector(joint1Mat[1][0], joint1Mat[1][1], joint1Mat[1][2]);
				planeTangentVector = MVector(-joint1Mat[2][0], -joint1Mat[2][1], -joint1Mat[2][2]);
				planeNormal = transPoint2;
			}
			else
			{
				planeTangentVector = transPoint1 ^ transPoint2;
				planeTangentVector.normalize();
				planeNormal = planeTangentVector ^ planeUPVector;
			}
			MVector joint2UpVector = transPoint2 ^ planeTangentVector;
			MVector joint1UpVector = planeTangentVector ^ transPoint1;
			double cosPlaneJoint = planeUPVector * transPoint1;
			if (cosPlaneJoint > 1)
				cosPlaneJoint = 1;
			if (cosPlaneJoint < -1)
				cosPlaneJoint = -1;
			double sinPlaneJoint = sqrt(1 - cosPlaneJoint * cosPlaneJoint);
			double cotPlaneJoint = cosPlaneJoint / sinPlaneJoint;
			double re_sinPlaneJoint = 1 / sinPlaneJoint;
			double collisionDeep_1 = (re_sinPlaneJoint - 1)*collisionDeep;
			collisionDeep = collisionDeep_1 + 1;
			double cosJoint2trans = -transPoint1 * transPoint2;
			double halfCos = sqrt(abs(1 + cosJoint2trans) / 2);
			double halfSin = sqrt(abs(1 - cosJoint2trans) / 2);
			MQuaternion joint2TransQuat(-planeTangentVector.x*halfSin, -planeTangentVector.y*halfSin, -planeTangentVector.z*halfSin, halfCos);
			MQuaternion joint1Quat;
			joint1Quat = joint1Mat;
			MQuaternion joint2Quat;
			joint2Quat = joint2Mat;
			MQuaternion offsetQuat = joint2Quat * (joint1Quat*joint2TransQuat).conjugate();
			if (offsetQuat.w < 0)
				offsetQuat.negateIt();
			MMatrix outSideJoint2M;
			outSideJoint2M[0][0] = transPoint2.x; outSideJoint2M[0][1] = transPoint2.y; outSideJoint2M[0][2] = transPoint2.z; outSideJoint2M[0][3] = 0;
			outSideJoint2M[1][0] = joint2UpVector.x; outSideJoint2M[1][1] = joint2UpVector.y; outSideJoint2M[1][2] = joint2UpVector.z; outSideJoint2M[1][3] = 0;
			outSideJoint2M[2][0] = -planeTangentVector.x; outSideJoint2M[2][1] = -planeTangentVector.y; outSideJoint2M[2][2] = -planeTangentVector.z; outSideJoint2M[2][3] = 0;
			MQuaternion outSideJoint2Q;
			outSideJoint2Q = outSideJoint2M;
			MMatrix outSideJoint1M;
			outSideJoint1M[0][0] = -transPoint1.x; outSideJoint1M[0][1] = -transPoint1.y; outSideJoint1M[0][2] = -transPoint1.z; outSideJoint1M[0][3] = 0;
			outSideJoint1M[1][0] = joint1UpVector.x; outSideJoint1M[1][1] = joint1UpVector.y; outSideJoint1M[1][2] = joint1UpVector.z; outSideJoint1M[1][3] = 0;
			outSideJoint1M[2][0] = -planeTangentVector.x; outSideJoint1M[2][1] = -planeTangentVector.y; outSideJoint1M[2][2] = -planeTangentVector.z; outSideJoint1M[2][3] = 0;
			MQuaternion outSideJoint1Q;
			outSideJoint1Q = outSideJoint1M;
			MVector outSideCenterX = planeTangentVector ^ planeUPVector;
			MMatrix outSideCenterM;
			outSideCenterM[0][0] = outSideCenterX.x; outSideCenterM[0][1] = outSideCenterX.y; outSideCenterM[0][2] = outSideCenterX.z; outSideCenterM[0][3] = 0;
			outSideCenterM[1][0] = planeUPVector.x; outSideCenterM[1][1] = planeUPVector.y; outSideCenterM[1][2] = planeUPVector.z; outSideCenterM[1][3] = 0;
			outSideCenterM[2][0] = -planeTangentVector.x; outSideCenterM[2][1] = -planeTangentVector.y; outSideCenterM[2][2] = -planeTangentVector.z; outSideCenterM[2][3] = 0;
			MQuaternion outSideCenterQ;
			outSideCenterQ = outSideCenterM;
			if (dot(outSideJoint2Q, outSideCenterQ) < 0)
				outSideJoint2Q *= -1;
			if (dot(outSideJoint1Q, outSideCenterQ) < 0)
				outSideJoint1Q *= -1;

			for (int id = 0; id < l; ++id)
			{
				int i = mapping(id, &item->weightIndexes);
				if (item->weights[id] < 0.0001)
					continue;
				double influenceWeight = influenceVertexes[id].weight;
				if (influenceWeight > 0)
				{
					MQuaternion averageOffsetQuat;
					if (influenceWeight < twistCutWeight2)
					{
						double w = (influenceWeight / twistCutWeight2)*0.5 + 0.5;
						w = (1 - w) * 2;
						double _w = w * w / 2;
						w = 1 - _w;
						averageOffsetQuat = offsetQuat * w;
						averageOffsetQuat.w += (1 - w);
						averageOffsetQuat.normalizeIt();
					}
					else
						averageOffsetQuat = offsetQuat;
					MVector p(0, influenceVertexes[id].offsetY, influenceVertexes[id].offsetZ);
					p *= averageOffsetQuat;
					p *= joint1Quat;
					double deepY = joint1UpVector * p;
					double deepZ = -(planeTangentVector*p);
					if (influenceWeight > 1)
					{
						MPoint start = deepY * joint2UpVector + -deepZ * planeTangentVector + joint2endPoint;
						auto &&calcuPoint = (start + transPoint2 * (influenceWeight - 1)*jointLength)*item->weights[id];
						InterlockedExchangeAdd64(&resultPoints[i].x, calcuPoint.x * 10000);
						InterlockedExchangeAdd64(&resultPoints[i].y, calcuPoint.y * 10000);
						InterlockedExchangeAdd64(&resultPoints[i].z, calcuPoint.z * 10000);
					}
					else if (deepY < 0)
					{
						double outsideWeightCut = -deepY * abs(cosPlaneJoint)*outsideCutProportion / jointLength;
						if (outsideWeightCut > 0.5)
							outsideWeightCut = 0.5;
						if (influenceWeight < outsideWeightCut)
						{
							double w = influenceWeight / outsideWeightCut;
							MQuaternion transQuat(outSideJoint2Q*w + outSideCenterQ * (1 - w));
							transQuat.normalizeIt();
							MPoint pointBuffer(0, deepY, deepZ);
							pointBuffer *= transQuat;
							pointBuffer += centerPoint + planeUPVector * (planeUPVector*pointBuffer*outsideHeight*(1 - w * w));
							auto &&calcuPoint = pointBuffer * item->weights[id];
							InterlockedExchangeAdd64(&resultPoints[i].x, calcuPoint.x * 10000);
							InterlockedExchangeAdd64(&resultPoints[i].y, calcuPoint.y * 10000);
							InterlockedExchangeAdd64(&resultPoints[i].z, calcuPoint.z * 10000);
						}
						else
						{
							double w = (influenceWeight - outsideWeightCut) / (1 - outsideWeightCut);
							double realW = outSideExtendWeight2 * (1 - outsideWeightCut) + outsideWeightCut;
							MPoint midPoint = deepY * joint2UpVector + -deepZ * planeTangentVector + centerPoint + transPoint2 * (jointLength*realW);
							if (w < outSideExtendWeight2)
							{
								MPoint start(0, deepY, deepZ);
								start *= outSideJoint2Q;
								start += centerPoint;
								MPoint pointBuffer = start + (midPoint - start)*(w / outSideExtendWeight2);
								auto &calcuPoint = pointBuffer * item->weights[id];
								InterlockedExchangeAdd64(&resultPoints[i].x, calcuPoint.x * 10000);
								InterlockedExchangeAdd64(&resultPoints[i].y, calcuPoint.y * 10000);
								InterlockedExchangeAdd64(&resultPoints[i].z, calcuPoint.z * 10000);
							}
							else
							{
								MPoint pointBuffer = midPoint + transPoint2 * ((influenceWeight - realW)*jointLength);
								auto &calcuPoint = pointBuffer * item->weights[id];
								InterlockedExchangeAdd64(&resultPoints[i].x, calcuPoint.x * 10000);
								InterlockedExchangeAdd64(&resultPoints[i].y, calcuPoint.y * 10000);
								InterlockedExchangeAdd64(&resultPoints[i].z, calcuPoint.z * 10000);
							}
						}
					}
					else
					{
						MVector vectorZ = -deepZ * planeTangentVector;
						double proportionY = (cosPlaneJoint*collisionHeight + 1)*deepY;
						double sideScale = (abs(deepY) / sqrt(deepY*deepY + deepZ * deepZ) - 1)*sideSmooth2 + 1;
						MPoint baseStart = proportionY * planeUPVector;
						MPoint start = baseStart * (collisionDeep_1*collisionOffset + 1);
						MPoint mid = proportionY * cotPlaneJoint*transPoint2 + sideScale * proportionY*joint2UpVector;
						MPoint end = deepY * joint2UpVector + joint2endVector;
						MVector mid1off = (start - mid)*collisionSmooth2;
						MPoint mid1 = mid + mid1off;
						MVector end_mid = end - mid;
						MPoint mid2 = end;
						double e_mLength = end_mid.length();
						double mid1offLength = mid1off.length();
						if (e_mLength > mid1offLength)
						{
							end_mid.x /= e_mLength;
							end_mid.y /= e_mLength;
							end_mid.z /= e_mLength;
							mid2 = mid + end_mid * mid1offLength;
						}
						MVector mid1_start(mid1 - start);
						MVector mid2_mid1(mid2 - mid1);
						MVector end_mid2(end - mid2);
						double length1 = mid1_start.length();
						double length2 = mid2_mid1.length();
						double length3 = end_mid2.length();
						double lengthSum = length1 + length2 + length3;
						double midWeight1 = length1 / lengthSum;
						double midWeight2 = (length1 + length2) / lengthSum;
						start = baseStart * collisionDeep;
						mid1 = mid + (start - mid)*collisionSmooth2;
						mid1_start = mid1 - start;
						MPoint pointBuffer;
						if (influenceWeight < midWeight1)
						{
							double w = influenceWeight / midWeight1;
							pointBuffer = start + mid1_start * w;
						}
						else if (influenceWeight < midWeight2)
						{
							double w = (influenceWeight - midWeight1) / (midWeight2 - midWeight1);
							double w2 = w * w;
							pointBuffer = (w2 - 2 * w + 1)*mid1 + (-2 * w2 + 2 * w)*mid + w2 * mid2;
						}
						else
						{
							double w = (influenceWeight - midWeight2) / (1 - midWeight2);
							pointBuffer = mid2 + end_mid2 * w;
						}
						auto &&calcuPoint = (pointBuffer + vectorZ + centerPoint)*item->weights[id];
						InterlockedExchangeAdd64(&resultPoints[i].x, calcuPoint.x * 10000);
						InterlockedExchangeAdd64(&resultPoints[i].y, calcuPoint.y * 10000);
						InterlockedExchangeAdd64(&resultPoints[i].z, calcuPoint.z * 10000);
					}
				}
				else
				{
					influenceWeight = -influenceWeight;
					MVector p(0, influenceVertexes[id].offsetY, influenceVertexes[id].offsetZ);
					if (influenceWeight < twistCutWeight1)
					{
						double w = (1 - (influenceWeight / twistCutWeight1))*0.5;
						w *= 2;
						w = w * w / 2;
						MQuaternion averageOffsetQuat = offsetQuat * w;
						averageOffsetQuat.w += (1 - w);
						averageOffsetQuat.normalizeIt();
						p *= averageOffsetQuat;
					}
					p *= joint1Quat;
					double deepY = joint1UpVector * p;
					double deepZ = -(planeTangentVector*p);
					if (influenceWeight > 1)
					{
						MPoint start = deepY * joint1UpVector + -deepZ * planeTangentVector + joint1endPoint;
						auto &&calcuPoint = (start + transPoint1 * (influenceWeight - 1)*parentJointLength)*item->weights[id];
						InterlockedExchangeAdd64(&resultPoints[i].x, calcuPoint.x * 10000);
						InterlockedExchangeAdd64(&resultPoints[i].y, calcuPoint.y * 10000);
						InterlockedExchangeAdd64(&resultPoints[i].z, calcuPoint.z * 10000);
					}
					else if (deepY < 0)
					{
						double outsideWeightCut = -deepY * abs(cosPlaneJoint)*outsideCutProportion / parentJointLength;
						if (outsideWeightCut > 0.5)
							outsideWeightCut = 0.5;
						if (influenceWeight < outsideWeightCut)
						{
							double w = influenceWeight / outsideWeightCut;
							MQuaternion transQuat(outSideJoint1Q*w + outSideCenterQ * (1 - w));
							transQuat.normalizeIt();
							MPoint pointBuffer(0, deepY, deepZ);
							pointBuffer *= transQuat;
							pointBuffer += centerPoint + planeUPVector * (planeUPVector*pointBuffer*outsideHeight*(1 - w * w));
							auto &&calcuPoint = pointBuffer * item->weights[id];
							InterlockedExchangeAdd64(&resultPoints[i].x, calcuPoint.x * 10000);
							InterlockedExchangeAdd64(&resultPoints[i].y, calcuPoint.y * 10000);
							InterlockedExchangeAdd64(&resultPoints[i].z, calcuPoint.z * 10000);
						}
						else
						{
							double w = (influenceWeight - outsideWeightCut) / (1 - outsideWeightCut);
							double realW = outSideExtendWeight1 * (1 - outsideWeightCut) + outsideWeightCut;
							MPoint midPoint = deepY * joint1UpVector + -deepZ * planeTangentVector + centerPoint + transPoint1 * (parentJointLength*realW);
							if (w < outSideExtendWeight1)
							{
								MPoint start(0, deepY, deepZ);
								start *= outSideJoint1Q;
								start += centerPoint;
								MPoint pointBuffer = start + (midPoint - start)*(w / outSideExtendWeight1);
								auto &&calcuPoint = pointBuffer * item->weights[id];
								InterlockedExchangeAdd64(&resultPoints[i].x, calcuPoint.x * 10000);
								InterlockedExchangeAdd64(&resultPoints[i].y, calcuPoint.y * 10000);
								InterlockedExchangeAdd64(&resultPoints[i].z, calcuPoint.z * 10000);
							}
							else
							{
								MPoint pointBuffer = midPoint + transPoint1 * ((influenceWeight - realW)*parentJointLength);
								auto &&calcuPoint = pointBuffer * item->weights[id];
								InterlockedExchangeAdd64(&resultPoints[i].x, calcuPoint.x * 10000);
								InterlockedExchangeAdd64(&resultPoints[i].y, calcuPoint.y * 10000);
								InterlockedExchangeAdd64(&resultPoints[i].z, calcuPoint.z * 10000);
							}
						}
					}
					else
					{
						MVector vectorZ = -deepZ * planeTangentVector;
						double proportionY = (cosPlaneJoint*collisionHeight + 1)*deepY;
						double sideScale = (abs(deepY) / sqrt(deepY*deepY + deepZ * deepZ) - 1)*sideSmooth1 + 1;
						MPoint baseStart = proportionY * planeUPVector;
						MPoint start = baseStart * (collisionDeep_1*collisionOffset + 1);
						MPoint mid = proportionY * cotPlaneJoint*transPoint1 + sideScale * proportionY*joint1UpVector;
						MPoint end = deepY * joint1UpVector + joint1endVector;
						MVector mid1off = (start - mid)*collisionSmooth1;
						MPoint mid1 = mid + mid1off;
						MVector end_mid = end - mid;
						MPoint mid2 = end;
						double e_mLength = end_mid.length();
						double mid1offLength = mid1off.length();
						if (e_mLength > mid1offLength)
						{
							end_mid.x /= e_mLength;
							end_mid.y /= e_mLength;
							end_mid.z /= e_mLength;
							mid2 = mid + end_mid * mid1offLength;
						}
						MVector mid1_start(mid1 - start);
						MVector mid2_mid1(mid2 - mid1);
						MVector end_mid2(end - mid2);
						double length1 = mid1_start.length();
						double length2 = mid2_mid1.length();
						double length3 = end_mid2.length();
						double lengthSum = length1 + length2 + length3;
						double midWeight1 = length1 / lengthSum;
						double midWeight2 = (length1 + length2) / lengthSum;
						start = baseStart * collisionDeep;
						mid1 = mid + (start - mid)*collisionSmooth1;
						mid1_start = mid1 - start;
						MPoint pointBuffer;
						if (influenceWeight < midWeight1)
						{
							double w = influenceWeight / midWeight1;
							pointBuffer = start + mid1_start * w;
						}
						else if (influenceWeight < midWeight2)
						{
							double w = (influenceWeight - midWeight1) / (midWeight2 - midWeight1);
							double w2 = w * w;
							pointBuffer = (w2 - 2 * w + 1)*mid1 + (-2 * w2 + 2 * w)*mid + w2 * mid2;
						}
						else
						{
							double w = (influenceWeight - midWeight2) / (1 - midWeight2);
							pointBuffer = mid2 + end_mid2 * w;
						}
						auto &&calcuPoint = (pointBuffer + vectorZ + centerPoint)*item->weights[id];
						InterlockedExchangeAdd64(&resultPoints[i].x, calcuPoint.x * 10000);
						InterlockedExchangeAdd64(&resultPoints[i].y, calcuPoint.y * 10000);
						InterlockedExchangeAdd64(&resultPoints[i].z, calcuPoint.z * 10000);
					}
				}
			}
		}
	}
}
void NTAPI collisionOffsetSkin::bindThread(_Inout_ PTP_CALLBACK_INSTANCE, _Inout_opt_ PVOID parameter, _Inout_ PTP_WORK)
{
	BindThreadParameter* para = (BindThreadParameter*)parameter;
	collisionOffsetSkin* skinPtr = para->first;
	int index = para->second;
	auto &influenceInfo = skinPtr->influenceInfo;
	auto &sourcePoints = skinPtr->sourcePoints;
	influenceInfo[index].itemNeedBind = 0;
	double jointLength = influenceInfo[index].jointLength;
	double parentJointLength = influenceInfo[index].parentJointLength;
	double twistCutWeight1 = influenceInfo[index].twistCutWeight1;
	double twistCutWeight2 = influenceInfo[index].twistCutWeight2;
	double outsideCutProportion = influenceInfo[index].outsideCutProportion;
	double outSideExtendWeight1 = influenceInfo[index].outSideExtendWeight1;
	double outSideExtendWeight2 = influenceInfo[index].outSideExtendWeight2;
	MMatrix joint2Mat = influenceInfo[index].globalOffsetMat*influenceInfo[index].bindMat;
	MMatrix joint1Mat = influenceInfo[index].globalOffsetMat*influenceInfo[index].bindParentOffsetMat*influenceInfo[index].bindParentMat;
	MPoint cPoint(joint2Mat[3][0], joint2Mat[3][1], joint2Mat[3][2]);
	joint1Mat[3][0] = joint2Mat[3][0];
	joint1Mat[3][1] = joint2Mat[3][1];
	joint1Mat[3][2] = joint2Mat[3][2];
	MVector transPoint2 = MVector(joint2Mat[0][0], joint2Mat[0][1], joint2Mat[0][2]);
	MVector transPoint1 = MVector(-joint1Mat[0][0], -joint1Mat[0][1], -joint1Mat[0][2]);
	MPoint joint2endPoint = cPoint + transPoint2 * jointLength;
	MPoint joint1endPoint = cPoint + transPoint1 * parentJointLength;
	MVector planeUPVector = (transPoint2 + transPoint1) / 2;
	planeUPVector.normalize();
	MVector planeTangentVector;
	MVector planeNormal;
	if (planeUPVector.x<0.001&&planeUPVector.x>-0.001&&planeUPVector.y<0.001&&planeUPVector.y>-0.001&&planeUPVector.z<0.001&&planeUPVector.z>-0.001)
	{
		planeUPVector = MVector(joint1Mat[1][0], joint1Mat[1][1], joint1Mat[1][2]);
		planeTangentVector = MVector(-joint1Mat[2][0], -joint1Mat[2][1], -joint1Mat[2][2]);
		planeNormal = transPoint2;
	}
	else
	{
		planeTangentVector = transPoint1 ^ transPoint2;
		planeTangentVector.normalize();
		planeNormal = planeTangentVector ^ planeUPVector;
	}
	MVector joint2UpVector = transPoint2 ^ planeTangentVector;
	MVector joint1UpVector = planeTangentVector ^ transPoint1;
	double cosPlaneJoint = planeUPVector * transPoint1;
	if (cosPlaneJoint > 1)
		cosPlaneJoint = 1;
	if (cosPlaneJoint < -1)
		cosPlaneJoint = -1;
	double sinPlaneJoint = sqrt(1 - cosPlaneJoint * cosPlaneJoint);
	double cosJoint2trans = -transPoint1 * transPoint2;
	double halfCos = sqrt(abs(1 + cosJoint2trans) / 2);
	double halfSin = sqrt(abs(1 - cosJoint2trans) / 2);
	MQuaternion joint2TransQuat(-planeTangentVector.x*halfSin, -planeTangentVector.y*halfSin, -planeTangentVector.z*halfSin, halfCos);
	MQuaternion joint1Quat;
	joint1Quat = joint1Mat;
	MQuaternion joint2Quat;
	joint2Quat = joint2Mat;
	MQuaternion joint2WithoutTwistQ = joint1Quat * joint2TransQuat;
	MQuaternion offsetQuat = joint2Quat * joint2WithoutTwistQ.conjugate();
	if (offsetQuat.w < 0)
		offsetQuat.negateIt();
	MMatrix outSideJoint2M;
	outSideJoint2M[0][0] = transPoint2.x; outSideJoint2M[0][1] = transPoint2.y; outSideJoint2M[0][2] = transPoint2.z; outSideJoint2M[0][3] = 0;
	outSideJoint2M[1][0] = joint2UpVector.x; outSideJoint2M[1][1] = joint2UpVector.y; outSideJoint2M[1][2] = joint2UpVector.z; outSideJoint2M[1][3] = 0;
	outSideJoint2M[2][0] = -planeTangentVector.x; outSideJoint2M[2][1] = -planeTangentVector.y; outSideJoint2M[2][2] = -planeTangentVector.z; outSideJoint2M[2][3] = 0;
	outSideJoint2M[3][0] = cPoint.x; outSideJoint2M[3][1] = cPoint.y; outSideJoint2M[3][2] = cPoint.z; outSideJoint2M[3][3] = 1;
	MQuaternion outSideJoint2Q;
	outSideJoint2Q = outSideJoint2M;
	MMatrix outSideJoint1M;
	outSideJoint1M[0][0] = -transPoint1.x; outSideJoint1M[0][1] = -transPoint1.y; outSideJoint1M[0][2] = -transPoint1.z; outSideJoint1M[0][3] = 0;
	outSideJoint1M[1][0] = joint1UpVector.x; outSideJoint1M[1][1] = joint1UpVector.y; outSideJoint1M[1][2] = joint1UpVector.z; outSideJoint1M[1][3] = 0;
	outSideJoint1M[2][0] = -planeTangentVector.x; outSideJoint1M[2][1] = -planeTangentVector.y; outSideJoint1M[2][2] = -planeTangentVector.z; outSideJoint1M[2][3] = 0;
	outSideJoint1M[3][0] = cPoint.x; outSideJoint1M[3][1] = cPoint.y; outSideJoint1M[3][2] = cPoint.z; outSideJoint1M[3][3] = 1;
	MQuaternion outSideJoint1Q;
	outSideJoint1Q = outSideJoint1M;
	MVector outSideCenterX = planeTangentVector ^ planeUPVector;
	MMatrix outSideCenterM;
	outSideCenterM[0][0] = outSideCenterX.x; outSideCenterM[0][1] = outSideCenterX.y; outSideCenterM[0][2] = outSideCenterX.z; outSideCenterM[0][3] = 0;
	outSideCenterM[1][0] = planeUPVector.x; outSideCenterM[1][1] = planeUPVector.y; outSideCenterM[1][2] = planeUPVector.z; outSideCenterM[1][3] = 0;
	outSideCenterM[2][0] = -planeTangentVector.x; outSideCenterM[2][1] = -planeTangentVector.y; outSideCenterM[2][2] = -planeTangentVector.z; outSideCenterM[2][3] = 0;
	outSideCenterM[3][0] = cPoint.x; outSideCenterM[3][1] = cPoint.y; outSideCenterM[3][2] = cPoint.z; outSideCenterM[3][3] = 1;
	MQuaternion outSideCenterQ;
	outSideCenterQ = outSideCenterM;
	if (dot(outSideJoint2Q, outSideCenterQ) < 0)
		outSideJoint2Q *= -1;
	if (dot(outSideJoint1Q, outSideCenterQ) < 0)
		outSideJoint1Q *= -1;
	MQuaternion withoutTwistUpOffQ = outSideJoint2Q * joint2WithoutTwistQ.conjugate();
	MMatrix joint1UpOffM = outSideJoint1M * joint1Mat.inverse();
	influenceInfo[index].influenceVertexes.clear();
	MMatrix joint1InvOutsideMat = outSideJoint1M.inverse();
	MMatrix joint2InvOutsideMat = outSideJoint2M.inverse();

	int(*mapping)(int, std::vector<int>*);
	auto &weightIndexes = influenceInfo[index].weightIndexes;
	int l;
	if (skinPtr->isWeightsLocked)
	{
		l = weightIndexes.size();
		mapping = [](int i, std::vector<int>* weightIndexes) {return weightIndexes->operator[](i); };
	}
	else
	{
		l = sourcePoints.length();
		mapping = [](int i, std::vector<int>*) {return i; };
	}
	for (int id = 0; id < l; ++id)
	{
		vertexBindInfo bindInfo;
		int i = mapping(id, &weightIndexes);
		MPoint &pnt = sourcePoints[i];
		if (planeNormal*(pnt - cPoint) >= 0)
		{
			MPoint p = pnt * joint2InvOutsideMat;
			double tw;
			if (p.x >= jointLength)
				tw = p.x / jointLength;
			else if (p.y < 0)
			{
				if (p.x < 0)
				{
					MVector Y(-p.x, -p.y, 0);
					double deepY = Y.length();
					double outsideWeightCut = deepY * abs(cosPlaneJoint)*outsideCutProportion / jointLength;
					if (outsideWeightCut > 0.5)
						outsideWeightCut = 0.5;
					Y *= outSideJoint2M;
					Y.normalize();
					MVector X = planeTangentVector ^ Y;
					MMatrix mat;
					mat[0][0] = X.x; mat[0][1] = X.y; mat[0][2] = X.z; mat[0][3] = 0;
					mat[1][0] = Y.x; mat[1][1] = Y.y; mat[1][2] = Y.z; mat[1][3] = 0;
					mat[2][0] = -planeTangentVector.x; mat[2][1] = -planeTangentVector.y; mat[2][2] = -planeTangentVector.z; mat[2][3] = 0;
					mat[3][0] = 0; mat[3][1] = 0; mat[3][2] = 0; mat[3][3] = 1;
					MQuaternion quat;
					quat = mat;
					tw = getQuatWeight(outSideJoint2Q, outSideCenterQ,quat);
					tw *= outsideWeightCut;
					p.y = -deepY;
				}
				else
				{
					double outsideWeightCut = -p.y*abs(cosPlaneJoint)*outsideCutProportion / jointLength;
					if (outsideWeightCut > 0.5)
						outsideWeightCut = 0.5;
					double realW = outSideExtendWeight2 * (1 - outsideWeightCut) + outsideWeightCut;
					double midX = jointLength * realW;
					if (p.x < midX)
						tw = p.x / midX * (realW - outsideWeightCut) + outsideWeightCut;
					else tw = (p.x - midX) / (jointLength - midX)*(1 - realW) + realW;
				}
			}
			else
			{
				double StartX = p.y*cosPlaneJoint / sinPlaneJoint;
				tw = (p.x - StartX) / (jointLength - StartX);
			}
			if (tw < 0)
				tw = 0;
			MQuaternion quat;
			if (tw < twistCutWeight2)
			{
				double w = (tw / twistCutWeight2)*0.5 + 0.5;
				w = (1 - w) * 2;
				double _w = w * w / 2;
				w = 1 - _w;
				quat = offsetQuat * w;
				quat.w += (1 - w);
				quat.normalizeIt();
			}
			else quat = offsetQuat;
			p *= withoutTwistUpOffQ;
			p *= quat.conjugateIt();
			bindInfo = vertexBindInfo{ double(tw),double(p.y),double(p.z) };
		}
		else
		{
			MPoint p = pnt * joint1InvOutsideMat;
			double tw;
			if (-p.x >= parentJointLength)
				tw = -p.x / parentJointLength;
			else if (p.y < 0)
			{
				if (p.x > 0)
				{
					MVector Y(-p.x, -p.y, 0);
					double deepY = Y.length();
					double outsideWeightCut = deepY * abs(cosPlaneJoint)*outsideCutProportion / parentJointLength;
					if (outsideWeightCut > 0.5)
						outsideWeightCut = 0.5;
					Y *= outSideJoint1M;
					Y.normalize();
					MVector X = planeTangentVector ^ Y;
					MMatrix mat;
					mat[0][0] = X.x; mat[0][1] = X.y; mat[0][2] = X.z; mat[0][3] = 0;
					mat[1][0] = Y.x; mat[1][1] = Y.y; mat[1][2] = Y.z; mat[1][3] = 0;
					mat[2][0] = -planeTangentVector.x; mat[2][1] = -planeTangentVector.y; mat[2][2] = -planeTangentVector.z; mat[2][3] = 0;
					mat[3][0] = 0; mat[3][1] = 0; mat[3][2] = 0; mat[3][3] = 1;
					MQuaternion quat;
					quat = mat;
					tw = getQuatWeight(outSideJoint1Q, outSideCenterQ, quat);
					tw *= outsideWeightCut;
					p.y = -deepY;
				}
				else
				{
					double outsideWeightCut = -p.y*abs(cosPlaneJoint)*outsideCutProportion / parentJointLength;
					if (outsideWeightCut > 0.5)
						outsideWeightCut = 0.5;
					double realW = outSideExtendWeight1 * (1 - outsideWeightCut) + outsideWeightCut;
					double midX = parentJointLength * realW;
					if (-p.x < midX)
						tw = -p.x / midX * (realW - outsideWeightCut) + outsideWeightCut;
					else tw = (-p.x - midX) / (parentJointLength - midX)*(1 - realW) + realW;
				}
			}
			else
			{
				double StartX = p.y*cosPlaneJoint / sinPlaneJoint;
				tw = (-p.x - StartX) / (parentJointLength - StartX);
			}
			if (tw < 0)
				tw = 0;
			if (tw < twistCutWeight1)
			{
				double w = (1 - tw / twistCutWeight1)*0.5;
				w *= 2;
				w = w * w / 2;
				MQuaternion quat = offsetQuat * w;
				quat.w += (1 - w);
				quat.normalizeIt();
				p *= quat.conjugateIt();
			}
			p *= joint1UpOffM;
			bindInfo = vertexBindInfo{ double(-tw),double(p.y),double(p.z) };
		}
		influenceInfo[index].influenceVertexes.push_back(bindInfo);
	}
}