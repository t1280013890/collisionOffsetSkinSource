#pragma once

#include <time.h>
#include <maya/MTypeId.h>
#include <maya/MPxSkinCluster.h>
#include <vector>
#include <maya/MPointArray.h>
#include <maya/MMatrix.h>
#include <maya/MPlugArray.h>
#include <maya/MDoubleArray.h>
#include <Windows.h>
#include <tuple>


struct vertexBindInfo
{

	double weight;
	double offsetY;
	double offsetZ;
};
struct InfluenceInfo
{
	MMatrix bindMat;
	MMatrix bindParentMat;
	MMatrix bindParentOffsetMat;
	MMatrix mat;

	MMatrix parentMat;
	MMatrix globalOffsetMat;
	std::vector<int> weightIndexes;
	std::vector<double>weights;
	std::vector<vertexBindInfo> influenceVertexes;
	double jointLength;
	double parentJointLength;
	double twistCutWeight1;
	double twistCutWeight2;
	double outSideExtendWeight1;
	double outSideExtendWeight2;
	double outsideCutProportion;
	double sideSmooth1;
	double sideSmooth2;
	double collisionDeep;
	double collisionHeight;
	double collisionOffset;
	double outsideHeight;
	double startAngleCos;
	double endAngleCos;
	double collisionSmooth1;
	double collisionSmooth2;
	char itemNeedBind = -1;
	bool isLinear;
};
struct longPoint
{
	longPoint() {}
	longPoint(const int&X, const int&Y, const int& Z) :x(X), y(Y), z(Z) {}
	long long x;
	long long y;
	long long z;
};
class collisionOffsetSkin :public MPxSkinCluster
{

public:
	typedef std::pair<collisionOffsetSkin*, InfluenceInfo*> DeformThreadParameter;
	typedef std::pair<collisionOffsetSkin*, int> BindThreadParameter;
	collisionOffsetSkin();
	~collisionOffsetSkin();
	static void* creator();
	static MStatus initialize();
	MStatus compute(const MPlug&, MDataBlock&)override;
	MStatus deform(MDataBlock& block, MItGeometry&iter, const MMatrix&mat, unsigned int multiIndex)override;
	MStatus setDependentsDirty(const MPlug&plug, MPlugArray&plugArray)override;
	void refreshInfluenceInfo();
	void refreshWeights(MDataBlock& data);
	void postConstructor()override;
	
	friend class collisionOffsetSkinCmd;
	static MObject inGeoChangeEventP;
	static MObject influenceInfoP;
	static MObject bindMatP;
	static MObject bindParentMatP;
	static MObject bindParentOffsetP;
	static MObject globalOffsetMatP;
	static MObject jointLengthP;
	static MObject parentJointLengthP;
	static MObject twistCutWeight1P;
	static MObject twistCutWeight2P;
	static MObject outsideCutProportionP;
	static MObject collisionDeepP;
	static MObject collisionHeightP;
	static MObject collisionOffsetP;
	static MObject outsideHeightP;
	static MObject startAngleCosP;
	static MObject endAngleCosP;
	static MObject collisionSmooth1P;
	static MObject collisionSmooth2P;
	static MObject sideSmooth1P;
	static MObject sideSmooth2P;
	static MObject outSideExtendWeight1P;
	static MObject outSideExtendWeight2P;
	static MObject isLinearP;

	static const MTypeId id;
	static void NTAPI calcuThread(_Inout_ PTP_CALLBACK_INSTANCE, _Inout_opt_ PVOID, _Inout_ PTP_WORK);

	static void NTAPI bindThread(_Inout_ PTP_CALLBACK_INSTANCE, _Inout_opt_ PVOID, _Inout_ PTP_WORK);
private:
	enum plugEnum
	{
		bindMatE = 1,
		bindParentOffsetE,
		bindParentMatE,
		globalOffsetMatE,
		jointLengthE,
		parentJointLengthE,
		twistCutWeight1E,
		twistCutWeight2E,
		outsideCutProportionE,
		collisionDeepE,
		collisionHeightE,
		collisionOffsetE,
		outsideHeightE,
		startAngleCosE,
		endAngleCosE,
		collisionSmooth1E,
		collisionSmooth2E,
		sideSmooth1E,
		sideSmooth2E,
		outSideExtendWeight1E,
		outSideExtendWeight2E,
		isLinearE,
	};
	std::vector<std::pair<plugEnum, int>> dirtyInfPlugEnumArray;

	std::vector<InfluenceInfo> influenceInfo;
	int needBind = false;
	bool needRefresh = true;
	bool isWeightsLocked;
	bool paintWeightsDirty = false;
	bool weightPlugDirty = false;

	std::vector<longPoint> resultPoints;
	MObject blendWeightsObj;
	MPlug paintWeightsP;

	MPointArray sourcePoints;
	MDoubleArray paintWeights;
	PTP_POOL threadPool;
	TP_CALLBACK_ENVIRON env;
	std::vector<std::pair<DeformThreadParameter, PTP_WORK>> deformParameters;
	std::vector<std::pair<BindThreadParameter, PTP_WORK>> bindParameters;

};