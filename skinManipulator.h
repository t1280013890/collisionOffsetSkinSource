#pragma once
#include <maya/MPxManipContainer.h>
#include <maya/MPxSelectionContext.h>
#include <maya/MPxContextCommand.h>
#include <maya/MCallbackIdArray.h>


class skinManip :public MPxManipContainer
{
public:
	skinManip();
	static void* creator();
	static MStatus initialize();
	MStatus createChildren() override;
	virtual void drawUI(MHWRender::MUIDrawManager& drawManager, const MHWRender::MFrameContext& frameContext)const override;
	MStatus connectPlugArray(MPlugArray&plug, MMatrix&parentMat, MMatrix&globaleOff);
	MManipData rotationChangedCallback(unsigned index);
	MManipData rotateToManipCallback(unsigned index);
	int rotatePlugIndex;
	MMatrix rotateManipMat;
	MMatrix parentMat;
	MMatrix parentMatInv;
	MMatrix offsetMat;
	MMatrix globalOffMat;
	MMatrix globalOffMatInv;
	MDagPath rotateManipDagPath;
	static MTypeId id;
};
class skinManipContext :public MPxSelectionContext
{
public:
	skinManipContext();
	void toolOnSetup(MEvent &event)override;
	void toolOffCleanup()override;
	void refreshManipulators();
	static void updateManipulators(void *data);
	MCallbackId id1;
};
class skinManipContextCmd :public MPxContextCommand
{
public:
	skinManipContextCmd() {};
	MPxContext* makeObj()override;
	static void *creator();
};