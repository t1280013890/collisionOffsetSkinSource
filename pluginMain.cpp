#include "skin.h"
#include "skinCmd.h"
#include <maya/MFnPlugin.h>
#include "skinManipulator.h"
#include <maya/MGlobal.h>
#include "pythonScript.h"


MStatus initializePlugin( MObject obj )
{ 
	MFnPlugin plugin( obj, "newSkin", "2018", "Any");
	plugin.registerNode(
		"collisionOffsetSkin",
		collisionOffsetSkin::id,
		&collisionOffsetSkin::creator,
		&collisionOffsetSkin::initialize,
		MPxNode::kSkinCluster);
	plugin.registerCommand("collisionOffsetSkin", collisionOffsetSkinCmd::creator, collisionOffsetSkinCmd::createSyntax);
	plugin.registerContextCommand("skinManipContext", &skinManipContextCmd::creator);
	plugin.registerNode("skinManip", skinManip::id, &skinManip::creator, &skinManip::initialize, MPxNode::kManipContainer);
	MGlobal::executePythonCommand(pythonScript1);
	MGlobal::executePythonCommand(pythonScript2);
	MGlobal::executePythonCommand(pythonScript3);
	MGlobal::executePythonCommand(initializeCmd);
	MGlobal::executeCommand(AETemplateCmd);
	collisionOffsetSkinCmd::initWeightPaintEvent();
	return MS::kSuccess;
}
MStatus uninitializePlugin( MObject obj )
{
	MFnPlugin plugin( obj );
	plugin.deregisterNode(collisionOffsetSkin::id);
	plugin.deregisterCommand("collisionOffsetSkin");
	plugin.deregisterContextCommand("skinManipContext");
	plugin.deregisterNode(skinManip::id);
	MGlobal::executePythonCommand(uninitializeCmd);
	collisionOffsetSkinCmd::uninitWeightPaintEvent();
	return MS::kSuccess;
}
