#pragma once
#define WITHCGAL
#include <maya/MMessage.h>
#include <maya/MPxCommand.h>
#include <maya/MArgList.h>
#include <maya/MSyntax.h>
#include "skinManipulator.h"
#include <maya/MDagPathArray.h>

#ifdef WITHCGAL
#include <CGAL/Simple_cartesian.h>
#include <CGAL/Cartesian.h>
#include <CGAL/Polyhedron_3.h>
#include <CGAL/HalfedgeDS_vector.h>
#include <CGAL/Polyhedron_incremental_builder_3.h>
#include <CGAL/Modifier_base.h>
#endif
class emptyPoint
{
public:
	template<typename... T>
	emptyPoint(T ...arg){}
	template<typename R,typename... T>
	emptyPoint& operator=(T... t)
	{
		return *this;
	}
};

#ifdef WITHCGAL
class MyPoint :public CGAL::Simple_cartesian<float>::Point_3 
{
public:
	template<typename... T>
	MyPoint(T ...arg):CGAL::Simple_cartesian<float>::Point_3(arg...){}
	template<typename... T>
	MyPoint& operator=(T... t)
	{
		CGAL::Simple_cartesian<float>::Point_3::operator=(t...);
		return this*;
	}
};
class MyCartesian :public CGAL::Simple_cartesian<float>
{
public:
	typedef emptyPoint Point_3;
};


class MyItem
{
public:
	template<class Refs,class Traits>
	struct Vertex_wrapper {
		typedef typename Traits::Point_3 Point;
		typedef CGAL::HalfedgeDS_vertex_base<Refs, CGAL::Tag_true, Point>Vertex;
	};
	template <class Refs,class Traits>
	struct Halfedge_wrapper {
		typedef CGAL::HalfedgeDS_halfedge_base<Refs, CGAL::Tag_false, CGAL::Tag_true, CGAL::Tag_true>Halfedge;
	};
	template<class Refs,class Traits>
	struct Face_wrapper {
		typedef typename Traits::Plane_3 Plane;
		typedef CGAL::HalfedgeDS_face_base<Refs, CGAL::Tag_false> Face;
	};
};
#endif

class collisionOffsetSkinCmd :public MPxCommand
{
public:
#ifdef WITHCGAL
	typedef CGAL::Polyhedron_3<MyCartesian, MyItem, CGAL::HalfedgeDS_vector> Poly;

	typedef Poly::Halfedge_handle Halfedge_handle;
	typedef Poly::Point_3 Point;
#endif
	collisionOffsetSkinCmd() {};
	MStatus doIt(const MArgList &args)override;
	static skinManipContext* manip;
	static MSyntax createSyntax();
	static void *creator();
	bool isUndoable()const { return false; }
#ifdef WITHCGAL
	void initWeight(MObject &nodeObj, int smooth);
	void smoothWeight(MObject &nodeObj, int smooth);
#endif
	static void rebindAll();
	static MCallbackId toolchangeID;
	static void initWeightPaintEvent();
	static void uninitWeightPaintEvent();
#ifdef WITHCGAL
	static void mayaObjToPoly(MObject &obj, Poly &poly);
#endif
};