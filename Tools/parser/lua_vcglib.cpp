#ifdef __cplusplus
extern "C" {
#endif

#include "lua.h"
#include "lualib.h"
#include "lauxlib.h"

#ifdef __cplusplus
}
#endif


#include <iostream>
#include <string>
// vcglib headers
#include <GL/gl.h>
#include <vcg/complex/complex.h>
#include <wrap/io_trimesh/import.h>
#include <wrap/io_trimesh/import_dae.h>
#include <wrap/io_trimesh/export_vrml.h>

using namespace std;
using namespace vcg;

class MyVertex; class MyEdge; class MyFace;
struct MyUsedTypes : public vcg::UsedTypes<vcg::Use<MyVertex>   ::AsVertexType,
                                             vcg::Use<MyEdge>     ::AsEdgeType,
                                             vcg::Use<MyFace>     ::AsFaceType>{};
class MyVertex  : public vcg::Vertex< MyUsedTypes, vcg::vertex::Coord3f, vcg::vertex::Normal3f, vcg::vertex::BitFlags  >{};
class MyFace    : public vcg::Face<   MyUsedTypes, vcg::face::FFAdj,  vcg::face::VertexRef, vcg::face::BitFlags > {};
class MyEdge    : public vcg::Edge<   MyUsedTypes> {};
class MyMesh    : public vcg::tri::TriMesh< std::vector<MyVertex>, std::vector<MyFace> , std::vector<MyEdge>  > {};

static int lua_vcglib_dae2vrml(lua_State *L) {
  std::string daefilename(luaL_checkstring(L, 1));
//  std::cout << daefilename << std::endl;
  std::string vrmlfilename(daefilename);
  vrmlfilename.replace(vrmlfilename.length() - 3, 3, "wrl");
//  std::cout << vrmlfilename << std::endl;
  MyMesh m;
  int mask = 0;
  tri::io::InfoDAE  info;
	if (!tri::io::ImporterDAE<MyMesh>::LoadMask(daefilename.c_str(), info))
	  return luaL_error(L, "Can not load Mask");
//  m.Enable(info.mask);
	int result = tri::io::ImporterDAE<MyMesh>::Open(m, daefilename.c_str(),info);
	if (result != tri::io::ImporterDAE<MyMesh>::E_NOERROR)
	{
		cout << "DAE Opening Error" << tri::io::ImporterDAE<MyMesh>::ErrorMsg(result) << endl;
		return luaL_error(L, "load file error");
	}

	result = tri::io::ExporterWRL<MyMesh>::Save(m,vrmlfilename.c_str(),mask, 0);
	if(result!=0)
	{
		return luaL_error(L, tri::io::ExporterWRL<MyMesh>::ErrorMsg(result));
	}

  return 1;
}


static int lua_vcglib_stl2vrml(lua_State *L) {
  std::string stlfilename(luaL_checkstring(L, 1));
  std::string vrmlfilename(stlfilename);
  vrmlfilename.replace(vrmlfilename.length() - 3, 3, "wrl");
//  std::cout << vrmlfilename << std::endl;
  MyMesh m;
  int mask = 0;
	if (!tri::io::ImporterSTL<MyMesh>::LoadMask(stlfilename.c_str(), mask))
	  return luaL_error(L, "Can not load Mask");

	int result = tri::io::ImporterSTL<MyMesh>::Open(m, stlfilename.c_str(), mask, 0);
	if (result != 0)
	{
		cout << "STL Opening Error" << tri::io::ImporterSTL<MyMesh>::ErrorMsg(result) << endl;
		return luaL_error(L, "load file error");
	}

	result = tri::io::ExporterWRL<MyMesh>::Save(m,vrmlfilename.c_str(),mask, 0);
	if(result!=0)
	{
		return luaL_error(L, tri::io::ExporterWRL<MyMesh>::ErrorMsg(result));
	}

  return 1;
}

static int lua_vcglib_sdf2vrml(lua_State *L) {
  std::string sdffilename(luaL_checkstring(L, 1));
  cout << sdffilename << endl;
  std::string vrmlfilename(sdffilename);
  vrmlfilename.replace(vrmlfilename.length() - 3, 3, "wrl");
  cout << vrmlfilename << endl;

  MyMesh m;
  int mask = 0;
	tri::io::ImporterPLY<MyMesh>::LoadMask(sdffilename.c_str(), mask); 
	// small patch to allow the loading of per wedge color into faces.  
	if(mask & tri::io::Mask::IOM_WEDGCOLOR) mask |= tri::io::Mask::IOM_FACECOLOR;
 
	int result = tri::io::ImporterPLY<MyMesh>::Open(m, sdffilename.c_str(), mask, 0);
	if (result != 0) // all the importers return 0 on success
	{
		if(tri::io::ImporterPLY<MyMesh>::ErrorCritical(result) )
			return luaL_error(L, tri::io::ImporterPLY<MyMesh>::ErrorMsg(result));
	}

	result = tri::io::ExporterWRL<MyMesh>::Save(m,vrmlfilename.c_str(),mask, 0);
	if(result!=0)
	{
		return luaL_error(L, tri::io::ExporterWRL<MyMesh>::ErrorMsg(result));
	}

  return 1;
}

static const struct luaL_reg vcg_lib [] = {
  {"dae2vrml", lua_vcglib_dae2vrml},
  {"stl2vrml", lua_vcglib_stl2vrml},
  {"sdf2vrml", lua_vcglib_sdf2vrml},
  {NULL, NULL}
};

extern "C"
int luaopen_vcglib(lua_State *L) {
  luaL_register(L, "vcglib", vcg_lib);
  return 1;
}
