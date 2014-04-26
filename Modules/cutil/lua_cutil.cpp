/**
 * Lua module to expose some common c utilties
 *
 * University of Pennsylvania
 * 2010
 */

#include <lua.hpp>
#include <stdlib.h>
#include <stdint.h>
#include <math.h>
#include <vector>
#include <string>
#include <map>
#ifdef TORCH
#include <torch/luaT.h>
#ifdef __cplusplus
extern "C"
{
#endif
#include <torch/TH/TH.h>
#ifdef __cplusplus
}
#endif
#endif

const char ascii_lut[] = "0123456789abcdef";

//Pack 32 possible label color bits into 6 types (for monitoring)
//Priority: Orange > Yellow > Cyan > White > Green > Black
//Black:0, Orange:1, Yellow:2, Cyan:4, Green:8, White: 16
//Map : 0,      1,       2,      3,       4,        5

const int8_t label_color_pack_lut[]=
	{ 0, 1, 
	  2, 1, 
          3, 1, 2, 1, 
          4, 1, 2, 1, 3, 1, 2, 1, 
          5, 1, 2, 1, 3, 1, 2, 1, 5, 1, 2, 1, 3, 1, 2, 1};

const int8_t label_color_unpack_lut[]={0,1,2,4,8,16};

std::map<std::string, int> dataTypeMap;

// use matlab support datatype names
void init_dataTypeMap() {
  dataTypeMap["int8"]     = sizeof(int8_t);
  dataTypeMap["int16"]    = sizeof(int16_t);
  dataTypeMap["int32"]    = sizeof(int32_t);
  dataTypeMap["int64"]    = sizeof(int64_t);
  dataTypeMap["uint8"]    = sizeof(uint8_t);
  dataTypeMap["uint16"]   = sizeof(uint16_t);
  dataTypeMap["uint32"]   = sizeof(uint32_t);
  dataTypeMap["uint64"]   = sizeof(uint64_t);
  dataTypeMap["single"]   = sizeof(float);
  dataTypeMap["double"]   = sizeof(double);
}


static int lua_array2string(lua_State *L) {
  uint8_t *data = (uint8_t *) lua_touserdata(L, 1);
  if ((data == NULL) || !lua_islightuserdata(L, 1)) {
    return luaL_error(L, "Input image not light user data");
  }
  
  int width = luaL_checkint(L, 2);
  int height = luaL_checkint(L, 3);
  std::string dtype(luaL_checkstring(L, 4));
  std::string name(luaL_checkstring(L, 5));

  std::map<std::string, int>::iterator idataTypeMap = dataTypeMap.find(dtype);
  if (idataTypeMap == dataTypeMap.end()) {
    return luaL_error(L, "unkown dtype: %s", dtype.c_str());
  }
  int nbytes = idataTypeMap->second;
  int size = width*height * nbytes;
  char cdata[(2*size) + 1];

  int ind = 0;
  int cind = 0;
  while (ind < size) {
    cdata[cind] = ascii_lut[(data[ind] & 0xf0) >> 4];
    cdata[cind+1] = ascii_lut[(data[ind] & 0x0f)];
    ind += 1;
    cind += 2;
  }
  cdata[(2*size)] = '\0';

  // create lua table
  lua_createtable(L, 0, 5);

  lua_pushstring(L, "name");
  lua_pushstring(L, name.c_str());
  lua_settable(L, -3);

  lua_pushstring(L, "width");
  lua_pushnumber(L, width);
  lua_settable(L, -3);

  lua_pushstring(L, "height");
  lua_pushnumber(L, height);
  lua_settable(L, -3);

  lua_pushstring(L, "dtype");

  lua_createtable(L, 0, 2);
  lua_pushstring(L, "name");
  lua_pushstring(L, dtype.c_str());
  lua_settable(L, -3);

  lua_pushstring(L, "nbytes");
  lua_pushnumber(L, nbytes);
  lua_settable(L, -3);

  lua_settable(L, -3);

  lua_pushstring(L, "data");
  lua_pushstring(L, cdata);
  lua_settable(L, -3);

  return 1;
}

static int lua_string2userdata(lua_State *L) {
  uint8_t *dout = (uint8_t *) lua_touserdata(L, 1);
  if ((dout == NULL) || !lua_islightuserdata(L, 1)) {
    return luaL_error(L, "output argument not light user data");
  }

  const char *cdata = luaL_checkstring(L, 2);

  int ind = 0;
  int cind = 0;
  while (cdata[cind] != '\0' && cdata[cind+1] != '\0') {
    uint8_t bh = cdata[cind] >= 'a' ? cdata[cind] - 'a' + 10 : cdata[cind] - '0';
    uint8_t bl = cdata[cind+1] >= 'a' ? cdata[cind+1] - 'a' + 10 : cdata[cind+1] - '0';
    dout[ind] = (uint8_t)((bh<<4) | bl);

    ind += 1;
    cind += 2;
  }

  return 1;
}




//Custom function for YUYV, where we throw out every other line

static int lua_array2string2(lua_State *L) {
  uint8_t *data = (uint8_t *) lua_touserdata(L, 1);
  if ((data == NULL) || !lua_islightuserdata(L, 1)) {
    return luaL_error(L, "Input image not light user data");
  }
  
  int width = luaL_checkint(L, 2);
  int height = luaL_checkint(L, 3);
  std::string dtype(luaL_checkstring(L, 4));
  std::string name(luaL_checkstring(L, 5));

  std::map<std::string, int>::iterator idataTypeMap = dataTypeMap.find(dtype);
  if (idataTypeMap == dataTypeMap.end()) {
    return luaL_error(L, "unkown dtype: %s", dtype.c_str());
  }
  int nbytes = idataTypeMap->second;
  int size = width*height * nbytes / 2;//half the size
  char cdata[(2*size) + 1];

  int ind = 0;
  int cind = 0;

  for (int i=0;i<height/2;i++){
    for (int j=0;j<width*nbytes;j++){
      cdata[cind] = ascii_lut[(data[ind] & 0xf0) >> 4];
      cdata[cind+1] = ascii_lut[(data[ind] & 0x0f)];
      ind += 1;
      cind += 2;
    }
    ind += width*nbytes;//skip every other line
  }

  cdata[(2*size)] = '\0';

  // create lua table
  lua_createtable(L, 0, 5);

  lua_pushstring(L, "name");
  lua_pushstring(L, name.c_str());
  lua_settable(L, -3);

  lua_pushstring(L, "width");
  lua_pushnumber(L, width);
  lua_settable(L, -3);

  lua_pushstring(L, "height");
  lua_pushnumber(L, height);
  lua_settable(L, -3);

  lua_pushstring(L, "dtype");

  lua_createtable(L, 0, 2);
  lua_pushstring(L, "name");
  lua_pushstring(L, dtype.c_str());
  lua_settable(L, -3);

  lua_pushstring(L, "nbytes");
  lua_pushnumber(L, nbytes);
  lua_settable(L, -3);

  lua_settable(L, -3);

  lua_pushstring(L, "data");
  lua_pushstring(L, cdata);
  lua_settable(L, -3);

  return 1;
}

//function for halved yuyv data string
static int lua_string2userdata2(lua_State *L) {
  uint8_t *dout = (uint8_t *) lua_touserdata(L, 1);
  if ((dout == NULL) || !lua_islightuserdata(L, 1)) {
    return luaL_error(L, "output argument not light user data");
  }

  const char *cdata = luaL_checkstring(L, 2);
  int width = luaL_checkint(L, 3);
  int height = luaL_checkint(L, 4);

  int ind = 0;
  int cind = 0;
  int nbyte = 4; //this function is only used for yuyv

  for (int i=0;i<height/2;i++){
    for (int j=0;j<width*nbyte;j++){
      uint8_t bh = cdata[cind] >= 'a' ? 
		cdata[cind] - 'a' + 10 : cdata[cind] - '0';
      uint8_t bl = cdata[cind+1] >= 'a' ? 
		cdata[cind+1] - 'a' + 10 : cdata[cind+1] - '0';
      dout[ind] = (uint8_t)((bh<<4) | bl);
      ind += 1;
      cind += 2;
    }
    //Copy previous line
    for (int j=0;j<width*nbyte;j++){
      dout[ind] = dout[ind-width*nbyte];
      ind += 1;
    }
  }
  return 1;
}

//Label-specific string conversion function
//Exploit label range (0-31) to pack a pixel into a single byte

static int lua_label2string(lua_State *L) {
  uint8_t *data = (uint8_t *) lua_touserdata(L, 1);
  if ((data == NULL) || !lua_islightuserdata(L, 1)) {
    return luaL_error(L, "Input image not light user data");
  }
  
  int arr_size = luaL_checkint(L, 2);
  std::string dtype(luaL_checkstring(L, 3));
  std::string name(luaL_checkstring(L, 4));

  std::map<std::string, int>::iterator idataTypeMap = dataTypeMap.find(dtype);
  if (idataTypeMap == dataTypeMap.end()) {
    return luaL_error(L, "unkown dtype: %s", dtype.c_str());
  }
  int nbytes = idataTypeMap->second;

  int size = arr_size * nbytes;
  char cdata[size + 1];

  int ind = 0;
  int cind = 0;
  while (ind < size) {
    cdata[cind] = '0'+data[ind];
    ind += 1;
    cind += 1;
  }
  cdata[size] = '\0';

  // create lua table
  lua_createtable(L, 0, 5);

  lua_pushstring(L, "name");
  lua_pushstring(L, name.c_str());
  lua_settable(L, -3);

  lua_pushstring(L, "size");
  lua_pushnumber(L, arr_size);
  lua_settable(L, -3);

  lua_pushstring(L, "dtype");

  lua_createtable(L, 0, 2);
  lua_pushstring(L, "name");
  lua_pushstring(L, dtype.c_str());
  lua_settable(L, -3);

  lua_pushstring(L, "nbytes");
  lua_pushnumber(L, nbytes);
  lua_settable(L, -3);

  lua_settable(L, -3);

  lua_pushstring(L, "data");
  lua_pushstring(L, cdata);
  lua_settable(L, -3);

  return 1;
}

static int lua_string2label(lua_State *L) {
  uint8_t *dout = (uint8_t *) lua_touserdata(L, 1);
  if ((dout == NULL) || !lua_islightuserdata(L, 1)) {
    return luaL_error(L, "output argument not light user data");
  }
  const char *cdata = luaL_checkstring(L, 2);
  int ind = 0;
  int cind = 0;
  while (cdata[cind] != '\0' && cdata[cind+1] != '\0') {
    dout[ind] = cdata[cind] - '0';
    ind += 1;
    cind += 1;
  }
  return 1;
}



//Label-specific string conversion function
//Bin each label into 6 class 
//And pack two pixel into one byte

static int lua_label2string_double(lua_State *L) {
  uint8_t *data = (uint8_t *) lua_touserdata(L, 1);
  if ((data == NULL) || !lua_islightuserdata(L, 1)) {
    return luaL_error(L, "Input image not light user data");
  }
  
  int arr_size = luaL_checkint(L, 2);
  std::string dtype(luaL_checkstring(L, 3));
  std::string name(luaL_checkstring(L, 4));

  std::map<std::string, int>::iterator idataTypeMap = dataTypeMap.find(dtype);
  if (idataTypeMap == dataTypeMap.end()) {
    return luaL_error(L, "unkown dtype: %s", dtype.c_str());
  }
  int nbytes = idataTypeMap->second;

  int size = arr_size * nbytes/2;
  char cdata[size + 1];

  int ind = 0;
  int cind = 0;
  while (ind < size*2) {
    //bin label data (0-31) to 6 class (0-5)
    char pixel1=label_color_pack_lut[data[ind++]];
    char pixel2=label_color_pack_lut[data[ind++]];
    //encode two pixels into a single byte
    //Now we use ascii #48 ~ #122 (total 75 characters)
    cdata[cind++] = '0' + pixel1 * 6 + pixel2;
  }
  cdata[size] = '\0';

  // create lua table
  lua_createtable(L, 0, 5);

  lua_pushstring(L, "name");
  lua_pushstring(L, name.c_str());
  lua_settable(L, -3);

  lua_pushstring(L, "size");
  lua_pushnumber(L, size);
  lua_settable(L, -3);

  lua_pushstring(L, "dtype");

  lua_createtable(L, 0, 2);
  lua_pushstring(L, "name");
  lua_pushstring(L, dtype.c_str());
  lua_settable(L, -3);

  lua_pushstring(L, "nbytes");
  lua_pushnumber(L, nbytes);
  lua_settable(L, -3);

  lua_settable(L, -3);

  lua_pushstring(L, "data");
  lua_pushstring(L, cdata);
  lua_settable(L, -3);

  return 1;
}

static int lua_string2label_double(lua_State *L) {
  uint8_t *dout = (uint8_t *) lua_touserdata(L, 1);
  if ((dout == NULL) || !lua_islightuserdata(L, 1)) {
    return luaL_error(L, "output argument not light user data");
  }
  const char *cdata = luaL_checkstring(L, 2);
  int ind = 0;
  int cind = 0;
  while (cdata[cind] != '\0' && cdata[cind+1] != '\0') {
    //We just use ascii values #48 ~ #122 to pack data
    char buffer = cdata[cind] - '0';
    dout[ind] = label_color_unpack_lut[buffer / 6];
    dout[ind+1] = label_color_unpack_lut[buffer % 6];
    ind += 2;
    cind += 1;
  }
  return 1;
}

//Label-specific string conversion function
//Bin each label into 6 class 
//And run Run-length encoding

static int lua_label2string_rle(lua_State *L) {
  uint8_t *data = (uint8_t *) lua_touserdata(L, 1);
  if ((data == NULL) || !lua_islightuserdata(L, 1)) {
    return luaL_error(L, "Input image not light user data");
  }
  
  int arr_size = luaL_checkint(L, 2);
  std::string dtype(luaL_checkstring(L, 3));
  std::string name(luaL_checkstring(L, 4));

  std::map<std::string, int>::iterator idataTypeMap = dataTypeMap.find(dtype);
  if (idataTypeMap == dataTypeMap.end()) {
    return luaL_error(L, "unkown dtype: %s", dtype.c_str());
  }
  int nbytes = idataTypeMap->second;

  int size = arr_size * nbytes;
  char cdata[size + 1];

  int ind = 0;
  int cind = 0;
  
  int last_data=label_color_pack_lut[data[0]];
  int current_size=1;
  int total_byte = 0;
  ind++;

  while (ind < size) {
    int current_data = label_color_pack_lut[data[ind]];
    if (ind==size-1) {
      //Multiple data : "012345"
      cdata[cind++] = '0' + last_data;
      cdata[cind++] = '0' + current_size;
      total_byte+=2;
      ind++;
    }else{
      if ((current_data==last_data) && (current_size<75)){
        current_size++;
        ind++;
      }else{
//	printf("C%dS%d, ",last_data,current_size);
	if (current_size>1){
          //Multiple data : "012345"
          cdata[cind++] = '0' + last_data;
          cdata[cind++] = '0' + current_size;
          total_byte+=2;
	}else{
          //Single data : "6789:;"
          cdata[cind++] = '6' + last_data;
          total_byte+=1;
	}
        last_data = current_data;
        current_size = 1;
        ind++;
      }
    }
  }
  cdata[cind] = '\0';

  // create lua table
  lua_createtable(L, 0, 5);

  lua_pushstring(L, "name");
  lua_pushstring(L, name.c_str());
  lua_settable(L, -3);

  lua_pushstring(L, "size");
  lua_pushnumber(L, arr_size);
  lua_settable(L, -3);

  lua_pushstring(L, "bsize");
  lua_pushnumber(L, total_byte);
  lua_settable(L, -3);

  lua_pushstring(L, "dtype");

  lua_createtable(L, 0, 2);
  lua_pushstring(L, "name");
  lua_pushstring(L, dtype.c_str());
  lua_settable(L, -3);

  lua_pushstring(L, "nbytes");
  lua_pushnumber(L, nbytes);
  lua_settable(L, -3);

  lua_settable(L, -3);

  lua_pushstring(L, "data");
  lua_pushstring(L, cdata);
  lua_settable(L, -3);

  return 1;
}



static int lua_string2label_rle(lua_State *L) {
  uint8_t *dout = (uint8_t *) lua_touserdata(L, 1);
  if ((dout == NULL) || !lua_islightuserdata(L, 1)) {
    return luaL_error(L, "output argument not light user data");
  }
  const char *cdata = luaL_checkstring(L, 2);
  int ind = 0;
  int cind = 0;
  while (cdata[cind] != '\0' && cdata[cind+1] != '\0') {
    int data1 = cdata[cind++]-'0';
    if (data1>5) {
      //Single data
      dout[ind++] = label_color_unpack_lut[data1-5];
    }else{
      //Multiple data
      int len = cdata[cind++]-'0';
      for (int i=0;i<len;i++){
	dout[ind++]=label_color_unpack_lut[data1];
      }
    }
  }
  return 1;
}









static int lua_ptradd(lua_State *L) {
  uint8_t *ptr = (uint8_t *) lua_touserdata(L, 1);
  if ((ptr == NULL) || !lua_islightuserdata(L, 1)) {
    return luaL_error(L, "ptr argument not light user data");
  }

  int n = luaL_checkinteger(L, 2);
  std::string dtype(luaL_checkstring(L, 3));

  std::map<std::string, int>::iterator idataTypeMap = dataTypeMap.find(dtype);
  if (idataTypeMap == dataTypeMap.end()) {
    return luaL_error(L, "unkown dtype: %s", dtype.c_str());
  }
  int nbytes = n * (idataTypeMap->second);

  lua_pushlightuserdata(L, ptr + nbytes); 

  return 1;
}

static int lua_sizeof(lua_State *L) {
  std::string dtype(luaL_checkstring(L, 1));

  std::map<std::string, int>::iterator idataTypeMap = dataTypeMap.find(dtype);
  if (idataTypeMap == dataTypeMap.end()) {
    return luaL_error(L, "unkown dtype: %s", dtype.c_str());
  }
  int nbytes = idataTypeMap->second;

  lua_pushinteger(L, nbytes); 

  return 1;
}

static int lua_testarray(lua_State *L) {
  static uint32_t *ptr = NULL;
  int size = 640*480;
  if (ptr == NULL) {
    ptr = (uint32_t*)malloc(size*sizeof(uint32_t));
    for (int i = 0; i < size; i++) {
      ptr[i] = i;
    }
  }

  lua_pushlightuserdata(L, ptr);

  return 1;
}

static int lua_bitand(lua_State *L) {
  int a = luaL_checkint(L, 1); 
  int b = luaL_checkint(L, 2); 

  lua_pushinteger(L, a & b);

  return 1;
}


static int lua_bitor(lua_State *L) {
  int a = luaL_checkint(L, 1); 
  int b = luaL_checkint(L, 2); 

  lua_pushinteger(L, a | b);

  return 1;
}

static int lua_bitxor(lua_State *L) {
  int a = luaL_checkint(L, 1); 
  int b = luaL_checkint(L, 2); 

  lua_pushinteger(L, a ^ b);

  return 1;
}

static int lua_bitnot(lua_State *L) {
  int a = luaL_checkint(L, 1); 

  lua_pushinteger(L, ~a);

  return 1;
}

static int lua_bitset(lua_State *L) {
	int a = luaL_checkint(L, 1); 
	int bit = luaL_checkint(L, 2);
	int value = luaL_checkint(L, 3);

	if (value)
		lua_pushinteger(L, a | (1 << bit));
	else
		lua_pushinteger(L, a & ~(1 << bit));

	return 1;
}

static int lua_bitget(lua_State *L) {
	int a = luaL_checkint(L, 1); 
	int bit = luaL_checkint(L, 2);

	if (a & (1 << bit))
		lua_pushinteger(L, 1);
	else
		lua_pushinteger(L, 0);

	return 1;
}

static int lua_bitlshift(lua_State *L) {
	int a = luaL_checkint(L, 1); 
	int bit = luaL_checkint(L, 2);

  lua_pushinteger(L, a << bit);
  return 1;
}

static int lua_bitrshift(lua_State *L) {
	int a = luaL_checkint(L, 1); 
	int bit = luaL_checkint(L, 2);

  lua_pushinteger(L, a >> bit);
  return 1;
}

#ifdef TORCH
static int lua_torch_to_userdata (lua_State *L) {
  /* Grab the destination tensor */
  const char* t_name = luaT_typename(L,1);
  THByteTensor *b_t = (THByteTensor *)luaT_checkudata(L, 1, t_name);
  lua_pushlightuserdata(L,b_t->storage->data);
  return 1;
}

/* Copies string data to the torch tensor */
static int lua_string2tensor(lua_State *L) {
  /* Grab the source string */
  size_t n_str;
  char * src = (char*)lua_tolstring(L, 1, &n_str);
  /* Check that we have a src (should be true...) */
  if( src==NULL )
    return luaL_error(L, "Bad source string.");
  /* Grab the destination tensor */
  const char* t_name = luaT_typename(L,2);
  THByteTensor *b_t = (THByteTensor *)luaT_checkudata(L, 2, t_name);
	/* Safety checks */
	THArgCheck(b_t->nDimension == 1, 2, "Tensor must have one dimension.");
	THArgCheck(b_t->stride[0] == 1, 2, "Improper tensor memory layout (non-contiguous).");
	/* Find the destination pointer */
	unsigned char* dest = b_t->storage->data;
  if( dest == NULL )
    return luaL_error(L, "Bad destination.");
	/* Size adjustment */
	size_t type_sz;
	size_t byte_sz = b_t->size[0];
	char type = t_name[6];
  switch (type) {
  case 'B':
		type_sz = sizeof(unsigned char);
    break;
  case 'C':
		type_sz = sizeof(char);
    break;
  case 'S':
		type_sz = sizeof(short);
    break;
  case 'L':
		type_sz = sizeof(long);
    break;
  case 'I':
    type_sz = sizeof(int);
    break;
  case 'F':
    type_sz = sizeof(float);
    break;
  case 'D':
    type_sz = sizeof(double);
    break;
  default:
    return luaL_error(L, "Bad tensor type.");
  }
	/* Find the byte size of the storage segment */
	byte_sz *= type_sz;
	/* Add an offset */
	dest += b_t->storageOffset * type_sz;
	/* Number of string elements to copy */
  size_t n_copy = luaL_optint( L, 3, n_str/type_sz ) * type_sz;
  /* Optional offset (in elements) of the string */
  size_t offset = luaL_optint(L,4,0) * type_sz;
	/* Add the offset from copying the string */
	src += offset;
  /* Ensure that we are not overstepping our boundary of the tensor */
  if( n_copy > byte_sz || (n_copy+offset)>n_str ){
    return luaL_error(L,
				"Bad copy size.\nn_str: %d\nbyte_sz: %d\noffset: %d\nn_copy:%d",
				n_str,byte_sz,offset,n_copy);
	}
	/* Copy the data */
	memcpy( dest, src, n_copy );
  return 0;
}
/* Copies string data to the torch tensor */
static int lua_string2storage(lua_State *L) {
  /* Grab the source string */
  size_t n_str;
  char * src = (char*)lua_tolstring(L, 1, &n_str);
  /* Check that we have a src (should be true...) */
  if( src==NULL )
    return luaL_error(L, "Bad source string.");
  /* Grab the destination tensor */
  const char* s_name = luaT_typename(L,2);
  THByteStorage *b_s = (THByteStorage *)luaT_checkudata(L, 2, s_name);
	/* Find the destination pointer */
	unsigned char* dest = b_s->data;
  if( dest == NULL ){
    return luaL_error(L, "Bad destination.");
	}
	size_t byte_sz = b_s->size;
	/* Size adjustment */
	size_t type_sz;
	char type = s_name[6];
  switch (type) {
  case 'B':
		type_sz = sizeof(unsigned char);
    break;
  case 'C':
		type_sz = sizeof(char);
    break;
  case 'S':
		type_sz = sizeof(short);
    break;
  case 'L':
		type_sz = sizeof(long);
    break;
  case 'I':
    type_sz = sizeof(int);
    break;
  case 'F':
    type_sz = sizeof(float);
    break;
  case 'D':
    type_sz = sizeof(double);
    break;
  default:
    return luaL_error(L, "Bad tensor type.");
  }
	/* Find the byte size of the storage segment */
	byte_sz *= type_sz;
  /* Ensure that we are not overstepping our boundary of the tensor */
  if(n_str != byte_sz)
    return luaL_error(L, "Not same Storage size. str: %d storage: %d",n_str,byte_sz);
	/* Copy the data */
	memcpy( dest, src, n_str );
  return 0;
}
#endif

static const luaL_Reg cutil_lib [] = {
  {"array2string", lua_array2string},
  {"string2userdata", lua_string2userdata},
  {"array2string2", lua_array2string2},
  {"string2userdata2", lua_string2userdata2},
  {"label2string", lua_label2string},
  {"string2label", lua_string2label},
  {"label2string_double", lua_label2string_double},
  {"string2label_double", lua_string2label_double},
  {"label2string_rle", lua_label2string_rle},
  {"string2label_rle", lua_string2label_rle},
  {"ptr_add", lua_ptradd},
  {"bit_and", lua_bitand},
  {"bit_or", lua_bitor},
  {"bit_xor", lua_bitxor},
  {"bit_not", lua_bitnot},
	{"bit_set", lua_bitset},
	{"bit_get", lua_bitget},
	{"bit_lshift", lua_bitlshift},
	{"bit_rshift", lua_bitrshift},
  {"sizeof", lua_sizeof},
  {"test_array", lua_testarray},
#ifdef TORCH
  {"string2tensor", lua_string2tensor},
	{"string2storage", lua_string2storage},
  {"torch_to_userdata", lua_torch_to_userdata},
#endif
  {NULL, NULL}
};

#ifdef __cplusplus
extern "C"
#endif
int luaopen_cutil (lua_State *L) {
#if LUA_VERSION_NUM == 502
	luaL_newlib(L, cutil_lib);
#else
  luaL_register(L, "cutil", cutil_lib);
#endif
  init_dataTypeMap();
  
  return 1;
}

