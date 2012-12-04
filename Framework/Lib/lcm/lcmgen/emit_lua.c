#include <stdio.h>
#include <stdlib.h>
#include <stdint.h>
#include <assert.h>
#include <string.h>
#include <ctype.h>

#include <inttypes.h>

#include "lcmgen.h"
#include "sprintfalloc.h"

#define INDENT(n) (4*(n))

#define emit_start(n, ...) do { fprintf(f, "%*s", INDENT(n), ""); fprintf(f, __VA_ARGS__); } while (0)
#define emit_continue(...) do { fprintf(f, __VA_ARGS__); } while (0)
#define emit_end(...) do { fprintf(f, __VA_ARGS__); fprintf(f, "\n"); } while (0)
#define emit(n, ...) do { fprintf(f, "%*s", INDENT(n), ""); fprintf(f, __VA_ARGS__); fprintf(f, "\n"); } while (0)

#define FLAG_NONE 0

// flags for emit_c_array_loops_start
#define FLAG_EMIT_MALLOCS 1

// flags for emit_c_array_loops_end
#define FLAG_EMIT_FREES   2

static inline int imax(int a, int b)
{
    return (a > b) ? a : b;
}


static char *dots_to_underscores(const char *s)
{
    char *p = strdup(s);

    for (char *t=p; *t!=0; t++)
        if (*t == '.')
            *t = '_';

    return p;
}

static char *remove_dots_and_above(const char *s)
{
    char *p = strdup(s);

    int idx = 0;
    for (char *t=p; *t!=0; t++) {
        idx++;
        if (*t == '_') {
          break;
        }
    }
    int len = strlen(p);
    printf("%d %d\n", idx, len);
    int t = 0;
    for (t = 0; t < (len-idx); t++)
      p[t] = p[t+idx];
    p[t] = '\0';
    printf("%s\n", p);
    return p;
}

static void emit_auto_generated_warning(FILE *f)
{
    fprintf(f,
            "/**********************************************************\n"
            " ** This file is generated automatically by lcmgen\n"
            "**********************************************************/\n\n");
}

static const char *map_type(const char *t)
{
    if (!strcmp(t,"boolean"))
        return "(int8_t)";

    if (!strcmp(t,"string"))
        return "(char*)";

    if (!strcmp(t,"int8_t"))
        return "(int8_t)";

    if (!strcmp(t,"int16_t"))
        return "(int16_t)";

    if (!strcmp(t,"int32_t"))
        return "(int32_t)";

    if (!strcmp(t,"int64_t"))
        return "(int64_t)";

    if (!strcmp(t,"double"))
        return "(double)";

    if (!strcmp(t,"float"))
        return "(float)";

    if (!strcmp(t,"byte"))
        return "(uint8_t)";

    return dots_to_underscores (t);
}

static const char *map_array_type(const char *t)
{
    if (!strcmp(t,"boolean"))
        return "(int8_t*)";

    if (!strcmp(t,"string"))
        return "(char**)";

    if (!strcmp(t,"int8_t"))
        return "(int8_t*)";

    if (!strcmp(t,"int16_t"))
        return "(int16_t*)";

    if (!strcmp(t,"int32_t"))
        return "(int32_t*)";

    if (!strcmp(t,"int64_t"))
        return "(int64_t*)";

    if (!strcmp(t,"double"))
        return "(double*)";

    if (!strcmp(t,"float"))
        return "(float*)";

    if (!strcmp(t,"byte"))
        return "(uint8_t*)";

    return dots_to_underscores (t);
}

static const char *map_type_tocommand(const char *t, int flag)
{
    if (!strcmp(t,"boolean"))
        return "lua_toboolean";

    if (!strcmp(t,"string"))
        return "lua_tostring";

    if (!strcmp(t,"int8_t") || !strcmp(t, "int16_t")
        || !strcmp(t, "int32_t") || !strcmp(t, "int64_t"))
        return "lua_tointeger";
    
    if (!strcmp(t, "byte") && flag > 0)
        return "lua_tostring";
    else
        return "lua_tointeger";

    if (!strcmp(t,"double") || !strcmp(t, "float"))
        return "lua_tonumber";

    return dots_to_underscores (t);
}

static const char *map_type_pushcommand(const char *t, int flag)
{
    if (!strcmp(t,"boolean"))
        return "lua_pushboolean";

    if (!strcmp(t,"string"))
        return "lua_pushstring";

    if (!strcmp(t,"int8_t") || !strcmp(t, "int16_t")
        || !strcmp(t, "int32_t") || !strcmp(t, "int64_t"))
        return "lua_pushinteger";
    
    if (!strcmp(t, "byte") && flag > 0)
        return "lua_pushstring";
    else
        return "lua_pushinteger";

    if (!strcmp(t,"double") || !strcmp(t, "float"))
        return "lua_pushnumber";

    return dots_to_underscores (t);
}
void setup_lua_options(getopt_t *gopt)
{
    getopt_add_string(gopt, 0,   "lpath",     ".",         
            "Lua destination directory");
}

/** Emit output that is common to every header file **/
static void emit_header_top(lcmgen_t *lcm, FILE *f, char *name)
{
    fprintf(f, "#ifndef _%s_h\n", name);
    fprintf(f, "#define _%s_h\n", name);
    fprintf(f, "\n");

    fprintf(f, "#ifdef __cplusplus\n");
    fprintf(f, "extern \"C\" {\n");
    fprintf(f, "#endif\n");

    fprintf(f, "\n");
    fprintf(f, "#include \"lua.h\"\n");
    fprintf(f, "#include \"lualib.h\"\n");
    fprintf(f, "#include \"lauxlib.h\"\n");
    fprintf(f, "\n");

}

/** Emit common lua open part tp header file **/
static void emit_header_luaopen(lcmgen_t *lcm, FILE *f, char *name)
{
    fprintf(f, "// Lua Module Open\n");
    fprintf(f, "int luaopen_%s(lua_State *L);\n", name);
    fprintf(f, "\n");
}

static void emit_lua_module_struct(lcmgen_t *lcm, FILE *f, char *name, char *shortname)
{
//    printf("%s\n", name);    
    fprintf(f, "static const struct luaL_reg %s_methods[] = {\n", name);
    fprintf(f, "%*s{\"%s_publish\", lua_%s_publish},\n", INDENT(1), "", shortname, name);
    fprintf(f, "%*s{\"%s_subscribe\", lua_%s_subscribe},\n", INDENT(1), "", shortname, name);
    fprintf(f, "%*s{\"%s_unsubscribe\", lua_%s_unsubscribe},\n", INDENT(1), "", shortname, name);
    fprintf(f, "%*s{\"%s_subscription_set_queue_capacity\",\n", INDENT(1), "", shortname);
    fprintf(f, "%*slua_%s_subscription_set_queue_capacity},\n", INDENT(4), "", name);
    fprintf(f, "%*s{NULL, NULL},\n", INDENT(1), "");
    fprintf(f, "};\n");
    fprintf(f, "\n");


    fprintf(f, "int luaopen_%s(lua_State *L)\n", name);
    fprintf(f, "{\n");
    fprintf(f, "%*sif (luaL_loadstring(L, \"require(\'lcm\')\") || lua_pcall(L, 0, 0, 0))\n", INDENT(1), "");
    fprintf(f, "%*sreturn luaL_error(L, \"Unable to require lcm module\");\n", INDENT(2), "");
    fprintf(f, "\n");
    fprintf(f, "%*sluaL_getmetatable(L, \"lcm_mt\");\n", INDENT(1), "");
    fprintf(f, "%*sluaL_register(L, NULL, %s_methods);\n", INDENT(1), "", name);
    fprintf(f, "%*sreturn 1;\n", INDENT(1), "");
    fprintf(f, "};\n");
    fprintf(f, "\n");


}

/** Emit output that is common to every header file **/
static void emit_header_bottom(lcmgen_t *lcm, FILE *f)
{
    fprintf(f, "#ifdef __cplusplus\n");
    fprintf(f, "}\n");
    fprintf(f, "#endif\n");
    fprintf(f, "\n");
    fprintf(f, "#endif\n");
}

// Create an accessor for member lm, whose name is "n". For arrays,
// the dim'th dimension is accessed. E.g., dim=0 will have no
// additional brackets, dim=1 has [a], dim=2 has [a][b].
static char *make_accessor(lcm_member_t *lm, const char *n, int dim)
{
    char *tmp = (char *) malloc(128);

    if (g_ptr_array_size(lm->dimensions) == 0) {
        sprintf(tmp, "&(%s[element].%s)", n, lm->membername);
    } else {
        int pos = sprintf(tmp, "%s[element].%s", n, lm->membername);
        for (unsigned int d = 0; d < dim; d++) {
            pos += sprintf(&tmp[pos], "[%c]", d + 'a');
        }
    }
    return tmp;
}

static char *make_array_size(lcm_member_t *lm, const char *n, int dim)
{
    if (g_ptr_array_size(lm->dimensions) == 0)
        return sprintfalloc("1");
    else {
        lcm_dimension_t *ld = (lcm_dimension_t *) g_ptr_array_index(lm->dimensions, dim);
        switch (ld->mode)
        {
        case LCM_CONST:
            return sprintfalloc("%s", ld->size);
        case LCM_VAR:
            return sprintfalloc("%s[element].%s", n, ld->size);
        }
    }
    assert(0);
    return NULL;
}

static void emit_lua_encode(lcmgen_t *lcm, FILE *f, lcm_struct_t *ls)
{
    char *tn = ls->structname->lctypename;
    char *tn_ = dots_to_underscores(tn);

    emit(0,"static void lua_%s_encode(lua_State *L, %s *msg)", tn_, tn_);

    emit(0,"{");
    for (unsigned int m = 0; m < g_ptr_array_size(ls->members); m++) {
        lcm_member_t *lm = (lcm_member_t *) g_ptr_array_index(ls->members, m);

        emit(1,"lua_getfield(L, 3, \"%s\");", lm->membername);
        emit(1,"if (!lua_isnil(L, 4))");

        int ndim = g_ptr_array_size(lm->dimensions);
        if (ndim == 0) {
            emit(2, "msg->%s = %s%s(L, 4);", lm->membername, map_type(lm->type->lctypename), 
                                map_type_tocommand(lm->type->lctypename, 0));
        } else {
            if (lcm_is_constant_size_array(lm)) {
                emit_start(2, "msg->%s = ", lm->membername);
                emit_continue( map_type_tocommand(lm->type->lctypename, 0),"%s" );
//                for (unsigned int d = 0; d < ndim; d++) {
//                    lcm_dimension_t *ld = (lcm_dimension_t *) g_ptr_array_index(lm->dimensions, d);
//                    emit_continue("[%s]", ld->size);
//                }
                emit_end("(L, 4);");
            } else {
//                emit(2, "msg->%s = %s%s(L, 4);", lm->membername, map_type_change(lm->type->lctypename),
//                                map_type_tocommand(lm->type->lctypename, 1));
                emit_start(2, "msg->%s = ", lm->membername);
                emit_continue(map_array_type(lm->type->lctypename),"%s");
                emit_continue(map_type_tocommand(lm->type->lctypename, 1),"%s");
                emit_end("(L, 4);");
            }
        }
        emit(1, "lua_pop(L ,1);\n");
    }

    emit(0,"}");
    emit(0,"");
}

static void emit_lua_decode(lcmgen_t *lcm, FILE *f, lcm_struct_t *ls)
{
    char *tn = ls->structname->lctypename;
    char *tn_ = dots_to_underscores(tn);

    emit(0,"static void lua_%s_decode(lua_State *L, const %s *msg)", tn_, tn_);

    emit(0,"{");
    for (unsigned int m = 0; m < g_ptr_array_size(ls->members); m++) {
        lcm_member_t *lm = (lcm_member_t *) g_ptr_array_index(ls->members, m);
        emit(1,"lua_pushstring(L, \"%s\");", lm->membername);

        int ndim = g_ptr_array_size(lm->dimensions);
        if (ndim == 0) {
            emit(1, "%s(L, msg->%s);", map_type_pushcommand(lm->type->lctypename, 0), lm->membername);
        } else {
            printf("size not 0 %s\n", lm->membername);
            if (lcm_is_constant_size_array(lm)) {
//                for (unsigned int d = 0; d < ndim; d++) {
//                    lcm_dimension_t *ld = (lcm_dimension_t *) g_ptr_array_index(lm->dimensions, d);
//                    emit_continue("[%s]", ld->size);
//                }
//                emit_end(";");
            } else {
                  emit(1, "%s(L, msg->%s);", map_type_pushcommand(lm->type->lctypename, 1), lm->membername);

//                for (unsigned int d = 0; d < ndim; d++)
//                    emit_continue("*");
//                emit_end("%s;", lm->membername);
            }
        }
        emit(1, "lua_settable(L ,-3);\n");
    }

    emit(0,"}");
    emit(0,"");
}

static void emit_lua_handler(lcmgen_t *lcm, FILE *f, lcm_struct_t *ls)
{
    char *tn = ls->structname->lctypename;
    char *tn_ = dots_to_underscores(tn);

    emit(0,"static void lua_%s_handler(const lcm_recv_buf_t *rbuf,", tn_);
    emit(3,"const char *channel, const %s *msg, void *userdata)", tn_);
    emit(0,"{");
    emit(0,"  lua_lcm_handler_t *handler = (lua_lcm_handler_t *)userdata;");
    emit(0,"  lua_State *L = handler->L;");
    emit(0,"");
    emit(0,"  /* push lua callback function */");
    emit(0,"  lua_rawgeti(L, LUA_REGISTRYINDEX, handler->callback_reference);");
    emit(0,"");
    emit(0,"  /* push channel name */");
    emit(0,"  lua_pushstring(L, channel);");
    emit(0,"");
    emit(0,"  /* push message table */");
    emit(0,"  lua_newtable(L);");
    emit(0,"  lua_lcm_rpc_request_t_decode(L, msg);");
    emit(0,"");
    emit(0,"  /* push userdata */");
    emit(0,"  lua_rawgeti(L, LUA_REGISTRYINDEX, handler->userdata_reference);");
    emit(0,"");
    emit(0,"  /* call handler */");
    emit(0,"  lua_call(L, 3, 0);");
    emit(0,"}");
    emit(0,"");
}

static void emit_lua_struct_publish(lcmgen_t *lcm, FILE *f, lcm_struct_t *lr)
{
    char *tn = lr->structname->lctypename;
    char *tn_ = dots_to_underscores(tn);
    char *ltn_ = sprintfalloc("lua_%s", tn_);
    fprintf(f,
            "static int %s_publish(lua_State *L)\n"
            "{\n"
            "  lua_lcm_t *lcm = lua_checklcm(L, 1);\n"
            "  const char *channel = luaL_checkstring(L, 2);\n"
            "  %s msg;\n"
            "  int result;\n"
            "\n"
            "  /* get message struct */\n"
            "  if (!lua_istable(L, 3))\n"
            "   return luaL_error(L, \"invalid message type\");\n"
            "  lua_%s_encode(L, &msg);\n"
            "\n"
            "  /* publish message */\n"
            "  result = %s_publish(lcm->lcm, channel, &msg);\n"
            "  lua_pushinteger(L, result);\n"
            "  return 1;\n"
            "}\n\n", ltn_, tn_, tn_, tn_);
}

static void emit_lua_struct_subscribe(lcmgen_t *lcm, FILE *f, lcm_struct_t *lr)
{
    const char *tn = lr->structname->lctypename;
    char *tn_ = dots_to_underscores(tn);
    char *ltn_ = sprintfalloc("lua_%s", tn_);

    fprintf(f,
            "static int %s_subscribe(lua_State *L)\n"
            "{\n"
            "  lua_lcm_t *lcm = lua_checklcm(L, 1);\n"
            "  const char *channel = luaL_checkstring(L, 2);\n"
            "  %s_subscription_t *subs;\n"
            "  lua_lcm_handler_t *handler;\n"
            "\n"
            "  if (!lua_isfunction(L, 3))\n"
            "    return luaL_error(L, \"invalid callback function\");\n"
            "  if (lua_gettop(L) > 4)\n"
            "    lua_pop(L, lua_gettop(L) - 4);\n"
            "  if (lua_gettop(L) < 4)\n"
            "    lua_pushnil(L);\n"
            "\n"
            "  if (lcm->n_handlers >= MAX_HANDLERS)\n"
            "  {\n"
            "    /* max handlers exceeded */\n"
            "    lua_pushnil(L);\n"
            "    return 1;\n"
            "  }\n"
            "\n"
            "  /* initialize message handler */\n"
            "  handler = &(lcm->handlers[lcm->n_handlers++]);\n"
            "  handler->userdata_reference = luaL_ref(L, LUA_REGISTRYINDEX);\n"
            "  handler->callback_reference = luaL_ref(L, LUA_REGISTRYINDEX);\n"
            "  handler->L = L;\n"
            "\n"
            "  /* subscribe to message */\n"
            "  subs = %s_subscribe(lcm->lcm, channel,\n"
            "                                     lua_%s_handler, handler);\n"
            "  lua_pushlightuserdata(L, subs);\n"
            "  return 1;\n"
            "}\n\n", ltn_, tn_, tn_, tn_);
   
    fprintf(f,
            "static int %s_unsubscribe(lua_State *L)\n"
            "{\n"
            "  lua_lcm_t *lcm = lua_checklcm(L, 1);\n"
            "  %s_subscription_t *subs = NULL;\n"
            "  int result;\n"
            "\n"
            "  if (!lua_islightuserdata(L, 2))\n"
            "    return luaL_error(L, \"invalid subscription handle\");\n"
            "  subs = (%s_subscription_t *)lua_touserdata(L, 2);\n"
            "\n"
            "  /* unsubscribe from message */\n"
            "  result = %s_unsubscribe(lcm->lcm, subs);\n"
            "  lua_pushinteger(L, result);\n"
            "  return 1;\n"
            "}\n\n", ltn_, tn_, tn_, tn_);

    fprintf(f,
            "static int %s_subscription_set_queue_capacity(lua_State *L)\n"
            "{\n"
            "  lua_lcm_t *lcm = lua_checklcm(L, 1);\n"
            "  %s_subscription_t *subs = NULL;\n"
            "  int num_messages, result;\n"
            "\n"
            "  if (!lua_islightuserdata(L, 2))\n"
            "    return luaL_error(L, \"invalid subscription handle\");\n"
            "  subs = (%s_subscription_t *)lua_touserdata(L, 2);\n"
            "\n"  
            "  /* set queue capacity for subscription */\n"
            "  num_messages = luaL_checkint(L, 3);\n"
            "  result = %s_subscription_set_queue_capacity(subs, num_messages);\n"
            "  lua_pushinteger(L, result);\n"
            "  return 1;\n"
            "}\n\n", ltn_, tn_, tn_, tn_);


}


int emit_lua_enum(lcmgen_t *lcmgen, lcm_enum_t *le) {
    char *tn = le->enumname->lctypename;
    printf("typename: %s\n", tn);
    
    return 0;
}

int emit_lua_struct(lcmgen_t *lcmgen, lcm_struct_t *lr) {
    char *tn = lr->structname->lctypename;
    char *tn_ = dots_to_underscores(tn);
    char *ltn_ = sprintfalloc("lua%s", tn_);
    char *n_ = remove_dots_and_above(tn_);
    char *header_name = sprintfalloc("%s/%s.h", 
                        getopt_get_string(lcmgen->gopt, "lpath"), ltn_);
    char *c_name      = sprintfalloc("%s/%s.c", 
                        getopt_get_string(lcmgen->gopt, "lpath"), ltn_);

    if (lcm_needs_generation(lcmgen, lr->lcmfile, header_name)) {
        FILE *f = fopen(header_name, "w");
        if (f == NULL)
            return -1;

        emit_header_top(lcmgen, f, ltn_);
        emit_auto_generated_warning(f);
        emit_header_luaopen(lcmgen, f, tn_);
        emit_header_bottom(lcmgen, f);
        fclose(f);
    } 

    // STRUCT C file
    if (lcm_needs_generation(lcmgen, lr->lcmfile, c_name)) {
        FILE *f = fopen(c_name, "w");
        if (f == NULL)
            return -1;

        emit_auto_generated_warning(f);

        fprintf(f, "#include \"lualcm.h\"\n");
        fprintf(f, "#include \"%s.h\"\n", ltn_);
        fprintf(f, "#include \"%s.h\"\n", tn_);
        fprintf(f, "#include <lcm/lcm.h>\n");
        fprintf(f, "\n");

        fprintf(f, "#include <stdint.h>\n");
        fprintf(f, "\n");
        
        fprintf(f, "\n");


        emit_lua_encode(lcmgen, f, lr);

        emit_lua_decode(lcmgen, f, lr);
  
        emit_lua_handler(lcmgen, f, lr);
        emit_lua_struct_publish(lcmgen, f, lr );
        emit_lua_struct_subscribe(lcmgen, f, lr );

        emit_lua_module_struct(lcmgen, f, tn_, n_);

        fclose(f);
    }

    return 0;
}

int emit_lua(lcmgen_t *lcmgen)
{
    printf("emit_lua, enum: %d, struct: %d\n", g_ptr_array_size(lcmgen->enums),
                    g_ptr_array_size(lcmgen->structs));
    ////////////////////////////////////////////////////////////
    // ENUMS
    for (unsigned int i = 0; i < g_ptr_array_size(lcmgen->enums); i++) {

        lcm_enum_t *le = (lcm_enum_t *) g_ptr_array_index(lcmgen->enums, i);
        if (emit_lua_enum(lcmgen, le))
            return -1;
    }

    ////////////////////////////////////////////////////////////
    // STRUCTS
    for (unsigned int i = 0; i < g_ptr_array_size(lcmgen->structs); i++) {
        lcm_struct_t *lr = (lcm_struct_t *) g_ptr_array_index(lcmgen->structs, i);

        if (emit_lua_struct(lcmgen, lr))
            return -1;
    }

    return 0;
}
