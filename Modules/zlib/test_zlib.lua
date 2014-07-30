local zlib = require'ffi'
src = "lua c api: how to push a string with                                                                                               a null character in the middle?";
print('Input string:',src);
print('Input length:',#src);
src_c = zlib.compress(src)
print('Compressed length:',#src_c);

local zlib = require'zlib'
src_uc = zlib.uncompress(src_c)
print('Output length:',#src_uc);
print('Output string:',src_uc);
