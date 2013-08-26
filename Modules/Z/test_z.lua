local Z = require('Z')

src = "lua c api: how to push a string with                                                                                               a null character in the middle?";
print('Input string:',src);
print('Input length:',#src);
src_c = Z.compress(src, #src)
print('Compressed length:',#src_c);
src_uc = Z.uncompress(src_c, #src_c)
print('Output length:',#src_uc);
print('Output string:',src_uc);
