ffi.cdef[[
// Standard library
setjmp
//
PNG_COLOR_TYPE_GRAY
PNG_COLOR_TYPE_RGB
PNG_LIBPNG_VER_STRING
PNG_INTERLACE_NONE
PNG_COMPRESSION_TYPE_BASE
PNG_FILTER_TYPE_BASE
//
png_byte
mem_encode
png_structp
png_bytep
//
png_set_compression_level
png_create_write_struct
png_set_write_fn
png_infop
png_create_info_struct
png_jmpbuf
png_set_IHDR
png_write_info
png_write_image
png_write_end
]]
local png = ffi.load'png'