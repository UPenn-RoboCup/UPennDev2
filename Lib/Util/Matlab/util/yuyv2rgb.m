function rgb=yuyv2rgb(yuyv);

siz = size(yuyv);
yuyv_u8 = reshape(typecast(yuyv(:), 'uint8'), [4 siz(1,1,1) size (1,2,1) 2*size(1,3,1)]);
ycbcr = yuyv_u8([1 2 4], :, 1:2:end);
ycbcr = permute(ycbcr, [3 2 1]);
rgb = ycbcr2rgb(ycbcr);
