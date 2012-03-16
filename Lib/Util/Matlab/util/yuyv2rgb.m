function rgb=yuyv2rgb(yuyv)

siz = size(yuyv);
yuyv_u8 = reshape(typecast(yuyv(:), 'uint8'), [4 siz]);
ycbcr = uint8(zeros(3,2*siz(1),siz(2)));
ycbcr(:,1:2:end,:) = yuyv_u8([1 2 4], :, :);
ycbcr(:,2:2:end,:) = yuyv_u8([1 2 4], :, :);
ycbcr = permute(ycbcr, [3 2 1]);
rgb = ycbcr2rgb(ycbcr);
