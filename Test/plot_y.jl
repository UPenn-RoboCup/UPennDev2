using Winston
f=open("y.raw","r")
im=read(f,Uint8,(320,240))
close(f)
imagesc(im)
im2 = int(im)

k = Int[
				0 0 1 0 0;
				0 1 2 1 0;
				1 2 -15 2 1;
				0 1 2 1 0;
				0 0 1 0 0;
				]
c = conv2(im2,k)
imagesc(c)
