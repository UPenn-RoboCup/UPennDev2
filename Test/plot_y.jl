using Winston
f=open("y.raw","r")
im=read(f,Int32,(320,240))
close(f)

# Show the incoming image
#imagesc(im)

# Form the kernel
k = Int32[
				0 0 1 0 0;
				0 1 2 1 0;
				1 2 -15 2 1;
				0 1 2 1 0;
				0 0 1 0 0;
				]
c = conv2(im,k)
imagesc(c)

# Open the torch conv2 just for sanity
f=open("c.raw","r")
#ct=read(f,Int32,(324,244))
ct=read(f,Int32,(316,236))
close(f)
imagesc(ct)
