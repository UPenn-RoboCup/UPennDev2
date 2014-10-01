require'freenect2'
freenect2.init()
for i=1,1 do
	print('Update', i)
	freenect2.update()
end
freenect2.shutdown()
