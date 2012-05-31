all:
	g++ sound_comm.cc alsa_util.cc -lasound

mex:
	mex myfft.cc
	mex mydtmf.cc dtmf.cc

clean:
	rm -f *mexa64
	rm -f a.out
