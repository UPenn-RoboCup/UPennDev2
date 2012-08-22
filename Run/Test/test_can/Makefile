
all: socketcan_write socketcan_read socketcan_reply socketcan_roundtrip

socketcan_write: socketcan_write.c
	g++ -o socketcan_write socketcan_write.c
	
socketcan_read: socketcan_read.c
	gcc -o socketcan_read socketcan_read.c

socketcan_reply: socketcan_reply.c
	gcc -o socketcan_reply socketcan_reply.c

socketcan_roundtrip: socketcan_roundtrip.c
	g++ -o socketcan_roundtrip socketcan_roundtrip.c
	
clean:
	rm -rf socketcan_write socketcan_read socketcan_reply *~
