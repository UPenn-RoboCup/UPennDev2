ifeq ($(shell uname),Darwin)

all:
	@echo "No EtherCat SOEM on Mac."
clean:
	@echo "No cleaning Needed"

else

LIBNAME=youbot
include ../../Makefile.inc
LDFLAGS+=\
-lYouBotDriver -lboost_date_time -lboost_filesystem \
-lboost_system -lboost_thread

endif
