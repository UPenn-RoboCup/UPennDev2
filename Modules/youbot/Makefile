ifeq ($(shell uname),Darwin)

all:
	@echo "No EtherCat SOEM on Mac."
clean:
	@echo "No cleaning Needed"

else

LIBNAME=youbot
include ../../Makefile.inc
INCLUDES+=\
-I/usr/local/include/youbot/youbot \
-I/usr/local/include/youbot/youbot/src
LDFLAGS+=\
-lYouBotDriver -lboost_date_time-mt -lboost_filesystem-mt \
-lboost_system-mt -lboost_thread-mt

endif
