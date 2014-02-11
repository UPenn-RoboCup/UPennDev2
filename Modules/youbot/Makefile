ifeq ($(shell uname),Darwin)

all:
	@echo "No EtherCat SOEM on Mac."
clean:
	@echo "No cleaning Needed"

else

LIBNAME=youbot
include ../../Makefile.inc
INCLUDES+= \
-I/home/youbot/youbot_driver \
-I/home/youbot/youbot_driver/soem/src \
-I/usr/local/include/youbot_driver \
-I/usr/local/include/youbot_driver/soem/src

LDFLAGS+=-L/home/youbot/youbot_driver/lib \
				 -lYouBotDriver -lboost_date_time-mt -lboost_filesystem-mt \
				 -lboost_system-mt -lboost_thread-mt
endif
