# Webots Makefile system 
#
# You may add some variable definitions hereafter to customize the build process
# See documentation in $(WEBOTS_HOME_PATH)/resources/Makefile.include

OSTYPE=$(shell uname -s|awk '{print tolower($$0)}')
WEBOTS_HOME=/usr/local/webots
ifeq ($(OSTYPE),darwin)
	WEBOTS_HOME=/Applications/Webots
endif

# Do not modify the following: this includes Webots global Makefile.include
space :=
space +=
WEBOTS_HOME_PATH=$(subst $(space),\ ,$(strip $(subst \,/,$(WEBOTS_HOME))))
include $(WEBOTS_HOME_PATH)/resources/Makefile.include
