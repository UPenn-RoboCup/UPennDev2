ADD_LIBRARY( GameControlPacket SHARED luagamecontrolpacket.cpp )
TARGET_LINK_LIBRARIES( GameControlPacket ${COMMLIBS} )
SET_TARGET_PROPERTIES( GameControlPacket PROPERTIES PREFIX "" )
INSTALL_TARGETS( ${LIBRARY_INST} GameControlPacket )
INSTALL_FILES( ${LIBRARY_INST} .lua )

		
