NAME = cameraServer
CONF  = ../Makefile.conf
RULES = ../Makefile.rules

##
## System config variables
##

include $(CONF)

##
## Variables
##

#CFLAGS      += -DDEBUG_CAMERA	# debugging info for the frame grabbers

CFLAGS      += -DG_DEBUG_PERFORMANCE # define, if you want performance output

CFLAGS      += -Wall		# print all possible errors during compiling

CFLAGS      += -DDO_DA_BEEP	# define, if the PC should beep for each saved frame

CFLAGS      += -DDO_DA_COUNTER	# define, if you want framenumber while saving

DEPENDENCIES += *.c

##
## local build variables
##

OBJS := $(OBJ_DIR)/display.o \
	$(OBJ_DIR)/grab.o \
	$(OBJ_DIR)/handlers.o 

CAMERA_LIBS = -lbUtils -lraiClient -lrai -ltcx $(OS_LIBS) -lezx -lX11 -lm 

CLIENT_OBJS = $(OBJ_DIR)/cameraClient.o  $(OBJ_DIR)/libcameraClient.o

##
##
ATTACH_LIBS     = -lcameraClient -lraiClient \
		  -lrai -lbUtils -ltcx $(OS_LIBS) -lezx -lm -lX11
ATTACH_OBJS     = $(OBJ_DIR)/display.o \
                  $(OBJ_DIR)/mainatt.o

##
##
CAMERACONTROL_LIBS   = -lcameraClient -lraiClient -lrai -lbUtils \
			-ldl -ltk -ltcl -ltix \
			-ltcx $(OS_LIBS) -lX11 -lm
CAMERACONTROL_OBJS   = $(OBJ_DIR)/camControlItf.o \
			$(OBJ_DIR)/camControl.o \

##
##
EXTRACTOR_LIBS  = -lm
EXTRACTOR_OBJS  =  $(OBJ_DIR)/extractframe.o

##
## Compulsory rules
##

all:
	$(MAKE) export $(DEPEND)
	$(MAKE) build
	chmod a+rx raw2ppm

build:  $(OBJ_DIR) \
	$(BIN_DIR) \
	$(LIB_DIR) \
	$(BIN_DIR)/cameraServer \
	$(LIB_DIR)/libcameraClient.a \
	$(BIN_DIR)/cameraAttachExample \
	$(BIN_DIR)/extractFrame 
#	$(BIN_DIR)/cameraControl


export:
	$(ECHO) "    ---- Installing header files"
	$(SILENT) $(MKDIR) $(INC_DIR)
	$(SILENT) $(MKDIR) $(BIN_DIR)
	$(INSTALL_H) `pwd`/CAMERA-messages.h $(INC_DIR)
	$(INSTALL_H) `pwd`/ioctl_meteor.h $(INC_DIR)
	$(INSTALL_H) `pwd`/imagesize.h $(INC_DIR)
	$(INSTALL_H) `pwd`/cameraClient.h $(INC_DIR)
	$(INSTALL_H) `pwd`/camControl.tcl $(BIN_DIR)
	$(INSTALL_H) `pwd`/stefan.xpm $(BIN_DIR)
	$(INSTALL_H) `pwd`/rwi.xpm $(BIN_DIR)

clean:
	$(RM) *.o *.a *~ core a.out obj obj_sun obj_solaris $(CLEAN_FILES)

# make clean is intended to leave a smaller but working system.
# use 'make distclean' to delete all generated files.
# 'make distclean' deletes the lib, bin, etc. dirs completely.
#
#	$(BIN_DIR)/cameraServer	$(LIB_DIR)/libcameraClient.a \
#	$(BIN_DIR)/cameraControl $(BIN_DIR)/cameraAttachExample \
#	$(BIN_DIR)/extractFrame

distclean: clean
	$(RM) $(DEPEND) Makefile.depend* \
		$(NAME)-src.tgz $(NAME)-bin.tgz $(DISTCLEAN_FILES)

##
## local rules
##

$(OBJ_DIR) $(LIB_DIR) $(BIN_DIR):
	$(SILENT) $(MKDIR) $@

$(BIN_DIR)/cameraServer: $(OBJS) $(LIB_DIR)/libbUtils.a \
	$(LIB_DIR)/libraiClient.a $(LIB_DIR)/librai.a \
	$(LIB_DIR)/libtcx.a $(LIB_DIR)/libezx.a
	$(ECHO) "    ->>- Creating $@"
	$(SILENT) $(CC) $(INCLUDES) $(LIB_DIRS) $(CFLAGS) -o $@ \
		$(OBJS) $(CAMERA_LIBS)

$(BIN_DIR)/cameraControl: $(CAMERACONTROL_OBJS) $(LIB_DIR)/libcameraClient.a \
	$(LIB_DIR)/libraiClient.a $(LIB_DIR)/librai.a $(LIB_DIR)/libbUtils.a \
	$(LIB_DIR)/libtcx.a
	$(ECHO) "    ->>- Creating $@"
	$(SILENT) $(CC) -o $@ $(CAMERACONTROL_OBJS) $(LIB_DIRS) $(CAMERACONTROL_LIBS) $(INCLUDES) 

$(BIN_DIR)/cameraAttachExample: $(ATTACH_OBJS) $(LIB_DIR)/libcameraClient.a \
	$(LIB_DIR)/libraiClient.a $(LIB_DIR)/librai.a $(LIB_DIR)/libbUtils.a \
	$(LIB_DIR)/libtcx.a $(LIB_DIR)/libezx.a
	$(ECHO) "    ->>- Creating $@"
	$(SILENT) $(CC) -o $@ $(ATTACH_OBJS) $(LIB_DIRS) $(ATTACH_LIBS) $(INCLUDES) 

$(BIN_DIR)/extractFrame: $(EXTRACTOR_OBJS)
	$(ECHO) "    ->>- Creating $@"
	$(SILENT) $(CC) -o $@ $(EXTRACTOR_OBJS) $(LIB_DIRS) $(EXTRACTOR_OBJS	) $(INCLUDES) 

$(LIB_DIR)/libcameraClient.a: $(CLIENT_OBJS)
	$(ECHO) "    ->>- Creating $@"
	$(SILENT) $(AR) $@ $(CLIENT_OBJS)
	$(SILENT) $(RANLIB) $@

##
## Standard suffix, version, packaging and dep rules
##

include $(RULES)
