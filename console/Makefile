#
# $Id: Makefile,v 1.4 1997/11/29 00:13:09 swa Exp $
#
NAME  = console
CONF  = ../Makefile.conf
RULES = ../Makefile.rules


## System config variables
##

include $(CONF)

##
## Variables
##

## >>>>>>>>> howto add flags <<<<<<<<<<<
CFLAGS       += -Wall 
#CFLAGS       += -DDEBUG		# if set, gives you a Tcl prompt

## >>>>>>>>> for purify <<<<<<<<<<<<<<<<
ifeq ($(OS_NAME),Linux)
CFLAGS       += -Di386
else
CC = purify -cache-dir=/tmp gcc
CFLAGS       += -Dsun
endif

## >>>>>>>>> lets be strict <<<<<<<<<<<<
#CC           += -pedantic

## >>>>>>>>> lemme check <<<<<<<<<<<<<<<
#SILENT        =

BUILD_FILES = 
BUILD_FILES += $(BIN_DIR)/console

##
##
##
CLEAN_FILES = $(BIN_DIR)/console
#	$(BIN_DIR)/stefan.xpm \
#	$(BIN_DIR)/run.xpm \
#	$(BIN_DIR)/running-small.xpm \
#	$(BIN_DIR)/rwi.xpm \
#	$(BIN_DIR)/stop.xpm \
#	$(BIN_DIR)/wspace-small.xpm \

CONSOLE_OBJS = $(OBJ_DIR)/console.o \
		$(OBJ_DIR)/consoleItf.o

CONSOLE_LIBS = -lraiClient -lrai -lbUtils \
		-ldl -ltk -ltcl -ltix -lBLT \
		-ltcx $(OS_LIBS) -lX11 -lm

##
## Compulsory rules
##

all:
	$(MAKE) export $(DEPEND)
	$(MAKE) build

build: 	$(OBJ_DIR) \
	$(BIN_DIR) \
	$(LIB_DIR) \
	$(BUILD_FILES)

export:
	$(ECHO) "    ---- Installing header files"
	$(SILENT) $(MKDIR) $(INC_DIR)
	$(ECHO) "    ---- Installing parameter files"
	$(SILENT) $(MKDIR) $(ETC_DIR)
	$(INSTALL_H) `pwd`/console.tcl $(BIN_DIR)
	$(INSTALL_H) `pwd`/stefan.xpm $(BIN_DIR)
	$(INSTALL_H) `pwd`/run.xpm $(BIN_DIR)
#	$(INSTALL_H) `pwd`/wspace.xpm $(BIN_DIR)
#	$(INSTALL_H) `pwd`/running-small.xpm $(BIN_DIR)
	$(INSTALL_H) `pwd`/rwi.xpm $(BIN_DIR)
	$(INSTALL_H) `pwd`/stop.xpm $(BIN_DIR)
#	$(INSTALL_H) `pwd`/wspace-small.xpm $(BIN_DIR)
	$(INSTALL_ETC) `pwd`/console.ini $(ETC_DIR)

clean:
	$(RM) *.o *.a *~ Makefile.depend* core a.out obj obj_sun obj_solaris $(CLEAN_FILES)

distclean: clean
	$(RM) $(DEPEND) Makefile.depend* \
	$(NAME)-src.tgz $(NAME)-bin.tgz $(DISTCLEAN_FILES)

##
## local rules
##

$(OBJ_DIR) $(LIB_DIR) $(BIN_DIR):
	$(SILENT) $(MKDIR) $@


$(BIN_DIR)/console: $(CONSOLE_OBJS) $(LIB_DIR)/libraiClient.a \
	$(LIB_DIR)/librai.a $(LIB_DIR)/libbUtils.a $(LIB_DIR)/libtcx.a
	$(ECHO) "    ->>- Creating $@"
	$(SILENT) $(CC) -o $@ $(CONSOLE_OBJS) $(LIB_DIRS) $(CONSOLE_LIBS) $(INCLUDES) 

##
## Standard suffix, version, packaging and dep rules
##

include $(RULES)

#
# $Log: Makefile,v $
# Revision 1.4  1997/11/29 00:13:09  swa
# Moved all stuff from console.ini into beeSoft.ini. Moved console2.ini to console.ini.
# Consider other definitions in beeSoft.ini if programOnHost is `***', so that exisiting
# definitions are re-used.
#
# Revision 1.3  1997/11/11 20:20:35  swa
# Added a -DDEBUG flag so that we suppress the Tcl prompt if we don't want
# to debug. User cannot enter Tcl stuff at the prompt then. Removed some
# stuff in Tcl_AppInit. For the debug stuff to work we use a different setup,
# and we needed (!) to call Tcl_FindExecutable first. Tcl--.
#
# Revision 1.2  1997/11/11 15:50:52  swa
# Ok, Sebastian bitched'n'moaned about the buttons and tabs semantics,
# so I changed the tabs to display no icon at all. We now have only
# `Start' buttons, that automatically change to a `Stop' button,
# whenever that particular program is running.
#
# Revision 1.1.1.1  1997/11/10 23:10:36  swa
# Stefan's new console manager.
#
# Revision 1.9  1997/11/09 21:45:36  swa
# Images are removed at make clean.
#
# Revision 1.8  1997/11/09 21:41:25  swa
# Neat about dialog and some other enhancements.
#
# Revision 1.7  1997/11/09 18:25:10  swa
# All programs and their dependencies are now taken care of. A second,
# user-specific ini file was added, console2.ini. All dependencies in
# console.ini should eventually go into beeSoft.ini and console2.ini
# should be renamed console.ini.
#
# Revision 1.6  1997/11/08 16:59:51  swa
# More icons.
#
# Revision 1.5  1997/11/08 04:40:04  swa
# Used a different icon for the `Start' button.
#
# Revision 1.4  1997/11/08 03:50:11  swa
# added the pictures to be moved to a fixed path, that is BIN_DIR
#
# Revision 1.3  1997/11/07 22:44:02  swa
# Fixed dependencies.
#
# Revision 1.2  1997/11/07 22:41:36  swa
# Compiles and runs nicely.
#
# Revision 1.1.1.1  1997/11/07 22:31:36  swa
# imported
#
#
