NAME = libXdw.a
CONF  = ../Makefile.conf
RULES = ../Makefile.rules

##
## System config variables
##

include $(CONF)

##
## Variables
##

DEPENDENCIES += *.c

##
## local build variables
##

OBJS =  $(OBJ_DIR)/Cache.o $(OBJ_DIR)/Drawing.o $(OBJ_DIR)/Figure.o $(OBJ_DIR)/Public.o $(OBJ_DIR)/libXdw.o

LIBS +=

ifeq ($(OS_NAME), Linux)
USER_DEFINES = -D_GNU_SOURCE -DX_LOCALE -DFUNCPROTO=15 -DNARROWPROTO 
else
USER_DEFINES = -D_GNU_SOURCE -DX_LOCALE
endif

##
## Compulsory rules
##

all:
	$(MAKE) export $(DEPEND)
	$(MAKE) build

build:	$(LIB_DIR) \
	$(LIB_DIR)/libXdw.a

export:
	$(ECHO) "    ---- Installing header files"
	$(SILENT) $(MKDIR) $(INC_DIR)
	$(INSTALL_H) `pwd`/Drawing.h $(INC_DIR)
	$(INSTALL_H) `pwd`/libXdw.h $(INC_DIR)
	$(ECHO) "    ---- Installing parameter files"

clean:
	$(RM) *.o *.a *~ core a.out obj obj_sun obj_sun4 obj_solaris \
		$(CLEAN_FILES)

distclean: clean
	$(RM) $(DEPEND) Makefile.depend* \
		$(NAME)-src.tgz $(NAME)-bin.tgz $(DISTCLEAN_FILES)

##
## local rules
##

$(OBJ_DIR) $(LIB_DIR) $(BIN_DIR):
	$(SILENT) $(MKDIR) $@

$(LIB_DIR)/libXdw.a: $(OBJ_DIR) $(OBJS)
	$(ECHO) "    ---- Creating $@"
	$(SILENT) $(AR) $@ $(OBJS)
	$(SILENT) $(RANLIB) $@

##
## Standard suffix, version, packaging and dep rules
##

include $(RULES)
