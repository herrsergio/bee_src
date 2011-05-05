NAME = libgd
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

OBJS := $(OBJ_DIR)/gd.o \
	$(OBJ_DIR)/gdfontt.o \
	$(OBJ_DIR)/gdfonts.o \
	$(OBJ_DIR)/gdfontmb.o \
	$(OBJ_DIR)/gdfontl.o \
	$(OBJ_DIR)/gdfontg.o

##
## Compulsory rules
##

all:
		$(MAKE) export $(DEPEND)
		$(MAKE) build

build: 		$(OBJ_DIR) \
		$(LIB_DIR) \
		$(LIB_DIR)/libgd.a

export:
	$(ECHO) "    ---- Installing header files"
	$(SILENT) $(MKDIR) $(INC_DIR)
	$(INSTALL_H) `pwd`/gd.h $(INC_DIR)
	$(INSTALL_H) `pwd`/gdfontt.h $(INC_DIR)
	$(INSTALL_H) `pwd`/gdfonts.h $(INC_DIR)
	$(INSTALL_H) `pwd`/gdfontmb.h $(INC_DIR)
	$(INSTALL_H) `pwd`/gdfontl.h $(INC_DIR)
	$(INSTALL_H) `pwd`/gdfontg.h $(INC_DIR)

clean:	
	$(SILENT) $(RM) *.o *.a *~ core a.out obj obj_sun obj_solaris $(CLEAN_FILES)

distclean: clean
	$(SILENT) $(RM) $(DEPEND) Makefile.depend* \
		$(NAME)-src.tgz $(NAME)-bin.tgz $(DISTCLEAN_FILES)

##
## local rules
##

CFLAGS := $(CFLAGS) -O

$(OBJ_DIR) $(LIB_DIR) $(BIN_DIR):
	$(SILENT) $(MKDIR) $@

$(LIB_DIR)/libgd.a: $(OBJS)
	$(SILENT) $(AR) $@ $(OBJS)
	$(SILENT) $(RANLIB) $@

##
## Standard suffix, version, packaging and dep rules
##

include $(RULES)
