NAME = msp
CONF  = ../Makefile.conf
RULES = ../Makefile.rules

##
## System config variables
##

include $(CONF)

##
## Variables
##

ifeq ($(OS_NAME),Linux)
DEPENDENCIES += abterm.c msp.c mspmodule.c mspterm.c utils.c
else
DEPENDENCIES += msp.c mspmodule.c
endif

##
## local build variables
##

LIBMSP_OBJS    = $(OBJ_DIR)/msp.o $(OBJ_DIR)/libmsp.o

MSPTERM_OBJS   = $(OBJ_DIR)/abterm.o $(OBJ_DIR)/mspterm.o \
			$(OBJ_DIR)/utils.o $(LIB_DIR)/libmsp.a \
			$(OBJ_DIR)/libcurses.a $(OBJ_DIR)/version.o

MSPTERM_LIBS   = -ltermcap -labus

MSPMODULE_OBJS = $(OBJ_DIR)/mspmodule.o $(OBJ_DIR)/libmspmodule.o

IRCAL_OBJS = $(OBJ_DIR)/irCal.o
IRCAL_LIBS = -lmspmodule -lrai -lutils -lmsp -labus -lbUtils

##
## Compulsory rules
##

all:
	$(MAKE) export $(DEPEND)
	$(MAKE) build

ifeq ($(OS_NAME),Linux)
build: $(OBJ_DIR) \
	$(LIB_DIR) \
	$(BIN_DIR) \
	$(LIB_DIR)/libmsp.a \
	$(LIB_DIR)/libmspmodule.a \
	$(BIN_DIR)/mspterm \
	$(BIN_DIR)/irCal
else
build: $(OBJ_DIR) \
	$(LIB_DIR) \
	$(BIN_DIR) \
	$(LIB_DIR)/libmsp.a \
	$(LIB_DIR)/libmspmodule.a
endif

export:
	$(ECHO) "    ---- Installing header files"
	$(SILENT) $(MKDIR) $(INC_DIR)
	$(INSTALL_H) `pwd`/msp.h $(INC_DIR)
	$(INSTALL_H) `pwd`/mspmodule.h $(INC_DIR)
	$(INSTALL_H) `pwd`/libmsp.h $(INC_DIR)
	$(INSTALL_H) `pwd`/libmspmodule.h $(INC_DIR)



clean:
	$(RM) *.o *.a *~ core a.out obj obj_sun obj_solaris $(CLEAN_FILES)

distclean: clean
	$(RM) $(DEPEND) Makefile.depend* \
		$(NAME)-src.tgz $(NAME)-bin.tgz $(DISTCLEAN_FILES)

##
## local rules
##

$(OBJ_DIR) $(LIB_DIR) $(BIN_DIR):
	$(SILENT) $(MKDIR) $@

$(OBJ_DIR)/libcurses.a: $(OBJ_DIR)
	$(SILENT) cp ./libcurses.a.hide-from-make-clean $(OBJ_DIR)/libcurses.a

$(LIB_DIR)/libmsp.a: $(LIBMSP_OBJS)
	$(ECHO) "    ---- Creating $@"
	$(SILENT) $(AR) $@ $(LIBMSP_OBJS)
	$(SILENT) $(RANLIB) $@

$(LIB_DIR)/libmspmodule.a: $(MSPMODULE_OBJS)
	$(ECHO) "    ---- Creating $@"
	$(SILENT) $(AR) $@ $(MSPMODULE_OBJS)
	$(SILENT) $(RANLIB) $@

$(BIN_DIR)/mspterm: $(MSPTERM_OBJS)
	$(ECHO) "    ---- Creating $@"
	$(SILENT) $(CC) $(INCLUDES) $(LIB_DIRS) $(CFLAGS) -o $@ \
		$(MSPTERM_OBJS) $(MSPTERM_LIBS)

$(BIN_DIR)/irCal: $(IRCAL_OBJS)
	$(ECHO) "    ---- Creating $@"
	$(SILENT) $(CC) $(INCLUDES) $(LIB_DIRS) $(CFLAGS) -o $@ \
		$(IRCAL_OBJS) $(IRCAL_LIBS)

##
## Standard suffix, version, packaging and dep rules
##

include $(RULES)
