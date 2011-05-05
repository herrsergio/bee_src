NAME = sonarint
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

OBJS := $(OBJ_DIR)/SONARINT.o $(OBJ_DIR)/file.o \
	$(OBJ_DIR)/graphics.o $(OBJ_DIR)/handlers.o $(OBJ_DIR)/init.o

SONARINT_LIBS = -lezx -lX11 -ltcx -lrobot $(OS_LIBS) -lbUtils -lm


##
## Compulsory rules
##

all:
	$(MAKE) export $(DEPEND)
	$(MAKE) build

build: $(OBJ_DIR) \
	$(BIN_DIR) \
	$(BIN_DIR)/sonarint

export:
	$(ECHO) "    ---- Installing header files"
	$(ECHO) "    ---- Installing parameter files"
	$(SILENT) $(MKDIR) $(ETC_DIR)
	$(INSTALL_ETC) `pwd`/sonarint.ini $(ETC_DIR)


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

$(BIN_DIR)/sonarint: $(OBJS)
	$(ECHO) "    ---- Creating $@"
	$(SILENT) $(CC) $(INCLUDES) $(LIB_DIRS) $(CFLAGS) -o $@ \
		$(OBJS) $(SONARINT_LIBS)

##
## Standard suffix, version, packaging and dep rules
##

include $(RULES)
