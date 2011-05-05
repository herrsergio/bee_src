NAME = map
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

OBJS := $(OBJ_DIR)/MAP.o $(OBJ_DIR)/file.o \
	$(OBJ_DIR)/handlers.o $(OBJ_DIR)/init.o $(OBJ_DIR)/map-graphics.o

MAP_LIBS = -lezx -lX11 -ltcx -lrobot $(OS_LIBS) -lbUtils -lgd -lm


##
## Compulsory rules
##

all:
	$(MAKE) export $(DEPEND)
	$(MAKE) build

build: $(OBJ_DIR) \
	$(BIN_DIR) \
	$(BIN_DIR)/map

export:
	$(ECHO) "    ---- Installing header files"
	$(SILENT) $(MKDIR) $(INC_DIR)
	$(INSTALL_H) `pwd`/MAP-messages.h $(INC_DIR)
	$(ECHO) "    ---- Installing parameter files"
	$(SILENT) $(MKDIR) $(ETC_DIR)
	$(INSTALL_ETC) `pwd`/map.ini $(ETC_DIR)

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

$(BIN_DIR)/map: $(OBJS)
	$(ECHO) "    ---- Creating $@"
	$(SILENT) $(CC) $(INCLUDES) $(LIB_DIRS) $(CFLAGS) -o $@ \
		$(OBJS) $(MAP_LIBS)

##
## Standard suffix, version, packaging and dep rules
##

include $(RULES)
