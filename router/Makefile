NAME = router
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

OBJS := $(OBJ_DIR)/ROUTER.o $(OBJ_DIR)/rbase-handlers.o \
	$(OBJ_DIR)/rcolli-handlers.o $(OBJ_DIR)/router.o \
	$(OBJ_DIR)/rsonar-handlers.o $(OBJ_DIR)/rlaser-handlers.o

ROUTER_LIBS = -lrobot -lezx -lX11 -ltcx -lbUtils $(OS_LIBS) -lm


##
## Compulsory rules
##

all:
	$(MAKE) export $(DEPEND)
	$(MAKE) build

build: $(OBJ_DIR) \
	$(BIN_DIR) \
	$(BIN_DIR)/router

export:
	$(ECHO) "    ---- Installing header files"


#install:
#	$(ECHO) "    ---- Installing bins"
#	$(SILENT) $(MKDIR) $(BIN_DIR)
#	$(INSTALL_EXEC) $(OBJ_DIR)/ROUTER $(BIN_DIR)

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

$(BIN_DIR)/router: $(OBJS)
	$(ECHO) "    ---- Creating $@"
	$(SILENT) $(CC) $(INCLUDES) $(LIB_DIRS) $(CFLAGS) -o $@ \
		$(OBJS) $(ROUTER_LIBS)

##
## Standard suffix, version, packaging and dep rules
##

include $(RULES)
