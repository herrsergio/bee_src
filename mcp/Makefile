NAME = mcp
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

LIBS +=

CLEAN_FILES += 

DISTCLEAN_FILES += 

##
## Compulsory rules
##

all:
	$(MAKE) export $(DEPEND)
	$(MAKE) build

build: $(OBJ_DIR) $(LIB_DIR) $(LIB_DIR)/libmcp.a

export:
	$(ECHO) "    ---- Installing header files"
	$(SILENT) $(MKDIR) $(INC_DIR)
	$(INSTALL_H) `pwd`/mcpStatus.h `pwd`/mcpIO.h $(INC_DIR)
	$(INSTALL_H) `pwd`/libmcp.h $(INC_DIR)


#install:
#	$(ECHO) "    ---- Installing libs"
#	$(SILENT) $(MKDIR) $(LIB_DIR)
#	$(INSTALL_LIB) $(OBJ_DIR)/libmcp.a $(LIB_DIR)

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

$(LIB_DIR)/libmcp.a: $(OBJ_DIR)/mcp.o $(OBJ_DIR)/libmcp.o
	$(ECHO) "    ---- Creating $@"
	$(SILENT) $(AR) $@ $(OBJ_DIR)/mcp.o $(OBJ_DIR)/libmcp.o
	$(SILENT) $(RANLIB) $@

##
## Standard suffix, version, packaging and dep rules
##

include $(RULES)
