NAME = tcx
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

OBJS = $(OBJ_DIR)/com.o $(OBJ_DIR)/comA.o $(OBJ_DIR)/data.o \
	$(OBJ_DIR)/formatters.o $(OBJ_DIR)/hash.o $(OBJ_DIR)/lex.o \
	$(OBJ_DIR)/list.o $(OBJ_DIR)/module.o $(OBJ_DIR)/msg.o \
	$(OBJ_DIR)/parseFmttrs.o $(OBJ_DIR)/primFmttrs.o \
	$(OBJ_DIR)/reg.o $(OBJ_DIR)/tcaMatrix.o \
        $(OBJ_DIR)/tcaMem.o $(OBJ_DIR)/global.o $(OBJ_DIR)/ws_gethostname.o \
	$(OBJ_DIR)/tcxcheck.o $(OBJ_DIR)/version.o

LIBS += -lbUtils

CLEAN_FILES += 
DISTCLEAN_FILES += 

# use this flag if you want tons of output
#CFLAGS       += -DVERBOSE

##
## Compulsory rules
##

all:
	$(MAKE) export $(DEPEND)
	$(MAKE) build

build: $(OBJ_DIR) \
	$(LIB_DIR) \
	$(BIN_DIR) \
	$(LIB_DIR)/libtcx.a \
	$(BIN_DIR)/tcxServer

export:
	$(ECHO) "    ---- Installing header files"
	$(SILENT) $(MKDIR) $(INC_DIR)
	$(INSTALL_H) `pwd`/tcx.h $(INC_DIR)
	$(INSTALL_H) `pwd`/tcxP.h $(INC_DIR)
	$(INSTALL_H) `pwd`/global.h $(INC_DIR)
	$(INSTALL_H) `pwd`/lex.h $(INC_DIR)
	$(INSTALL_H) `pwd`/list.h $(INC_DIR)
	$(INSTALL_H) `pwd`/hash.h $(INC_DIR)
	$(INSTALL_H) `pwd`/formatters.h $(INC_DIR)


#install:
#	$(ECHO) "    ---- Installing libs"
#	$(SILENT) $(MKDIR) $(LIB_DIR)
#	$(INSTALL_LIB) $(OBJ_DIR)/libtcx.a $(LIB_DIR)
#	$(ECHO) "    ---- Installing bins"
#	$(SILENT) $(MKDIR) $(BIN_DIR)
#	$(INSTALL_EXEC) $(OBJ_DIR)/tcxServer $(BIN_DIR)

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

$(LIB_DIR)/libtcx.a: $(OBJS)
	$(ECHO) "    ---- Creating $@"
	$(SILENT) $(AR) $@ $(OBJS)
	$(SILENT) $(RANLIB) $@

$(BIN_DIR)/tcxServer: $(OBJ_DIR)/tcxServer.o $(LIB_DIR)/libtcx.a
	$(ECHO) "    ---- Creating $@"
	$(SILENT) $(CC) $(INCLUDES) $(CFLAGS) -O0 -g -o $@ $(OBJ_DIR)/tcxServer.o \
		$(LIB_DIRS) $(LIBS) $(OS_LIBS) $(LIB_DIR)/libtcx.a

$(BIN_DIR)/a1: $(OBJ_DIR)/a1.o $(LIB_DIR)/libtcx.a
	$(ECHO) "    ---- Creating $@"
	$(SILENT) $(CC) $(INCLUDES) $(CFLAGS) -o $@ \
		$(OBJ_DIR)/a1.o $(LIB_DIR)/libtcx.a $(OS_LIBS) 

$(BIN_DIR)/b1: $(OBJ_DIR)/b1.o $(LIB_DIR)/libtcx.a
	$(ECHO) "    ---- Creating $@"
	$(SILENT) $(CC) $(INCLUDES) $(CFLAGS) -o $@ \
		$(OBJ_DIR)/b1.o $(LIB_DIR)/libtcx.a $(OS_LIBS) 

test: $(LIB_DIR)/libtcx.a $(BIN_DIR)/tcxServer \
	$(BIN_DIR)/a1 $(BIN_DIR)/b1

##
## Standard suffix, version/packaging and dep rules
##

include $(RULES)
