NAME = baseServer
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

MODULE_OBJS = $(OBJ_DIR)/base.o $(OBJ_DIR)/statusReport.o $(OBJ_DIR)/libbase.o
SERVER_OBJS = $(OBJ_DIR)/baseServer.o
CLIENT_OBJS = $(OBJ_DIR)/baseClient.o $(OBJ_DIR)/libbaseClient.o

SERVER_LIBS = -lrai -lbase -lmcp -lmspmodule -lmsp -lutils -ltcx \
	-labus -lbUtils $(OS_LIBS) -lm


##
## Compulsory rules
##

all:
	$(MAKE) export $(DEPEND)
	$(MAKE) build

build: $(OBJ_DIR) $(LIB_DIR) $(BIN_DIR) \
	$(OBJ_DIR)/libbase.a \
	$(BIN_DIR)/baseServer \
	$(LIB_DIR)/libbaseClient.a

export:
	$(ECHO) "    ---- Installing header files"
	$(SILENT) $(MKDIR) $(INC_DIR)
#	$(INSTALL_H) `pwd`/base.h $(INC_DIR)
	$(INSTALL_H) `pwd`/baseClient.h $(INC_DIR)
	$(INSTALL_H) `pwd`/baseMessages.h $(INC_DIR)
	$(INSTALL_H) `pwd`/baseOpcodes.h $(INC_DIR)
	$(INSTALL_H) `pwd`/irClient.h $(INC_DIR)
	$(INSTALL_H) `pwd`/sensorMessages.h $(INC_DIR)
	$(INSTALL_H) `pwd`/sonarClient.h $(INC_DIR)
	$(INSTALL_H) `pwd`/tactileClient.h $(INC_DIR)
	$(INSTALL_H) `pwd`/libbase.h $(INC_DIR)
	$(INSTALL_H) `pwd`/libbaseClient.h $(INC_DIR)
	$(INSTALL_H) `pwd`/odoClient.h $(INC_DIR)


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

$(OBJ_DIR)/libbase.a: $(MODULE_OBJS)
	$(ECHO) "    ---- Creating $@"
	$(SILENT) $(AR) $@ $(MODULE_OBJS)
	$(SILENT) $(RANLIB) $@

$(BIN_DIR)/baseServer: $(SERVER_OBJS)
	$(ECHO) "    ---- Creating $@"
	$(SILENT) $(CC) $(INCLUDES) $(LIB_DIRS) $(CFLAGS) -o $@ \
		$(SERVER_OBJS) $(SERVER_LIBS)

$(LIB_DIR)/libbaseClient.a: $(CLIENT_OBJS)
	$(ECHO) "    ---- Creating $@"
	$(SILENT) $(AR) $@ $(CLIENT_OBJS)
	$(SILENT) $(RANLIB) $@

##
## Standard suffix, version, packaging and dep rules
##

include $(RULES)
