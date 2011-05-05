NAME = armServer
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

MODULE_OBJS = $(OBJ_DIR)/arm.o $(OBJ_DIR)/statusReport.o
SERVER_OBJS = $(OBJ_DIR)/armServer.o $(OBJ_DIR)/libarm.a
CLIENT_OBJS = $(OBJ_DIR)/armClient.o $(OBJ_DIR)/libarmClient.o

SERVER_LIBS = -lrai -lmcp -lutils -ltcx -lbUtils $(OS_LIBS)

##
## Compulsory rules
##

all:
	$(MAKE) export $(DEPEND)
	$(MAKE) build

build: $(OBJ_DIR) $(LIB_DIR) $(BIN_DIR) \
	$(OBJ_DIR)/libarm.a \
	$(BIN_DIR)/armServer \
	$(LIB_DIR)/libarmClient.a

export:
	$(ECHO) "    ---- Installing header files"
	$(SILENT) $(MKDIR) $(INC_DIR)
	$(INSTALL_H) `pwd`/armMessages.h $(INC_DIR)
	$(INSTALL_H) `pwd`/armClient.h $(INC_DIR)
	$(INSTALL_H) `pwd`/libarmClient.h $(INC_DIR)

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

$(OBJ_DIR)/libarm.a: $(MODULE_OBJS)
	$(ECHO) "    ---- Creating $@"
	$(SILENT) $(AR) $@ $(MODULE_OBJS)
	$(SILENT) $(RANLIB) $@


$(BIN_DIR)/armServer: $(SERVER_OBJS)
	$(ECHO) "    ---- Creating $@"
	$(SILENT) $(CC) $(INCLUDES) $(LIB_DIRS) $(CFLAGS) -o $@ \
		$(SERVER_OBJS) $(SERVER_LIBS)

$(LIB_DIR)/libarmClient.a: $(CLIENT_OBJS)
	$(ECHO) "    ---- Creating $@"
	$(SILENT) $(AR) $@ $(CLIENT_OBJS)
	$(SILENT) $(RANLIB) $@

##
## Standard suffix, version, packaging and dep rules
##

include $(RULES)
