NAME = speechServer
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

MODULE_OBJS = $(OBJ_DIR)/speech.o $(OBJ_DIR)/libspeech.o
SERVER_OBJS = $(OBJ_DIR)/speechServer.o
CLIENT_OBJS = $(OBJ_DIR)/speechClient.o $(OBJ_DIR)/libspeechClient.o

SERVER_LIBS = -lspeech -lutils -ltcx -lbUtils $(OS_LIBS)

##
## Compulsory rules
##

all:
	$(MAKE) export $(DEPEND)
	$(MAKE) build

build: $(OBJ_DIR) \
	$(LIB_DIR) \
	$(BIN_DIR) \
	$(OBJ_DIR)/libspeech.a \
	$(BIN_DIR)/speechServer \
	$(LIB_DIR)/libspeechClient.a

export:
	$(ECHO) "    ---- Installing header files"
	$(SILENT) $(MKDIR) $(INC_DIR)
#	$(INSTALL_H) `pwd`/speech.h $(INC_DIR)
	$(INSTALL_H) `pwd`/speechClient.h $(INC_DIR)
	$(INSTALL_H) `pwd`/libspeech.h $(INC_DIR)
	$(INSTALL_H) `pwd`/libspeechClient.h $(INC_DIR)


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

$(OBJ_DIR)/libspeech.a: $(MODULE_OBJS)
	$(ECHO) "    ---- Creating $@"
	$(SILENT) $(AR) $@ $(MODULE_OBJS)
	$(SILENT) $(RANLIB) $@

$(BIN_DIR)/speechServer: $(SERVER_OBJS)
	$(ECHO) "    ---- Creating $@"
	$(SILENT) $(CC) $(INCLUDES) $(LIB_DIRS) $(CFLAGS) -o $@ \
		$(SERVER_OBJS) $(SERVER_LIBS)

$(LIB_DIR)/libspeechClient.a: $(CLIENT_OBJS)
	$(ECHO) "    ---- Creating $@"
	$(SILENT) $(AR) $@ $(CLIENT_OBJS)
	$(SILENT) $(RANLIB) $@

##
## Standard suffix, version, packaging and dep rules
##

include $(RULES)
