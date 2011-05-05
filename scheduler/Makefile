NAME = scheduler
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

build: $(OBJ_DIR) $(LIB_DIR) \
	$(LIB_DIR)/libraiClient.a $(LIB_DIR)/librai.a

export:
	$(ECHO) "    ---- Installing header files"
	$(SILENT) $(MKDIR) $(INC_DIR)
	$(INSTALL_H) `pwd`/rai.h `pwd`/raiClient.h `pwd`/raiServer.h $(INC_DIR)
	$(INSTALL_H) `pwd`/librai.h $(INC_DIR)
	$(INSTALL_H) `pwd`/libraiClient.h $(INC_DIR)



#install:
#	$(ECHO) "    ---- Installing libs"
#	$(SILENT) $(MKDIR) $(LIB_DIR)
#	$(INSTALL_LIB) $(OBJ_DIR)/librai.a $(LIB_DIR)
#	$(INSTALL_LIB) $(OBJ_DIR)/libraiClient.a $(LIB_DIR)

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

$(LIB_DIR)/librai.a: $(OBJ_DIR)/rai.o $(OBJ_DIR)/librai.o
	$(ECHO) "    ---- Creating $@"
	$(SILENT) $(AR) $@ $(OBJ_DIR)/rai.o $(OBJ_DIR)/librai.o
	$(SILENT) $(RANLIB) $@

$(LIB_DIR)/libraiClient.a: $(OBJ_DIR)/raiClient.o $(OBJ_DIR)/libraiClient.o
	$(ECHO) "    ---- Creating $@"
	$(SILENT) $(AR) $@ $(OBJ_DIR)/raiClient.o $(OBJ_DIR)/libraiClient.o
	$(SILENT) $(RANLIB) $@

##
## Standard suffix, version, packaging and dep rules
##

include $(RULES)
