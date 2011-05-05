NAME = utils
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
	$(LIB_DIR)/libutils.a \
	$(LIB_DIR)/libbUtils.a \
	$(BIN_DIR)/bWatch \
	$(BIN_DIR)/bCheck \

export:
	$(ECHO) "    ---- Installing header files"
	$(SILENT) $(MKDIR) $(INC_DIR)
	$(INSTALL_H) `pwd`/*.h $(INC_DIR)


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

$(LIB_DIR)/libutils.a: $(OBJ_DIR)/libutils.o \
			$(OBJ_DIR)/itrig.o \
			$(OBJ_DIR)/utils.o
	$(ECHO) "    ---- Creating $@"
	$(SILENT) $(AR) $@ $(OBJ_DIR)/utils.o $(OBJ_DIR)/itrig.o \
	$(OBJ_DIR)/libutils.o 
	$(SILENT) $(RANLIB) $@

$(BIN_DIR)/bWatch: bWatch
	$(ECHO) "    ---- Installing $@"
	$(SILENT) $(CP) bWatch $(BIN_DIR)
	$(SILENT) chmod a+x $(BIN_DIR)/bWatch

$(BIN_DIR)/bCheck: bCheck
	$(ECHO) "    ---- Installing $@"
	$(SILENT) $(CP) bCheck $(BIN_DIR)
	$(SILENT) chmod a+x $(BIN_DIR)/bCheck

$(LIB_DIR)/libbUtils.a: $(OBJ_DIR)/libbUtils.o $(OBJ_DIR)/bUtils.o
	$(ECHO) "    ---- Creating $@"
	$(SILENT) $(AR) $@ $(OBJ_DIR)/bUtils.o $(OBJ_DIR)/libbUtils.o
	$(SILENT) $(RANLIB) $@

bUtilsTest: $(OBJ_DIR)/bUtilsTest.o $(LIB_DIR)/libbUtils.a
	$(ECHO) "    ---- Creating $@"
	$(SILENT) $(CC) $(INCLUDES) $(LIB_DIRS) $(CFLAGS) -o $@ \
		$(OBJ_DIR)/bUtilsTest.o -lbUtils

##
## Standard suffix, version, packaging rules and dep
##

include $(RULES)
