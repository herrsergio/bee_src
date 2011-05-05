NAME = controller
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

OBJS := $(OBJ_DIR)/corr.o $(OBJ_DIR)/file.o $(OBJ_DIR)/graphics.o \
	$(OBJ_DIR)/init.o $(OBJ_DIR)/main.o $(OBJ_DIR)/mouse.o \
	$(OBJ_DIR)/task.o $(OBJ_DIR)/tour.o \
	$(OBJ_DIR)/action.o
#	$(OBJ_DIR)/sample.o 

CONTROLLER_LIBS = -lezx -lX11 -ltcx -lrobot $(OS_LIBS) -lbUtils -lm


##
## Compulsory rules
##

all:
	$(MAKE) export $(DEPEND)
	$(MAKE) build

build: $(OBJ_DIR) $(BIN_DIR) \
	$(BIN_DIR)/commander

export:
	$(ECHO) "    ---- Installing header files"
	$(ECHO) "    ---- Installing parameter files"
	$(SILENT) $(MKDIR) $(ETC_DIR)
	$(INSTALL_ETC) `pwd`/controller.ini $(ETC_DIR)

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

$(BIN_DIR)/commander: $(OBJS)
	$(ECHO) "    ---- Creating $@"
	$(SILENT) $(CC) $(INCLUDES) $(LIB_DIRS) $(CFLAGS) -o $@ \
		$(OBJS) $(CONTROLLER_LIBS)

##
## Standard suffix, version, packaging and dep rules
##

include $(RULES)
