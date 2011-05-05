NAME = simulator2
CONF  = ../Makefile.conf
RULES = ../Makefile.rules

##
## System config variables
##

include $(CONF)

INSTALL_ETC = $(INSTALL_H)

##
## Variables
##

DEPENDENCIES += *.c *.cc

##
## local build variables
##

OBJS += $(OBJ_DIR)/main.o $(OBJ_DIR)/simxw.o $(OBJ_DIR)/robot.o 
OBJS += $(OBJ_DIR)/base.o $(OBJ_DIR)/SIMULATOR_TCX.o $(OBJ_DIR)/sonar.o
OBJS += $(OBJ_DIR)/laser.o $(OBJ_DIR)/playground.o $(OBJ_DIR)/obstacles.o
OBJS += $(OBJ_DIR)/rectangle.o $(OBJ_DIR)/circle.o $(OBJ_DIR)/linalg.o
OBJS += $(OBJ_DIR)/store.o $(OBJ_DIR)/doors.o $(OBJ_DIR)/trigofkt.o
OBJS += $(OBJ_DIR)/surface.o $(OBJ_DIR)/reader.o $(OBJ_DIR)/schedule.o
OBJS += $(OBJ_DIR)/human.o $(OBJ_DIR)/ir.o $(OBJ_DIR)/tactile.o

LIBS = -lX11 -lXt -lXaw -lXmu -lXext -lICE -lSM -lXdw -ltcx -lrobot \
	-lbUtils -lm -lstdc++

ifeq ($(OS_NAME), Linux)
USER_DEFINES = -D_GNU_SOURCE -DX_LOCALE -DFUNCPROTO=15 -DNARROWPROTO 
else
USER_DEFINES = -D_GNU_SOURCE -DX_LOCALE
endif

##
## Compulsory rules
##

all:
	$(MAKE) export $(DEPEND)
	$(MAKE) build

build:  $(OBJ_DIR) \
	$(BIN_DIR) \
	$(LIB_DIR) \
	$(LIB_DIR)/libXdw.a \
	$(BIN_DIR)/simulator
#	$(BIN_DIR)/map2gridmap

export:
	$(ECHO) "    ---- Installing header files"
	$(SILENT) $(MKDIR) $(INC_DIR)
	$(INSTALL_H) `pwd`/SIMULATOR-messages.h $(INC_DIR)
	$(ECHO) "    ---- Installing parameter files"
	$(SILENT) $(MKDIR) $(ETC_DIR)
	$(INSTALL_ETC) `pwd`/simulator2.ini $(ETC_DIR)

clean:
	$(RM) *.o *.a *~ core a.out obj obj_sun obj_sun4 obj_solaris \
		$(CLEAN_FILES)

distclean: clean
	$(RM) $(DEPEND) Makefile.depend* \
		$(NAME)-src.tgz $(NAME)-bin.tgz $(DISTCLEAN_FILES)

##
## local rules
##

$(OBJ_DIR) $(LIB_DIR) $(BIN_DIR):
	$(SILENT) $(MKDIR) $@

$(BIN_DIR)/simulator: $(OBJS)
	$(ECHO) "    ---- Creating $@"
	$(SILENT) $(CC) $(INCLUDES) $(LIB_DIRS) $(CFLAGS) -o $@ \
		$(OBJS) $(LIBS) $(OS_LIBS)

$(BIN_DIR)/map2gridmap: $(OBJ_DIR)/map2gridmap.o  $(OBJ_DIR)/store.o $(OBJ_DIR)/obstacles.o $(OBJ_DIR)/rectangle.o $(OBJ_DIR)/circle.o $(OBJ_DIR)/linalg.o $(OBJ_DIR)/doors.o $(OBJ_DIR)/surface.o $(OBJ_DIR)/xwstub.o                 
	$(ECHO) "    ---- Creating $@"
	$(SILENT) $(CC) $(INCLUDES) $(LIB_DIRS) $(CFLAGS) -o $@ \
		$(OBJ_DIR)/map2gridmap.o $(OBJ_DIR)/store.o $(OBJ_DIR)/obstacles.o $(OBJ_DIR)/rectangle.o $(OBJ_DIR)/circle.o  $(OBJ_DIR)/linalg.o $(OBJ_DIR)/doors.o $(OBJ_DIR)/surface.o  $(OBJ_DIR)/xwstub.o $(LIBS) $(OS_LIBS)

##
## Standard suffix, version, packaging and dep rules
##

include $(RULES)
