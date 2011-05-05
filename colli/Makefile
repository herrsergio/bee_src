NAME = colli
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

OBJS := $(OBJ_DIR)/ROBOT.o $(OBJ_DIR)/rwibase_interface.o \
	$(OBJ_DIR)/sonar_interface.o $(OBJ_DIR)/laser_interface.o \
	$(OBJ_DIR)/server.o $(OBJ_DIR)/base-handlers.o \
	$(OBJ_DIR)/sonar-handlers.o \
	$(OBJ_DIR)/colli-handlers.o $(OBJ_DIR)/colliTools.o \
	$(OBJ_DIR)/colliTcx.o $(OBJ_DIR)/colliBase.o \
	$(OBJ_DIR)/colliTargets.o \
	$(OBJ_DIR)/colliTrajectories.o $(OBJ_DIR)/colliExceptions.o \
	$(OBJ_DIR)/colliModes.o $(OBJ_DIR)/colliGnuPlot.o \
	$(OBJ_DIR)/mathTools.o $(OBJ_DIR)/colliFiles.o \
	$(OBJ_DIR)/colliSonar.o \
	$(OBJ_DIR)/colliLaser.o \
	$(OBJ_DIR)/collision.o $(OBJ_DIR)/colliArm.o \
	$(OBJ_DIR)/colliArmExceptions.o \
	$(OBJ_DIR)/BASE.o \
	$(OBJ_DIR)/colliCleanUp.o $(OBJ_DIR)/colliCleanUpPick.o \
	$(OBJ_DIR)/colliCleanUpDrop.o $(OBJ_DIR)/colliCleanUpTools.o \
	$(OBJ_DIR)/colliCleanUpArm.o $(OBJ_DIR)/colliCleanUpSunvis.o \
	$(OBJ_DIR)/colliBumper.o $(OBJ_DIR)/colliIr.o 

COLLI_LIBS = -lX11 -ltcx -lrobot $(OS_LIBS) -lbUtils -lm -lezx 

CLIENT_OBJS = $(OBJ_DIR)/colliClient.o  $(OBJ_DIR)/libcolliClient.o

##
## Compulsory rules
##

all:
	$(MAKE) export $(DEPEND)
	$(MAKE) build

build: $(OBJ_DIR) \
	$(BIN_DIR) \
	$(LIB_DIR) \
	$(BIN_DIR)/colliServer \
	$(LIB_DIR)/libcolliClient.a \
#	$(BIN_DIR)/OBSTACLE_SERVER

export:
	$(ECHO) "    ---- Installing header files"
	$(SILENT) $(MKDIR) $(INC_DIR)
	$(INSTALL_H) `pwd`/BASE-messages.h $(INC_DIR)
	$(INSTALL_H) `pwd`/COLLI-messages.h $(INC_DIR)
	$(INSTALL_H) `pwd`/IR-messages.h $(INC_DIR)
	$(INSTALL_H) `pwd`/LASER-messages.h $(INC_DIR)
	$(INSTALL_H) `pwd`/SONAR-messages.h $(INC_DIR)
	$(INSTALL_H) `pwd`/rwibase_interface.h $(INC_DIR)
	$(INSTALL_H) `pwd`/laser_interface.h $(INC_DIR)
	$(INSTALL_H) `pwd`/collision.h $(INC_DIR)
	$(INSTALL_H) `pwd`/ROBOT.h $(INC_DIR)
	$(INSTALL_H) `pwd`/base-handlers.h $(INC_DIR)
	$(INSTALL_H) `pwd`/sonar_interface.h $(INC_DIR)
	$(INSTALL_H) `pwd`/colli-handlers.h $(INC_DIR)
	$(INSTALL_H) `pwd`/colliClient.h $(INC_DIR)
	$(INSTALL_H) `pwd`/libcolliClient.h $(INC_DIR)
	$(ECHO) "    ---- Installing parameter files"
	$(SILENT) $(MKDIR) $(ETC_DIR)
	$(INSTALL_ETC) `pwd`/colli_modes.ini $(ETC_DIR)


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

$(BIN_DIR)/colliServer: $(OBJS)
	$(ECHO) "    ---- Creating $@"
	$(SILENT) $(CC) $(INCLUDES) $(LIB_DIRS) $(CFLAGS) -o $@ \
		$(OBJS) $(COLLI_LIBS)

$(LIB_DIR)/libcolliClient.a: $(CLIENT_OBJS)
	$(ECHO) "    ---- Creating $@"
	$(SILENT) $(AR) $@ $(CLIENT_OBJS)
	$(SILENT) $(RANLIB) $@

$(BIN_DIR)/OBSTACLE_SERVER: obstacleServer.o obstacleServerTools.o 
	$(CC) $(INCLUDES) $(LIB_DIRS) $(CFLAGS) -DUNIBONN -o $@ \
	obstacleServer.o obstacleServerTools.o $(COLLI_LIBS) -llocalize -lGetDistance

##
## Standard suffix, version, packaging and dep rules
##

include $(RULES)

