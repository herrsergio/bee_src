NAME = laserServer
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

CFLAGS      += -Wall		# for development purposes

#CFLAGS      += -DTCX_debug	# for development purposes

##
## local build variables
##

OBJS := $(OBJ_DIR)/mainlaser.o \
	$(OBJ_DIR)/io.o \
	$(OBJ_DIR)/laserHandlers.o 

LASER_LIBS = -lrobot -lbUtils -lrobot -ltcx -lezx -lX11 $(OS_LIBS) -lm 

CLIENT_OBJS = $(OBJ_DIR)/laserClient.o

##
##
EXAMPLE_LIBS = -llaserClient -lraiClient \
		-lrai -lbUtils -ltcx $(OS_LIBS) -lm
EXAMPLE_OBJS = $(OBJ_DIR)/laserExample.o

##
## Compulsory rules
##

all:
	$(MAKE) export $(DEPEND)
	$(MAKE) build

build: $(OBJ_DIR) \
	$(BIN_DIR) \
	$(LIB_DIR) \
	$(BIN_DIR)/laserServer \
	$(LIB_DIR)/liblaserClient.a \
	$(BIN_DIR)/laserExample 

export:
	$(INSTALL_H) `pwd`/laserClient.h $(INC_DIR)
	$(INSTALL_H) `pwd`/LASER_SERVER-messages.h $(INC_DIR)

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

$(BIN_DIR)/laserServer: $(OBJS) $(LIB_DIR)/librobot.a $(LIB_DIR)/libbUtils.a\
	$(LIB_DIR)/libtcx.a $(LIB_DIR)/libezx.a
	$(ECHO) "    ->>- Creating $@"
	$(SILENT) $(CC) $(INCLUDES) $(LIB_DIRS) $(CFLAGS) -o $@ \
		$(OBJS) $(LASER_LIBS)

$(BIN_DIR)/laserExample: $(EXAMPLE_OBJS) $(LIB_DIR)/liblaserClient.a \
	$(LIB_DIR)/libraiClient.a $(LIB_DIR)/librai.a $(LIB_DIR)/libbUtils.a \
	$(LIB_DIR)/libtcx.a
	$(ECHO) "    ->>- Creating $@"
	$(SILENT) $(CC) -o $@ $(EXAMPLE_OBJS) $(LIB_DIRS) $(EXAMPLE_LIBS) $(INCLUDES) 

$(LIB_DIR)/liblaserClient.a: $(CLIENT_OBJS)
	$(ECHO) "    ->>- Creating $@"
	$(SILENT) $(AR) $@ $(CLIENT_OBJS)
	$(SILENT) $(RANLIB) $@

##
## Standard suffix, version, packaging and dep rules
##

include $(RULES)
