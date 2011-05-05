NAME = buttonServer
CONF  = ../Makefile.conf
RULES = ../Makefile.rules

##
## System config variables
##

include $(CONF)

##
## Variables
##

#CFLAGS       += -Wall -g

# leave the following two comments in the makefile!!
#CFLAGS       += -DBUTTON_DEBUG
#CFLAGS       += -DTCX_DEBUG

DEPENDENCIES += *.c

#SILENT        = 

##
## local build variables
##

OBJS := $(OBJ_DIR)/button.o $(OBJ_DIR)/buttonHandlers.o

BUTTON_LIBS =  -lbUtils  -ltcx -lrobot $(OS_LIBS) -lm

CLIENT_OBJS = $(OBJ_DIR)/buttonClient.o

##
##
EXAMPLE_LIBS = -lbuttonClient -lraiClient -lrai -lbUtils -ltcx $(OS_LIBS) -lm
EXAMPLE_OBJS = $(OBJ_DIR)/example.o

##
## Compulsory rules
##

all:
	$(MAKE) export $(DEPEND)
	$(MAKE) build

build: $(OBJ_DIR) \
	$(BIN_DIR) \
	$(LIB_DIR) \
	$(BIN_DIR)/buttonServer \
	$(LIB_DIR)/libbuttonClient.a \
	$(BIN_DIR)/buttonExample

export:
	$(ECHO) "    ---- Installing header files"
	$(SILENT) $(MKDIR) $(INC_DIR)
	$(INSTALL_H) `pwd`/BUTTONS-messages.h $(INC_DIR)
	$(INSTALL_H) `pwd`/buttonClient.h $(INC_DIR)

clean:
	$(RM) *.o *.a *~ core a.out obj obj_sun obj_solaris $(CLEAN_FILES) \
	$(BIN_DIR)/buttonExample $(BIN_DIR)/buttonServer $(LIB_DIR)/libbuttonClient.a

distclean: clean
	$(RM) $(DEPEND) Makefile.depend* \
		$(NAME)-src.tgz $(NAME)-bin.tgz $(DISTCLEAN_FILES)

##
## local rules
##

$(OBJ_DIR) $(LIB_DIR) $(BIN_DIR):
	$(SILENT) $(MKDIR) $@

$(BIN_DIR)/buttonServer: $(OBJS) $(LIB_DIR)/libbUtils.a $(LIB_DIR)/libtcx.a $(LIB_DIR)/librobot.a
	$(ECHO) "    ->>- Creating $@"
	$(SILENT) $(CC) $(INCLUDES) $(LIB_DIRS) $(CFLAGS) -o $@ \
		$(OBJS) $(BUTTON_LIBS)

$(BIN_DIR)/buttonExample: $(EXAMPLE_OBJS) $(LIB_DIR)/libbuttonClient.a\
	 $(LIB_DIR)/libraiClient.a $(LIB_DIR)/librai.a $(LIB_DIR)/libbUtils.a $(LIB_DIR)/libtcx.a
	$(ECHO) "    ->>- Creating $@"
	$(SILENT) $(CC) -o $@ $(EXAMPLE_OBJS) $(LIB_DIRS) $(EXAMPLE_LIBS) $(INCLUDES) 

$(LIB_DIR)/libbuttonClient.a: $(CLIENT_OBJS)
	$(ECHO) "    ->>- Creating $@"
	$(SILENT) $(AR) $@ $(CLIENT_OBJS)
	$(SILENT) $(RANLIB) $@

##
## Standard suffix, version, packaging and dep rules
##

include $(RULES)
