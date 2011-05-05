NAME = ezx
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

OBJS = $(OBJ_DIR)/colorx.o $(OBJ_DIR)/drawx.o $(OBJ_DIR)/initx.o \
	$(OBJ_DIR)/cursorx.o $(OBJ_DIR)/grey.o $(OBJ_DIR)/textx.o \
	$(OBJ_DIR)/windowx.o $(OBJ_DIR)/o-graphics.o \
	$(OBJ_DIR)/libezx.o

LIBS +=

CLEAN_FILES += 
DISTCLEAN_FILES += 

##
## Compulsory rules
##

all:
	$(MAKE) export $(DEPEND)
	$(MAKE) build

build: $(OBJ_DIR) \
	$(LIB_DIR) \
	$(LIB_DIR)/libezx.a 


export:
	$(ECHO) "    ---- Installing header files"
	$(SILENT) $(MKDIR) $(INC_DIR)
	$(INSTALL_H) `pwd`/EZX11.h $(INC_DIR)
	$(INSTALL_H) `pwd`/o-graphics.h $(INC_DIR)
	$(INSTALL_H) `pwd`/libezx.h $(INC_DIR)


#install:
#	$(ECHO) "    ---- Installing libs"
#	$(SILENT) $(MKDIR) $(LIB_DIR)
#	$(INSTALL_LIB) $(OBJ_DIR)/libezx.a $(LIB_DIR)
#	$(ECHO) "    ---- Installing bins"
#	$(SILENT) $(MKDIR) $(BIN_DIR)

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

$(LIB_DIR)/libezx.a: $(OBJS)
	$(ECHO) "    ---- Creating $@"
	$(SILENT) $(AR) $@ $(OBJS)
	$(SILENT) $(RANLIB) $@

##
## Standard suffix, version/packaging and dep rules
##

include $(RULES)
