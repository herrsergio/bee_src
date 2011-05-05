# Makefile for the Linux ACCESS.bus device driver
#
# The loadable kernel module version
#
# Tyson D. Sawyer, Real World Interface, Inc.
# tyson@rwii.com
#

NAME  = abus
CONF  = ../Makefile.conf
RULES = ../Makefile.rules

##
## System config variables
##

include $(CONF)

##
## Variables
##

ifeq ($(OS_NAME),Linux)
DEPENDENCIES := abus.c abusd.c abusk.c
else
DEPENDENCIES := abus.c
endif

CLEAN_FILES = abusd libabus.so* $(LIB_DIR)/abusk.o

##
## local build variables
##

ifeq ($(SMP),)
else
CFLAGS += -D__SMP__

ifeq ($(SMP_PROF),)
else
CFLAGS += -D__SMP_PROF__
endif
endif

CFLAGS += -I/usr/include -Idefault

##
## Compulsory rules
##

all:
	$(MAKE) export $(DEPEND)
	$(MAKE) build

ifeq ($(OS_NAME),Linux)
build: $(OBJ_DIR) $(LIB_DIR) $(BIN_DIR) \
		$(LIB_DIR)/abusk.o \
		$(BIN_DIR)/abusd \
		$(LIB_DIR)/libabus.a
	@echo "VERSION            = $(VERSION)"
	@echo "PATCHLEVEL         = $(PATCHLEVEL)"
	@echo "SUBLEVEL           = $(SUBLEVEL)"
	@echo "LINUX_VERSION_CODE = $(LINUX_VERSION_CODE)"
	@echo "SMP                = $(SMP)"
	@echo "SMP_PROF           = $(SMP_PROF)"
	@echo "CFLAGS             = $(CFLAGS)"
else
build: $(OBJ_DIR) $(LIB_DIR) $(LIB_DIR)/libabus.a
endif

export:
	$(ECHO) "    ---- Installing header files"
	$(SILENT) $(MKDIR) $(INC_DIR)/acb
	$(INSTALL_H) `pwd`/acb/*.h $(INC_DIR)/acb
	$(INSTALL_H) `pwd`/libabus.h $(INC_DIR)


clean:
	$(RM) *.o *.a *~ core a.out obj obj_sun obj_solaris $(CLEAN_FILES)

distclean: clean
	$(RM) $(DEPEND) Makefile.depend* \
		$(NAME)-src.tgz $(NAME)-bin.tgz $(DISTCLEAN_FILES)

##
## local rules
##

ifeq ($(OS_NAME),Linux)
remove_old:
	-sudo mv /usr/sbin/abusd /tmp
	-sudo rm -f /tmp/abusd
	-sudo rm -rf /usr/local/include/acb
	-sudo rm -rf /usr/include/acb
	-find /lib/modules -name abusk.o -exec rm {} \;
	/bin/ln -fs ~bee/lib/abusk.o /lib/modules/default/misc/abusk.o

dev:
	/bin/mknod -m 666 /dev/acb0 c 28 0
	/bin/ln -s /dev/acb0 /dev/abus
	/bin/ln -fs ~bee/lib/abusk.o /lib/modules/default/misc/abusk.o
endif

$(LIB_DIR)/abusk.o: abusk.c
	$(ECHO) "    ---- Compiling $< (C)"
	$(SILENT) $(CC) $(CFLAGS) $(INCLUDES) -c -o $@ $<

$(OBJ_DIR) $(LIB_DIR) $(BIN_DIR):
	$(SILENT) $(MKDIR) $@

$(BIN_DIR)/abusd: $(OBJ_DIR) $(BIN_DIR) $(OBJ_DIR)/abusd.o
	$(ECHO) "    ---- Creating $@"
	$(SILENT) $(CC) $(INCLUDES) $(CFLAGS) -o $@ \
		$(OBJ_DIR)/abusd.o

$(OBJ_DIR)/abus-pic.o: $(OBJ_DIR) abus.c
	$(ECHO) "    ---- Creating $@"
	$(SILENT) $(CC) -c -g -fPIC $(CFLAGS) $(INCLUDES) -c -o $@ abus.c

$(OBJ_DIR)/libabus.so.1.0.0: $(OBJ_DIR) $(LIB_DIR) $(OBJ_DIR)/abus-pic.o
	$(ECHO) "    ---- Creating $@"
	$(SILENT) $(CC) -shared -Wl,-soname,libabus.so.1 -o $@ \
		$(OBJ_DIR)/abus-pic.o

$(LIB_DIR)/libabus.a: $(OBJ_DIR) $(LIB_DIR) $(OBJ_DIR)/abus.o $(OBJ_DIR)/libabus.o
	$(ECHO) "    ---- Creating $@"
	$(SILENT) $(AR) $@ $(OBJ_DIR)/abus.o $(OBJ_DIR)/libabus.o
	$(SILENT) $(RANLIB) $@

##
## Standard suffix, version, packaging and dep rules
##

include $(RULES)
