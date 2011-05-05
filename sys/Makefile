NAME = sys
CONF  = ../Makefile.conf
RULES = ../Makefile.rules

##
## System config variables
##

include $(CONF)

##
## Variables
##


##
## local build variables
##


##
## Compulsory rules
##

all:
	$(MAKE) export $(DEPEND)
	$(MAKE) build

build:

export:
	$(ECHO) "    ---- Installing header files"
	$(SILENT) $(MKDIR) $(INC_DIR)
	$(INSTALL_H) `pwd`/mspMessages.h $(INC_DIR)
	$(ECHO) "    ---- Installing etc scripts"
	$(SILENT) $(MKDIR) $(ETC_DIR)
	$(INSTALL_H) `pwd`/beeSoft.ini $(ETC_DIR)

build:


clean:
	$(RM) *.o *.a *~ core a.out obj obj_sun obj_solaris $(CLEAN_FILES)

distclean: clean
	$(RM) $(DEPEND) Makefile.depend* \
		$(NAME)-src.tgz $(NAME)-bin.tgz $(DISTCLEAN_FILES)

##
## local rules
##


##
## Standard suffix, version/packaging and dep rules
##

include $(RULES)
