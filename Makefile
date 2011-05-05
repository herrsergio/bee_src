NAME  = beeSoft

CONF  = Makefile.conf
RULES = Makefile.rules

include $(CONF)

TAR_SRC += Makefile $(CONF) $(RULES) Makefile.example
TAR_SRC += README

TAR_BIN = include lib bin etc doc

PACKAGES = 	abus sys scheduler utils mcp msp consoleLib tcx robot ezx \
		baseServer speechServer armServer laserServer \
		sound libgd network \
		colli buttons plan controller \
		map sonarint Drawing10 simulator2\
		pantilt vision speech \
		laserint router collgraph cameraServer beeExamples 


# PACKAGES +=	sunvis cd arm tracker flow convert localize \
#		motion showpic

# PACKAGES +=	detection hli meteor

# PACKAGES +=	$(DOC_DIR_2)/rhino $(DOC_DIR_2)/rai

RECURSE = $(SILENT) \
	echo "Considering $(PACKAGES)" ; \
	for i in $(PACKAGES) ; do \
		if [ -d $$i ] ; then \
			if ( cd $$i ; echo ; echo "    MODULE:"  $$i ; \
				${MAKE} $@ ; ) then \
				echo -n ; \
			else \
				exit 1 ; \
			fi ; \
		fi \
	done

##
## Compulsory rules
##

all:  home export $(DEPEND) build success

build:
	$(ECHO)
	$(ECHO) "       B U I L D"
	$(ECHO)
	$(RECURSE)

export:
	$(ECHO)
	$(ECHO) "       E X P O R T"
	$(ECHO)
	$(RECURSE)

install: all

new:  clean all tags

clean:
	$(ECHO)
	$(ECHO) "       C L E A N"
	$(ECHO)
	$(RECURSE)
	$(RM) *~

distclean:
	$(ECHO)
	$(ECHO) "       D I S T     C L E A N"
	$(ECHO)
	$(RECURSE)
	$(RM) ../include ../lib* ../bin* ../etc ../log \
		$(NAME)-src.tgz $(NAME)-bin.tgz *~

dep $(DEPEND) depend: 
	$(ECHO)
	$(ECHO) "       D E P E N D"
	$(ECHO)
	$(RECURSE)


home:
	$(MKDIR) sys/$(LIB_DIR)
	chmod -R a+r sys/$(LIB_DIR)
	chmod a+rx sys/$(LIB_DIR)
	$(MKDIR) sys/$(BIN_DIR)
	chmod -R a+rx sys/$(BIN_DIR)
	$(MKDIR) sys/$(ETC_DIR)
	chmod -R a+r sys/$(ETC_DIR)
	chmod a+rx sys/$(ETC_DIR)
	$(MKDIR) sys/$(INC_DIR)
	chmod -R a+r sys/$(INC_DIR)
	chmod a+rx sys/$(INC_DIR)
	rm -rf sys/$(LOG_DIR)
	$(MKDIR) sys/$(LOG_DIR)
	chmod -R a+rw sys/$(LOG_DIR)
	chmod a+rwx sys/$(LOG_DIR)
	chmod -R a+r .
	chmod a+rx .

success:
	$(ECHO)
	$(ECHO) "Software successfully compiled."
