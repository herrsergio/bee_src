
/***************************************************************************
 ***************************************************************************
 ***************************************************************************
 *****
 ***** Welcome!
 *****
 ***** This file is part of the BeeSoft robot control software.
 ***** The version number is:
 *****
 *****                  v1.3.8 (released Aug 5, 1998)
 *****                  this is not an official BeeSoft version
 *****
 ***** Please refer to bee/src/COPYRIGHT for copyright and liability
 ***** information.
 *****
 ***************************************************************************
 ***************************************************************************
 ***************************************************************************/
!#########################################
!### 
!###      M A K E F I L E
!###      for VMS systems
!###
!###  On the AXP system which this port was
!###  done on, the /noopt CFLAG was required
!###  to keep the server from crashing as soon
!###  as a client connected. :(
!###
!### ---> Questions? contact tyson@wpi.edu
!###
!###
!
$ TCX_DIR = "DISK$APG:[TYSON.PROJ.TCX"
$ BIN_DIR = TCX_DIR + ".-.BIN]"
$ LIB_DIR = TCX_DIR + ".-.LIB]"
!
$ CFLAGS = "/standard=vaxc /noopt /float=IEEE"
$ LFLAGS = ""
$ INCLUDES = "/include=" + TCX_DIR + ".include]"
!
!$ define sys$output make.out
!
$ CC 'CFLAGS' 'INCLUDES' COM.C 
$ CC 'CFLAGS' 'INCLUDES' COMA.C
$ CC 'CFLAGS' 'INCLUDES' DATA.C
$ CC 'CFLAGS' 'INCLUDES' FORMATTERS.C
$ CC 'CFLAGS' 'INCLUDES' GLOBAL.C
$ CC 'CFLAGS' 'INCLUDES' HASH.C
$ CC 'CFLAGS' 'INCLUDES' LEX.C
$ CC 'CFLAGS' 'INCLUDES' LIST.C
$ CC 'CFLAGS' 'INCLUDES' MODULE.C
$ CC 'CFLAGS' 'INCLUDES' MSG.C
$ CC 'CFLAGS' 'INCLUDES' PARSEFMTTRS.C
$ CC 'CFLAGS' 'INCLUDES' PRIMFMTTRS.C
$ CC 'CFLAGS' 'INCLUDES' REG.C
$ CC 'CFLAGS' 'INCLUDES' TCAMATRIX.C
$ CC 'CFLAGS' 'INCLUDES' TCAMEM.C
$ CC 'CFLAGS' 'INCLUDES' TCXSERVER.C
$ CC 'CFLAGS' 'INCLUDES' A1.C
$ CC 'CFLAGS' 'INCLUDES' B1.C 
!
$ LIBRARY /CREATE libtcx com,comA,data,formatters,hash,lex,list,module,msg
$ LIBRARY /INSERT libtcx parseFmttrs,primFmttrs,reg,tcaMatrix,tcaMem,global
!
$ LINK 'LFLAGS' /exe=tcx.exe tcxserver,libtcx/lib
$ LINK 'LFLAGS' /exe=a1.exe a1,libtcx/lib
$ LINK 'LFLAGS' /exe=b1.exe b1,libtcx/lib
!
$ COPY TCX.EXE 'BIN_DIR'
$ COPY LIBTCX.OLB 'LIB_DIR'
