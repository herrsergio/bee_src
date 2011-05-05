#
# $Id: console.tcl,v 1.5 1998/02/09 22:59:30 swa Exp $
#
# (c) 1997 Stefan Waldherr <swa@cs.cmu.edu>
#

#
# change the font globally
#
option add *Font {-Adobe-Helvetica-Bold-R-Normal-*-*-120-*-*-*-*-*-*}

#option add *Font {-adobe-courier-*-r-*-*-12-*-*-*-*-*-*-*}

#the program name
# set programName { 
#     "out"
#     "err"
# }


# set howStart1 {
#     "/bin/sh"
#     "/bin/sh"
# }


# set howStart2 {
#     "-c"
#     "-c"
# }

# set howStart3 { 
#     "/home/swa/bee/bin/putout 2>&1"
#     "/home/swa/bee/bin/puterr 2>&1"
# }

# #
# set howKill1 {
#     "/bin/sh"
#     "/bin/sh"
# }

# #
# set howKill2 {
#     "-c"
#     "-c"
# }

# #
# set howKill3 { 
#     "killall putout 2>&1"
#     "killall puterr 2>&1"
# }

# #where to run the program
# set programOnHost {
#     ""
#     ""
# }

# tix addbitmapdir /home/swa/bee/bin

set img0 [tix getimage run]
set img1 [tix getimage stop]
# set img2 [tix getimage wspace-small]
# set img3 [tix getimage running-small]

##################################################################
#
# the routine that sets up the window etc.
#
##################################################################
proc setup { w } {
    
    global programName img0 img1 scrollauto
#     global img2 img3

    frame $w.mbar -relief raised -bd 2
    pack $w.mbar -side top -fill x
    
    menubutton $w.mbar.file -text "File" -underline 0 -menu $w.mbar.file.menu
    menubutton $w.mbar.options -text "Options" -underline 0 -menu $w.mbar.options.menu
    menubutton $w.mbar.help -text "Help" -underline 0 -menu $w.mbar.help.menu

    pack $w.mbar.file -side left
    pack $w.mbar.options -side left
    pack $w.mbar.help -side right

    set m [ menu $w.mbar.file.menu -tearoff 0 ]
    $m add command -label "Quit" -command "myQuit $w" 

    set m [ menu $w.mbar.options.menu -tearoff 0 ]
    $m add command -label "Display parsed data ..." -command "displayParsedData"
    $m add checkbutton -label "Scroll automatically" -under 0 \
	-variable scrollauto -onvalue 1 -offvalue 0

    set m [ menu $w.mbar.help.menu -tearoff 0 ]
    $m add command -label "About ..." -command "aboutDlg"

    # frame that holds the first row
    frame $w.fr1
    $w.fr1 config -bg gray

    # frame that holds a second row
#     frame $w.fr2
#     $w.fr2 config -bg gray

    # frame that holds the output window(s)
    frame $w.fr3
    $w.fr3 config -bg gray
#    pack $w.fr1 $w.fr2 $w.fr3 -side top
    pack $w.fr1 $w.fr3 -side top
    
    # the start row
    set i -1
    foreach b $programName {
	incr i
 	button $w.fr1.$b -command "doIt $w $i 1"
	set startImg [image create compound -window $w.fr1.$b]
 	$startImg add line
 	$startImg add text -text "$b" -underline 0
 	$startImg add space -width 7
 	$startImg add image -image $img0
 	$w.fr1.$b config -image $startImg
 	pack $w.fr1.$b -side left -ipady 2m -pady 2m
    }

    # the stop row
#     set i -1
#     foreach b $programName {
# 	incr i
#  	button $w.fr2.$b -command "doIt $w $i 0"
# 	set stopImg [image create compound -window $w.fr2.$b]
#  	$stopImg add line
#  	$stopImg add text -text "$b" -underline 0
#  	$stopImg add space -width 7
#  	$stopImg add image -image $img1
#  	$w.fr2.$b config -image $stopImg
#  	pack $w.fr2.$b -side left -ipady 2m -pady 2m
#     }

    # now put a ton of notebooks into frame 3
    # these notebooks contain the output of the program
    # in a scrollwindow
    
    tixNoteBook $w.fr3.nb -ipadx 6 -ipady 6
    $w config -bg gray
    $w.fr3.nb subwidget nbframe config -backpagecolor gray
    # create each notebook tab
    set i -1
    foreach b $programName {
	incr i
	set staleImg [image create compound -window [$w.fr3.nb subwidget nbframe] -pady 0]
	$staleImg add line
	$staleImg add text -text "$b" -underline 0
	$staleImg add space -width 7
# 	$staleImg add image -image $img2
	$w.fr3.nb add nb_$i -image $staleImg
	pack $w.fr3.nb -expand yes -fill both -padx 5 -pady 5 -side top
    }
    # and fill each notebook tab
    set i -1
    foreach b $programName {
	incr i
	set f [$w.fr3.nb subwidget nb_$i]
	tixScrolledText $f.text_$i -width 800 -options { 
	    entry.width 120
	    font {-adobe-courier-*-r-*-*-12-*-*-*-*-*-*-*}
	    label.font "Times"
	}
	pack $f.text_$i
    }

}

################################################################## 
#
# changes the start into stop button and updates the notebook tab
#
################################################################## 
proc makeStopButton { w programName i} {

    global img0 img1 
#    global img2 img3

    # change button
    set stopImg [image create compound -window $w.fr1.$programName]
    $stopImg add line
    $stopImg add text -text "$programName" -underline 0
    $stopImg add space -width 7
    $stopImg add image -image $img1
    $w.fr1.$programName config -image $stopImg
    $w.fr1.$programName config -command "doIt $w $i 0"

    # change tab
    set runningImg [image create compound -window [$w.fr3.nb subwidget nbframe] -pady 0]
    $runningImg add line
    $runningImg add text -text "$programName" -underline 0
    $runningImg add space -width 7
#     $runningImg add image -image $img3

    set tabb [$w.fr3.nb subwidget nbframe]
    $tabb pageconfigure nb_$i -image $runningImg

}

################################################################## 
#
# changes the stop into start button and updates the notebook tab
#
################################################################## 
proc makeStartButton { w programName i } {

    global img0 img1 programRunning
#     global img2 img3 

    # change button
    set startImg [image create compound -window $w.fr1.$programName]
    $startImg add line
    $startImg add text -text "$programName" -underline 0
    $startImg add space -width 7
    $startImg add image -image $img0
    $w.fr1.$programName config -image $startImg
    $w.fr1.$programName config -command "doIt $w $i 1"
    
    # change tab
    set staleImg [image create compound -window [$w.fr3.nb subwidget nbframe] -pady 0]
    $staleImg add line
    $staleImg add text -text "$programName" -underline 0
    $staleImg add space -width 7
#     $staleImg add image -image $img2

    set tabb [$w.fr3.nb subwidget nbframe]
    $tabb pageconfigure nb_$i -image $staleImg

    set programRunning($i) 0

}

################################################################## 
#
#
################################################################## 
proc TraceEnd { name1 name2 op } {

    global mainwindow programName programRunning

    set programNameNum [ lindex $programName $name2 ]

    set programRunning($name2) 0

    # set the button text to start
    makeStartButton $mainwindow $programNameNum $name2

}

################################################################## 
# Name1 and name2 give the name(s) for the variable being accessed: if the
# variable is a scalar then name1 gives the variable's name and name2 is an
# empty string; if the variable is an array element then name1 gives the name
# of the array and name2 gives the index into the array; if an entire array is
# being deleted and the trace was registered on the overall array, rather than
# a single element, then name1 gives the array name and name2 is an empty
# string. Name1 and name2 are not necessarily the same as the name used in the
# trace variable command: the upvar command allows a procedure to reference a
# variable under a different name. Op indicates what operation is being
# performed on the variable, and is one of r, w, or u as defined above.
##################################################################
proc ShowOutput { name1 name2 op } {

    global myInfo mainwindow myStatus scrollauto

    # get the text window of the notebook-tab according to $num
    set f [$mainwindow.fr3.nb subwidget nb_$name2]
    set log [ $f.text_$name2 subwidget text ]
    
    $log insert end "$myInfo($name2)\n"

    if { $scrollauto > 0 } {
	$log yview scroll 1 units
    }

    # and delete myInfo($name2)
    set myInfo($name2) {}

}

##################################################################
#
# starts/stops a specific program.
#
##################################################################
proc doIt { w num start } {

    global programName programOnHost programID
    global howStart1 howStart2 howStart3 howKill3 howKill1 howKill2 
    global myInfo pid myStatus programRunning dep
    global pathToMyPrograms
    global scrollauto

    set programNameNum [ lindex $programName $num ]
    set programOnHostNum [ lindex $programOnHost $num ]
    set howStart1Num [ lindex $howStart1 $num ]
    set howStart2Num [ lindex $howStart2 $num ]
    set howStart3Num [ lindex $howStart3 $num ]
    set howKill1Num [ lindex $howKill1 $num ]
    set howKill2Num [ lindex $howKill2 $num ]
    set howKill3Num [ lindex $howKill3 $num ]

    # get the text window of the notebook-tab according to $num
    set f [$w.fr3.nb subwidget nb_$num]
    set log [ $f.text_$num subwidget text ]
    
    # take care of dependencies
    if { $dep($num) == {} } {
    }  else {
	foreach depp $dep($num) {
	    set index [lsearch -exact $programID $depp ]
	    if { $index != -1 && $programRunning($index) == 0 } {
		doIt $w $index 1
	    }
	}
    }

    set realstart "/bin/sh -c 'PATH=$pathToMyPrograms $howStart3Num 2>&1'"

    if { $start == 1 } {

	$log insert end "----------------------------------------------------------\n"
	$log insert end "Starting $programNameNum with: $howStart1Num $howStart2Num $programOnHostNum $realstart.\n"

	if { $scrollauto > 0 } {
	    $log yview scroll 2 units
	}

	set pid($num) [bgexec myStatus($num) \
			   -update myInfo($num) \
			   "$howStart1Num" "$howStart2Num" \
			   "$programOnHostNum" \
			   "$realstart" &]

# the following stuff works for fflushed stdout programs
# 	set pid($num) [bgexec myStatus($num) -update myInfo($num) \
# 			   "/usr/bin/rsh" "-n" "noonan" \
# 			   "/home/swa/bee/bin/putout" &]

# and this works for stderr programs
# 	set pid($num) [bgexec myStatus($num) \
# 			   -update myInfo($num) \
# 			   "/usr/bin/ssh" "-n" "noonan" \
# 			   "/bin/sh -c '/home/swa/bee/bin/puterr 2>&1'" &]

	set programRunning($num) 1

	# set the button text to stop
	makeStopButton $w $programNameNum $num
	
    } else {
	
	$log insert end "----------------------------------------------------------\n"
	$log insert end "Killing $programNameNum with: $howKill1Num $howKill2Num $programOnHostNum $howKill3Num.\n"

	if { $scrollauto > 0 } {
	    $log yview scroll 2 units
	}
	bgexec myStatus($num) -update myInfo($num) \
	    "$howKill1Num" "$howKill2Num" "$programOnHostNum" "$howKill3Num" &

    }


}

##################################################################
#
# quits the program. cleans up.
#
##################################################################
proc myQuit { w } {

    global programName programRunning

    # count all running processes
    set i -1
    set labeltext ""
    foreach b $programName {
	incr i
	if { $programRunning($i) == 1 } {
	    append labeltext [ lindex $programName $i ]
	    append labeltext " "
	}
    }
    
    if { [string compare $labeltext ""] == 0 } {
	destroy $w
    } else {
	set labeltext "Ok, There are one or more processes (namely $labeltext) left. Kill and quit?"
	# there are still processes left, do we really want to quit?
	set res [ dialog .d {Warning} $labeltext {question} -1 Ok Cancel ]
	if { $res == 0 } {
	    # and kill all running processes
	    set i -1
	    foreach b $programName {
		incr i
		if { $programRunning($i) == 1 } {
		    doIt $w $i 0
		}
	    }
	    destroy $w
	}
    }
   
}

##################################################################
#
# displays the parsed dependencies in a text window
#
##################################################################
proc displayParsedData { } {

    global programName programOnHost programID
    global howStart1 howStart2 howStart3 howKill3 howKill1 howKill2 
    global dep
    global pathToMyPrograms

    wm withdraw .
    set w .parsed_data
    toplevel $w
    wm title .parsed_data "These are the programs' parameters and dependencies"

    # We create the frame and the ScrolledText widget
    # at the top of the dialog box
    #
    frame $w.top -relief raised -bd 1

    # Create a Scrolled Text widget.
    #
    tixScrolledText $w.top.a -options { 
	entry.width 80
	font {-adobe-courier-*-r-*-*-12-*-*-*-*-*-*-*}
    }
    pack $w.top.a -expand yes -fill both -padx 10 -pady 10 -side left

    # Use a ButtonBox to hold the buttons.
    #
    tixButtonBox $w.box -orientation horizontal
    $w.box add ok -text Ok -underline 0 -command "destroy $w" -width 6

    pack $w.box -side bottom -fill x
    pack $w.top -side top -fill both -expand yes

    # put pathToMyPrograms into the 
    $w.top.a subwidget text insert end "\n"
    $w.top.a subwidget text insert end "\n"
    $w.top.a subwidget text insert end "Path to the programs is: $pathToMyPrograms\n"
    $w.top.a subwidget text insert end "\n"
    $w.top.a subwidget text insert end "\n"

    # Put the dependencies inside the text subwidget of the ScrolledText widget
    #
    set i -1
    foreach p $programName {
	incr i
	set programIDNum [ lindex $programID $i ]
	set programNameNum [ lindex $programName $i ]
	set programOnHostNum [ lindex $programOnHost $i ]
	set howStart1Num [ lindex $howStart1 $i ]
	set howStart2Num [ lindex $howStart2 $i ]
	set howStart3Num [ lindex $howStart3 $i ]
	set howKill1Num [ lindex $howKill1 $i ]
	set howKill2Num [ lindex $howKill2 $i ]
	set howKill3Num [ lindex $howKill3 $i ]
	$w.top.a subwidget text insert end "$i.programID....... $programIDNum\n"
	$w.top.a subwidget text insert end "$i.programName .... $programNameNum\n"
	$w.top.a subwidget text insert end "$i.howStart1....... $howStart1Num\n"
	$w.top.a subwidget text insert end "$i.howStart2....... $howStart2Num\n"
	$w.top.a subwidget text insert end "$i.howStart3....... $howStart3Num\n"
	$w.top.a subwidget text insert end "$i.howKill1........ $howKill1Num\n"
	$w.top.a subwidget text insert end "$i.howKill2........ $howKill2Num\n"
	$w.top.a subwidget text insert end "$i.howKill3........ $howKill3Num\n"
	$w.top.a subwidget text insert end "$i.programOnHost... $programOnHostNum\n"
	if { $dep($i) == {} } {
	}  else {
	    $w.top.a subwidget text insert end "$i.dep............. "
	    foreach depp $dep($i) {
		set index [lsearch -exact $programID $depp ]
		set deppp [ lindex $programID $index ]
		$w.top.a subwidget text insert end "$deppp "
	    }
	    $w.top.a subwidget text insert end "\n"
	}
	$w.top.a subwidget text insert end "\n"
    }

}

##################################################################
#
# shows the about dialog
#
##################################################################
proc aboutDlg { } {

    wm withdraw .
    set w .about
    toplevel $w
    wm title .about About

    set stefanimg [tix getimage stefan]
    set buttonimg [tix getimage rwi]

    #
    frame $w.top -relief raised -bd 1
    frame $w.bottom -relief raised -bd 1
    frame $w.top.left -relief raised -bd 1
    frame $w.top.right -relief raised -bd 1
    
    #
    label $w.top.right.version -text "Console\n\$Revision: 1.5 $"
    pack $w.top.right.version -side top -padx 5 -pady 19

    #
    label $w.top.right.about -text "(c) Stefan Waldherr <swa@cs.cmu.edu>"
    pack $w.top.right.about -side top -padx 5 -pady 8

    #
    button $w.top.left.image -padx 4 -pady 1 -command "destroy $w"
    set stefan_img [image create compound -window $w.top.left.image]
    $stefan_img add image -image $stefanimg
    $w.top.left.image config -image $stefan_img
    pack $w.top.left.image

    #
    button $w.bottom.ok -padx 4 -pady 1 -command "destroy $w"
    set ok_img [image create compound -window $w.bottom.ok]
    $ok_img add image -image $buttonimg
    $w.bottom.ok config -image $ok_img
    pack $w.bottom.ok

    # and put them into place
    pack $w.top.left -side left -fill both -expand yes
    pack $w.bottom -side bottom -fill x
    pack $w.top -side top -fill both -expand yes
    pack $w.top.right -side left -fill both -expand yes
    

}

###################################################################
#
# first function to be executed upon start of the program ...
#
###################################################################
if { ![info exists tix_demo_running] } {

    global mainwindow myInfo myStatus

    wm withdraw .

    set mainwindow .console

    toplevel $mainwindow

    setup $mainwindow

    wm title .console "Console"

    # trace the stati and info information
    set i -1
    foreach b $programName {
	incr i
	trace variable myInfo($i) w ShowOutput
	trace variable myStatus($i) w TraceEnd
	set programRunning($i) 0
    }

    bind $mainwindow <Destroy> {if {"%W" == ".console"} exit}

}

###################################################################
#
# tk_dialog:
#
# This procedure displays a dialog box, waits for a button in the dialog
# to be invoked, then returns the index of the selected button.
#
# Arguments:
# w -           Window to use for dialog top-level.
# title -       Title to display in dialog's decorative frame.
# text -        Message to display in dialog.
# bitmap -      Bitmap to display in dialog (empty string means none).
# default -     Index of button that is to display the default ring
#               (-1 means none).
# args -        One or more strings to display in buttons across the
#               bottom of the dialog box.
###################################################################

proc dialog {w title text bitmap default args} {
    global tkPriv

    # 1. Create the top-level window and divide it into top
    # and bottom parts.

    catch {destroy $w}
    toplevel $w -class Dialog
    wm title $w $title
    wm iconname $w Dialog
    wm protocol $w WM_DELETE_WINDOW { }
    wm transient $w [winfo toplevel [winfo parent $w]]
    frame $w.top -relief raised -bd 1
    pack $w.top -side top -fill both
    frame $w.bot -relief raised -bd 1
    pack $w.bot -side bottom -fill both

    # 2. Fill the top part with bitmap and message.

    label $w.msg -wraplength 3i -justify left -text $text 
    pack $w.msg -in $w.top -side right -expand 1 -fill both -padx 3m -pady 3m
    if {$bitmap != ""} {
        label $w.bitmap -bitmap $bitmap
        pack $w.bitmap -in $w.top -side left -padx 3m -pady 3m
    }

    # 3. Create a row of buttons at the bottom of the dialog.

    set i 0
    foreach but $args {
        button $w.button$i -text $but -command "set tkPriv(button) $i"
        if {$i == $default} {
            frame $w.default -relief sunken -bd 1
            raise $w.button$i $w.default
            pack $w.default -in $w.bot -side left -expand 1 -padx 3m -pady 2m
            pack $w.button$i -in $w.default -padx 2m -pady 2m
            bind $w <Return> "$w.button$i flash; set tkPriv(button) $i"
        } else {
            pack $w.button$i -in $w.bot -side left -expand 1 \
                    -padx 3m -pady 2m
        }
        incr i
    }

    # 4. Withdraw the window, then update all the geometry information
    # so we know how big it wants to be, then center the window in the
    # display and de-iconify it.

    wm withdraw $w
    update idletasks
    set x [expr [winfo screenwidth $w]/2 - [winfo reqwidth $w]/2 \
            - [winfo vrootx [winfo parent $w]]]
    set y [expr [winfo screenheight $w]/2 - [winfo reqheight $w]/2 \
            - [winfo vrooty [winfo parent $w]]]
    wm geom $w +$x+$y
    wm deiconify $w

    # 5. Set a grab and claim the focus too.

    set oldFocus [focus]
    grab $w
    tkwait visibility $w
    if {$default >= 0} {
        focus $w.button$default
    } else {
        focus $w
    }

    # 6. Wait for the user to respond, then restore the focus and
    # return the index of the selected button.  Restore the focus
    # before deleting the window, since otherwise the window manager
    # may take the focus away so we can't redirect it.

    tkwait variable tkPriv(button)
    catch {focus $oldFocus}
    destroy $w
    return $tkPriv(button)
}

#
# $Log: console.tcl,v $
# Revision 1.5  1998/02/09 22:59:30  swa
# Added a parameter that controls if we automatically scroll to the end of
# each log window.
#
# Revision 1.4  1997/11/28 18:26:01  swa
# Added a variable to replace absolute paths. More user friendly. Use only
# ssh for simplicity and security. Update TODO with tyson's thoughts.
#
# Revision 1.3  1997/11/11 15:50:53  swa
# Ok, Sebastian bitched'n'moaned about the buttons and tabs semantics,
# so I changed the tabs to display no icon at all. We now have only
# `Start' buttons, that automatically change to a `Stop' button,
# whenever that particular program is running.
#
# Revision 1.2  1997/11/10 23:51:33  swa
# Fixed some problems with the menu.
#
# Revision 1.1.1.1  1997/11/10 23:10:36  swa
# Stefan's new console manager.
#
# Revision 1.19  1997/11/09 23:39:41  swa
# Changed the about dialog slightly.
#
# Revision 1.18  1997/11/09 23:19:51  swa
# Changed the dialog substantially. Now we ask politely if we want to
# kill remaining processes before we quit.
#
# Revision 1.17  1997/11/09 22:35:59  swa
# Forgot to list dependencies in the window.
#
# Revision 1.16  1997/11/09 21:45:13  swa
# Forgot to comment out the header.
#
# Revision 1.15  1997/11/09 21:41:25  swa
# Neat about dialog and some other enhancements.
#
# Revision 1.14  1997/11/09 20:37:19  swa
# Added displayParsedParams to an Options menu. Changed the fonts to
# be fixed.
#
# Revision 1.13  1997/11/09 18:34:45  swa
# Minor stuff.
#
# Revision 1.12  1997/11/09 18:25:11  swa
# All programs and their dependencies are now taken care of. A second,
# user-specific ini file was added, console2.ini. All dependencies in
# console.ini should eventually go into beeSoft.ini and console2.ini
# should be renamed console.ini.
#
# Revision 1.11  1997/11/08 23:36:23  swa
# All dependent processes get started. Also on remote machines. beeSoft
# current processes and their dependencies are coded in console.ini.
#
# Revision 1.10  1997/11/08 18:01:07  swa
# Programs which depend on other programs are now taken care of.
#
# Revision 1.9  1997/11/08 17:36:15  swa
# Hold the status (if or if not running) of a program in programRunning.
#
# Revision 1.8  1997/11/08 17:00:32  swa
# Retrieved code at the beginning of the file (for testing purposes) and
# changed the notbooks' tabs to display the status of a process.
#
# Revision 1.7  1997/11/08 04:40:04  swa
# Used a different icon for the `Start' button.
#
# Revision 1.6  1997/11/08 04:30:32  swa
# Now, we set the stop button iff on of the program stops (for some
# reason or the other).
#
# Revision 1.5  1997/11/08 04:14:58  swa
# Use a single button for both, start and stop. Next fix must be,
# that the button goes back to start iff a program exits and the
# user didn't click the button stop.
#
# Revision 1.4  1997/11/08 03:51:06  swa
# Most variables are no longer set in the .tcl script, instead they are
# read from the .ini file and evaluated by our Tcl interpreter.
#
# Revision 1.3  1997/11/08 01:14:31  swa
# renamed commander to console to avoid confusion with sebastian's
# cr*ppy commander ;-)
#
# Revision 1.2  1997/11/08 01:09:24  swa
# Start and Stop icons added.
#
# Revision 1.1.1.1  1997/11/07 22:31:36  swa
# imported
#
#
