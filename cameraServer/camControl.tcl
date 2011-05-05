#
# $Id: camControl.tcl,v 1.6 1997/11/10 23:54:42 swa Exp $
#
# (c) 1997 Stefan Waldherr <swa@cs.cmu.edu>
#
camControl mycamControl

#
# change the font globally
#
option add *Font {-Adobe-Helvetica-Bold-R-Normal-*-*-120-*-*-*-*-*-*}

set filename "/tmp/frames.raw"

set maxframenum 15

# the following options are now set by the c program,
# where we know, how many frame grabbers we have
# set source_options {0 1}
# set source_labels(0) "One" 
# set source_labels(1) "Two"

set source 0

##################################################################
#
# the routine that sets up the window etc.
#
##################################################################
proc setup { w } {

    global filename maxframenum source source_options source_labels

    frame $w.mbar -relief raised -bd 2
    pack $w.mbar -side top -fill x
    
    menubutton $w.mbar.file -text "File" -underline 0 -menu $w.mbar.file.menu
    menubutton $w.mbar.help -text "Help" -underline 0 -menu $w.mbar.help.menu

    pack $w.mbar.file -side left
    pack $w.mbar.help -side right

    set m [ menu $w.mbar.file.menu -tearoff 0 ]
    $m add command -label "Quit" -command "myQuit $w" 

    set m [ menu $w.mbar.help.menu -tearoff 0 ]
    $m add command -label "About ..." -command "aboutDlg"

    frame $w.fr1 -bd 1 -relief sunken
    frame $w.fr2 -bd 1 -relief sunken
    frame $w.fr3 -bd 1 -relief sunken

    pack $w.fr1 -side top -fill both
    pack $w.fr2 -side top -fill both
    pack $w.fr3 -side top -fill both

    label $w.fr1.filename -text "Filename: " -anchor e \
	-width 15 -padx 3 -pady 10
    entry $w.fr1.entry_filename -textvariable filename -width 30 -relief sunken
    grid $w.fr1.filename $w.fr1.entry_filename -sticky w

    label $w.fr1.hostname \
	-text "(this is local to the machine which runs\nthe cameraServer)" 
    grid $w.fr1.hostname -column 1 -sticky w -ipadx 5

    tixOptionMenu $w.fr1.source -label "Frame grabber: " \
	-variable source \
	-options {
	    label.width  15
	    label.anchor e
	    menubutton.width 5
	}
    foreach sou $source_options {
 	$w.fr1.source add command $sou -label $source_labels($sou)
    }

    grid $w.fr1.source -columnspan 2 -sticky w -pady 3

    button $w.fr2.startc -text "Start" -command "startc"
    label $w.fr2.saving_cont -text "saving continuously."
    grid $w.fr2.startc $w.fr2.saving_cont -pady 10

    button $w.fr2.stopc -text "Stop" -command "stopc"
    label $w.fr2.saving_conttoo -text "saving continuously."
    grid $w.fr2.stopc $w.fr2.saving_conttoo  -pady 10

    button $w.fr2.start -text "Start" -command "re_ad"
    label $w.fr2.reading_cont -text "reading continuously."
    grid $w.fr2.start $w.fr2.reading_cont  -pady 10
    
    button $w.fr3.startn -text "Start" -command "save"
    tixControl $w.fr3.framenum -label "saving" -integer true \
	-variable maxframenum -min 1 -max 999\
	-validatecmd "validateMe $w.fr3.startn" \
	-options {
	    entry.width 4
	    label.width 6
	    label.anchor w
	}

    label $w.fr3.frames -text "frames."
    grid $w.fr3.startn $w.fr3.framenum $w.fr3.frames -pady 15

}

##################################################################
#
# validates an integer input between 1 and 999
#
##################################################################
proc validateMe {w value} {

    if { $value > 0 && $value <= 999 } {
	return $value
    } else {
	return 0
    }
}

##################################################################
#
# handles continuous saving - start
#
##################################################################
proc startc { } {

    global filename source

    if { $source==0 || $source==1 } {
    } else {
	dialog .d {Error} \
	{Please specify 0 for the first or 1 for the second frame grabber.} \
	warning -1 OK
	return 
    }

    set error [ catch { mycamControl startc $filename $source } ]
    
    if { $error != 0 } {
	dialog .d {Error} {Error saving.} warning -1 OK
    } else {
    }

}

##################################################################
#
# handles continuous saving - stop
#
##################################################################
proc stopc { } {

    global filename source

    if { $source==0 || $source==1 } {
    } else {
	dialog .d {Error} \
	{Please specify 0 for the first or 1 for the second frame grabber.} \
	warning -1 OK
	return 
    }

    set error [ catch { mycamControl stopc $filename $source } ]
    
    if { $error != 0 } {
	dialog .d {Error} {Error stopping.} warning -1 OK
    } else {
    }

}

##################################################################
#
# handles saving of num frames
#
##################################################################
proc save { } {

    # check if num is an integer here (so we don't have to do it in c

    global filename maxframenum source

    if { $source==0 || $source==1 } {
    } else {
	dialog .d {Error} \
	{Please specify 0 for the first or 1 for the second frame grabber.} \
	warning -1 OK
	return 
    }

    set error [ catch { mycamControl save $filename $maxframenum $source} ]
    
    if { $error != 0 } {
	dialog .d {Error} {Error saving.} warning -1 OK
    } else {
    }

}

##################################################################
#
# handles reading of frames
#
##################################################################
proc re_ad { } {

    global filename source

    if { $source==0 || $source==1 } {
    } else {
	dialog .d {Error} \
	{Please specify 0 for the first or 1 for the second frame grabber.} \
	warning -1 OK
	return 
    }

    set error [ catch { mycamControl re_ad $filename $source } ]
    
    if { $error != 0 } {
	dialog .d {Error} {Error reading.} warning -1 OK
    } else {
    }

}

##################################################################
#
# quits the program. cleans up.
#
##################################################################
proc myQuit { w } {

    destroy $w
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
    label $w.top.right.version -text "cameraControl\n\$Revision: 1.6 $"
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
    wm withdraw .
    set w .camControl
    toplevel $w
    setup $w
    wm title .camControl "cameraControl"
    bind $w <Destroy> {if {"%W" == ".camControl"} exit}
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
# $Log: camControl.tcl,v $
# Revision 1.6  1997/11/10 23:54:42  swa
# Fixed some problems with the menubar.
#
# Revision 1.5  1997/11/10 22:45:00  swa
# Some changes to the Tcl/Tk interface have been made (user now selects framegrabber
# and frames, he doesn't have to enter them). About dialog pepped up. Now, we
# require Tix to be installed for the Tcl/Tk program.
#
# Revision 1.4  1997/10/04 00:13:05  swa
# First working version of the cameraServer that supports two (count'em
# two!) framegrabbers.  Although the server seems to work just fine, it
# has not yet been fully tested.
#
# Revision 1.3  1997/08/05 17:41:09  swa
# It is no longer req. to be in the same directory of camControl.tcl when you
# start camControl.
#
# Revision 1.2  1997/07/24 00:54:24  swa
# Fixed some minor bugs, extended the README file and tested the version. The
# cameraAttachExample does not yet die when the cameraServer dies. Will be
# fixed in future versions.
#
# Revision 1.1  1997/07/23 22:43:32  swa
# This is the first version of camControl, a Tcl program that remotely controls
# the cameraServer. The user can save and read frames. recorder was renamed
# to camControl, since the program will not only record but also read frames.
#
#