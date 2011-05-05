
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



/* @(#)struct.h	1.13 11/12/92 */

/*
 * Structure for a single track.  This is pretty much self-explanatory --
 * one of these exists for each track on the current CD.
 */
struct trackinfo {
	char	*songname;	/* Name of song, dynamically allocated */
	char	*otherdb;	/* Unrecognized info for this track */
	char	*otherrc;
	int	length;		/* Length of track in seconds or Kbytes */
	int	start;		/* Starting position (f+s*75+m*60*75) */
	int	volume;		/* Per-track volume (1-32, 0 to disable) */
	int	track;		/* Physical track number */
	int	section;	/* Section number (0 if track not split) */
	char	contd;		/* Flag: continuation of previous track */
	char	avoid;		/* Flag: don't play this track. */
	char	data;		/* Flag: data track */
};

/*
 * Structure for internal playlist management.  The internal playlist is
 * simply the list of track ranges that are being played currently.  This
 * is built whenever the CD starts playing; it's used in normal and shuffle
 * modes as well as playlist mode.
 *
 * The "starttime" element represents how much time has elapsed by the time
 * we get to this entry.  For instance, if the list begins with a 5-minute
 * track and a 3-minute track, the third entry would have a starttime of 8
 * minutes.  This is used so that the elapsed play time can be displayed
 * even in shuffle or playlist modes.
 *
 * The last member of the list has a start track of 0, and its starttime is
 * the total playing time of the playlist (which will usually be overestimated,
 * since we don't play leadouts in some cases.)
 */
struct play {
	int	start;		/* Start track, or 0 if end of list */
	int	end;		/* last track plus 1 */
	int	starttime;	/* Number of seconds elapsed previously */
};

/*
 * Structure for playlists (as seen by the user.)  This is simply a name
 * followed by a zero-terminated list of track numbers to play.  The list
 * is terminated by a NULL name.
 */
struct playlist {
	char	*name;		/* Name of this playlist */
	int	*list;		/* List of tracks */
};

struct cdinfo {
	char	artist[84];	/* Artist's name */
	char	cdname[84];	/* Disc's name */
	int	ntracks;	/* Number of tracks on the disc */
	int	length;		/* Total running time in seconds */
	int	autoplay;	/* Start playing CD immediately */
	int	playmode;	/* How to play the CD */
	int	volume;		/* Default volume (1-32, 0 for none) */
	struct trackinfo *trk;	/* struct trackinfo[ntracks] */
	struct playlist *lists;	/* User-specified playlists */
	char	*whichdb;	/* Which database is this entry from? */
	char	*otherdb;	/* Unrecognized lines from this entry */
	char	*otherrc;
};

/* The global variable "cd" points to the struct for the CD that's playing. */
extern struct cdinfo *cd;

/* struct playlist *new_list(); */

#ifdef UNIBONN
#define NUM_MSGs 313
#else
#define NUM_MSGs 7
#endif

typedef struct {
  int start_track;
  int start_time;
  int start_frame;
  int end_track;
  int end_time;
  int end_frame;
  char *text;
} CD_msg_type;

extern CD_msg_type MSGs[];
