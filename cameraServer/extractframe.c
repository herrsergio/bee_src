
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
#include <stdio.h>
#include <stdlib.h>
#include <imagesize.h>

#ifdef i386
#include <getopt.h>
#endif

#include <string.h>

char dump[ROWS*COLS*4];

#ifdef i386
/* ---------------------------------------------------------
 *
 *
 * --------------------------------------------------------*/
int openAndReadFile( char *filename, int num ) {

  FILE *fp;

  /* open the file and read the frames */
  fp = fopen( filename, "rb" );
  if ( !fp ) {
    perror("\nError");
    return -1;
  }

  if ( fseek( fp, num*COLS*ROWS*4,SEEK_SET) != 0 ) {
    perror( "fseek() reports:");
    fclose( fp );
    return -1;
  }

  if ( fread(dump,1,ROWS*COLS*4,fp)!=ROWS*COLS*4 ) {
    fprintf( stderr, "\nError reading frame %d from file, eof?\n", num);
    fclose( fp );
    return -1;
  }

  fclose( fp );

  return 0;
}

/* ---------------------------------------------------------
 *
 * dumps the ppm image. note, that the source data is BGRX
 * and NOT RGBX.
 *
 * --------------------------------------------------------*/
int writePPMimage( char *filename ) {

  FILE *fp;
  char realdump[COLS*ROWS*3];
  int i,j;

  /* dump the current image into a .ppm file */
  fp = fopen( filename, "wb" );
  if ( !fp ) {
    perror("Error");
    return -1;
  }

  i=0; j=0;
  do {
    realdump[j+0] = dump[i+2];
    realdump[j+1] = dump[i+1];
    realdump[j+2] = dump[i+0];
    i+=4; j+=3;
  } while (i<ROWS*COLS*4);

  fprintf(fp,"P6\n%d %d\n255\n",COLS,ROWS);
  if ( ROWS*COLS*3 != fwrite( realdump,sizeof(char),ROWS*COLS*3,fp ) ) {
    perror("writePPMimage");
    exit(-1);
  }
	
  fclose( fp );

  return 0;
}
#endif

/* ---------------------------------------------------------
 *
 * -i for inputfile
 * -o for outputfile
 * -n for framenumber
 *
 * --------------------------------------------------------*/
int main( int argc, char** argv ) {

#ifdef i386
  int num=-1;
  char *ifilename=NULL;
  char *ofilename=NULL;

  while (1) {
    int c,optindex;
    
    c = getopt_long(argc,argv,"i:o:n:", NULL, &optindex);
    if (c == EOF) break;
    
    switch (c) {
    case 'i':
      ifilename=strdup(optarg);
      break;
    case 'o':
      ofilename=strdup(optarg);
      break;
    case 'n':
      num=atoi(optarg);
      break;
    default:
      exit(1);      
      break;
    }
  }

  if ( ifilename == NULL || ofilename == NULL || num < 0 ) {
    fprintf( stderr,"%s: Wrong parameters.\n", __FILE__ );
  }

/*   fprintf( stderr,"ifilename = %s\n", ifilename ); */
/*   fprintf( stderr,"ofilename = %s\n", ofilename ); */
/*   fprintf( stderr,"      num = %d\n", num ); */

  if ( openAndReadFile( ifilename, num ) != 0 ) {
    return -1;
  }

  if ( writePPMimage( ofilename ) != 0 ) {
    return -1;
  }
#else

  fprintf( stderr, "This program requires Linux.\n");

#endif

  return 0;

}
  

/*
 * $Log: extractframe.c,v $
 * Revision 1.3  1997/07/25 16:34:32  swa
 * extractframe now compiles under SUN as well.
 *
 * Revision 1.2  1997/07/25 02:41:42  swa
 * Added support for mpeg. Changed README accordingly.
 *
 * Revision 1.1  1997/07/24 21:21:00  swa
 * Added extractFrame, which extracts a particular frame from a raw imagefile,
 * and writes that frame to a ppm file.
 *
 *
 */
