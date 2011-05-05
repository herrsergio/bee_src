
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



#include <unistd.h>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <netinet/in.h>  /* for byteorder */

#include <bUtils.h>

#include <rai.h>
#include <utils.h>    /* for openRaw and fopenEtcFile */
#include <lockNames.h>
#include "mspmodule.h"

#ifdef B14
#define CONFIG_B14
#undef  CONFIG_B14_PUSHER
#undef  CONFIG_B21
#endif

#ifdef B14_PUSHER
#define CONFIG_B14
#define CONFIG_B14_PUSHER
#undef  CONFIG_B21
#endif

#ifdef B21
#define CONFIG_B21
#undef  CONFIG_B14
#undef  CONFIG_B14_PUSHER
#endif

#define  ENCL_MSP_NUM      (0x20)

#define   SONARMASTER                    (0) /* index into sonar_msps[] */
#define   MSP_INIT_TIMEOUT               (5000) /* 5 sec */


typedef struct mspModuleOpsType {

  /* These are callbacks */
  tactileCallbackType tactileRep;
  irCallbackType irRep;
  sonarCallbackType sonarRep;
} mspModuleOpsType;


unsigned long ping1[] = {
  0x2001, 0x2011, 0x2101, 0x2111, 0x2201, 0x2211, 0x0000};
unsigned long ping2[] = {
  0x2002, 0x2012, 0x2102, 0x2112, 0x2202, 0x2212, 0x0000};
unsigned long ping3[] = {
  0x2003, 0x2013, 0x2103, 0x2113, 0x2203, 0x2213, 0x0000};
unsigned long ping4[] = {
  0x2004, 0x2014, 0x2104, 0x2114, 0x2204, 0x2214, 0x0000};

static const unsigned long *globalSonTable[] = {
  ping1, ping2, ping3, ping4, NULL}; /* NULL to end table */

/* sonaddr -> sonar number */

#ifdef CONFIG_B14 /* B14 (B12-486) 16 tranducer setup */
static unsigned short sonAddr2sonNum_LUT[] = {
  0x2001,0x2002,0x2003,0x2004,	/* msp 0 chain 0 */
  0x2011,0x2012,0x2013,0x2014,  /* msp 0 chain 1 */
  0x2101,0x2102,0x2103,0x2104,  /* msp 1 chain 0 */
  0x2111,0x2112,0x2113,0x2114,	/* msp 1 chain 1 */
  0x2201,0x2202,0x2203,0x2204,	/* msp 2 chain 0 */
  0x2211,0x2212,0x2213,0x2214,	/* msp 2 chain 1 */
  0x0000
};
#else /* B21 arrangment with 24 transducers and 3 MSPs */
static unsigned short sonAddr2sonNum_LUT[] = {
  0x2011,0x2012,0x2013,0x2014,  /* msp 0 chain 1 */
  0x2101,0x2102,0x2103,0x2104,  /* msp 1 chain 0 */
  0x2111,0x2112,0x2113,0x2114,	/* msp 1 chain 1 */
  0x2201,0x2202,0x2203,0x2204,	/* msp 2 chain 0 */
  0x2211,0x2212,0x2213,0x2214,	/* msp 2 chain 1 */
  0x2001,0x2002,0x2003,0x2004,	/* msp 0 chain 0 */
  0x0000
};
#endif

#ifdef CONFIG_B14

static long mspAddr2mspNum_LUT[] = {
  0x20, 0x21, 0x80, 0x00};

#else

static long mspAddr2mspNum_LUT[] = {
  0x20, 0x21, 0x22, 0x30, 0x31, 0x32, 0x33, 0x00};

#endif

static irNum2irRow_LUT[] = {
  0, 0, 0, 0, 0, 0, 0, 0,	/* msp 0, enclusure */
  0, 0, 0, 0, 0, 0, 0, 0,	/* msp 1, enclusure */
  0, 0, 0, 0, 0, 0, 0, 0,	/* msp 2, enclusure */
  1, 1, 2, 1, 1, 1, 2, 1,	/* msp 3, base */
  1, 1, 2, 1, 1, 1, 2, 1,	/* msp 4, base */
  1, 1, 2, 1, 1, 1, 2, 1,	/* msp 5, base */
  1, 1, 2, 1, 1, 1, 2, 1	/* msp 6, base */
};

#ifdef CONFIG_B14 /* B14 (B12-486) 16 tranducer setup */
static irNum2irCol_LUT[] = {
   0,  1,  2,  3,  4,  5,  6,  7, /* msp 0, enclosure: row 0 */
   8,  9, 10, 11, 12, 13, 14, 15, /* msp 1, enclosure: row 0 */
  16, 16, 16, 16, 16, 16, 16, 16, /* msp 2, enclosure: row 0 */
  16, 16, 16, 16, 16, 16, 16, 16, /* msp 3, row 1, 1, 2, 1, 1, 1, 2, 1 */
  16, 16, 16, 16, 16, 16, 16, 16, /* map 4, row 1, 1, 2, 1, 1, 1, 2, 1 */
  16, 16, 16, 16, 16, 16, 16, 16, /* map 5, row 1, 1, 2, 1, 1, 1, 2, 1 */
  16, 16, 16, 16, 16, 16, 16, 16, /* map 6, row 1, 1, 2, 1, 1, 1, 2, 1 */
};

#else /* B21 arrangment with 24 transducers and 3 MSPs */

static irNum2irCol_LUT[] = {
  20, 21, 22, 23,  0,  1,  2,  3, /* msp 0, enclosure: row 0 */
   4,  5,  6,  7,  8,  9, 10, 11, /* msp 1, enclosure: row 0 */
  12, 13, 14, 15, 16, 17, 18, 19, /* msp 2, enclosure: row 0 */
  23,  0,  0,  1,  2,  3,  1,  4, /* msp 3, row 1, 1, 2, 1, 1, 1, 2, 1 */
   5,  6,  2,  7,  8,  9,  3, 10, /* map 4, row 1, 1, 2, 1, 1, 1, 2, 1 */
  11, 12,  4, 13, 14, 15,  5, 16, /* map 5, row 1, 1, 2, 1, 1, 1, 2, 1 */
  17, 18,  6, 19, 20, 21,  7, 22  /* map 6, row 1, 1, 2, 1, 1, 1, 2, 1 */
};
#endif

#ifdef CONFIG_B14

#ifdef CONFIG_B14_PUSHER

static tactNum2tactRow_LUT[] = {
  3, 3, 3, 3, 3, 3, 0, 0,	/* msp 0, pusher */
  3, 3, 3, 3, 3, 3, 0, 0,	/* msp 1, pusher */
  2, 2, 2, 2, 2, 2, 0, 0	/* MCP on base */
};

static tactNum2tactCol_LUT[] = {
   0,  1,  2,  3,  4,  5,  0,  0, /* msp 0, pusher */
  11, 10,  9,  8,  7,  6,  0,  0, /* msp 1, pusher */
   0,  1,  2,  3,  4,  5,  0,  0  /* MCP on base */
};

#else /* CONFIG_B14_PUSHER */

static tactNum2tactRow_LUT[] = {
  0, 1, 0, 1, 0, 1, 0, 1,	/* msp 0, enclosure */
  0, 1, 0, 1, 0, 1, 0, 1,	/* msp 1, enclusure */
  2, 2, 2, 2, 2, 2, 2, 2	/* MCP on base */
};

static tactNum2tactCol_LUT[] = {
   0,  0,  1,  1,  2,  2,  3,  3, /* msp 0, enclosure */
   4,  4,  5,  5,  6,  6,  7,  7, /* msp 1, enclosure */
   0,  1,  2,  3,  4,  5,  6,  6  /* MCP on base */
};
#endif /* CONFIG_B14_PUSHER */

#else /* CONFIG_B14 */

static tactNum2tactRow_LUT[] = {
  0, 1, 0, 1, 0, 1, 0, 1,	/* msp 0, enclusure */
  0, 1, 0, 1, 0, 1, 0, 1,	/* msp 1, enclusure */
  0, 1, 0, 1, 0, 1, 0, 1,	/* msp 2, enclusure */
  2, 3, 2, 3, 2, 3, 2, 3,	/* msp 3, base */
  2, 3, 2, 3, 2, 3, 2, 3,	/* msp 4, base */
  2, 3, 2, 3, 2, 3, 2, 3,	/* msp 5, base */
  2, 3, 2, 3, 2, 3, 2, 3	/* msp 6, base */
};

static tactNum2tactCol_LUT[] = {
  10, 10, 11, 11,  0,  0,  1,  1, /* msp 0, enclosure */
   2,  2,  3,  3,  4,  4,  5,  5, /* msp 1, enclosure */
   6,  6,  7,  7,  8,  8,  9,  9, /* msp 2, enclosure */
  15, 15,  0,  0,  1,  1,  2,  2, /* msp 3, base */
   3,  3,  4,  4,  5,  5,  6,  6, /* map 4, base */
   7,  7,  8,  8,  9,  9, 10, 10, /* map 5, base */
  11, 11, 12, 12, 13, 13, 14, 14  /* map 6, base */
};
#endif /* CONFIG_B14 */

#define    sonaddr_msp(son_addr)     ((son_addr)&0xFF00)
#define    sonaddr_chain(son_addr)   ((son_addr)&0x00F0)
#define    sonaddr_sonar(son_addr)   ((son_addr)&0x000F)

static int msp[NUM_MSPS];
static mspModuleOpsType moduleOps;
static mspOpsType mspOps;

static int        mspModuleFd            = -1;
static RaiModule  *mspModule             = NULL;
static int        mspModuleInitialized   = 0;
static int        sonarsPinging          = 0;
static int        sonarsRequested        = 0;

/* stuff for normalizing the IR data */

int normalizeIr = 1;
/* This is kinda ugly - we should really allocate these dynamically */
static int irMin[B_MAX_SENSOR_ROWS][B_MAX_SENSOR_COLS];
static int irMax[B_MAX_SENSOR_ROWS][B_MAX_SENSOR_COLS];

static char bmpMask[B_MAX_SENSOR_ROWS][B_MAX_SENSOR_COLS];

/************************************************************************/
/*  
     Global structures required by the sensor APIs we are supporting 
*/
/************************************************************************/

sonarType sonars[B_MAX_SENSOR_COLS];

irType irs[B_MAX_SENSOR_ROWS][B_MAX_SENSOR_COLS];

/* for passing irs to user callback*/
irType *irPtr[B_MAX_SENSOR_ROWS];



tactileType tactiles[B_MAX_SENSOR_ROWS][B_MAX_SENSOR_COLS];


/* for passing irs to user callback*/
tactileType *tactilePtr[B_MAX_SENSOR_ROWS];


/************************
 * utility functions
 ************************/

static int
sonAddr2sonNum (unsigned short sonAddr) {
  int ii;

  ii=0;
  while (sonAddr2sonNum_LUT[ii]) {
    if (sonAddr2sonNum_LUT[ii] == sonAddr) return(ii);
    ii++;
  }
  
  fprintf (stderr,
	   "sonaddr->sonar: invalid sonAddr(0x%04X)\n", sonAddr);
  return (-1);
}

static int
mspAddr2mspNum (unsigned short mspAddr) {
  int ii;

  ii=0;
  while (mspAddr2mspNum_LUT[ii]) {
    if (mspAddr2mspNum_LUT[ii] == mspAddr) return(ii);
    ii++;
  }
  
  fprintf (stderr,
	   "mspAddr->mspNum: invalid mspAddr(0x%04X)\n", mspAddr);
  return (-1);
}

static int
irNum2irRow (unsigned short mspAddr, int irNum) {
  int irIndex;
  
  irIndex = (mspAddr2mspNum(mspAddr) << 3) + irNum;
  return(irNum2irRow_LUT[irIndex]);
}

static int
irNum2irCol (unsigned short mspAddr, int irNum) {
  int irIndex;
  
  irIndex = (mspAddr2mspNum(mspAddr) << 3) + irNum;
  return(irNum2irCol_LUT[irIndex]);
}

static int
tactNum2tactRow (unsigned short mspAddr, int tactNum) {
  int tactIndex;
  int row;

  tactIndex = (mspAddr2mspNum(mspAddr) << 3) + tactNum;
  row = tactNum2tactRow_LUT[tactIndex];
#if 0
  fprintf(stdout, "tactNum2tactRow(msp=%02X, tactNum=%d) = %d : index %02X\n",
	  mspAddr, tactNum, row, tactIndex);
#endif /* 0 */
  return(row);
}

static int
tactNum2tactCol (unsigned short mspAddr, int tactNum) {
  int tactIndex;
  int col;

  tactIndex = (mspAddr2mspNum(mspAddr) << 3) + tactNum;
  col = tactNum2tactCol_LUT[tactIndex];
#if 0
  fprintf(stdout, "tactNum2tactCol(msp=%02X, tactNum=%d) = %d : index %02X\n",
	  mspAddr, tactNum, col, tactIndex);
#endif /* 0 */
  return(col);
}

/************************
 * libmsp callbacks
 ************************/

static int
irReportHandler (long mspAddr, const unsigned long *ir) {
  int irRow, irCol, irNum;
  struct timeval time;
  int value;

  gettimeofday(&time, NULL);	/* get time first */


  /* make previous data old */
  for (irRow = 0; irRow< bRobot.ir_rows; irRow++) {
    for (irCol = 0; irCol < bRobot.ir_cols[irRow]; irCol++) {
      irs[irRow][irCol].mostRecent = FALSE;
    }
  }

  /* iterate through new data */

  for (irNum=0; irNum<8; irNum++) {
    irRow = irNum2irRow(mspAddr, irNum);
    irCol = irNum2irCol(mspAddr, irNum);
    irs[irRow][irCol].mostRecent = TRUE;
    memcpy(&irs[irRow][irCol].time, &time,sizeof(timeval));

    value= ir[irNum];
    if (normalizeIr) {
      if ((irMax[irRow][irCol]==0) ||
	  (irMax[irRow][irCol]-irMin[irRow][irCol]<0x60)) {
	value = 0;
      }
      else {
	value = (value-irMin[irRow][irCol])*99;
	value /= irMax[irRow][irCol]-irMin[irRow][irCol];
	if (value<0) value=0;
      }
    }
    irs[irRow][irCol].value = value;
  }

  /* call callback */
  if (moduleOps.irRep) return(moduleOps.irRep(irPtr));
  return(0);
}

int
tactileReportHandler (long mspAddr, const unsigned long switches) {
  int tactRow, tactCol, tactNum;
  struct timeval time;
  char value;
#if 0
  char buf[40];
  int ii;
   
  for (ii=0; ii<8; ii++) buf[ii] = ((switches>>(7-ii))&0x01)+'0';
  buf[ii] = 0;
  fprintf(stderr, "{%08X} BMP : %s\n", (unsigned)mspAddr, buf);
#endif /* 0 */
  gettimeofday(&time, NULL);	/* get time first */

  /* iterate through new data */

  for (tactNum=0; tactNum<8; tactNum++) {
    value = (char)((switches>>tactNum) & 0x1);
    tactRow = tactNum2tactRow(mspAddr, tactNum);
    tactCol = tactNum2tactCol(mspAddr, tactNum);
    memcpy(&tactiles[tactRow][tactCol].time, &time,
	   sizeof(tactiles[tactRow][tactCol].time));
    if (!bmpMask[tactRow][tactCol]) value = 0;
    tactiles[tactRow][tactCol].value = value;
  }

#if 0
  for (row = 0; row < bRobot.tactile_rows; row++) {
    fprintf(stderr, "%s(A) - ", __FUNCTION__);
    for (num = 0; num < bRobot.tactile_cols[row]; num++) {
      fprintf(stderr, "%d", tactilePtr[tactRow][tactCol].value);
    }
    fprintf(stderr, "\n");
  }
  fprintf(stderr, "\n");
#endif

  /* call callback */
  if (moduleOps.tactileRep) return(moduleOps.tactileRep(tactilePtr));

  return(0);
}

static int
sonReportHandler (long mspAddr, const unsigned long *table[]) {
  int sonarNum, sonar;
  unsigned long value;
  struct timeval time;

  gettimeofday(&time, NULL);	/* get time first */

  /* make previous data old */
  for (sonar = 0; sonar < B_MAX_SENSOR_COLS; sonar++) {
    sonars[sonar].mostRecent = FALSE;
  }
  
  /* iterate through new data */
  sonar = 0;
  while (table[sonar]) {
    sonarNum = sonAddr2sonNum(table[sonar][0]);
    if (sonar<bRobot.sonar_cols[0] && sonar>=0) {
      value = table[sonar][1]; /* skip multiple echos for now */
      sonars[sonarNum].mostRecent = TRUE;
      memcpy(&sonars[sonarNum].time, &time,
	     sizeof(sonars[sonarNum].time));

      if (value < NO_RETURN) {
        /* This assumes that we're still reporting sonar values in mm and */
        /* that base_radius is in cm.  Otherwise, we will have to fudge   */
        /* things differently.                                            */
        value = (value * MM_PER_CLICK) +
	  (bRobot.enclosure_radius * 10.0) - 30.0;
      }

      if (value > NO_RETURN) value = NO_RETURN - 1;
      sonars[sonarNum].value = value;
    }
    sonar++;
  }

  /* call callback */
  if (moduleOps.sonarRep) return(moduleOps.sonarRep(sonars));
  return(0);
}

static int
mspConnectHandler (long mspAddr, int connectFlag) {
  int mspNum;
  mspIrParmsType irParms;

  memset(&irParms, 0xFF, sizeof(irParms));

  irParms.interval = 75;

  #ifdef VERBOSE
  if (connectFlag)
    fprintf(stderr, "MSP #%02X connected\n", (unsigned)mspAddr);
  else
    fprintf(stderr, "MSP #%02X disconnected\n", (unsigned)mspAddr);
  #endif
    
  mspNum = mspAddr2mspNum(mspAddr);
  if (mspNum<0) return (-1);
  msp[mspNum] = connectFlag;

  if (connectFlag) {

    mspSetIrParms(mspAddr, &irParms);

    if (sonarsPinging) {
      sonarStop();	/* stop-start to include new MSP */
      sonarStart();
    }
    else {
      if ((mspNum == 0) && sonarsRequested) {
	sonarStart();	/* master MSP up. start pinging */
      }
    }
  }
  else {
    if (mspNum == 0) {
      sonarsPinging = 0;	/* master MSP down.  Sonar has stopped */
    }
  }
  return (0);
}

/**********************************
 * module init and RAI callbacks
 **********************************/

static void
mspModuleSelect (RaiModule * mod) {
  mspLibSelect();
}

static void
mspModuleShutdown (RaiModule * mod) {

  if (sonarsPinging) {
    sonarStop();
  }
  
  /*
   * close our open file descriptor.
   * this is sick, cause we have to depend on the
   * internals of the RaiModule struct; yuck!
   */
  
  close(mod->fd);
}


void mspModuleInit() {
  int ii;
 /* int validMspIds[] = {0x20, 0x21, 0x22, 0x30, 0x31, 0x32, 0x33, 0x00}; */

  if (!mspModuleInitialized) {
    /*
     * What do we lock?  Sonar?  MSPs? No, we are not using all
     * MSPs and there may be other sonar type devices.  We could
     * lock baseMSP or something like that.  Lock MSPs for now.
     */
    
    if (makeLock(MSP_LOCK)<0) {
      fprintf(stderr,
	      "SonarInit:  Could not get lock for msp\n"
	      "SonarInit:  Some other process must have it locked\n");
      exit(-1);
    }
         
    moduleOps.tactileRep = NULL;
    moduleOps.irRep = NULL;
    moduleOps.sonarRep = NULL;

    /* Read in IR calibration data */

    {
      int val, num, row, numPerRow, maxMin;
      FILE *calFile;

      calFile = fopenEtcFile("irCal.dat");

      if (calFile!=NULL) {
	fprintf(stderr, "File opened.\n");
	do {
	  num=fscanf(calFile, "%x %x %x ", &row, &numPerRow, &maxMin);
	  if (num<1) break;
	  
	  for (ii=0; ii<numPerRow; ii++) {
	    num=fscanf(calFile, "%x", &val);
	    if (num<1) {
	      normalizeIr=0;
	      break;
	    }
	    if (maxMin==1) irMin[row][ii] = val;
	    else irMax[row][ii] = val;
	  }
	} while (num!=EOF);
      }
      else {
	normalizeIr = 0;
      }

      if (normalizeIr) {
	fprintf(stderr, "Normalizing IR data.\n");
#if 0
	for(row=0;row<NUM_IR_ROWS;row++) {
	  
	  fprintf(stderr, "%X %02X 1 ",row, irsInRow[row]);
	  for(num=0;num<irsInRow[row];num++) {
	    fprintf(stderr, "%02X ", irMin[row][num]);
	  }
	  fprintf(stderr, "\n");
	  
	  fprintf(stderr, "%X %02X 2 ",row, irsInRow[row]);
	  for(num=0;num<irsInRow[row];num++) {
	    fprintf(stderr, "%02X ", irMax[row][num]);
	  }
	  fprintf(stderr, "\n\n");
	  
	}
#endif /* 0 */
      }
      else {
	fprintf(stderr, "Not normalizing IR data.\n");
      }
    }

    /* XXX Read in tactile mask data */

    {
      int val, num, row, numPerRow;
      FILE *calFile;
      int maskBumps = 1;

      calFile = fopenEtcFile("bmpMask.dat");

      for (row = 0; row < bRobot.tactile_rows; row++) {
        for (num = 0; num < bRobot.tactile_cols[row]; num++) {
	  bmpMask[row][num] = 1;
	}
      }

      if (calFile!=NULL) {
	fprintf(stderr, "File opened.\n");
	do {
	  num=fscanf(calFile, "%x %x ", &row, &numPerRow);
	  if (num<1) break;
	  
	  for (ii=0; ii<numPerRow; ii++) {
	    num=fscanf(calFile, "%x", &val);
	    if (num<1) {
	      maskBumps=0;
	      break;
	    }
	    bmpMask[row][ii] = val;
	  }
	} while (num!=EOF);
      } else {
	maskBumps = 0;
      }

      if (!maskBumps) {
        for (row = 0; row < bRobot.tactile_rows; row++) {
          for (num = 0; num < bRobot.tactile_cols[row]; num++) {
	    bmpMask[row][num] = 1;
	  }
	}
      }

      for (row = 0; row < bRobot.tactile_rows; row++) {
        fprintf(stderr, "%X %02X ",row, bRobot.tactile_cols[row]);
        for (num = 0; num < bRobot.tactile_cols[row]; num++) {
	  fprintf(stderr, "%X ", bmpMask[row][num]);
	}
	fprintf(stderr, "\n");
      }
      fprintf(stderr, "\n");
    }

    RaiInit();   /* will do nothing if it is already initialized */
    mspModule = makeModule("base MPSs", mspModuleShutdown);
    
    /* initialize libmsp */

    memset(&mspOps, 0, sizeof(mspOps));
    mspOps.validMspId = NULL; /* XXX validMspIds; */
    mspOps.mspConnect = mspConnectHandler;
    mspOps.bmpRep     = tactileReportHandler;
    mspOps.irRep      = irReportHandler;
    mspOps.sonRep     = sonReportHandler;

    mspModuleFd = mspLibInit(ABUS_FILE, &mspOps);

    if (mspModuleFd < 0) {
      fprintf (stderr, "MspModuleInit: cannot open %s\n",
	       ABUS_FILE);
      exit (-1);
    }

    #ifdef VERBOSE
    fprintf (stderr, "MspModuleInit: opened %s on fd %d\n",
	     ABUS_FILE,
	     mspModuleFd);
    #endif

    addSelect(mspModule, mspModuleSelect, mspModuleFd);
    
    /* done w/ libmsp initialization */


    for (ii=0; ii<NUM_MSPS; ii++) {
      msp[ii] = 0;
    }

    mspModuleInitialized = TRUE;
    #ifdef VERBOSE
    fprintf (stderr, "MspModuleInit: initialized.\n");
    #endif
  }
  
#ifdef VERBOSE
  fprintf(RaiError,"Done with mspModuleInit\n");
#endif
  
}

/***************** PUBLIC API **********************************/

void sonarInit()
{
  if (!mspModuleInitialized) 
    mspModuleInit();
  sonarStart();
}

void irInit()
{
  int i;
  for(i = 0; i < bRobot.ir_rows; i++)
    irPtr[i] = irs[i];
  if (!mspModuleInitialized) 
    mspModuleInit();
}

void tactileInit()
{
  int i;

#if 0
  fprintf(stderr, "%s:%6d:%s()\n", __FILE__, __LINE__, __FUNCTION__);
#endif

  for(i = 0; i < bRobot.tactile_rows; i++) {
    tactilePtr[i] = tactiles[i];
  }

  if (!mspModuleInitialized) 
    mspModuleInit();
}

void registerSonarCallback(sonarCallbackType sonarCB)
{
  moduleOps.sonarRep = sonarCB;
}

void registerIrCallback(irCallbackType irCB)
{
  moduleOps.irRep = irCB;
}

void registerTactileCallback(tactileCallbackType tacCB)
{
  moduleOps.tactileRep = tacCB;
}




/*****************************
 * exported control functions
 *****************************/

void sonarStart(void) {
  int mspAddr;

  sonarsRequested = 1;

  #ifdef VERBOSE
  fprintf(stderr, "sonarStart()\n");
  #endif

  if (msp[0]) {
    mspAddr = mspAddr2mspNum_LUT[0];

  #ifdef VERBOSE
    fprintf(stderr, "Starting sonars using mspNum %02X\n",(unsigned)mspAddr);
  #endif

    mspSetSonTable(mspAddr, globalSonTable);
    mspReqSonStart(mspAddr);
    sonarsPinging = 1;
  }
  else {
    sonarsPinging = 0;
  }
}

void sonarStop(void) {

  if (msp[0]) {
    mspReqSonStop(mspAddr2mspNum_LUT[0]);
  }
  sonarsRequested = 0;
  sonarsPinging = 0;
}

void irStart(void) {};
void irStop(void) {};


void tactileStart(void) {};
void tactileStop(void) {};
