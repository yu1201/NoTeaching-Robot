/PROG ceshi20260418
/ATTR
OWNER   = MNEDITOR;
COMMENT    = "ceshi20260418 by VC OLP";
PROG_SIZE = 64000;
CREATE    = DATE 26-04-18 TIME 02:52:42;
MODIFIED  = DATE 26-04-18 TIME 02:52:42;
FILE_NAME = ceshi20260418;
VERSION   = 0;
LINE_COUNT = 4;
MEMORY_SIZE = 64000;
PROTECT   = READ_WRITE;
TCD: STACK_SIZE = 0,
     TASK_PRIORITY  = 50,
     TIME_SLICE = 0,
     BUSY_LAMP_OFF  = 0,
     ABORT_REQUEST  = 0,
     PAUSE_REQUEST  = 0;
DEFAULT_GROUP = 1,*,*,*,*;
CONTROL_CODE  = 00000000 00000000;
/APPL
  ARC : TRUE ;
  ARC Welding Equipment : 1,*,*,*,*;
/MN
   1:  UFRAME_NUM=0 ;
   2:  UTOOL_NUM=1 ;
   3:L P[1] 30mm/sec FINE    ;
   4:L P[2] 30mm/sec FINE    ;
/POS
P[1]{
   GP1:
	UF : 0, UT : 1,		CONFIG : 'N U T, 0, 0, 0',
	X = 1275.826 mm,	Y = 0.0 mm,	Z = 872.197 mm,
	W = 0.0 deg,	P = -60.0 deg,	R = 180.0 deg
};
P[2]{
   GP1:
	UF : 0, UT : 1,		CONFIG : 'N U T, 0, 0, 0',
	X = 1275.826 mm,	Y = 394.034 mm,	Z = 872.197 mm,
	W = -0.0 deg,	P = -60.0 deg,	R = -180.0 deg
};
/END
