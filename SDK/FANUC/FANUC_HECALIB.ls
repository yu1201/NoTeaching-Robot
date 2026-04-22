/PROG  FANUC_HECALIB
/ATTR
OWNER		= MNEDITOR;
COMMENT		= "Auto hand-eye calib";
PROG_SIZE	= 0;
CREATE		= DATE 26-04-22  TIME 11:30:00;
MODIFIED	= DATE 26-04-22  TIME 11:30:00;
FILE_NAME	= ;
VERSION		= 0;
LINE_COUNT	= 66;
MEMORY_SIZE	= 0;
PROTECT		= READ_WRITE;
TCD:  STACK_SIZE	= 0,
      TASK_PRIORITY	= 50,
      TIME_SLICE	= 0,
      BUSY_LAMP_OFF	= 0,
      ABORT_REQUEST	= 0,
      PAUSE_REQUEST	= 0;
DEFAULT_GROUP	= 1,*,*,*,*;
CONTROL_CODE	= 00000000 00000000;
/MN
   1:  UFRAME_NUM=0 ;
   2:  UTOOL_NUM=1 ;
   3:  R[90]=0 ;
   4:  R[91]=0 ;
   5:  J PR[10] 20% FINE ;
   6:  R[90]=10 ;
   7:  LBL[10] ;
   8:  IF R[91]=-1,JMP LBL[99] ;
   9:  IF R[91]=10,JMP LBL[11] ;
  10:  JMP LBL[10] ;
  11:  LBL[11] ;
  12:  R[91]=0 ;
  13:  J PR[11] 20% FINE ;
  14:  R[90]=11 ;
  15:  LBL[12] ;
  16:  IF R[91]=-1,JMP LBL[99] ;
  17:  IF R[91]=11,JMP LBL[13] ;
  18:  JMP LBL[12] ;
  19:  LBL[13] ;
  20:  R[91]=0 ;
  21:  J PR[12] 20% FINE ;
  22:  R[90]=12 ;
  23:  LBL[14] ;
  24:  IF R[91]=-1,JMP LBL[99] ;
  25:  IF R[91]=12,JMP LBL[15] ;
  26:  JMP LBL[14] ;
  27:  LBL[15] ;
  28:  R[91]=0 ;
  29:  J PR[13] 20% FINE ;
  30:  R[90]=13 ;
  31:  LBL[16] ;
  32:  IF R[91]=-1,JMP LBL[99] ;
  33:  IF R[91]=13,JMP LBL[17] ;
  34:  JMP LBL[16] ;
  35:  LBL[17] ;
  36:  R[91]=0 ;
  37:  J PR[14] 20% FINE ;
  38:  R[90]=14 ;
  39:  LBL[18] ;
  40:  IF R[91]=-1,JMP LBL[99] ;
  41:  IF R[91]=14,JMP LBL[19] ;
  42:  JMP LBL[18] ;
  43:  LBL[19] ;
  44:  R[91]=0 ;
  45:  J PR[15] 20% FINE ;
  46:  R[90]=15 ;
  47:  LBL[20] ;
  48:  IF R[91]=-1,JMP LBL[99] ;
  49:  IF R[91]=15,JMP LBL[21] ;
  50:  JMP LBL[20] ;
  51:  LBL[21] ;
  52:  R[91]=0 ;
  53:  J PR[16] 20% FINE ;
  54:  R[90]=16 ;
  55:  LBL[22] ;
  56:  IF R[91]=-1,JMP LBL[99] ;
  57:  IF R[91]=16,JMP LBL[23] ;
  58:  JMP LBL[22] ;
  59:  LBL[23] ;
  60:  R[91]=0 ;
  61:  R[90]=999 ;
  62:  END ;
  63:  LBL[99] ;
  64:  R[91]=0 ;
  65:  R[90]=-1 ;
  66:  END ;
/POS
/END
