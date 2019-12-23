/*
FILENAME...   EthercatMCAxis.h
*/

#ifndef ETHERCATMCAXIS_H
#define ETHERCATMCAXIS_H

#include "asynMotorAxis.h"
#include <stdint.h>
#include <asynInt32SyncIO.h>
#include <asynFloat64SyncIO.h>
#include <asynInt8ArraySyncIO.h>

#define AMPLIFIER_ON_FLAG_CREATE_AXIS  (1)
#define AMPLIFIER_ON_FLAG_AUTO_ON      (1<<1)
#define AMPLIFIER_ON_FLAG_USING_CNEN   (1<<2)

// ECMC ###########################################################################################
#define ECMC_MAX_ASYN_DIAG_STR_LEN    256
#define ECMC_MAX_ASYN_DRVINFO_STR_LEN 128
// Statuses
#define ECMC_ASYN_AXIS_STAT_STRING          "T_SMP_MS=%d/TYPE=asynInt32/ax%d.status?"
#define ECMC_ASYN_AXIS_DIAG_STRING          "T_SMP_MS=%d/TYPE=asynInt8ArrayIn/ax%d.diagnostic?"
#define ECMC_ASYN_AXIS_DIAG_BIN_STRING      "T_SMP_MS=%d/TYPE=asynInt8ArrayIn/ax%d.diagnosticbin?"
// Control
#define ECMC_ASYN_AXIS_CONT_STRING          "T_SMP_MS=%d/TYPE=asynInt32/ax%d.control="
#define ECMC_ASYN_AXIS_TARG_POS_STRING      "T_SMP_MS=%d/TYPE=asynFloat64/ax%d.targetpos="
#define ECMC_ASYN_AXIS_TARG_VEL_STRING      "T_SMP_MS=%d/TYPE=asynFloat64/ax%d.targetvel="
#define ECMC_ASYN_AXIS_TARG_ACC_STRING      "T_SMP_MS=%d/TYPE=asynFloat64/ax%d.targetacc="
#define ECMC_ASYN_AXIS_SOFT_LIM_BWD_STRING  "T_SMP_MS=%d/TYPE=asynFloat64/ax%d.softlimbwd="
#define ECMC_ASYN_AXIS_SOFT_LIM_FWD_STRING  "T_SMP_MS=%d/TYPE=asynFloat64/ax%d.softlimfwd="


// TODO Use common h file with ECMC for below types (or merge ECMC and this driver)
typedef struct {
  unsigned char              enabled:1;
  unsigned char              execute:1;
  unsigned char              busy:1;
  unsigned char              attarget:1;
  unsigned char              moving:1;
  unsigned char              limitfwd:1;
  unsigned char              limitbwd:1;
  unsigned char              homeswitch:1;
  unsigned char              instartup:1;
  unsigned char              inrealtime:1;
  unsigned char              trajsource:1;
  unsigned char              encsource:1;
  unsigned char              plccmdallowed:1;
  unsigned char              softlimfwdena:1;
  unsigned char              softlimbwdena:1;
  unsigned char              unused:1;
  unsigned char              seqstate:8;
  unsigned char              lastilock:8;
} ecmcAxisStatusWordType;

typedef struct {
  unsigned char              enable:1;
  unsigned char              execute:1;
  unsigned char              reset:1;
  unsigned char              softlimfwdena:1;
  unsigned char              softlimbwdena:1;
  unsigned char              unused_1:3;
  unsigned char              unused_2:8;
  unsigned char              cmd:8;
  unsigned int               cmddata:8;
}ecmcAxisControlWordType;

// 6,0.000000,0.000000,0.000000,0.000000,0.000000,0,0.000000,0.000000,0.000000,0.000000,0,8598,0,0,0,0,0,0,0,1,0,0,0,0,1,1,1,1,1
typedef struct {
  int    axId;            // 1
  double setpos;          // 2
  double actpos;          // 3
  double poserr;          // 4
  double targpos;         // 5
  double targposerr;      // 6
  int    rawpos;          // 7
  double cntrlout;        // 8
  double setvel;          // 9
  double actvel;          // 10
  double rawvelff;        // 11
  int    rawvel;          // 12     
  int    cyclecnt;        // 13
  int    error;           // 14
  int    cmd;             // 15
  int    cmddata;         // 16
  int    seqstate;        // 17
  int    ilock;           // 18
  int    ilocklastactive; // 19
  int    trajsource;      // 20
  int    encsource;       // 21
  int    enable;          // 22
  int    enabled;         // 23
  int    execute;         // 24
  int    busy;            // 25
  int    attarget;        // 26
  int    homed;           // 27
  int    lowlim;          // 28
  int    highlim;         // 29
  int    homesensor;      // 30
} ecmcDiagStringData;

enum motionCommandTypes {
  ECMC_CMD_NOCMD      = -1,
  ECMC_CMD_JOG        = 0,
  ECMC_CMD_MOVEVEL    = 1,
  ECMC_CMD_MOVEREL    = 2,
  ECMC_CMD_MOVEABS    = 3,
  ECMC_CMD_MOVEMODULO = 4,   // NOT IMPLEMENTED
  ECMC_CMD_HOMING     = 10,  // PARTLY IMPLEMENTED
  // NOT IMPLEMENTED (implemented in another way..)
  ECMC_CMD_SUPERIMP   = 20,  // NOT IMPLEMENTED
  // NOT IMPLEMENTED (implemented in another way..)
  ECMC_CMD_GEAR       = 30,
};

enum interlockTypes {
  ECMC_INTERLOCK_NONE                              = 0,
  ECMC_INTERLOCK_SOFT_BWD                          = 1,
  ECMC_INTERLOCK_SOFT_FWD                          = 2,
  ECMC_INTERLOCK_HARD_BWD                          = 3,
  ECMC_INTERLOCK_HARD_FWD                          = 4,
  ECMC_INTERLOCK_NO_EXECUTE                        = 5,
  ECMC_INTERLOCK_POSITION_LAG                      = 6,
  ECMC_INTERLOCK_BOTH_LIMITS                       = 7,
  ECMC_INTERLOCK_EXTERNAL                          = 8,
  ECMC_INTERLOCK_TRANSFORM                         = 9,
  ECMC_INTERLOCK_MAX_SPEED                         = 10,
  ECMC_INTERLOCK_CONT_HIGH_LIMIT                   = 11,
  ECMC_INTERLOCK_CONT_OUT_INCREASE_AT_LIMIT_SWITCH = 12,
  ECMC_INTERLOCK_AXIS_ERROR_STATE                  = 13,
  ECMC_INTERLOCK_UNEXPECTED_LIMIT_SWITCH_BEHAVIOUR = 14,
  ECMC_INTERLOCK_VELOCITY_DIFF                     = 15,
  ECMC_INTERLOCK_ETHERCAT_MASTER_NOT_OK            = 16,
  ECMC_INTERLOCK_PLC_NORMAL                        = 17,
  ECMC_INTERLOCK_PLC_BWD                           = 18,
  ECMC_INTERLOCK_PLC_FWD                           = 19,
};

enum dataSource {
  ECMC_DATA_SOURCE_INTERNAL           = 0,
  ECMC_DATA_SOURCE_EXTERNALENCODER    = 1,
  ECMC_DATA_SOURCE_EXTERNALTRAJECTORY = 2
};

typedef struct {
  double             positionSetpoint;
  double             positionActual;
  double             positionError;
  double             positionTarget;
  double             cntrlError;
  double             cntrlOutput;
  double             velocityActual;
  double             velocitySetpoint;
  double             velocityFFRaw;
  int64_t            positionRaw;
  int                error;
  int                velocitySetpointRaw;
  int                seqState;
  int                cmdData;
  motionCommandTypes command;
  interlockTypes     trajInterlock;
  interlockTypes     lastActiveInterlock;
  dataSource         trajSource;
  dataSource         encSource;
  bool               enable;
  bool               enabled;
  bool               execute;
  bool               busy;
  bool               atTarget;
  bool               homed;
  bool               limitFwd;
  bool               limitBwd;
  bool               homeSwitch;
  bool               sumIlockFwd;
  bool               sumIlockBwd;
} ecmcAxisStatusOnChangeType;

typedef struct {
  int                        axisID;
  int                        cycleCounter;
  double                     acceleration;
  double                     deceleration;
  bool                       reset;
  bool                       moving;
  bool                       stall;
  ecmcAxisStatusOnChangeType onChangeData;
} ecmcAxisStatusType;

//ECMC End

extern const char *modNamEMC;

extern "C" {
  int EthercatMCCreateAxis(const char *EthercatMCName, int axisNo,
                      int axisFlags, const char *axisOptionsStr);
  asynStatus writeReadOnErrorDisconnect_C(asynUser *pasynUser,
                                          const char *outdata, size_t outlen,
                                          char *indata, size_t inlen);
  asynStatus checkACK(const char *outdata, size_t outlen, const char *indata);
  double EthercatMCgetNowTimeSecs(void);
}

typedef struct {
  /* V1 members */
  int bEnable;           /*  1 */
  int bReset;            /*  2 */
  int bExecute;          /*  3 */
  int nCommand;          /*  4 */
  int nCmdData;          /*  5 */
  double fVelocity;      /*  6 */
  double fPosition;      /*  7 */
  double fAcceleration;  /*  8 */
  double fDecceleration; /*  9 */
  int bJogFwd;           /* 10 */
  int bJogBwd;           /* 11 */
  int bLimitFwd;         /* 12 */
  int bLimitBwd;         /* 13 */
  double fOverride;      /* 14 */
  int bHomeSensor;       /* 15 */
  int bEnabled;          /* 16 */
  int bError;            /* 17 */
  int nErrorId;          /* 18 */
  double fActVelocity;   /* 19 */
  double fActPosition;   /* 20 */
  double fActDiff;       /* 21 */
  int bHomed;            /* 22 */
  int bBusy;             /* 23 */
  /* V2 members */
  double encoderRaw;
  int atTarget;
  /* neither V1 nor V2, but calculated here */
  int mvnNRdyNex; /* Not in struct. Calculated in poll() */
  int motorStatusDirection; /* Not in struct. Calculated in pollAll() */
  int motorDiffPostion;     /* Not in struct. Calculated in poll() */
} st_axis_status_type;

class epicsShareClass EthercatMCAxisEcmc : public asynMotorAxis
{
public:
  /* These are the methods we override from the base class */
  EthercatMCAxisEcmc(class EthercatMCController *pC, int axisNo,
            int axisFlags, const char *axisOptionsStr);
  void report(FILE *fp, int level);
  asynStatus mov2(double posEGU, int nCommand, double maxVeloEGU, double accEGU);
  asynStatus move(double position, int relative, double min_velocity, double max_velocity, double acceleration);
  asynStatus moveVelocity(double min_velocity, double max_velocity, double acceleration);
  asynStatus setPosition(double);

  asynStatus home(double min_velocity, double max_velocity, double acceleration, int forwards);
  asynStatus stop(double acceleration);
  void       callParamCallbacksUpdateError();
  //asynStatus pollAll(bool *moving);
  //asynStatus pollAll(bool *moving, st_axis_status_type *pst_axis_status);
  asynStatus poll(bool *moving);

  // ECMC ##############
  ecmcAxisStatusType *getDiagBinDataPtr();
  // ECMC End ##############
private:
  typedef enum
  {
    eeAxisErrorIOCcomError = -1,
    eeAxisErrorNoError,
    eeAxisErrorMCUError,
    eeAxisErrorCmdError,
    eeAxisErrorNotFound,
    eeAxisErrorNotHomed,
    eeAxisIllegalInTargetWindow
  } eeAxisErrorType;

  typedef enum
  {
    eeAxisWarningNoWarning,
    eeAxisWarningCfgZero,
    eeAxisWarningVeloZero,
    eeAxisWarningSpeedLimit
  } eeAxisWarningType;

  typedef enum
  {
    pollNowReadScaling,
    pollNowReadMonitoring,
    pollNowReadBackSoftLimits,
    pollNowReadBackVelocities
  } eeAxisPollNowType;

  EthercatMCController *pC_;          /**< Pointer to the asynMotorController to which this axis belongs.
                                   *   Abbreviated because it is used very frequently */
  struct {
    st_axis_status_type old_st_axis_status;
    double scaleFactor;
    double eres;
    const char *externalEncoderStr;
    const char *cfgfileStr;
    const char *cfgDebug_str;
    int axisFlags;
    int MCU_nErrorId;     /* nErrorID from MCU */
    int old_MCU_nErrorId; /* old nErrorID from MCU */
    int old_EPICS_nErrorId; /* old nErrorID from MCU */

    int old_bError;   /* copy of bError */
#ifndef motorWaitPollsBeforeReadyString
    unsigned int waitNumPollsBeforeReady;
#endif
    int nCommandActive;
    int old_nCommandActive;
    int homed;
    unsigned int illegalInTargetWindow :1;
    eeAxisWarningType old_eeAxisWarning;
    eeAxisWarningType eeAxisWarning;
    eeAxisErrorType old_eeAxisError;
    eeAxisErrorType eeAxisError;
    eeAxisPollNowType eeAxisPollNow;
    /* Which values have changed in the EPICS IOC, but are not updated in the
       motion controller */
    struct {
      int          nMotionAxisID;     /* Needed for ADR commands */
      unsigned int statusVer        :1;
      unsigned int oldStatusDisconnected : 1;
      unsigned int sErrorMessage    :1; /* From MCU */
      unsigned int initialPollNeeded :1;
    }  dirty;

    struct {
      int          statusVer;           /* 0==V1, busy old style 1==V1, new style*/
      unsigned int bV1BusyNewStyle  :1;
    }  supported;

    /* Error texts when we talk to the controller, there is not an "OK"
       Or, failure in setValueOnAxisVerify() */
    char cmdErrorMessage[80]; /* From driver */
    char sErrorMessage[80]; /* From controller */
    char adsport_str[15]; /* "ADSPORT=12345/" */ /* 14 should be enough, */
    char adsport_zero[1]; /* 15 + 1 for '\' keep us aligned in memory */
    unsigned int adsPort;
  } drvlocal;

  asynStatus handleConnect(void);
  asynStatus writeReadControllerPrint(int traceMask);
  asynStatus writeReadControllerPrint(void);
  asynStatus readConfigLine(const char *line, const char **errorTxt_p);
  asynStatus readConfigFile(void);
  asynStatus readBackAllConfig(int axisID);
  asynStatus updateCfgValue(int function, int    newValue, const char *name);
  asynStatus updateCfgValue(int function, double newValue, const char *name);
  asynStatus readBackSoftLimits(void);
  asynStatus readBackHoming(void);
  asynStatus readScaling(int axisID);
  asynStatus readMonitoring(int axisID);
  asynStatus readBackVelocities(int axisID);
  asynStatus readBackEncoders(int axisID);
  asynStatus initialPoll(void);
  asynStatus initialPollInternal(void);
  asynStatus setValueOnAxis(const char* var, int value);
  asynStatus setValueOnAxisVerify(const char *var, const char *rbvar,
                                  int value, unsigned int retryCount);
  asynStatus setValueOnAxis(const char* var, double value);
  asynStatus setValuesOnAxis(const char* var1, double value1, const char* var2, double value2);
  int getMotionAxisID(void);
  asynStatus setSAFValueOnAxis(unsigned indexGroup,
                               unsigned indexOffset,
                               int value);

  asynStatus setSAFValueOnAxisVerify(unsigned indexGroup,
                                     unsigned indexOffset,
                                     int value,
                                     unsigned int retryCount);

  asynStatus setSAFValueOnAxis(unsigned indexGroup,
                               unsigned indexOffset,
                               double value);

  asynStatus setSAFValueOnAxisVerify(unsigned indexGroup,
                                     unsigned indexOffset,
                                     double value,
                                     unsigned int retryCount);

  asynStatus getSAFValueFromAxisPrint(unsigned indexGroup,
                                      unsigned indexOffset,
                                      const char *name,
                                      int *value);

  asynStatus getSAFValuesFromAxisPrint(unsigned iIndexGroup,
                                       unsigned iIndexOffset,
                                       const char *iname,
                                       int *iValue,
                                       unsigned fIndexGroup,
                                       unsigned fIndexOffset,
                                       const char *fname,
                                       double *fValue);

  asynStatus getSAFValueFromAxisPrint(unsigned indexGroup,
                                      unsigned indexOffset,
                                      const char *name,
                                      double *value);

  asynStatus getValueFromAxis(const char* var, int *value);
  asynStatus getValueFromAxis(const char* var, double *value);
  asynStatus getStringFromAxis(const char* var, char *value, size_t maxlen);
  asynStatus getValueFromController(const char* var, double *value);

  asynStatus resetAxis(void);
  //bool       pollPowerIsOn(void);
  asynStatus enableAmplifier(int);
  asynStatus sendVelocityAndAccelExecute(double maxVeloEGU, double accEGU);
  asynStatus setClosedLoop(bool closedLoop);
  asynStatus setIntegerParam(int function, int value);
  asynStatus setDoubleParam(int function, double value);
  asynStatus setStringParamDbgStrToMcu(const char *value);
  asynStatus setStringParam(int function, const char *value);
  asynStatus stopAxisInternal(const char *function_name, double acceleration);
#ifndef motorMessageTextString
  void updateMsgTxtFromDriver(const char *value);
#endif

  // ECMC specific
  asynStatus connectEcmcAxis();
  asynStatus readStatusWd();
  asynStatus readDiagStr();       // Read ascii diag data over int8array interface
  asynStatus readDiagBin();       // Read binary diag data over int8array interface
  asynStatus readAllStatus();
  asynStatus readControlWd(ecmcAxisControlWordType *controlWd);
  asynStatus writeControlWd(ecmcAxisControlWordType controlWd);
  asynStatus writeTargetPos(double pos);
  asynStatus writeTargetVel(double vel);
  asynStatus writeTargetAcc(double acc);
  asynStatus writeSoftLimBwd(double softlimbwd);
  asynStatus writeSoftLimFwd(double softlimfwd);
  asynStatus printDiagBinData();
  // Temporary convert betwwen differnt structure types.. Remove later
  asynStatus uglyConvertFunc(ecmcAxisStatusType*in ,st_axis_status_type *out);
  asynUser *asynUserStatWd_;      // "T_SMP_MS=%d/TYPE=asynInt32/ax%d.status?"
  asynUser *asynUserDiagStr_;     // "T_SMP_MS=%d/TYPE=asynInt8ArrayIn/ax%d.diagnostic?"  
  asynUser *asynUserDiagBin_;     // "T_SMP_MS=%d/TYPE=asynInt8ArrayIn/ax%d.diagnosticbin?"  
  asynUser *asynUserDiagBinIntr_; // "T_SMP_MS=%d/TYPE=asynInt8ArrayIn/ax%d.diagnosticbin?"  
  asynUser *asynUserCntrlWd_;     // "T_SMP_MS=%d/TYPE=asynInt32/ax%d.control="
  asynUser *asynUserTargPos_;     // "T_SMP_MS=%d/TYPE=asynFloat64/ax%d.targetpos="
  asynUser *asynUserTargVel_;     // "T_SMP_MS=%d/TYPE=asynFloat64/ax%d.targetvel="
  asynUser *asynUserTargAcc_;     // "T_SMP_MS=%d/TYPE=asynFloat64/ax%d.targetacc="
  asynUser *asynUserSoftLimBwd_;  // "T_SMP_MS=%d/TYPE=asynFloat64/ax%d.soflimbwd="
  asynUser *asynUserSoftLimFwd_;  // "T_SMP_MS=%d/TYPE=asynFloat64/ax%d.soflimfwd="
  char      diagStringBuffer_[ECMC_MAX_ASYN_DIAG_STR_LEN];
  int       axisId_;
  //double    actPos_;
  ecmcAxisStatusWordType statusWd_;
  ecmcDiagStringData     diagData_;
  ecmcAxisStatusType     diagBinData_;
  asynInterface *pasynIFDiagBinIntr_;
  asynInt8Array *pIFDiagBinIntr_;
  void          *interruptDiagBinPvt_;

  double oldPositionAct_;  // needed for uglyConvertFunc().. 
  // ECMC end
  friend class EthercatMCController;
};

#endif
