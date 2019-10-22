#ifndef ETHERCATMCGVLAXIS_H
#define ETHERCATMCGVLAXIS_H

#include <stdint.h>


extern "C" {
  int EthercatMCCreateGvlAxis(const char *EthercatMCName, int axisNo,
                              int axisFlags, const char *axisOptionsStr);
};

typedef struct {
  unsigned  wStatusWord;
} st_gvl_axis_status_type;

class epicsShareClass EthercatMCGvlAxis : public asynMotorAxis
{
public:
  /* These are the methods we override from the base class */
  EthercatMCGvlAxis(class EthercatMCController *pC, int axisNo,
                    int axisFlags, const char *axisOptionsStr);
  void report(FILE *fp, int level);
  asynStatus move(double position, int relative, double min_velocity, double max_velocity, double acceleration);
  asynStatus moveVelocity(double min_velocity, double max_velocity, double acceleration);
  asynStatus setPosition(double);

  asynStatus home(double min_velocity, double max_velocity, double acceleration, int forwards);
  asynStatus stop(double acceleration);
  void       callParamCallbacksUpdateError();
  asynStatus pollAll(bool *moving);
  asynStatus pollAll(bool *moving, st_axis_status_type *pst_axis_status);
  asynStatus poll(bool *moving);

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

  EthercatMCController *pC_;
  struct {
    st_gvl_axis_status_type old_st_axis_status;
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
    int homed;
    eeAxisWarningType old_eeAxisWarning;
    eeAxisWarningType eeAxisWarning;
    eeAxisErrorType old_eeAxisError;
    eeAxisErrorType eeAxisError;
    eeAxisPollNowType eeAxisPollNow;

    struct {
      unsigned int oldStatusDisconnected : 1;
      unsigned int initialPollNeeded :1;
      unsigned int sErrorMessage :1;
    }  dirty;

    /* Error texts when we talk to the controller, there is not an "OK"
       Or, failure in setValueOnAxisVerify() */
    char cmdErrorMessage[80]; /* From driver */
    char sErrorMessage[80]; /* From controller */
    char adsport_str[15]; /* "ADSPORT=12345/" */ /* 14 should be enough, */
    char adsport_zero[1]; /* 15 + 1 for '\' keep us aligned in memory */
    unsigned int adsPort;
  } drvlocal;

  asynStatus initialPoll(void);
  asynStatus initialPollInternal(void);
  asynStatus setValueOnAxis(const char* var, int value);
  asynStatus setValueOnAxisVerify(const char *var, const char *rbvar,
                                  int value, unsigned int retryCount);
  asynStatus setValueOnAxis(const char* var, double value);
  asynStatus setValuesOnAxis(const char* var1, double value1, const char* var2, double value2);
  asynStatus getValueFromAxis(const char* var, int *value);
  asynStatus getValueFromAxis(const char* var, double *value);
  asynStatus getStringFromAxis(const char* var, char *value, size_t maxlen);
  asynStatus resetAxis(void);
  bool       pollPowerIsOn(void);
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

  friend class EthercatMCController;
};

#endif
