/*
  FILENAME... EthercatMCGvlAxis.cpp
*/

#include <stdio.h>
#include <string.h>
#include <stdlib.h>
#include <math.h>
#include <errno.h>
#include <unistd.h>

#include <epicsThread.h>

#include "motor.h"
#include "EthercatMCController.h"
#include "EthercatMCGvlAxis.h"

#ifndef ASYN_TRACE_INFO
#define ASYN_TRACE_INFO      0x0040
#endif

#ifndef ASYN_TRACE_DEBUG
#define ASYN_TRACE_DEBUG     0x0080
#endif

typedef enum {
  idxStatusCodeRESET    = 0,
  idxStatusCodeIDLE     = 1,
  idxStatusCodePOWEROFF = 2,
  idxStatusCodeWARN     = 3,
  idxStatusCodeERR4     = 4,
  idxStatusCodeSTART    = 5,
  idxStatusCodeBUSY     = 6,
  idxStatusCodeSTOP     = 7,
  idxStatusCodeERROR    = 8,
  idxStatusCodeERR9     = 9,
  idxStatusCodeERR10    = 10,
  idxStatusCodeERR11    = 11,
  idxStatusCodeERR12    = 12,
  idxStatusCodeERR13    = 13,
  idxStatusCodeERR14    = 14,
  idxStatusCodeERR15    = 15
} idxStatusCodeType;

extern "C" const char *idxStatusCodeTypeToStr(idxStatusCodeType idxStatusCode);

/* The maximum number of polls we wait for the motor
   to "start" (report moving after a new move command */
#define WAITNUMPOLLSBEFOREREADY 3


//
// These are the EthercatMCGvlAxis methods
//

/** Creates a new EthercatMCGvlAxis object.
 * \param[in] pC Pointer to the EthercatMCController to which this axis belongs.
 * \param[in] axisNo Index number of this axis, range 1 to pC->numAxes_. (0 is not used)
 *
 *
 * Initializes register numbers, etc.
 */
EthercatMCGvlAxis::EthercatMCGvlAxis(EthercatMCController *pC, int axisNo,
                                     int axisFlags, const char *axisOptionsStr)
  : asynMotorAxis(pC, axisNo),
    pC_(pC)
{
#ifdef motorFlagsDriverUsesEGUString
    setIntegerParam(pC_->motorFlagsDriverUsesEGU_,1);
#endif
#ifdef motorFlagsAdjAfterHomedString
    setIntegerParam(pC_->motorFlagsAdjAfterHomed_, 1);
#endif
#ifdef motorWaitPollsBeforeReadyString
  setIntegerParam(pC_->motorWaitPollsBeforeReady_ , WAITNUMPOLLSBEFOREREADY);
#endif
  memset(&drvlocal, 0, sizeof(drvlocal));
  memset(&drvlocal.dirty, 0xFF, sizeof(drvlocal.dirty));
  drvlocal.old_eeAxisError = eeAxisErrorIOCcomError;
  drvlocal.axisFlags = axisFlags;

  /* We pretend to have an encoder (fActPosition) */
  setIntegerParam(pC_->motorStatusHasEncoder_, 1);
#ifdef motorFlagsNoStopProblemString
  setIntegerParam(pC_->motorFlagsNoStopProblem_, 1);
#endif
#ifdef motorFlagsNoStopOnLsString
  setIntegerParam(pC_->motorFlagsNoStopOnLS_, 1);
#endif
#ifdef motorFlagsLSrampDownString
  setIntegerParam(pC_->motorFlagsLSrampDown_, 1);
#endif
#ifdef motorFlagsPwrWaitForOnString
  setIntegerParam(pC_->motorFlagsPwrWaitForOn_, 1);
#endif
#if 0
#ifdef POWERAUTOONOFFMODE2
  if (axisFlags & AMPLIFIER_ON_FLAG_WHEN_HOMING) {
    setIntegerParam(pC_->motorPowerAutoOnOff_, POWERAUTOONOFFMODE2);
    setDoubleParam(pC_->motorPowerOnDelay_,   6.0);
    setDoubleParam(pC_->motorPowerOffDelay_, -1.0);
  }
#endif
#endif
#ifdef motorShowPowerOffString
    setIntegerParam(pC_->motorShowPowerOff_, 1);
#endif
#ifdef  motorNotHomedProblemString
    setIntegerParam(pC_->motorNotHomedProblem_, MOTORNOTHOMEDPROBLEM_ERROR);
#endif

  //drvlocal.scaleFactor = 1.0;
#if 0
  if (axisFlags & AMPLIFIER_ON_FLAG_USING_CNEN) {
    setIntegerParam(pC->motorStatusGainSupport_, 1);
  }
#endif
  pC_->features_ |= FEATURE_BITS_GVL;
  if (axisOptionsStr && axisOptionsStr[0]) {
    const char * const encoder_is_str = "encoder=";
    const char * const cfgfile_str = "cfgFile=";
    const char * const cfgDebug_str = "getDebugText=";
#ifndef motorFlagsDriverUsesEGUString
    const char * const stepSize_str = "stepSize=";
#endif
    const char * const homProc_str = "HomProc=";
    const char * const homPos_str  = "HomPos=";
    const char * const adsPort_str  = "adsPort=";
    const char * const scaleFactor_str = "scaleFactor=";
    const char * const sFeatures_str = "sFeatures=";

    char *pOptions = strdup(axisOptionsStr);
    char *pThisOption = pOptions;
    char *pNextOption = pOptions;

    while (pNextOption && pNextOption[0]) {
      pNextOption = strchr(pNextOption, ';');
      if (pNextOption) {
        *pNextOption = '\0'; /* Terminate */
        pNextOption++;       /* Jump to (possible) next */
      }
      if (!strncmp(pThisOption, encoder_is_str, strlen(encoder_is_str))) {
        pThisOption += strlen(encoder_is_str);
        drvlocal.externalEncoderStr = strdup(pThisOption);
      }  else if (!strncmp(pThisOption, cfgfile_str, strlen(cfgfile_str))) {
        pThisOption += strlen(cfgfile_str);
        drvlocal.cfgfileStr = strdup(pThisOption);
      } else if (!strncmp(pThisOption, cfgDebug_str, strlen(cfgDebug_str))) {
        pThisOption += strlen(cfgDebug_str);
        drvlocal.cfgDebug_str = strdup(pThisOption);
#ifndef motorFlagsDriverUsesEGUString
      } else if (!strncmp(pThisOption, stepSize_str, strlen(stepSize_str))) {
        pThisOption += strlen(stepSize_str);
        /* This option is obsolete, depending on motor */
        drvlocal.scaleFactor = atof(pThisOption);
#endif
      } else if (!strncmp(pThisOption, adsPort_str, strlen(adsPort_str))) {
        pThisOption += strlen(adsPort_str);
        int adsPort = atoi(pThisOption);
        if (adsPort > 0) {
          /* Save adsport_str for the poller */
          snprintf(drvlocal.adsport_str, sizeof(drvlocal.adsport_str),
                   "ADSPORT=%d/", adsPort);
          drvlocal.adsPort = (unsigned)adsPort;
        }
      } else if (!strncmp(pThisOption, homProc_str, strlen(homProc_str))) {
        pThisOption += strlen(homProc_str);
        int homProc = atoi(pThisOption);
        setIntegerParam(pC_->EthercatMCHomProc_, homProc);
      } else if (!strncmp(pThisOption, homPos_str, strlen(homPos_str))) {
        pThisOption += strlen(homPos_str);
        double homPos = atof(pThisOption);
        setDoubleParam(pC_->EthercatMCHomPos_, homPos);
      } else if (!strncmp(pThisOption, scaleFactor_str, strlen(scaleFactor_str))) {
        pThisOption += strlen(scaleFactor_str);
        drvlocal.scaleFactor = atof(pThisOption);
      } else if (!strncmp(pThisOption, sFeatures_str, strlen(sFeatures_str))) {
        pThisOption += strlen(sFeatures_str);
        if (!strcmp(pThisOption, "Gvl")) {
        }
      }
      pThisOption = pNextOption;
    }
    free(pOptions);
  }
  /* Set the module name to "" if we have FILE/LINE enabled by asyn */
  if (pasynTrace->getTraceInfoMask(pC_->pasynUserController_) & ASYN_TRACEINFO_SOURCE) modNamEMC = "";
  initialPoll();
}


extern "C" int EthercatMCCreateGvlAxis(const char *EthercatMCName, int axisNo,
                                       int axisFlags, const char *axisOptionsStr)
{
  EthercatMCController *pC;

  pC = (EthercatMCController*) findAsynPortDriver(EthercatMCName);
  if (!pC)
  {
    printf("Error port %s not found\n", EthercatMCName);
    return asynError;
  }
  pC->lock();
  new EthercatMCGvlAxis(pC, axisNo, axisFlags, axisOptionsStr);
  pC->unlock();
  return asynSuccess;
}


asynStatus EthercatMCGvlAxis::initialPoll(void)
{
  asynStatus status;
  if (!drvlocal.dirty.initialPollNeeded)
    return asynSuccess;

  status = initialPollInternal();
  asynPrint(pC_->pasynUserController_, ASYN_TRACE_INFO,
            "%sinitialPoll(%d) status=%d\n",
            modNamEMC, axisNo_, status);
  if (status == asynSuccess) drvlocal.dirty.initialPollNeeded = 0;
  return status;
}


/** Connection status is changed, the dirty bits must be set and
 *  the values in the controller must be updated
 * \param[in] AsynStatus status
 *
 * Sets the dirty bits
 */
asynStatus EthercatMCGvlAxis::initialPollInternal(void)
{
  asynStatus status = asynSuccess;
  return status;
}

/** Reports on status of the axis
 * \param[in] fp The file pointer on which report information will be written
 * \param[in] level The level of report detail desired
 *
 * After printing device-specific information calls asynMotorAxis::report()
 */
void EthercatMCGvlAxis::report(FILE *fp, int level)
{
  if (level > 0) {
    fprintf(fp, "  axis %d\n", axisNo_);
  }

  // Call the base class method
  asynMotorAxis::report(fp, level);
}



/** Move the axis to a position, either absolute or relative
 * \param[in] position in steps
 * \param[in] relative (0=absolute, otherwise relative)
 * \param[in] minimum velocity, steps/sec
 * \param[in] maximum velocity, steps/sec
 * \param[in] acceleration,  steps/sec/sec
 *
 */
asynStatus EthercatMCGvlAxis::move(double position, int relative, double minVelocity, double maxVelocity, double acceleration)
{
  asynStatus status = asynError;
  return status;
}


/** Home the motor, search the home position
 * \param[in] minimum velocity, mm/sec
 * \param[in] maximum velocity, mm/sec
 * \param[in] acceleration, seconds to maximum velocity
 * \param[in] forwards (0=backwards, otherwise forwards)
 *
 */
asynStatus EthercatMCGvlAxis::home(double minVelocity, double maxVelocity, double acceleration, int forwards)
{
  asynStatus status = asynError;
#ifndef motorWaitPollsBeforeReadyString
  drvlocal.waitNumPollsBeforeReady += WAITNUMPOLLSBEFOREREADY;
#endif
  return status;
}


/** jog the the motor, search the home position
 * \param[in] minimum velocity, mm/sec (not used)
 * \param[in] maximum velocity, mm/sec (positive or negative)
 * \param[in] acceleration, seconds to maximum velocity
 *
 */
asynStatus EthercatMCGvlAxis::moveVelocity(double minVelocity, double maxVelocity, double acceleration)
{
  asynStatus status = asynError;
  return status;

}



/**
 * See asynMotorAxis::setPosition
 */
asynStatus EthercatMCGvlAxis::setPosition(double value)
{
  asynStatus status = asynError;
  return status;
}

asynStatus EthercatMCGvlAxis::resetAxis(void)
{
  asynStatus status = asynError;
  return status;
}

bool EthercatMCGvlAxis::pollPowerIsOn(void)
{
  return false;
}

/** Enable the amplifier on an axis
 *
 */
asynStatus EthercatMCGvlAxis::enableAmplifier(int on)
{
  asynStatus status = asynError;
  return status;

}

/** Stop the axis
 *
 */
asynStatus EthercatMCGvlAxis::stopAxisInternal(const char *function_name, double acceleration)
{
  asynStatus status = asynError;
  return status;
}

/** Stop the axis, called by motor Record
 *
 */
asynStatus EthercatMCGvlAxis::stop(double acceleration )
{
  drvlocal.eeAxisWarning = eeAxisWarningNoWarning;
  return stopAxisInternal(__FUNCTION__, acceleration);
}

void EthercatMCGvlAxis::callParamCallbacksUpdateError()
{
  const char *msgTxtFromDriver = NULL;
  int EPICS_nErrorId = drvlocal.MCU_nErrorId;
  drvlocal.eeAxisError = eeAxisErrorNoError;
  if (EPICS_nErrorId) {
    /* Error from MCU */
    drvlocal.eeAxisError = eeAxisErrorMCUError;
    msgTxtFromDriver = &drvlocal.sErrorMessage[0];
  } else if (drvlocal.dirty.sErrorMessage) {
    /* print error below */
    drvlocal.eeAxisError = eeAxisErrorIOCcomError;
  } else if (drvlocal.cmdErrorMessage[0]) {
    drvlocal.eeAxisError = eeAxisErrorCmdError;
    msgTxtFromDriver = &drvlocal.cmdErrorMessage[0];
#if 0
  } else if (!drvlocal.homed &&
             (drvlocal.nCommandActive != NCOMMANDHOME)) {
    drvlocal.eeAxisError = eeAxisErrorNotHomed;
  } else if (drvlocal.illegalInTargetWindow) {
    drvlocal.eeAxisError = eeAxisIllegalInTargetWindow;
    msgTxtFromDriver = "E: InTargetPosWin";
#endif
  }
  if (drvlocal.eeAxisError != drvlocal.old_eeAxisError ||
      drvlocal.eeAxisWarning != drvlocal.old_eeAxisWarning ||
      drvlocal.old_EPICS_nErrorId != EPICS_nErrorId) {

    if (!msgTxtFromDriver && drvlocal.eeAxisWarning) {
       /* No error to show yet */
      switch(drvlocal.eeAxisWarning) {
      case eeAxisWarningCfgZero:
        msgTxtFromDriver = "E: scaleFactor is 0.0";
        break;
      case eeAxisWarningVeloZero:
        msgTxtFromDriver = "E: velo is 0.0";
        break;
      case eeAxisWarningSpeedLimit:
        msgTxtFromDriver = "I: Speed Limit";
        break;
      case eeAxisWarningNoWarning:
        break;
      }
    }
    /* No warning to show yet */
    if (!msgTxtFromDriver) {
      msgTxtFromDriver = "I: Moving";
    }
    /* End of error/warning text messages */

    /* Axis has a problem: Report to motor record */
    /*
     * Site note: Some versions of the motor module needed and
     *  #ifdef motorFlagsNoStopProblemString
     * here. Today these versions are history, and the
     * motorFlagsNoStopProblemString is no longer defined in the
     * motor module. So we need to remove the #ifdef here.
     */
    setIntegerParam(pC_->motorStatusProblem_,
                    drvlocal.eeAxisError != eeAxisErrorNoError);
    /* MCU has a problem: set the red light in CSS */
    setIntegerParam(pC_->EthercatMCErr_,
                    drvlocal.eeAxisError == eeAxisErrorMCUError);
    setIntegerParam(pC_->EthercatMCErrId_, EPICS_nErrorId);
    asynPrint(pC_->pasynUserController_, ASYN_TRACE_INFO,
              "%spoll(%d) callParamCallbacksUpdateError"
              " Error=%d old=%d ErrID=0x%x old=0x%x Warn=%d txt=%s\n",
              modNamEMC, axisNo_, drvlocal.eeAxisError, drvlocal.old_eeAxisError,
              EPICS_nErrorId, drvlocal.old_EPICS_nErrorId,
              drvlocal.eeAxisWarning,
              msgTxtFromDriver ? msgTxtFromDriver : "NULL");

    if (!drvlocal.cfgDebug_str) {
      updateMsgTxtFromDriver(msgTxtFromDriver);
    }
    drvlocal.old_eeAxisError = drvlocal.eeAxisError;
    drvlocal.old_eeAxisWarning = drvlocal.eeAxisWarning;
    drvlocal.old_EPICS_nErrorId = EPICS_nErrorId;
  }

  callParamCallbacks();
}

asynStatus EthercatMCGvlAxis::setIntegerParamLog(int function,
                                                 int newValue,
                                                 const char *name)
{
  int oldValue;
  asynStatus status = pC_->getIntegerParam(axisNo_, function, &oldValue);
  if (status || (newValue != oldValue)) {
    asynPrint(pC_->pasynUserController_, ASYN_TRACE_INFO,
              "%spoll(%d) %s=%d\n",
              modNamEMC, axisNo_, name, newValue);
  }
  return setIntegerParam(function, newValue);
}


asynStatus EthercatMCGvlAxis::pollAll(bool *moving, st_axis_status_type *pst_axis_status)
{
  return asynSuccess;
}


/** Polls the axis.
 * This function reads the motor position, the limit status, the home status, the moving status,
 * and the drive power-on status.
 * It calls setIntegerParam() and setDoubleParam() for each item that it polls,
 * and then calls callParamCallbacks() at the end.
 * \param[out] moving A flag that is set indicating that the axis is moving (true) or done (false). */
asynStatus EthercatMCGvlAxis::poll(bool *moving)
{
  unsigned traceMask = ASYN_TRACE_INFO;
  double targetPosition = 0.0;
  double actPosition = 0.0;
  int nvals = 0;
  asynStatus comStatus = asynError;
  uint32_t statusReasonAux;
  idxStatusCodeType idxStatusCode;
  uint16_t errorID = 0; //0xFFFF;
  bool nowMoving = false;
  int powerIsOn = 0;
  int statusValid = 0;
  int hasError = 0;
  unsigned idxReasonBits = 0;
  unsigned idxAuxBits = 0;

  snprintf(pC_->outString_, sizeof(pC_->outString_),
           "%sMAIN.aAxesState_EPICS[%d].AxisStatus.wStatusWord?",
           drvlocal.adsport_str, axisNo_);
  comStatus = pC_->writeReadOnErrorDisconnect();
  if (comStatus) {
    asynPrint(pC_->pasynUserController_, ASYN_TRACE_ERROR|ASYN_TRACEIO_DRIVER,
              "%sout=%s in=%s return=%s (%d)\n",
              modNamEMC, pC_->outString_, pC_->inString_,
              EthercatMCstrStatus(comStatus), (int)comStatus);
    return asynError;
  }
  nvals = sscanf(pC_->inString_, "%u", &statusReasonAux);
  if (nvals != 1) {
    /* rubbish on the line */
    asynPrint(pC_->pasynUserController_, ASYN_TRACE_ERROR|ASYN_TRACEIO_DRIVER,
              "%spoll(%d) nvals=%d out=%s in=%s \n",
              modNamEMC, axisNo_, nvals, pC_->outString_, pC_->inString_);
    return asynError;
  }
  idxStatusCode = (idxStatusCodeType)(statusReasonAux >> 28);
  idxReasonBits = (statusReasonAux >> 24) & 0x0F;
  idxAuxBits    =  statusReasonAux  & 0x0FFFFFF;



  setIntegerParam(pC_->EthercatMCErrId_, errorID);
  setDoubleParam(pC_->motorPosition_, actPosition);
  drvlocal.hasProblem = 0;
  setIntegerParam(pC_->EthercatMCStatusCode_, idxStatusCode);
  if ((statusReasonAux != drvlocal.old_statusReasonAux) ||
      (idxAuxBits      != drvlocal.old_idxAuxBits)) {
    if (errorID) {
      asynPrint(pC_->pasynUserController_, traceMask,
                "%spoll(%d) actPos=%f targetPos=%f statusReasonAux=0x%x (%s) errorID=0x%x\n",
                modNamEMC, axisNo_,
                actPosition, targetPosition, statusReasonAux,
                idxStatusCodeTypeToStr(idxStatusCode), errorID);
    } else {
      asynPrint(pC_->pasynUserController_, traceMask,
                "%spoll(%d) actPos=%f targetPos=%f statusReasonAux=0x%x (%s)\n",
                modNamEMC, axisNo_,
                actPosition, targetPosition, statusReasonAux,
                idxStatusCodeTypeToStr(idxStatusCode));
    }
    drvlocal.old_statusReasonAux = statusReasonAux;
    drvlocal.old_idxAuxBits      = idxAuxBits;
  }

  switch (idxStatusCode) {
    /* After RESET, START, STOP the bits are not valid */
  case idxStatusCodeIDLE:
  case idxStatusCodeWARN:
    powerIsOn = 1;
    statusValid = 1;
    break;
  case idxStatusCodePOWEROFF:
    statusValid = 1;
    drvlocal.hasProblem = 1;
    powerIsOn = 0;
    break;
  case idxStatusCodeBUSY:
    powerIsOn = 1;
    statusValid = 1;
    nowMoving = true;
    break;
  case idxStatusCodeERROR:
    hasError = 1;
    statusValid = 1;
    drvlocal.hasProblem = 1;
    break;
  default:
    drvlocal.hasProblem = 1;
  }
  if (statusValid) {
    const char *msgTxtFromDriver = NULL;
    int hls = idxReasonBits & 0x8 ? 1 : 0;
    int lls = idxReasonBits & 0x4 ? 1 : 0;
    setIntegerParamLog(pC_->motorStatusLowLimit_, lls,  "LLS");
    setIntegerParamLog(pC_->motorStatusHighLimit_, hls, "HLS");
    setIntegerParam(pC_->motorStatusMoving_, nowMoving);
    setIntegerParam(pC_->motorStatusDone_, !nowMoving);
    setIntegerParam(pC_->EthercatMCStatusBits_, statusReasonAux);
    setIntegerParam(pC_->EthercatMCErr_, hasError);
    if (drvlocal.auxBitsNotHomedMask) {
      int homed = idxAuxBits & drvlocal.auxBitsNotHomedMask ? 0 : 1;
      setIntegerParamLog(pC_->motorStatusHomed_, homed, "homed");
      if (!homed) {
        drvlocal.hasProblem = 1;
      }
    }
    if (hasError) {
      char sErrorMessage[40];
      const char *errIdString = errStringFromErrId(errorID);
      memset(&sErrorMessage[0], 0, sizeof(sErrorMessage));
      if (errIdString[0]) {
        snprintf(sErrorMessage, sizeof(sErrorMessage)-1, "E: %s %x",
                 errIdString, errorID);
      }  else {
        snprintf(sErrorMessage, sizeof(sErrorMessage)-1,
                 "E: TwinCAT Err %x", errorID);
      }
      msgTxtFromDriver = sErrorMessage;
    }
    updateMsgTxtFromDriver(msgTxtFromDriver);
  }
  *moving = nowMoving;
  setIntegerParam(pC_->EthercatMCStatusCode_, idxStatusCode);
  setIntegerParam(pC_->motorStatusProblem_, drvlocal.hasProblem);
  setIntegerParam(pC_->motorStatusPowerOn_, powerIsOn);

  drvlocal.old_statusReasonAux = statusReasonAux;
  drvlocal.old_idxAuxBits      = idxAuxBits;
#if 0
  callParamCallbacksUpdateError();
#else
  callParamCallbacks();
#endif
  return asynSuccess;
}

/** Set the motor closed loop status
  * \param[in] closedLoop true = close loop, false = open looop. */
asynStatus EthercatMCGvlAxis::setClosedLoop(bool closedLoop)
{
  asynStatus status = asynError;
  return status;
}

asynStatus EthercatMCGvlAxis::setIntegerParam(int function, int value)
{
  asynStatus status;
  if (function == pC_->motorUpdateStatus_) {
    asynPrint(pC_->pasynUserController_, ASYN_TRACE_INFO,
              "%ssetIntegerParam(%d motorUpdateStatus_)=%d\n", modNamEMC, axisNo_, value);
  } else if (function == pC_->motorStatusCommsError_) {
    asynPrint(pC_->pasynUserController_, ASYN_TRACE_FLOW,
              "%ssetIntegerParam(%d pC_->motorStatusCommsError_)=%d\n",
              modNamEMC, axisNo_, value);
    if (value && !drvlocal.dirty.oldStatusDisconnected) {
      asynPrint(pC_->pasynUserController_, ASYN_TRACE_ERROR|ASYN_TRACEIO_DRIVER,
                "%s Communication error(%d)\n", modNamEMC, axisNo_);
      memset(&drvlocal.dirty, 0xFF, sizeof(drvlocal.dirty));
      drvlocal.MCU_nErrorId = 0;
    }
#ifdef motorPowerAutoOnOffString
  } else if (function == pC_->motorPowerAutoOnOff_) {
    asynPrint(pC_->pasynUserController_, ASYN_TRACE_INFO,
              "%ssetIntegerParam(%d motorPowerAutoOnOff_)=%d\n", modNamEMC, axisNo_, value);
#endif
#ifdef EthercatMCHomProcString
  } else if (function == pC_->EthercatMCHomProc_) {
    int motorNotHomedProblem = 0;
    setIntegerParam(pC_->EthercatMCHomProc_RB_, value);
    asynPrint(pC_->pasynUserController_, ASYN_TRACE_INFO,
              "%ssetIntegerParam(%d HomProc_)=%d motorNotHomedProblem=%d\n",
              modNamEMC, axisNo_, value, motorNotHomedProblem);
#endif
#ifdef EthercatMCErrRstString
  } else if (function == pC_->EthercatMCErrRst_) {
    if (value) {
      asynPrint(pC_->pasynUserController_, ASYN_TRACE_INFO,
                "%ssetIntegerParam(%d ErrRst_)=%d\n",
                modNamEMC, axisNo_, value);
      /*  We do not want to call the base class */
      return resetAxis();
    }
    /* If someone writes 0 to the field, just ignore it */
    return asynSuccess;
#endif
  }

  //Call base class method
  status = asynMotorAxis::setIntegerParam(function, value);
  return status;
}

/** Set a floating point parameter on the axis
 * \param[in] function, which parameter is updated
 * \param[in] value, the new value
 *
 * When the IOC starts, we will send the soft limits to the controller.
 * When a soft limit is changed, and update is send them to the controller.
 */
asynStatus EthercatMCGvlAxis::setDoubleParam(int function, double value)
{
  asynStatus status;

  if (function == pC_->motorMoveRel_) {
    asynPrint(pC_->pasynUserController_, ASYN_TRACE_INFO,
              "%ssetDoubleParam(%d motorMoveRel_)=%g\n", modNamEMC, axisNo_, value);
  } else if (function == pC_->motorMoveAbs_) {
    asynPrint(pC_->pasynUserController_, ASYN_TRACE_INFO,
              "%ssetDoubleParam(%d motorMoveAbs_)=%g\n", modNamEMC, axisNo_, value);
  } else if (function == pC_->motorMoveVel_) {
    asynPrint(pC_->pasynUserController_, ASYN_TRACE_INFO,
              "%ssetDoubleParam(%d motorMoveVel_)=%g\n", modNamEMC, axisNo_, value);
  } else if (function == pC_->motorHome_) {
    asynPrint(pC_->pasynUserController_, ASYN_TRACE_INFO,
              "%ssetDoubleParam(%d motorHome__)=%g\n", modNamEMC, axisNo_, value);
  } else if (function == pC_->motorStop_) {
    asynPrint(pC_->pasynUserController_, ASYN_TRACE_INFO,
              "%ssetDoubleParam(%d motorStop_)=%g\n", modNamEMC, axisNo_, value);
  } else if (function == pC_->motorVelocity_) {
    asynPrint(pC_->pasynUserController_, ASYN_TRACE_INFO,
              "%ssetDoubleParam(%d motorVelocity_=%g\n", modNamEMC, axisNo_, value);
  } else if (function == pC_->motorVelBase_) {
    asynPrint(pC_->pasynUserController_, ASYN_TRACE_INFO,
              "%ssetDoubleParam(%d motorVelBase_)=%g\n", modNamEMC, axisNo_, value);
  } else if (function == pC_->motorAccel_) {
    asynPrint(pC_->pasynUserController_, ASYN_TRACE_INFO,
              "%ssetDoubleParam(%d motorAccel_)=%g\n", modNamEMC, axisNo_, value);
  } else if (function == pC_->motorDeferMoves_) {
    asynPrint(pC_->pasynUserController_, ASYN_TRACE_INFO,
              "%ssetDoubleParam(%d motmotorDeferMoves_=%g\n", modNamEMC, axisNo_, value);
  } else if (function == pC_->motorMoveToHome_) {
    asynPrint(pC_->pasynUserController_, ASYN_TRACE_INFO,
              "%ssetDoubleParam(%d motmotorMoveToHome_=%g\n", modNamEMC, axisNo_, value);
  } else if (function == pC_->motorResolution_) {
    asynPrint(pC_->pasynUserController_, ASYN_TRACE_INFO,
              "%ssetDoubleParam(%d motorResolution_=%g\n",  modNamEMC, axisNo_, value);
    /* Limits handled above */

#ifdef motorPowerOnDelayString
  } else if (function == pC_->motorPowerOnDelay_) {
    asynPrint(pC_->pasynUserController_, ASYN_TRACE_INFO,
              "%ssetDoubleParam(%d motorPowerOnDelay_)=%g\n", modNamEMC, axisNo_, value);
#endif
#ifdef motorPowerOffDelayString
  } else if (function == pC_->motorPowerOffDelay_) {
    asynPrint(pC_->pasynUserController_, ASYN_TRACE_INFO,
              "%ssetDoubleParam(%d motorPowerOffDelay_=%g\n", modNamEMC, axisNo_, value);
#endif
#ifdef motorPowerOffFractionString
  } else if (function == pC_->motorPowerOffFraction_) {
    asynPrint(pC_->pasynUserController_, ASYN_TRACE_INFO,
              "%ssetDoubleParam(%d motomotorPowerOffFraction_=%g\n", modNamEMC, axisNo_, value);
#endif
#ifdef motorPostMoveDelayString
  } else if (function == pC_->motorPostMoveDelay_) {
    asynPrint(pC_->pasynUserController_, ASYN_TRACE_INFO,
              "%ssetDoubleParam(%d motorPostMoveDelay_=%g\n", modNamEMC, axisNo_, value);
#endif
  } else if (function == pC_->motorStatus_) {
    asynPrint(pC_->pasynUserController_, ASYN_TRACE_INFO,
              "%ssetDoubleParam(%d motorStatus_)=%g\n", modNamEMC, axisNo_, value);
#ifdef EthercatMCHVELFRMString
  } else if (function == pC_->EthercatMCHVELfrm_) {
    asynPrint(pC_->pasynUserController_, ASYN_TRACE_INFO,
              "%ssetDoubleParam(%d HVELfrm_)=%g\n", modNamEMC, axisNo_, value);
#endif
#ifdef EthercatMCHomPosString
  } else if (function == pC_->EthercatMCHomPos_) {
    pC_->setDoubleParam(axisNo_, pC_->EthercatMCHomPos_RB_, value);
    asynPrint(pC_->pasynUserController_, ASYN_TRACE_INFO,
              "%ssetDoubleParam(%d HomPos_)=%f\n", modNamEMC, axisNo_, value);
#endif
  }
  // Call the base class method
  status = asynMotorAxis::setDoubleParam(function, value);
  return status;
}

asynStatus EthercatMCGvlAxis::setStringParamDbgStrToMcu(const char *value)
{
    const char * const Main_this_str = "Main.this.";
    const char * const Sim_this_str = "Sim.this.";
#if 0
    unsigned adsport;
    unsigned indexOffset;
    int      ivalue;
    double   fvalue;
    int nvals = 0;
    int retryCount = 1;
#endif

    asynPrint(pC_->pasynUserController_, ASYN_TRACE_INFO,
              "%ssetStringParamDbgStrToMcu(%d)=\"%s\"\n",
              modNamEMC, axisNo_, value);
    /* empty strings are not send to the controller */
    if (!value[0]) return asynSuccess;

    /* Check the string. E.g. Main.this. and Sim.this. are passed
       as Main.M1 or Sim.M1
       ADR commands are handled below */
    if (!strncmp(value, Main_this_str, strlen(Main_this_str))) {
      snprintf(pC_->outString_, sizeof(pC_->outString_), "%sMain.M%d.%s",
              drvlocal.adsport_str, axisNo_, value + strlen(Main_this_str));
      return pC_->writeReadACK(ASYN_TRACE_INFO);
    }
    /* caput IOC:m1-DbgStrToMCU Sim.this.log=M1.log */
    if (!strncmp(value, Sim_this_str, strlen(Sim_this_str))) {
      snprintf(pC_->outString_, sizeof(pC_->outString_), "Sim.M%d.%s",
              axisNo_, value + strlen(Sim_this_str));
      return pC_->writeReadACK(ASYN_TRACE_INFO);
    }
#if 0
    nvals = sscanf(value, "Sim.M%u.", &ivalue);
    if (nvals == 1) {
      snprintf(pC_->outString_, sizeof(pC_->outString_), "%s", value);
      return pC_->writeReadACK();
    }
    /* ADR commands integer
     *  # in  target position monitoring
     *  setADRinteger 501 0x4000 0x15 1
     */
    nvals = sscanf(value, "setADRinteger %u %x %x %d",
                   &indexGroup, &indexOffset, &ivalue);
    if (nvals == 3) {
      return setSAFValueOnAxisVerify(indexGroup, indexOffset,
                                     ivalue, retryCount);
    }
    /* ADR commands floating point
     *  # Target position monitoring window, mm
     *  setADRdouble  501 0x4000 0x6 0.1 */
    nvals = sscanf(value, "setADRdouble %u %x %x %lf",
                   &indexGroup, &indexOffset, &fvalue);

    if (nvals == 3) {
      return setSAFValueOnAxisVerify(indexGroup, indexOffset,
                                     fvalue, retryCount);
    }
#endif
    /* If we come here, the command was not understood */
    return asynError;
}

asynStatus EthercatMCGvlAxis::setStringParam(int function, const char *value)
{
  if (function == pC_->EthercatMCDbgStrToMcu_) {
    return setStringParamDbgStrToMcu(value);
  } else {
    /* Call base class method */
    return asynMotorAxis::setStringParam(function, value);
  }
}

#ifndef motorMessageTextString
void EthercatMCGvlAxis::updateMsgTxtFromDriver(const char *value)
{
  if (value && value[0]) {
    setStringParam(pC_->EthercatMCMCUErrMsg_,value);
  } else {
    setStringParam(pC_->EthercatMCMCUErrMsg_, "");
  }
}
#endif
