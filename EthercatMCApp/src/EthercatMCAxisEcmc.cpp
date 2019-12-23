/*
  FILENAME... EthercatMCAxis.cpp
*/

#include <stdio.h>
#include <string.h>
#include <stdlib.h>
#include <math.h>
#include <errno.h>
#include <unistd.h>

#include <epicsThread.h>

#include "motor.h"
#include "EthercatMCAxisEcmc.h"
#include "EthercatMCController.h"

#ifndef ASYN_TRACE_INFO
#define ASYN_TRACE_INFO      0x0040
#endif

#define NCOMMANDMOVEVEL  1
#define NCOMMANDMOVEREL  2
#define NCOMMANDMOVEABS  3
#define NCOMMANDHOME    10
#define HOMPROC_MANUAL_SETPOS    15

/* The maximum number of polls we wait for the motor
   to "start" (report moving after a new move command */
#define WAITNUMPOLLSBEFOREREADY 3

static EthercatMCController *pC;
static EthercatMCAxisEcmc   *axes;

/* Callback for axis data struct (binary)
*/
void diagBinCallback(void *userPvt, asynUser *pasynUser, epicsInt8 *value, size_t nelements) {

  EthercatMCAxisEcmc * axis = (EthercatMCAxisEcmc*)userPvt;
  if(!axis) {
    printf("Axis not found!!!!!!\n");
    return;
  }
  if(nelements!=sizeof(ecmcAxisStatusType)) {
    printf("Wrong byte count...ERROR!!!!!!\n");
    return;
  }
  memcpy(axis->getDiagBinDataPtr(),value,sizeof(ecmcAxisStatusType));
}

//
// These are the EthercatMCAxis methods
//

/** Creates a new EthercatMCAxis object.
 * \param[in] pC Pointer to the EthercatMCController to which this axis belongs.
 * \param[in] axisNo Index number of this axis, range 1 to pC->numAxes_. (0 is not used)
 *
 *
 * Initializes register numbers, etc.
 */
EthercatMCAxisEcmc::EthercatMCAxisEcmc(EthercatMCController *pC, int axisNo,
                               int axisFlags, const char *axisOptionsStr)
  : asynMotorAxis(pC, axisNo),
    pC_(pC)
{
  // ECMC  
  asynUserStatWd_      = NULL; // "T_SMP_MS=%d/TYPE=asynInt32/ax%d.status?"
  asynUserDiagStr_     = NULL; // "T_SMP_MS=%d/TYPE=asynInt8ArrayIn/ax%d.diagnostic?"
  asynUserDiagBin_     = NULL; // "T_SMP_MS=%d/TYPE=asynInt8ArrayIn/ax%d.diagnosticbin?"
  asynUserDiagBinIntr_ = NULL; // "T_SMP_MS=%d/TYPE=asynInt8ArrayIn/ax%d.diagnosticbin?"  
  asynUserCntrlWd_     = NULL; // "T_SMP_MS=%d/TYPE=asynInt32/ax%d.control="
  asynUserTargPos_     = NULL; // "T_SMP_MS=%d/TYPE=asynFloat64/ax%d.targetpos="
  asynUserTargVel_     = NULL; // "T_SMP_MS=%d/TYPE=asynFloat64/ax%d.targetvel="
  asynUserTargAcc_     = NULL; // "T_SMP_MS=%d/TYPE=asynFloat64/ax%d.targetacc="
  asynUserSoftLimBwd_  = NULL; // "T_SMP_MS=%d/TYPE=asynFloat64/ax%d.soflimbwd="
  asynUserSoftLimFwd_  = NULL; // "T_SMP_MS=%d/TYPE=asynFloat64/ax%d.soflimfwd="

  pasynIFDiagBinIntr_  = NULL;
  pIFDiagBinIntr_      = NULL;
  interruptDiagBinPvt_ = NULL;


  memset(&diagData_,0, sizeof(diagData_));
  memset(&diagBinData_,0, sizeof(diagBinData_));
  memset(&diagStringBuffer_[0],0,sizeof(diagStringBuffer_));
  axisId_ = axisNo;
  
  oldPositionAct_ = 0;
  //ECMC End
  
  int powerAutoOnOff = -1; /* undefined */
  /* Some parameters are only defined in the ESS fork of the motor module.
     So they have the ifdef */
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
  /* Force a printout of this 3 variables, once the controller is connected,
     they have 0 or 1 (but never -1) */
  drvlocal.old_st_axis_status.bHomed = -1;
  drvlocal.old_st_axis_status.bLimitBwd = -1;
  drvlocal.old_st_axis_status.bLimitFwd = -1;

  drvlocal.old_eeAxisError = eeAxisErrorIOCcomError;

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
#ifdef motorShowPowerOffString
    setIntegerParam(pC_->motorShowPowerOff_, 1);
#endif
#ifdef  motorNotHomedProblemString
    setIntegerParam(pC_->motorNotHomedProblem_, MOTORNOTHOMEDPROBLEM_ERROR);
#endif

  drvlocal.scaleFactor = 1.0;
  if (axisOptionsStr && axisOptionsStr[0]) {
    const char * const encoder_is_str = "encoder=";
    const char * const cfgfile_str = "cfgFile=";
    const char * const cfgDebug_str = "getDebugText=";
#ifndef motorFlagsDriverUsesEGUString
    /* The non-ESS motor needs a dummy "stepm-size" to compensate for MRES */
    const char * const stepSize_str = "stepSize=";
#endif
    const char * const homProc_str 	   = "HomProc=";
    const char * const homPos_str  	   = "HomPos=";
    const char * const adsPort_str         = "adsPort=";
    const char * const axisFlags_str       = "axisFlags=";
    const char * const powerAutoOnOff_str  = "powerAutoOnOff=";
    const char * const powerOffDelay_str   = "powerOffDelay=";
    const char * const powerOnDelay_str    = "powerOnDelay=";
    const char * const scaleFactor_str     = "scaleFactor=";

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
          drvlocal.adsPort = (unsigned)adsPort;
        }
      } else if (!strncmp(pThisOption, axisFlags_str, strlen(axisFlags_str))) {
        pThisOption += strlen(axisFlags_str);
        int myAxisFlags = atoi(pThisOption);
        if (myAxisFlags > 0) {
          axisFlags = myAxisFlags;
        }
      } else if (!strncmp(pThisOption, powerAutoOnOff_str, strlen(powerAutoOnOff_str))) {
        pThisOption += strlen(powerAutoOnOff_str);
	powerAutoOnOff = atoi(pThisOption);
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
      } else if (!strncmp(pThisOption, powerOffDelay_str, strlen(powerOffDelay_str))) {
        double powerOffDelay;
        pThisOption += strlen(powerOffDelay_str);
        powerOffDelay = atof(pThisOption);
        updateCfgValue(pC_->motorPowerOffDelay_, powerOffDelay, "powerOffDelay");
      } else if (!strncmp(pThisOption, powerOnDelay_str, strlen(powerOnDelay_str))) {
        double powerOnDelay;
        pThisOption += strlen(powerOnDelay_str);
        powerOnDelay = atof(pThisOption);
        updateCfgValue(pC_->motorPowerOnDelay_, powerOnDelay, "powerOnDelay");
      }
      pThisOption = pNextOption;
    }
    free(pOptions);
  }
  drvlocal.axisFlags = axisFlags;
  if (axisFlags & AMPLIFIER_ON_FLAG_USING_CNEN) {
    setIntegerParam(pC->motorStatusGainSupport_, 1);
  }
  if (powerAutoOnOff >= 0) {
    /* The new handling using options */
    setIntegerParam(pC_->motorPowerAutoOnOff_, powerAutoOnOff);
    /* the delays had been set up above */
  } else if (axisFlags & AMPLIFIER_ON_FLAG_AUTO_ON) {
    /* old, legacy, to support old start scripts where flags == 6 are used */
#ifdef POWERAUTOONOFFMODE2
    setIntegerParam(pC_->motorPowerAutoOnOff_, POWERAUTOONOFFMODE2);
    setDoubleParam(pC_->motorPowerOnDelay_,   6.0);
    setDoubleParam(pC_->motorPowerOffDelay_, -1.0);
#endif
  }
  /* Set the module name to "" if we have FILE/LINE enabled by asyn */
  if (pasynTrace->getTraceInfoMask(pC_->pasynUserController_) & ASYN_TRACEINFO_SOURCE) modNamEMC = "";

  asynStatus status = connectEcmcAxis();
  if (status!=asynSuccess) {
    asynPrint(pC_->pasynUserSelf, ASYN_TRACE_ERROR,
              "connectEcmcAxis() failed on port %s\n",pC_->mcuPortName_);
  }
  
  initialPoll();
}


extern "C" int EthercatMCCreateAxisEcmc(const char *EthercatMCName, int axisNo,
                                    int axisFlags, const char *axisOptionsStr)
{
  
  pC = (EthercatMCController*) findAsynPortDriver(EthercatMCName);
  if (!pC) {
    printf("Error port %s not found\n", EthercatMCName);
    return asynError;
  }
  pC->lock();
  new EthercatMCAxisEcmc(pC, axisNo, axisFlags, axisOptionsStr);
  pC->unlock();
  return asynSuccess;
  
}

asynStatus EthercatMCAxisEcmc::updateCfgValue(int function,
                                          double newValue,
                                          const char *name)
{
  double oldValue;
  asynStatus status = pC_->getDoubleParam(axisNo_, function, &oldValue);
  if (status) {
    /* First time that we write the value after IOC restart
       ECMC configures everything from the iocshell, no need to
       do a print here */
    if (!(pC_->features_ & FEATURE_BITS_ECMC)) {
      asynPrint(pC_->pasynUserController_, ASYN_TRACE_INFO,
                "%supdateCfgValue(%d) %s=%f\n",
                modNamEMC, axisNo_, name, newValue);
    }
  } else if (newValue != oldValue) {
    asynPrint(pC_->pasynUserController_, ASYN_TRACE_INFO,
              "%supdateCfgValue(%d) old%s=%f new%s=%f\n",
              modNamEMC, axisNo_, name, oldValue, name, newValue);
  }
  return pC_->setDoubleParam(axisNo_, function, newValue);
}

asynStatus EthercatMCAxisEcmc::updateCfgValue(int function,
                                          int newValue,
                                          const char *name)
{
  int oldValue;
  asynStatus status = pC_->getIntegerParam(axisNo_, function, &oldValue);
  if (status) {
    /* First time that we write the value after IOC restart
       ECMC configures everything from the iocshell, no need to
       do a print here */
    if (!((pC_->features_ & FEATURE_BITS_ECMC))) {
      asynPrint(pC_->pasynUserController_, ASYN_TRACE_INFO,
                "%supdateCfgValue(%d) %s=%d\n",
                modNamEMC, axisNo_, name, newValue);
    }
  } else if (newValue != oldValue) {
    asynPrint(pC_->pasynUserController_, ASYN_TRACE_INFO,
              "%supdateCfgValue(%d) old%s=%d new%s=%d\n",
              modNamEMC, axisNo_, name, oldValue, name, newValue);
  }
  return pC_->setIntegerParam(axisNo_, function, newValue);
}

asynStatus EthercatMCAxisEcmc::readBackSoftLimits(void)
{
  asynStatus status;
  int nvals;
  int axisID = getMotionAxisID();
  int enabledHigh = 0, enabledLow = 0;
  double fValueHigh = 0.0, fValueLow  = 0.0;
  double scaleFactor = drvlocal.scaleFactor;

  snprintf(pC_->outString_, sizeof(pC_->outString_),
           "ADSPORT=501/.ADR.16#%X,16#%X,2,2?;ADSPORT=501/.ADR.16#%X,16#%X,8,5?;"
           "ADSPORT=501/.ADR.16#%X,16#%X,2,2?;ADSPORT=501/.ADR.16#%X,16#%X,8,5?",
           0x5000 + axisID, 0xC,
           0x5000 + axisID, 0xE,
           0x5000 + axisID, 0xB,
           0x5000 + axisID, 0xD);

  status = pC_->writeReadOnErrorDisconnect();
  if (status)
    return status;
  nvals = sscanf(pC_->inString_, "%d;%lf;%d;%lf",
                 &enabledHigh, &fValueHigh, &enabledLow, &fValueLow);
  if (nvals != 4) {
    asynPrint(pC_->pasynUserController_, ASYN_TRACE_ERROR|ASYN_TRACEIO_DRIVER,
              "%snvals=%d command=\"%s\" response=\"%s\"\n",
              modNamEMC, nvals, pC_->outString_, pC_->inString_);
    enabledHigh = enabledLow = 0;
    fValueHigh = fValueLow = 0.0;
    return asynError;
  }
  asynPrint(pC_->pasynUserController_, ASYN_TRACE_FLOW,
            "%sout=%s in=%s CHLM_En=%d CHLM=%f CLLM_En=%d CLLM=%f\n",
            modNamEMC, pC_->outString_, pC_->inString_,
            enabledHigh, fValueHigh, enabledLow, fValueLow);
  /* EthercatMCCHLMXX are info(asyn:READBACK,"1"),
     so we must use pC_->setXXX(axisNo_..)  here */



  pC_->setIntegerParam(axisNo_, pC_->EthercatMCCfgDHLM_En_, enabledHigh);
  pC_->setDoubleParam(axisNo_, pC_->EthercatMCCfgDHLM_, fValueHigh);
  pC_->setIntegerParam(axisNo_, pC_->EthercatMCCfgDLLM_En_, enabledLow);
  pC_->setDoubleParam(axisNo_, pC_->EthercatMCCfgDLLM_, fValueLow);

  if (scaleFactor) {
    pC_->udateMotorLimitsRO(axisNo_, enabledHigh && enabledLow,
                            fValueHigh / scaleFactor, fValueLow / scaleFactor);
  }
  return status;
}

asynStatus EthercatMCAxisEcmc::readBackHoming(void)
{
  asynStatus status;
  int    homProc = 0;
  double homPos  = 0.0;

  /* Don't read it, when the driver has been configured with HomProc= */
  status = pC_->getIntegerParam(axisNo_, pC_->EthercatMCHomProc_,&homProc);
  if (status == asynSuccess) return status;

  status = getValueFromAxis("_EPICS_HOMPROC", &homProc);
  if (!status) setIntegerParam(pC_->EthercatMCHomProc_, homProc);
  status = getValueFromAxis("_EPICS_HOMPOS", &homPos);
  if (status) {
    /* fall back */
    status = getSAFValueFromAxisPrint(0x5000, 0x103, "homPos", &homPos);
  }
  if (!status) setDoubleParam(pC_->EthercatMCHomPos_, homPos);
  return asynSuccess;
}

asynStatus EthercatMCAxisEcmc::readScaling(int axisID)
{
  int nvals;
  asynStatus status;
  double srev = 0, urev = 0;

  double scaleFactor = drvlocal.scaleFactor;

  if (!scaleFactor) return asynError;
  snprintf(pC_->outString_, sizeof(pC_->outString_),
           "ADSPORT=501/.ADR.16#%X,16#%X,8,5?;"
           "ADSPORT=501/.ADR.16#%X,16#%X,8,5?;",
           0x5000 + axisID, 0x24,  // SREV
           0x5000 + axisID, 0x23   // UREV
           );
  status = pC_->writeReadOnErrorDisconnect();
  if (status) return status;
  nvals = sscanf(pC_->inString_, "%lf;%lf",
                 &srev, &urev);

  if (nvals != 2) {
    asynPrint(pC_->pasynUserController_, ASYN_TRACE_ERROR|ASYN_TRACEIO_DRIVER,
              "%snvals=%d\n", modNamEMC, nvals);
    return asynError;
  }
  updateCfgValue(pC_->EthercatMCCfgSREV_RB_, srev, "srev");
  updateCfgValue(pC_->EthercatMCCfgUREV_RB_, urev, "urev");
  return asynSuccess;
}

asynStatus EthercatMCAxisEcmc::readMonitoring(int axisID)
{
  int nvals;
  asynStatus status;
  double rdbd, rdbd_tim, poslag = -1, poslag_tim = -1;
  int rdbd_en, poslag_en = 0;
  double scaleFactor = drvlocal.scaleFactor;

  if (!scaleFactor) return asynError;
  snprintf(pC_->outString_, sizeof(pC_->outString_),
           "ADSPORT=501/.ADR.16#%X,16#%X,8,5?;"
           "ADSPORT=501/.ADR.16#%X,16#%X,8,5?;"
           "ADSPORT=501/.ADR.16#%X,16#%X,2,2?;"
           "ADSPORT=501/.ADR.16#%X,16#%X,8,5?;"
           "ADSPORT=501/.ADR.16#%X,16#%X,8,5?;"
           "ADSPORT=501/.ADR.16#%X,16#%X,2,2?",
           0x4000 + axisID, 0x16,  // RDBD_RB
           0x4000 + axisID, 0x17,  // RDBD_Tim
           0x4000 + axisID, 0x15,  // RDBD_En
           0x6000 + axisID, 0x12,  // PosLag
           0x6000 + axisID, 0x13,  // PosLag_Tim
           0x6000 + axisID, 0x10); // Poslag_En
  status = pC_->writeReadOnErrorDisconnect();
  if (status) return status;
  nvals = sscanf(pC_->inString_, "%lf;%lf;%d;%lf;%lf;%d",
                 &rdbd, &rdbd_tim, &rdbd_en, &poslag, &poslag_tim, &poslag_en
                 );
  if ((nvals != 6) && (nvals != 3)) {
    asynPrint(pC_->pasynUserController_, ASYN_TRACE_ERROR|ASYN_TRACEIO_DRIVER,
              "%sreadMonitoring(%d) nvals=%d out=%s in=%s \n",
              modNamEMC, axisNo_, nvals, pC_->outString_, pC_->inString_);
    return asynError;
  }
  updateCfgValue(pC_->EthercatMCCfgSPDB_RB_, rdbd, "spbd");
  updateCfgValue(pC_->EthercatMCCfgRDBD_RB_, rdbd, "rdbd");
  updateCfgValue(pC_->EthercatMCCfgRDBD_Tim_RB_, rdbd_tim , "rdbd_time");
  updateCfgValue(pC_->EthercatMCCfgRDBD_En_RB_, rdbd_en, "rdbd_en");

  drvlocal.illegalInTargetWindow = (!rdbd_en || !rdbd);

  if (nvals == 6) {
    updateCfgValue(pC_->EthercatMCCfgPOSLAG_RB_, poslag, "poslag");
    updateCfgValue(pC_->EthercatMCCfgPOSLAG_Tim_RB_, poslag_tim, "poslag_tim");
    updateCfgValue(pC_->EthercatMCCfgPOSLAG_En_RB_, poslag_en, "poslag_en");
  }
  return asynSuccess;
}


asynStatus EthercatMCAxisEcmc::readBackVelocities(int axisID)
{
  asynStatus status;
  int nvals;
  double scaleFactor = drvlocal.scaleFactor;
  double velo, vmax, jvel, accs;
  snprintf(pC_->outString_, sizeof(pC_->outString_),
           "ADSPORT=501/.ADR.16#%X,16#%X,8,5?;"
           "ADSPORT=501/.ADR.16#%X,16#%X,8,5?;"
           "ADSPORT=501/.ADR.16#%X,16#%X,8,5?;"
           "ADSPORT=501/.ADR.16#%X,16#%X,8,5?;",
           0x4000 + axisID, 0x9,   // VELO"
           0x4000 + axisID, 0x27,  // VMAX"
           0x4000 + axisID, 0x8,   // JVEL
           0x4000 + axisID, 0x101  // ACCS"
           );
  status = pC_->writeReadOnErrorDisconnect();
  if (status) return status;
  nvals = sscanf(pC_->inString_, "%lf;%lf;%lf;%lf",
                 &velo, &vmax, &jvel, &accs);
  if (nvals != 4) {
    asynPrint(pC_->pasynUserController_, ASYN_TRACE_ERROR|ASYN_TRACEIO_DRIVER,
              "%snvals=%d command=\"%s\" response=\"%s\"\n",
              modNamEMC, nvals, pC_->outString_, pC_->inString_);
    return asynError;
  }
  asynPrint(pC_->pasynUserController_, ASYN_TRACE_FLOW,
            "%svelo=%f vmax=%f jvel=%f accs=%f\n",
            modNamEMC, velo, vmax, jvel, accs);
  if (velo > 0.0) {
    updateCfgValue(pC_->EthercatMCCfgVELO_, velo / scaleFactor, "velo");
  }
  if (vmax > 0.0) {
    updateCfgValue(pC_->EthercatMCCfgVMAX_, vmax / scaleFactor, "vmax");
  }
  if (jvel > 0.0) {
    updateCfgValue(pC_->EthercatMCCfgJVEL_, jvel / scaleFactor, "jvel");
  }
  if (accs > 0.0) {
    updateCfgValue(pC_->EthercatMCCfgACCS_, accs / scaleFactor, "accs");
  }
  return asynSuccess;
}

asynStatus EthercatMCAxisEcmc::readBackEncoders(int axisID)
{
  /* https://infosys.beckhoff.com/english.php?content=../content/1033/tcadsdevicenc2/index.html&id= */
  asynStatus status;
  uint32_t numEncoders = 0;
  int nvals;

  snprintf(pC_->outString_, sizeof(pC_->outString_),
           "ADSPORT=501/.ADR.16#%X,16#%X,8,19?;",
           0x4000 + axisID, 0x57
           );
  status = pC_->writeReadOnErrorDisconnect();
  if (status) return status;
  nvals = sscanf(pC_->inString_, "%u",
                 &numEncoders);
  if (nvals != 1) {
    asynPrint(pC_->pasynUserController_, ASYN_TRACE_ERROR|ASYN_TRACEIO_DRIVER,
              "%snvals=%d command=\"%s\" response=\"%s\"\n",
              modNamEMC, nvals, pC_->outString_, pC_->inString_);
    return asynError;
  }
  asynPrint(pC_->pasynUserController_, ASYN_TRACE_INFO,
            "%sreadBackEncoders(%d) numEncoders=%u\n",
            modNamEMC, axisNo_, numEncoders);
  return asynSuccess;
}

asynStatus EthercatMCAxisEcmc::initialPoll(void)
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


asynStatus EthercatMCAxisEcmc::readBackAllConfig(int axisID)
{
  asynStatus status = asynSuccess;
  /* for ECMC homing is configured from EPICS, do NOT do the readback */
  if (!(pC_->features_ & FEATURE_BITS_ECMC)) {
    if (!drvlocal.scaleFactor) {
      asynPrint(pC_->pasynUserController_, ASYN_TRACE_INFO,
                "%sreadBackAllConfig(%d) drvlocal.scaleFactor=0.0\n",
                modNamEMC, axisNo_);
      return asynError;
    }
    if (status == asynSuccess) status = readBackHoming();
  }
  if (status == asynSuccess) status = readScaling(axisID);
  if (status == asynSuccess) status = readMonitoring(axisID);
  if (status == asynSuccess) status = readBackSoftLimits();
  if (status == asynSuccess) status = readBackVelocities(axisID);
  if (!(pC_->features_ & FEATURE_BITS_ECMC)) {
    if (status == asynSuccess) status = readBackEncoders(axisID);
  }
  return status;
}


/** Connection status is changed, the dirty bits must be set and
 *  the values in the controller must be updated
 * \param[in] AsynStatus status
 *
 * Sets the dirty bits
 */
asynStatus EthercatMCAxisEcmc::initialPollInternal(void)
{
  asynStatus status = asynSuccess;

  /*  Check for Axis ID */
  int axisID = getMotionAxisID();
  switch (axisID) {
  case -2:
    updateMsgTxtFromDriver("No AxisID");
    return asynSuccess;
  case -1:
    setIntegerParam(pC_->motorStatusCommsError_, 1);
    return asynError;
  case 0:
    return asynSuccess;
  default:
    if (axisID != axisNo_) {
      updateMsgTxtFromDriver("ConfigError AxisID");
      return asynError;
    }
  }
  status = readConfigFile();
  if (status) {
    asynPrint(pC_->pasynUserController_, ASYN_TRACE_ERROR|ASYN_TRACEIO_DRIVER,
              "%s(%d) readConfigFile() failed\n",
              modNamEMC, axisNo_);
    updateMsgTxtFromDriver("ConfigError Config File");
    return status;
  }
  if (drvlocal.axisFlags & AMPLIFIER_ON_FLAG_CREATE_AXIS) {
    /* Enable the amplifier when the axis is created,
       but wait until we have a connection to the controller.
       After we lost the connection, Re-enable the amplifier
       See AMPLIFIER_ON_FLAG */
    status = enableAmplifier(1);
  }
  if (status == asynSuccess) status = readBackAllConfig(axisID);
  if (status == asynSuccess && drvlocal.dirty.oldStatusDisconnected) {
    asynPrint(pC_->pasynUserController_, ASYN_TRACE_ERROR|ASYN_TRACEIO_DRIVER,
              "%sconnected(%d)\n", modNamEMC, axisNo_);
    drvlocal.dirty.oldStatusDisconnected = 0;
  }
  return status;
}

/** Reports on status of the axis
 * \param[in] fp The file pointer on which report information will be written
 * \param[in] level The level of report detail desired
 *
 * After printing device-specific information calls asynMotorAxis::report()
 */
void EthercatMCAxisEcmc::report(FILE *fp, int level)
{
  if (level > 0) {
    fprintf(fp, "  axis %d\n", axisNo_);
  }

  // Call the base class method
  asynMotorAxis::report(fp, level);
}


/** Set velocity and acceleration for the axis
 * \param[in] maxVelocity, mm/sec
 * \param[in] acceleration ???
 *
 */
asynStatus EthercatMCAxisEcmc::sendVelocityAndAccelExecute(double maxVeloEGU, double accEGU)
{
  asynStatus status;
  /* We don't use minVelocity */
  status = setValuesOnAxis("fAcceleration", accEGU,
                           "fDeceleration", accEGU);
  if (status) return status;
  status = setValueOnAxis("fVelocity", maxVeloEGU);
  if (status == asynSuccess) status = setValueOnAxis("bExecute", 1);
#ifndef motorWaitPollsBeforeReadyString
  drvlocal.waitNumPollsBeforeReady += WAITNUMPOLLSBEFOREREADY;
#endif
  return status;
}

/** Move the motor to an absolute location or by a relative amount.
 * \param[in] posEGU  The absolute position to move to (if relative=0) or the relative distance to move
 * by (if relative=1). Units=steps.
 * \param[in] relative  Flag indicating relative move (1) or absolute move (0).
 * \param[in] maxVeloEGU The maximum velocity, often called the slew velocity. Units=EGU/sec.
 * \param[in] accEGU The acceleration value. Units=EGU/sec/sec. */
asynStatus EthercatMCAxisEcmc::mov2(double posEGU, int nCommand, double maxVeloEGU, double accEGU)
{
  if (accEGU) {
    snprintf(pC_->outString_, sizeof(pC_->outString_),
             "%sMain.M%d.bExecute=0;"
             "%sMain.M%d.nCommand=%d;"
             "%sMain.M%d.nCmdData=0;"
             "%sMain.M%d.fPosition=%f;"
             "%sMain.M%d.fAcceleration=%f;"
             "%sMain.M%d.fDeceleration=%f;"
             "%sMain.M%d.fVelocity=%f;"
             "%sMain.M%d.bExecute=1",
             drvlocal.adsport_str, axisNo_,
             drvlocal.adsport_str, axisNo_, nCommand,
             drvlocal.adsport_str, axisNo_,
             drvlocal.adsport_str, axisNo_, posEGU,
             drvlocal.adsport_str, axisNo_, accEGU,
             drvlocal.adsport_str, axisNo_, accEGU,
             drvlocal.adsport_str, axisNo_, maxVeloEGU,
             drvlocal.adsport_str, axisNo_);
  } else {
    snprintf(pC_->outString_, sizeof(pC_->outString_),
             "%sMain.M%d.bExecute=0;"
             "%sMain.M%d.nCommand=%d;"
             "%sMain.M%d.nCmdData=0;"
             "%sMain.M%d.fPosition=%f;"
             "%sMain.M%d.fVelocity=%f;"
             "%sMain.M%d.bExecute=1",
             drvlocal.adsport_str, axisNo_,
             drvlocal.adsport_str, axisNo_, nCommand,
             drvlocal.adsport_str, axisNo_,
             drvlocal.adsport_str, axisNo_, posEGU,
             drvlocal.adsport_str, axisNo_, maxVeloEGU,
             drvlocal.adsport_str, axisNo_);
  }
#ifndef motorWaitPollsBeforeReadyString
  drvlocal.waitNumPollsBeforeReady += WAITNUMPOLLSBEFOREREADY;
#endif
  return pC_->writeReadACK(ASYN_TRACE_INFO);
}

/** Move the axis to a position, either absolute or relative
 * \param[in] position in steps
 * \param[in] relative (0=absolute, otherwise relative)
 * \param[in] minimum velocity, steps/sec
 * \param[in] maximum velocity, steps/sec
 * \param[in] acceleration,  steps/sec/sec
 *
 */
asynStatus EthercatMCAxisEcmc::move(double position, int relative, double minVelocity, double maxVelocity, double acceleration)
{
  asynStatus status = asynSuccess;
  asynPrint(pC_->pasynUserController_, ASYN_TRACE_FLOW,
            "%smove(%d) position=%f relative=%d maxVelocity=%f acceleration=%f\n",
            modNamEMC, axisNo_,
            position, relative, maxVelocity, acceleration);

  drvlocal.eeAxisWarning = eeAxisWarningNoWarning;
  /* Do range check */
  if (!drvlocal.scaleFactor) {
    drvlocal.eeAxisWarning = eeAxisWarningCfgZero;
    return asynSuccess;
  } else if (!maxVelocity) {
    drvlocal.eeAxisWarning = eeAxisWarningVeloZero;
    return asynSuccess;
  }

#if MAX_CONTROLLER_STRING_SIZE > 350
  return mov2(position * drvlocal.scaleFactor,
              relative ? NCOMMANDMOVEREL : NCOMMANDMOVEABS,
              maxVelocity * drvlocal.scaleFactor,
              acceleration * drvlocal.scaleFactor);
#else
  int nCommand = relative ? NCOMMANDMOVEREL : NCOMMANDMOVEABS;
  if (status == asynSuccess) status = stopAxisInternal(__FUNCTION__, 0);
  if (status == asynSuccess) status = setValueOnAxis("nCommand", nCommand);
  if (status == asynSuccess) status = setValueOnAxis("nCmdData", 0);
  if (status == asynSuccess) status = setValueOnAxis("fPosition", position * drvlocal.scaleFactor);
  if (status == asynSuccess) status = sendVelocityAndAccelExecute(maxVelocity * drvlocal.scaleFactor,
                                                                  acceleration * drvlocal.scaleFactor);
#endif
  return status;
}


/** Home the motor, search the home position
 * \param[in] minimum velocity, mm/sec
 * \param[in] maximum velocity, mm/sec
 * \param[in] acceleration, seconds to maximum velocity
 * \param[in] forwards (0=backwards, otherwise forwards)
 *
 */
asynStatus EthercatMCAxisEcmc::home(double minVelocity, double maxVelocity, double acceleration, int forwards)
{
  asynStatus status = asynSuccess;
  int nCommand = NCOMMANDHOME;

  int homProc = -1;
  double homPos = 0.0;

  drvlocal.eeAxisWarning = eeAxisWarningNoWarning;
  /* The homPos may be undefined, then use 0.0 */
  (void)pC_->getDoubleParam(axisNo_, pC_->EthercatMCHomPos_, &homPos);
  status = pC_->getIntegerParam(axisNo_, pC_->EthercatMCHomProc_,&homProc);
  if (homProc == HOMPROC_MANUAL_SETPOS)
    return asynError;
  /* The controller will do the home search, and change its internal
     raw value to what we specified in fPosition. */
  if (pC_->features_ & FEATURE_BITS_ECMC) {
    double velToHom;
    double velFrmHom;
    double accHom;
    if (!status) status = stopAxisInternal(__FUNCTION__, 0);
    if (!status) status = setValueOnAxis("fHomePosition", homPos);
    if (!status) status = pC_->getDoubleParam(axisNo_,
                                              pC_->EthercatMCVelToHom_,
                                              &velToHom);
    if (!status) status = pC_->getDoubleParam(axisNo_,
                                              pC_->EthercatMCVelFrmHom_,
                                              &velFrmHom);
    if (!status) status = pC_->getDoubleParam(axisNo_,
                                              pC_->EthercatMCAccHom_,
                                              &accHom);
    if (!status) status = setSAFValueOnAxis(0x4000, 0x6,
                                            velToHom);
    if (!status) status = setSAFValueOnAxis(0x4000, 0x7,
                                            velFrmHom);
    if (!status)  status = setValuesOnAxis("fAcceleration", accHom,
                                           "fDeceleration", accHom);
    if (!status) status = setValueOnAxis("nCommand", nCommand );
    if (!status) status = setValueOnAxis("nCmdData", homProc);
    if (!status) status = setValueOnAxis("bExecute", 1);
  } else {
    snprintf(pC_->outString_, sizeof(pC_->outString_),
             "%sMain.M%d.bExecute=0;"
             "%sMain.M%d.nCommand=%d;"
             "%sMain.M%d.nCmdData=%d;"
             "%sMain.M%d.fHomePosition=%f;"
             "%sMain.M%d.bExecute=1",
             drvlocal.adsport_str, axisNo_,
             drvlocal.adsport_str, axisNo_, nCommand,
             drvlocal.adsport_str, axisNo_, homProc,
             drvlocal.adsport_str, axisNo_, homPos,
             drvlocal.adsport_str, axisNo_);
    return pC_->writeReadACK(ASYN_TRACE_INFO);
  }
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
asynStatus EthercatMCAxisEcmc::moveVelocity(double minVelocity, double maxVelocity, double acceleration)
{
  drvlocal.eeAxisWarning = eeAxisWarningNoWarning;
  /* Do range check */
  if (!drvlocal.scaleFactor) {
    drvlocal.eeAxisWarning = eeAxisWarningCfgZero;
    return asynSuccess;
  } else if (!maxVelocity) {
    drvlocal.eeAxisWarning = eeAxisWarningVeloZero;
    return asynSuccess;
  }

#if MAX_CONTROLLER_STRING_SIZE > 350
  {
    double maxVeloEGU = maxVelocity * drvlocal.scaleFactor;
    double acc_in_EGU_sec2 = 0.0;
    if (acceleration > 0.0001) {
      double acc_in_seconds = maxVelocity / acceleration;
      acc_in_EGU_sec2 = maxVeloEGU / acc_in_seconds;
    }
    if (acc_in_EGU_sec2  < 0) acc_in_EGU_sec2 = 0 - acc_in_EGU_sec2 ;
    return mov2(0, NCOMMANDMOVEVEL, maxVeloEGU, acc_in_EGU_sec2);
  }
#else
  asynStatus status = asynSuccess;

  if (status == asynSuccess) status = stopAxisInternal(__FUNCTION__, 0);
  if (status == asynSuccess) setValueOnAxis("nCommand", NCOMMANDMOVEVEL);
  if (status == asynSuccess) status = setValueOnAxis("nCmdData", 0);
  if (status == asynSuccess) status = sendVelocityAndAccelExecute(maxVelocity * drvlocal.scaleFactor,
                                                                  acceleration * drvlocal.scaleFactor);

  return status;
#endif
}



/**
 * See asynMotorAxis::setPosition
 */
asynStatus EthercatMCAxisEcmc::setPosition(double value)
{
  asynStatus status = asynSuccess;
  int nCommand = NCOMMANDHOME;
  int homProc = 0;
  double homPos = value;

  status = pC_->getIntegerParam(axisNo_,
                                pC_->EthercatMCHomProc_,
                                &homProc);
  asynPrint(pC_->pasynUserController_, ASYN_TRACE_INFO,
            "%ssetPosition(%d  homProc=%d position=%g egu=%g\n",
            modNamEMC, axisNo_,  homProc, value, value * drvlocal.scaleFactor );

  if (homProc != HOMPROC_MANUAL_SETPOS)
    return asynError;
  if (status == asynSuccess) status = stopAxisInternal(__FUNCTION__, 0);
  if (status == asynSuccess) status = setValueOnAxis("fHomePosition", homPos);
  if (status == asynSuccess) status = setValueOnAxis("nCommand", nCommand );
  if (status == asynSuccess) status = setValueOnAxis("nCmdData", homProc);
  if (status == asynSuccess) status = setValueOnAxis("bExecute", 1);

  return status;
}

asynStatus EthercatMCAxisEcmc::resetAxis(void)
{
  asynStatus status = asynSuccess;
  int EthercatMCErr;
  bool moving;
  /* Reset command error, if any */
  drvlocal.eeAxisWarning = eeAxisWarningNoWarning;
  drvlocal.cmdErrorMessage[0] = 0;
  status = pC_->getIntegerParam(axisNo_, pC_->EthercatMCErr_, &EthercatMCErr);
  asynPrint(pC_->pasynUserController_, ASYN_TRACE_INFO,
            "%sresetAxis(%d status=%d EthercatMCErr)=%d\n",
            modNamEMC, axisNo_, (int)status, EthercatMCErr);

  if (EthercatMCErr) {
    /* Soft reset of the axis */
    status = setValueOnAxis("bExecute", 0);
    if (status) goto resetAxisReturn;
    status = setValueOnAxisVerify("bReset", "bReset", 1, 20);
    if (status) goto resetAxisReturn;
    epicsThreadSleep(.1);
    status = setValueOnAxisVerify("bReset", "bReset", 0, 20);
  }
 resetAxisReturn:
  asynPrint(pC_->pasynUserController_, ASYN_TRACE_INFO,
            "%sresetAxis(%d) status=%s (%d)\n",
            modNamEMC, axisNo_, EthercatMCstrStatus(status), (int)status);
  /* do a poll */
  poll(&moving);
  return status;
}

/*bool EthercatMCAxisEcmc::pollPowerIsOn(void)
{
  int ret = 0;
  asynStatus status = getValueFromAxis(".bEnabled", &ret);
  if (!status && ret)
    return true;
  else
    return false;
}*/

/** Enable the amplifier on an axis
 *
 */
asynStatus EthercatMCAxisEcmc::enableAmplifier(int on)
{
  asynStatus status = asynSuccess;
  unsigned counter = 10;
  bool moving;
  int ret;
  const char *enableEnabledReadback = "bEnabled";

#ifdef POWERAUTOONOFFMODE2
  {
    int autoPower;
    pC_->getIntegerParam(axisNo_, pC_->motorPowerAutoOnOff_, &autoPower);
    if (autoPower) {
      /* The record/driver will check for enabled - don't do that here */
      enableEnabledReadback = "bEnable";
    }
  }
#endif
  on = on ? 1 : 0; /* either 0 or 1 */
  status = getValueFromAxis(".bEnabled", &ret);
  /* Either it went wrong OR the amplifier IS as it should be */
  if (status || (ret == on)) return status;
  if (!on) {
    /* Amplifier is on and should be turned off.
       Stop the axis by setting bEnable to 0 */
    status = stopAxisInternal(__FUNCTION__, 0);
    if (status) return status;
  }
  status = setValueOnAxis("bEnable", on);
  if (status || !on) return status; /* this went wrong OR it should be turned off */
  while (counter) {
    epicsThreadSleep(.1);
    snprintf(pC_->outString_, sizeof(pC_->outString_),
             "%sMain.M%d.%s?;%sMain.M%d.%s?",
             drvlocal.adsport_str, axisNo_, "bBusy",
             drvlocal.adsport_str, axisNo_, enableEnabledReadback);
    status = pC_->writeReadOnErrorDisconnect();
    asynPrint(pC_->pasynUserController_, ASYN_TRACE_INFO,
              "%sout=%s in=%s status=%s (%d)\n",
              modNamEMC, pC_->outString_, pC_->inString_,
              EthercatMCstrStatus(status), (int)status);
    if (status) return status;
    if (!strcmp("0;1", pC_->inString_)) {
      /* bBusy == 0; bEnable(d) == 1 */
      goto enableAmplifierPollAndReturn;
    } else if (!strcmp("1;1", pC_->inString_)) {
      /* bBusy=1 is OK */
      goto enableAmplifierPollAndReturn;
    }
    counter--;
  }
  /* if we come here, it went wrong */
  if (!drvlocal.cmdErrorMessage[0]) {
    snprintf(drvlocal.cmdErrorMessage, sizeof(drvlocal.cmdErrorMessage)-1,
             "E: enableAmplifier(%d) failed. out=%s in=%s\n",
             axisNo_, pC_->outString_, pC_->inString_);
    /* The poller co-ordinates the writing into the parameter library */
  }
enableAmplifierPollAndReturn:
  poll(&moving);
  return status;

}

/** Stop the axis
 *
 */
asynStatus EthercatMCAxisEcmc::stopAxisInternal(const char *function_name, double acceleration)
{
  asynStatus status = asynError;
  asynPrint(pC_->pasynUserController_, ASYN_TRACE_INFO,
            "%sstopAxisInternal(%d) (%s)\n", modNamEMC, axisNo_, function_name);
  status = setValueOnAxisVerify("bExecute", "bExecute", 0, 1);
  return status;
}

/** Stop the axis, called by motor Record
 *
 */
asynStatus EthercatMCAxisEcmc::stop(double acceleration )
{
  drvlocal.eeAxisWarning = eeAxisWarningNoWarning;
  return stopAxisInternal(__FUNCTION__, acceleration);
}

void EthercatMCAxisEcmc::callParamCallbacksUpdateError()
{
  const char *msgTxtFromDriver = NULL;
  int EPICS_nErrorId = drvlocal.MCU_nErrorId;
  drvlocal.eeAxisError = eeAxisErrorNoError;
  if (drvlocal.supported.statusVer == -1) {
    drvlocal.eeAxisError = eeAxisErrorNotFound;
    msgTxtFromDriver = "Not found";
  } else if (EPICS_nErrorId) {
    /* Error from MCU */
    drvlocal.eeAxisError = eeAxisErrorMCUError;
    msgTxtFromDriver = &drvlocal.sErrorMessage[0];
  } else if (drvlocal.dirty.sErrorMessage) {
    /* print error below */
    drvlocal.eeAxisError = eeAxisErrorIOCcomError;
  } else if (drvlocal.cmdErrorMessage[0]) {
    drvlocal.eeAxisError = eeAxisErrorCmdError;
    msgTxtFromDriver = &drvlocal.cmdErrorMessage[0];
  } else if (!drvlocal.homed &&
             (drvlocal.nCommandActive != NCOMMANDHOME)) {
    drvlocal.eeAxisError = eeAxisErrorNotHomed;
  } else if (drvlocal.illegalInTargetWindow) {
    drvlocal.eeAxisError = eeAxisIllegalInTargetWindow;
    msgTxtFromDriver = "E: InTargetPosWin";
  }
  if (drvlocal.eeAxisError != drvlocal.old_eeAxisError ||
      drvlocal.eeAxisWarning != drvlocal.old_eeAxisWarning ||
      drvlocal.old_EPICS_nErrorId != EPICS_nErrorId ||
      drvlocal.old_nCommandActive != drvlocal.nCommandActive) {

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
        msgTxtFromDriver = "Speed Limit";
        break;
      case eeAxisWarningNoWarning:
        break;
      }
    }
    /* No warning to show yet */
    if (!msgTxtFromDriver) {
      switch(drvlocal.nCommandActive) {
#ifdef motorLatestCommandString
      case NCOMMANDMOVEVEL:
        setIntegerParam(pC_->motorLatestCommand_, LATEST_COMMAND_MOVE_VEL);
        break;
      case NCOMMANDMOVEREL:
        setIntegerParam(pC_->motorLatestCommand_, LATEST_COMMAND_MOVE_REL);
        break;
      case NCOMMANDMOVEABS:
        setIntegerParam(pC_->motorLatestCommand_, LATEST_COMMAND_MOVE_ABS);
        break;
      case NCOMMANDHOME:
        setIntegerParam(pC_->motorLatestCommand_, LATEST_COMMAND_HOMING);
        break;
#endif
      case 0:
        break;
      default:
        msgTxtFromDriver = "Moving";
      }
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
              " Error=%d old=%d ErrID=0x%x old=0x%x Warn=%d nCmd=%d old=%d txt=%s\n",
              modNamEMC, axisNo_, drvlocal.eeAxisError, drvlocal.old_eeAxisError,
              EPICS_nErrorId, drvlocal.old_EPICS_nErrorId,
              drvlocal.eeAxisWarning,
              drvlocal.nCommandActive, drvlocal.old_nCommandActive,
              msgTxtFromDriver ? msgTxtFromDriver : "NULL");

    if (!drvlocal.cfgDebug_str) {
      updateMsgTxtFromDriver(msgTxtFromDriver);
    }
    drvlocal.old_eeAxisError = drvlocal.eeAxisError;
    drvlocal.old_eeAxisWarning = drvlocal.eeAxisWarning;
    drvlocal.old_EPICS_nErrorId = EPICS_nErrorId;
    drvlocal.old_nCommandActive = drvlocal.nCommandActive;
  }

  callParamCallbacks();
}

// asynStatus EthercatMCAxisEcmc::pollAll(bool *moving, st_axis_status_type *pst_axis_status)
// {
//   asynStatus comStatus;

//   int motor_axis_no = 0;
//   int nvals = 0;
//   const char * const Main_dot_str = "Main.";
//   const size_t       Main_dot_len = strlen(Main_dot_str);
//   struct {
//     double velocitySetpoint;
//     double fDecceleration;
//     int cycleCounter;
//     unsigned int EtherCATtime_low32;
//     unsigned int EtherCATtime_high32;
//     int command;
//     int cmdData;
//     int reset;
//     int moving;
//     int stall;
//   } notUsed;
//   if (drvlocal.dirty.initialPollNeeded) {
//     comStatus = initialPoll();
//     if (comStatus) return comStatus;
//   }

//   if (pC_->features_ & FEATURE_BITS_V2) {
//     /* V2 is supported, use it. */
//     snprintf(pC_->outString_, sizeof(pC_->outString_),
//              "%sMain.M%d.stAxisStatusV2?", drvlocal.adsport_str, axisNo_);
//     comStatus = pC_->writeReadOnErrorDisconnect();
//     if (!strncasecmp(pC_->inString_,  Main_dot_str, Main_dot_len)) {
//       nvals = sscanf(&pC_->inString_[Main_dot_len],
//                      "M%d.stAxisStatusV2="
//                      "%lf,%lf,%lf,%lf,%lf,%lf,%lf,%d,%d,%d,%d,%d,%d,%d,%d,%d,%d,%d,%d,%d,%d,%d,%d,%d,%d,%d",
//                      &motor_axis_no,
//                      &pst_axis_status->fPosition,
//                      &pst_axis_status->fActPosition,
//                      &pst_axis_status->encoderRaw,          /* Send as uint64; parsed as double ! */
//                      &notUsed.velocitySetpoint,
//                      &pst_axis_status->fActVelocity,
//                      &pst_axis_status->fAcceleration,
//                      &notUsed.fDecceleration,
//                      &notUsed.cycleCounter,
//                      &notUsed.EtherCATtime_low32,
//                      &notUsed.EtherCATtime_high32,
//                      &pst_axis_status->bEnable,
//                      &pst_axis_status->bEnabled,
//                      &pst_axis_status->bExecute,
//                      &notUsed.command,
//                      &notUsed.cmdData,
//                      &pst_axis_status->bLimitBwd,
//                      &pst_axis_status->bLimitFwd,
//                      &pst_axis_status->bHomeSensor,
//                      &pst_axis_status->bError,
//                      &pst_axis_status->nErrorId,
//                      &notUsed.reset,
//                      &pst_axis_status->bHomed,
//                      &pst_axis_status->bBusy,
//                      &pst_axis_status->atTarget,
//                      &notUsed.moving,
//                      &notUsed.stall);
//     }
//     if (nvals == 27) {
//       pst_axis_status->mvnNRdyNex = pst_axis_status->bBusy || !pst_axis_status->atTarget;
//     }
//   } else if (pC_->features_ & FEATURE_BITS_V1) {
//     /* Read the complete Axis status */
//     snprintf(pC_->outString_, sizeof(pC_->outString_),
//              "%sMain.M%d.stAxisStatus?", drvlocal.adsport_str, axisNo_);
//     comStatus = pC_->writeReadOnErrorDisconnect();
//     if (comStatus) return comStatus;
//     if (!strncasecmp(pC_->inString_,  Main_dot_str, Main_dot_len)) {
//       nvals = sscanf(&pC_->inString_[Main_dot_len],
//                      "M%d.stAxisStatus="
//                      "%d,%d,%d,%u,%u,%lf,%lf,%lf,%lf,%d,"
//                      "%d,%d,%d,%lf,%d,%d,%d,%u,%lf,%lf,%lf,%d,%d",
//                      &motor_axis_no,
//                      &pst_axis_status->bEnable,        /*  1 */
//                      &pst_axis_status->bReset,         /*  2 */
//                      &pst_axis_status->bExecute,       /*  3 */
//                      &pst_axis_status->nCommand,       /*  4 */
//                      &pst_axis_status->nCmdData,       /*  5 */
//                      &pst_axis_status->fVelocity,      /*  6 */
//                      &pst_axis_status->fPosition,      /*  7 */
//                      &pst_axis_status->fAcceleration,  /*  8 */
//                      &pst_axis_status->fDecceleration, /*  9 */
//                      &pst_axis_status->bJogFwd,        /* 10 */
//                      &pst_axis_status->bJogBwd,        /* 11 */
//                      &pst_axis_status->bLimitFwd,      /* 12 */
//                      &pst_axis_status->bLimitBwd,      /* 13 */
//                      &pst_axis_status->fOverride,      /* 14 */
//                      &pst_axis_status->bHomeSensor,    /* 15 */
//                      &pst_axis_status->bEnabled,       /* 16 */
//                      &pst_axis_status->bError,         /* 17 */
//                      &pst_axis_status->nErrorId,       /* 18 */
//                      &pst_axis_status->fActVelocity,   /* 19 */
//                      &pst_axis_status->fActPosition,   /* 20 */
//                      &pst_axis_status->fActDiff,       /* 21 */
//                      &pst_axis_status->bHomed,         /* 22 */
//                      &pst_axis_status->bBusy           /* 23 */);
//     }
//     if (nvals != 24) {
//       goto pollAllWrongnvals;
//     }

//     /* V1 new style: mvnNRdyNex follows bBusy */
//     if (pC_->features_ & (FEATURE_BITS_ECMC | FEATURE_BITS_SIM))
//       drvlocal.supported.bV1BusyNewStyle = 1;

//     pst_axis_status->mvnNRdyNex = pst_axis_status->bBusy && pst_axis_status->bEnabled;
//     if (!drvlocal.supported.bV1BusyNewStyle) {
//       /* "V1 old style":done when bEcecute is 0 */
//       pst_axis_status->mvnNRdyNex &= pst_axis_status->bExecute;
//     }
//   } /* End of V1 */
//   /* From here on, either V1 or V2 is supported */
//   if (drvlocal.dirty.statusVer) {
//     if (pC_->features_ & FEATURE_BITS_V2)
//       drvlocal.supported.statusVer = 2;
//     else if ((pC_->features_ & FEATURE_BITS_V1) && !drvlocal.supported.bV1BusyNewStyle)
//       drvlocal.supported.statusVer = 0;
//     else if ((pC_->features_ & FEATURE_BITS_V1) && drvlocal.supported.bV1BusyNewStyle)
//       drvlocal.supported.statusVer = 1;
//     asynPrint(pC_->pasynUserController_, ASYN_TRACE_INFO,
//               "%spollAll(%d) nvals=%d V1=%d V2=%d sim=%d ecmc=%d bV1BusyNew=%d Ver=%d cmd/data=%d/%d fPos=%f fActPos=%f\n",
//               modNamEMC, axisNo_, nvals,
//               pC_->features_ & FEATURE_BITS_V1,
//               pC_->features_ & FEATURE_BITS_V2,
//               pC_->features_ & FEATURE_BITS_SIM,
//               pC_->features_ & FEATURE_BITS_ECMC,
//               drvlocal.supported.bV1BusyNewStyle,
//               drvlocal.supported.statusVer,
//               pst_axis_status->nCommand,
//               pst_axis_status->nCmdData,
//               pst_axis_status->fPosition,
//               pst_axis_status->fActPosition);
// #ifdef motorFlagsHomeOnLsString
//     setIntegerParam(pC_->motorFlagsHomeOnLs_, 1);
// #endif
// #ifdef motorFlagsStopOnProblemString
//     setIntegerParam(pC_->motorFlagsStopOnProblem_, 0);
// #endif
//     drvlocal.dirty.statusVer = 0;
//   }
//   if (axisNo_ != motor_axis_no) return asynError;

//   /* Use previous fActPosition and current fActPosition to calculate direction.*/
//   if (pst_axis_status->fActPosition > drvlocal.old_st_axis_status.fActPosition) {
//     pst_axis_status->motorDiffPostion = 1;
//     pst_axis_status->motorStatusDirection = 1;
//   } else if (pst_axis_status->fActPosition < drvlocal.old_st_axis_status.fActPosition) {
//     pst_axis_status->motorDiffPostion = 1;
//     pst_axis_status->motorStatusDirection = 0;
//   }
//   return asynSuccess;


// pollAllWrongnvals:
//   /* rubbish on the line */
//   asynPrint(pC_->pasynUserController_, ASYN_TRACE_ERROR|ASYN_TRACEIO_DRIVER,
//             "%spollAll(%d) nvals=%d out=%s in=%s \n",
//             modNamEMC, axisNo_, nvals, pC_->outString_, pC_->inString_);
//   return asynDisabled;
// }


/** Polls the axis.
 * This function reads the motor position, the limit status, the home status, the moving status,
 * and the drive power-on status.
 * It calls setIntegerParam() and setDoubleParam() for each item that it polls,
 * and then calls callParamCallbacks() at the end.
 * \param[out] moving A flag that is set indicating that the axis is moving (true) or done (false). */
asynStatus EthercatMCAxisEcmc::poll(bool *moving)
{
  asynStatus comStatus = asynSuccess;
  st_axis_status_type st_axis_status;

  double timeBefore = EthercatMCgetNowTimeSecs();
#ifndef motorWaitPollsBeforeReadyString
  int waitNumPollsBeforeReady_ = drvlocal.waitNumPollsBeforeReady;
#endif

  /* Driver not yet initialized, do nothing */
  if (!drvlocal.scaleFactor) return comStatus;

  if (drvlocal.supported.statusVer == -1) {
    callParamCallbacksUpdateError();
    return asynSuccess;
  }

  memset(&st_axis_status, 0, sizeof(st_axis_status));

  // ---------------------ECMC
  //readAllStatus();

  uglyConvertFunc(&diagBinData_,&st_axis_status);
  
  // ---------------------ECMC

  /* Disable these lines since data is fetched with readAllStatus() function*/

  // memset(&st_axis_status, 0, sizeof(st_axis_status));
  // comStatus = pollAll(moving, &st_axis_status);
  // if (comStatus) {
  //   asynPrint(pC_->pasynUserController_, ASYN_TRACE_ERROR|ASYN_TRACEIO_DRIVER,
  //             "%sout=%s in=%s return=%s (%d)\n",
  //             modNamEMC, pC_->outString_, pC_->inString_,
  //             EthercatMCstrStatus(comStatus), (int)comStatus);
  //   if (comStatus == asynDisabled) {
  //     return asynSuccess;
  //   }
  //   goto skip;
  // }

  if (drvlocal.cfgDebug_str) {
    asynStatus comStatus;
    snprintf(pC_->outString_, sizeof(pC_->outString_), "%s", drvlocal.cfgDebug_str);
    comStatus = pC_->writeReadOnErrorDisconnect();
    if (!comStatus) {
      updateMsgTxtFromDriver(pC_->inString_);
    }
  }

  setIntegerParam(pC_->motorStatusHomed_, st_axis_status.bHomed);
  drvlocal.homed = st_axis_status.bHomed;
  setIntegerParam(pC_->motorStatusCommsError_, 0);
  setIntegerParam(pC_->motorStatusAtHome_, st_axis_status.bHomeSensor);
  setIntegerParam(pC_->motorStatusLowLimit_, !st_axis_status.bLimitBwd);
  setIntegerParam(pC_->motorStatusHighLimit_, !st_axis_status.bLimitFwd);
  setIntegerParam(pC_->motorStatusPowerOn_, st_axis_status.bEnabled);
  setDoubleParam(pC_->EthercatMCVelAct_, st_axis_status.fActVelocity);
  setDoubleParam(pC_->EthercatMCAcc_RB_, st_axis_status.fAcceleration);

#ifndef motorWaitPollsBeforeReadyString
  if (drvlocal.waitNumPollsBeforeReady) {
    *moving = true;
  }
  else
#endif
    {
      *moving = st_axis_status.mvnNRdyNex ? true : false;
      if (!st_axis_status.mvnNRdyNex &&
          !(pC_->features_ & FEATURE_BITS_ECMC)) {
        /* not moving: poll the parameters for this axis */
        int axisID = getMotionAxisID();
        switch (drvlocal.eeAxisPollNow) {
        case pollNowReadScaling:
          readScaling(axisID);
          drvlocal.eeAxisPollNow = pollNowReadMonitoring;
          break;
        case pollNowReadMonitoring:
          readMonitoring(axisID);
          drvlocal.eeAxisPollNow = pollNowReadBackSoftLimits;
          break;
        case pollNowReadBackSoftLimits:
          readBackSoftLimits();
          drvlocal.eeAxisPollNow = pollNowReadBackVelocities;
          break;
        case pollNowReadBackVelocities:
        default:
          readBackVelocities(axisID);
          drvlocal.eeAxisPollNow = pollNowReadScaling;
          break;
        }
      }
    }

  if (st_axis_status.mvnNRdyNex)
    drvlocal.nCommandActive = st_axis_status.nCommand;
  else {
    drvlocal.nCommandActive = 0;
    if (drvlocal.eeAxisWarning == eeAxisWarningSpeedLimit) {
      drvlocal.eeAxisWarning = eeAxisWarningNoWarning;
    }
  }

  if (drvlocal.nCommandActive != NCOMMANDHOME) {
    setDoubleParam(pC_->motorPosition_,
                   st_axis_status.fActPosition / drvlocal.scaleFactor);
    setDoubleParam(pC_->motorEncoderPosition_,
                   st_axis_status.fActPosition / drvlocal.scaleFactor);
    drvlocal.old_st_axis_status.fActPosition = st_axis_status.fActPosition;
    setDoubleParam(pC_->EthercatMCVel_RB_, st_axis_status.fVelocity);
  }

  if (drvlocal.externalEncoderStr) {
    comStatus = getValueFromController(drvlocal.externalEncoderStr,
                                       &st_axis_status.encoderRaw);
    if (!comStatus) setDoubleParam(pC_->EthercatMCEncAct_,
                                   st_axis_status.encoderRaw);
  } else if (pC_->features_ & FEATURE_BITS_V2) {
    setDoubleParam(pC_->EthercatMCEncAct_, st_axis_status.encoderRaw);
  }

  if (drvlocal.old_st_axis_status.bHomed != st_axis_status.bHomed) {
    asynPrint(pC_->pasynUserController_, ASYN_TRACE_INFO,
              "%spoll(%d) homed=%d\n",
              modNamEMC, axisNo_, st_axis_status.bHomed);
    drvlocal.old_st_axis_status.bHomed =  st_axis_status.bHomed;
  }
  if (drvlocal.old_st_axis_status.bLimitBwd != st_axis_status.bLimitBwd) {
    asynPrint(pC_->pasynUserController_, ASYN_TRACE_INFO,
              "%spoll(%d) LLS=%d\n",
              modNamEMC, axisNo_, !st_axis_status.bLimitBwd);
    drvlocal.old_st_axis_status.bLimitBwd =  st_axis_status.bLimitBwd;
  }
  if (drvlocal.old_st_axis_status.bLimitFwd != st_axis_status.bLimitFwd) {
    asynPrint(pC_->pasynUserController_, ASYN_TRACE_INFO,
              "%spoll(%d) HLS=%d\n",
              modNamEMC, axisNo_,!st_axis_status.bLimitFwd);
    drvlocal.old_st_axis_status.bLimitFwd = st_axis_status.bLimitFwd;
  }

#ifndef motorWaitPollsBeforeReadyString
  if (drvlocal.waitNumPollsBeforeReady) {
    /* Don't update moving, done, motorStatusProblem_ */
    asynPrint(pC_->pasynUserController_, ASYN_TRACE_INFO,
              "%spoll(%d) mvnNRdyNexAt=%d Ver=%d bBusy=%d bExecute=%d bEnabled=%d atTarget=%d waitNumPollsBeforeReady=%d\n",
              modNamEMC,
              axisNo_, st_axis_status.mvnNRdyNex,
              drvlocal.supported.statusVer,
              st_axis_status.bBusy, st_axis_status.bExecute,
              st_axis_status.bEnabled, st_axis_status.atTarget,
              drvlocal.waitNumPollsBeforeReady);
    drvlocal.waitNumPollsBeforeReady--;
    callParamCallbacks();
  }
  else
#endif
    {
      if (drvlocal.old_st_axis_status.mvnNRdyNex != st_axis_status.mvnNRdyNex ||
          drvlocal.old_st_axis_status.bBusy      != st_axis_status.bBusy ||
          drvlocal.old_st_axis_status.bEnabled   != st_axis_status.bEnabled ||
          drvlocal.old_st_axis_status.bExecute   != st_axis_status.bExecute ||
          drvlocal.old_st_axis_status.atTarget   != st_axis_status.atTarget) {
        asynPrint(pC_->pasynUserController_, ASYN_TRACE_INFO,
                  "%spoll(%d) mvnNRdy=%d Ver=%d bBusy=%d bExe=%d bEnabled=%d atTarget=%d wf=%d ENC=%g fPos=%g fActPosition=%g time=%f\n",
                  modNamEMC, axisNo_, st_axis_status.mvnNRdyNex,
                  drvlocal.supported.statusVer,
                  st_axis_status.bBusy, st_axis_status.bExecute,
                  st_axis_status.bEnabled, st_axis_status.atTarget,
                  waitNumPollsBeforeReady_,
                  st_axis_status.encoderRaw, st_axis_status.fPosition,
                  st_axis_status.fActPosition,
                  EthercatMCgetNowTimeSecs() - timeBefore);
      }
    }
  setIntegerParam(pC_->motorStatusDirection_, st_axis_status.motorStatusDirection);
  setIntegerParam(pC_->motorStatusMoving_, st_axis_status.mvnNRdyNex);
  setIntegerParam(pC_->motorStatusDone_, !st_axis_status.mvnNRdyNex);

  drvlocal.MCU_nErrorId = st_axis_status.nErrorId;

  if (drvlocal.cfgDebug_str) {
    ; /* Do not do the following */
  } else if (drvlocal.old_bError != st_axis_status.bError ||
             drvlocal.old_MCU_nErrorId != drvlocal.MCU_nErrorId ||
             drvlocal.dirty.sErrorMessage) {
    char sErrorMessage[256];
    int nErrorId = st_axis_status.nErrorId;
    const char *errIdString = errStringFromErrId(nErrorId);
    sErrorMessage[0] = '\0';
    drvlocal.sErrorMessage[0] = '\0';
    asynPrint(pC_->pasynUserController_, ASYN_TRACE_INFO,
              "%spoll(%d) bError=%d st_axis_status.nErrorId=0x%x\n",
              modNamEMC, axisNo_, st_axis_status.bError,
              nErrorId);
    drvlocal.old_bError = st_axis_status.bError;
    drvlocal.old_MCU_nErrorId = nErrorId;
    drvlocal.dirty.sErrorMessage = 0;
    if (nErrorId) {
      /* Get the ErrorMessage to have it in the log file */
      (void)getStringFromAxis("sErrorMessage", (char *)&sErrorMessage[0],
                              sizeof(sErrorMessage));
      asynPrint(pC_->pasynUserController_, ASYN_TRACE_INFO,
                "%ssErrorMessage(%d)=\"%s\"\n",
                modNamEMC, axisNo_, sErrorMessage);
    }
    /* First choice: "well known" ErrorIds */
    if (errIdString[0]) {
      snprintf(drvlocal.sErrorMessage, sizeof(drvlocal.sErrorMessage)-1, "E: %s %x",
               errIdString, nErrorId);
    } else if ((pC_->features_ & FEATURE_BITS_ECMC) && nErrorId) {
      /* emcmc has error messages */
      snprintf(drvlocal.sErrorMessage, sizeof(drvlocal.sErrorMessage)-1, "E: %s",
               sErrorMessage);
    } else if (nErrorId) {
      snprintf(drvlocal.sErrorMessage, sizeof(drvlocal.sErrorMessage)-1, "E: Cntrl Error %x", nErrorId);
    }
    /* The poller will update the MsgTxt field */
    // updateMsgTxtFromDriver(drvlocal.sErrorMessage);
  }
  callParamCallbacksUpdateError();

  memcpy(&drvlocal.old_st_axis_status, &st_axis_status,
         sizeof(drvlocal.old_st_axis_status));
  return asynSuccess;

skip:
  return asynError;
}

/** Set the motor closed loop status
 * \param[in] closedLoop true = close loop, false = open looop. */
asynStatus EthercatMCAxisEcmc::setClosedLoop(bool closedLoop)
{
  int value = closedLoop ? 1 : 0;
  asynPrint(pC_->pasynUserController_, ASYN_TRACE_INFO,
            "%ssetClosedLoop(%d)=%d\n",  modNamEMC, axisNo_, value);
  if (drvlocal.axisFlags & AMPLIFIER_ON_FLAG_USING_CNEN) {
    return enableAmplifier(value);
  }
  return asynSuccess;
}

asynStatus EthercatMCAxisEcmc::setIntegerParam(int function, int value)
{
  asynStatus status;
  unsigned indexGroup5000 = 0x5000;
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
  } else if (function == pC_->EthercatMCHomProc_) {
    int motorNotHomedProblem = 0;
    setIntegerParam(pC_->EthercatMCHomProc_RB_, value);
    asynPrint(pC_->pasynUserController_, ASYN_TRACE_INFO,
              "%ssetIntegerParam(%d HomProc_)=%d motorNotHomedProblem=%d\n",
              modNamEMC, axisNo_, value, motorNotHomedProblem);
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
  } else if (function == pC_->EthercatMCCfgDHLM_En_) {
    asynPrint(pC_->pasynUserController_, ASYN_TRACE_INFO,
              "%ssetIntegerParam(%d EthercatMCCfgDHLM_En)=%d\n",
              modNamEMC, axisNo_, value);
    status = setSAFValueOnAxis(indexGroup5000, 0xC, value);
    readBackSoftLimits();
    return status;
  } else if (function == pC_->EthercatMCCfgDLLM_En_) {
    asynPrint(pC_->pasynUserController_, ASYN_TRACE_INFO,
              "%ssetIntegerParam(%d EthercatMCCfgDLLM_En)=%d\n",
              modNamEMC, axisNo_, value);
    status = setSAFValueOnAxis(indexGroup5000, 0xB, value);
    readBackSoftLimits();
    return status;
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
asynStatus EthercatMCAxisEcmc::setDoubleParam(int function, double value)
{
  asynStatus status;
  unsigned indexGroup5000 = 0x5000;

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
  } else if (function == pC_->EthercatMCCfgDHLM_) {
    asynPrint(pC_->pasynUserController_, ASYN_TRACE_INFO,
              "%ssetDoubleParam(%d EthercatMCCfgDHLM_)=%f\n", modNamEMC, axisNo_, value);
    status = setSAFValueOnAxis(indexGroup5000, 0xE, value);
    readBackSoftLimits();
    return status;
  } else if (function == pC_->EthercatMCCfgDLLM_) {
    asynPrint(pC_->pasynUserController_, ASYN_TRACE_INFO,
              "%ssetDoubleParam(%d EthercatMCCfgDLLM_)=%f\n", modNamEMC, axisNo_, value);
    status = setSAFValueOnAxis(indexGroup5000, 0xD, value);
    readBackSoftLimits();
    return status;
  } else if (function == pC_->EthercatMCCfgVELO_) {
    asynPrint(pC_->pasynUserController_, ASYN_TRACE_INFO,
              "%ssetDoubleParam(%d EthercatMCCfgVELO_)=%f\n", modNamEMC, axisNo_, value);
    status = setSAFValueOnAxis(0x4000, 0x9, value);
    return status;
  } else if (function == pC_->EthercatMCCfgVMAX_) {
    asynPrint(pC_->pasynUserController_, ASYN_TRACE_INFO,
              "%ssetDoubleParam(%d EthercatMCCfgVMAX_)=%f\n", modNamEMC, axisNo_, value);
    status = setSAFValueOnAxis(0x4000, 0x27, value);
    return status;
  } else if (function == pC_->EthercatMCCfgJVEL_) {
    asynPrint(pC_->pasynUserController_, ASYN_TRACE_INFO,
              "%ssetDoubleParam(%d EthercatMCCfgJVEL_)=%f\n", modNamEMC, axisNo_, value);
    status = setSAFValueOnAxis(0x4000, 0x8, value);
    return status;
  } else if (function == pC_->EthercatMCCfgACCS_) {
    asynPrint(pC_->pasynUserController_, ASYN_TRACE_INFO,
              "%ssetDoubleParam(%d EthercatMCCfgACCS_)=%f\n", modNamEMC, axisNo_, value);
    status = setSAFValueOnAxis(0x4000, 0x101, value);
    return status;
  }
  // Call the base class method
  status = asynMotorAxis::setDoubleParam(function, value);
  return status;
}

asynStatus EthercatMCAxisEcmc::setStringParamDbgStrToMcu(const char *value)
{
  const char * const Main_this_str = "Main.this.";
  const char * const Sim_this_str = "Sim.this.";

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
  /* If we come here, the command was not understood */
  return asynError;
}

asynStatus EthercatMCAxisEcmc::setStringParam(int function, const char *value)
{
  if (function == pC_->EthercatMCDbgStrToLog_) {
    asynPrint(pC_->pasynUserController_, ASYN_TRACE_INFO,
              "%ssetStringParamDbgStrToLog(%d)=\"%s\"\n",
              modNamEMC, axisNo_, value);
  } else if (function == pC_->EthercatMCDbgStrToMcu_) {
    return setStringParamDbgStrToMcu(value);
  }
  /* Call base class method */
  return asynMotorAxis::setStringParam(function, value);
}

#ifndef motorMessageTextString
void EthercatMCAxisEcmc::updateMsgTxtFromDriver(const char *value)
{
  if (value && value[0]) {
    setStringParam(pC_->EthercatMCMCUErrMsg_,value);
  } else {
    setStringParam(pC_->EthercatMCMCUErrMsg_, "");
  }
}
#endif


asynStatus EthercatMCAxisEcmc::readConfigLine(const char *line, const char **errorTxt_p)
{
  const char *setRaw_str = "setRaw "; /* Raw is Raw */
  const char *setValue_str = "setValue "; /* prefixed with ADSPORT */
  const char *setADRinteger_str = "setADRinteger ";
  const char *setADRdouble_str  = "setADRdouble ";
  const char *setSim_str = "setSim ";

  asynStatus status = asynError;
  const char *errorTxt = NULL;

  while (*line == ' ') line++;
  if (line[0] == '#') {
    /*  Comment line */
    return asynSuccess;
  }

  if (!strncmp(setRaw_str, line, strlen(setRaw_str))) {
    const char *cfg_txt_p = &line[strlen(setRaw_str)];
    while (*cfg_txt_p == ' ') cfg_txt_p++;

    snprintf(pC_->outString_, sizeof(pC_->outString_), "%s", cfg_txt_p);
    status = pC_->writeReadACK(ASYN_TRACE_INFO);
  } else if (!strncmp(setValue_str, line, strlen(setValue_str))) {
    const char *cfg_txt_p = &line[strlen(setValue_str)];
    while (*cfg_txt_p == ' ') cfg_txt_p++;

    snprintf(pC_->outString_, sizeof(pC_->outString_), "%s%s",
             drvlocal.adsport_str, cfg_txt_p);
    status = pC_->writeReadACK(ASYN_TRACE_INFO);
  } else if (!strncmp(setSim_str, line, strlen(setSim_str))) {
    if (pC_->features_ & FEATURE_BITS_SIM) {
      const char *cfg_txt_p = &line[strlen(setRaw_str)];
      while (*cfg_txt_p == ' ') cfg_txt_p++;

      snprintf(pC_->outString_, sizeof(pC_->outString_),
               "Sim.M%d.%s", axisNo_, cfg_txt_p);
      status = pC_->writeReadACK(ASYN_TRACE_INFO);
    } else {
      status = asynSuccess;
    }
  } else if (!strncmp(setADRinteger_str, line, strlen(setADRinteger_str))) {
    unsigned indexGroup;
    unsigned indexOffset;
    int value;
    int nvals = 0;
    const char *cfg_txt_p = &line[strlen(setADRinteger_str)];
    while (*cfg_txt_p == ' ') cfg_txt_p++;
    nvals = sscanf(cfg_txt_p, "%x %x %d",
                   &indexGroup, &indexOffset, &value);
    if (nvals == 3) {
      status = setSAFValueOnAxisVerify(indexGroup, indexOffset, value, 1);
    } else {
      errorTxt = "Need 4 values";
    }
  } else if (!strncmp(setADRdouble_str, line, strlen(setADRdouble_str))) {
    unsigned indexGroup;
    unsigned indexOffset;
    double value;
    int nvals = 0;
    const char *cfg_txt_p = &line[strlen(setADRdouble_str)];
    while (*cfg_txt_p == ' ') cfg_txt_p++;
    nvals = sscanf(cfg_txt_p, "%x %x %lf",
                   &indexGroup, &indexOffset, &value);
    if (nvals == 3) {
      status = setSAFValueOnAxisVerify(indexGroup, indexOffset,
                                       value, 1);
    } else {
      errorTxt = "Need 4 values";
    }
  } else {
    errorTxt = "Illegal command";
  }
  if (errorTxt_p && errorTxt) {
    *errorTxt_p = errorTxt;
  }
  return status;
}


asynStatus EthercatMCAxisEcmc::readConfigFile(void)
{
  const char *simOnly_str = "simOnly ";
  FILE *fp;
  char *ret = &pC_->outString_[0];
  int line_no = 0;
  asynStatus status = asynSuccess;
  const char *errorTxt = NULL;
  /* no config file, or successfully uploaded : return */
  if (!drvlocal.cfgfileStr) {
    return asynSuccess;
  }
  fp = fopen(drvlocal.cfgfileStr, "r");
  if (!fp) {
    int saved_errno = errno;
    char cwdbuf[4096];
    char errbuf[4196];

    char *mypwd = getcwd(cwdbuf, sizeof(cwdbuf));
    snprintf(errbuf, sizeof(errbuf)-1,
             "E: readConfigFile: %s\n%s/%s",
             strerror(saved_errno),
             mypwd ? mypwd : "",
             drvlocal.cfgfileStr);
    updateMsgTxtFromDriver(errbuf);
    asynPrint(pC_->pasynUserController_, ASYN_TRACE_ERROR|ASYN_TRACEIO_DRIVER,
              "%s(%d)%s\n", modNamEMC, axisNo_, errbuf);
    return asynError;
  }
  while (ret && !status && !errorTxt) {
    char rdbuf[256];
    size_t i;
    size_t len;

    line_no++;
    ret = fgets(rdbuf, sizeof(rdbuf), fp);
    if (!ret) break;    /* end of file or error */
    len = strlen(ret);
    if (!len) continue; /* empty line, no LF */
    for (i=0; i < len; i++) {
      /* No LF, no CR , no ctrl characters, */
      if (rdbuf[i] < 32) rdbuf[i] = 0;
    }
    len = strlen(ret);
    if (!len) continue; /* empty line with LF */
    asynPrint(pC_->pasynUserController_, ASYN_TRACE_ERROR|ASYN_TRACEIO_DRIVER,
              "%sreadConfigFile(%d) %s:%u %s\n",
              modNamEMC, axisNo_,
              drvlocal.cfgfileStr, line_no, rdbuf);

    if (!strncmp(simOnly_str, rdbuf, strlen(simOnly_str))) {
      /* "simOnly " Only for the simulator */
      if (pC_->features_ & FEATURE_BITS_SIM) {
        status = readConfigLine(&rdbuf[strlen(simOnly_str)], &errorTxt);
      }
    } else {
      status = readConfigLine(rdbuf, &errorTxt);
    }

    if (status || errorTxt) {
      char errbuf[256];
      errbuf[sizeof(errbuf)-1] = 0;
      if (status) {
        snprintf(errbuf, sizeof(errbuf)-1,
                 "E: %s:%d out=%s\nin=%s",
                 drvlocal.cfgfileStr, line_no, pC_->outString_, pC_->inString_);
      } else {
        snprintf(errbuf, sizeof(errbuf)-1,
                 "E: %s:%d \"%s\"\n%s",
                 drvlocal.cfgfileStr, line_no, rdbuf, errorTxt);
      }

      asynPrint(pC_->pasynUserController_, ASYN_TRACE_ERROR|ASYN_TRACEIO_DRIVER,
                "%sreadConfigFile %s\n", modNamEMC, errbuf);
      updateMsgTxtFromDriver(errbuf);
    }
  } /* while */

  if (ferror(fp) || status || errorTxt) {
    asynPrint(pC_->pasynUserController_, ASYN_TRACE_ERROR|ASYN_TRACEIO_DRIVER,
              "%sreadConfigFile %sstatus=%d errorTxt=%s (%s)\n",
              modNamEMC,
              ferror(fp) ? "ferror " : "",
              (int)status,
              errorTxt ? errorTxt : "",
              drvlocal.cfgfileStr);
    fclose(fp);
    return asynError;
  }
  return asynSuccess;
}

asynStatus EthercatMCAxisEcmc::setSAFValueOnAxisVerify(unsigned indexGroup,
                                                   unsigned indexOffset,
                                                   int value,
                                                   unsigned int retryCount)
{
  asynStatus status = asynSuccess;
  unsigned int counter = 0;
  int rbvalue = 0 - value;
  while (counter < retryCount) {
    status = getSAFValueFromAxisPrint(indexGroup, indexOffset, "value=", &rbvalue);
    if (status) break;
    if (rbvalue == value) break;
    status = setSAFValueOnAxis(indexGroup, indexOffset, value);
    counter++;
    if (status) break;
    epicsThreadSleep(.1);
  }
  return status;
}

/** Sets a floating point value on an axis
 * the values in the controller must be updated
 * \param[in] name of the variable to be updated
 * \param[in] value the (floating point) variable to be updated
 *
 */
asynStatus EthercatMCAxisEcmc::setValueOnAxis(const char* var, double value)
{
  snprintf(pC_->outString_, sizeof(pC_->outString_),
           pC_->features_ & FEATURE_BITS_GVL ? "%sGvl.axes[%d].%s=%g" : "%sMain.M%d.%s=%g",
           drvlocal.adsport_str, axisNo_, var, value);
  return pC_->writeReadACK(ASYN_TRACE_INFO);
}

/** Sets 2 floating point value on an axis
 * the values in the controller must be updated
 * \param[in] name of the variable to be updated
 * \param[in] value the (floating point) variable to be updated
 *
 */
asynStatus EthercatMCAxisEcmc::setValuesOnAxis(const char* var1, double value1,
                                           const char* var2, double value2)
{
  snprintf(pC_->outString_, sizeof(pC_->outString_),
           pC_->features_ & FEATURE_BITS_GVL
           ? "%sGvl.axes[%d].%s=%g;%sGvl.axes[%d].%s=%g" : "%sMain.M%d.%s=%g;%sMain.M%d.%s=%g",
           drvlocal.adsport_str, axisNo_, var1, value1,
           drvlocal.adsport_str, axisNo_, var2, value2);
  return pC_->writeReadACK(ASYN_TRACE_INFO);
}

asynStatus EthercatMCAxisEcmc::getSAFValueFromAxisPrint(unsigned indexGroup,
                                                    unsigned indexOffset,
                                                    const char *name,
                                                    int *value)
{
  int res;
  int nvals;
  asynStatus status;
  int axisID = getMotionAxisID();
  if (axisID <= 0) return asynError;
  snprintf(pC_->outString_, sizeof(pC_->outString_), "ADSPORT=%u/.ADR.16#%X,16#%X,2,2?",
          501, indexGroup + axisID, indexOffset);
  status = pC_->writeReadOnErrorDisconnect();
  if (status)
    return status;
  nvals = sscanf(pC_->inString_, "%d", &res);
  if (nvals != 1) {
    asynPrint(pC_->pasynUserController_, ASYN_TRACE_ERROR|ASYN_TRACEIO_DRIVER,
              "%snvals=%d command=\"%s\" response=\"%s\"\n",
              modNamEMC, nvals, pC_->outString_, pC_->inString_);
    return asynError;
  }
  asynPrint(pC_->pasynUserController_, ASYN_TRACE_INFO,
            "%sout=%s in=%s %s=%d\n",
            modNamEMC, pC_->outString_, pC_->inString_,name, res);
  *value = res;
  return asynSuccess;
}

asynStatus EthercatMCAxisEcmc::getSAFValueFromAxisPrint(unsigned indexGroup,
                                                    unsigned indexOffset,
                                                    const char *name,
                                                    double *value)
{
  double res;
  int nvals;
  asynStatus status;
  int axisID = getMotionAxisID();
  if (axisID <= 0) return asynError;
  snprintf(pC_->outString_, sizeof(pC_->outString_), "ADSPORT=%u/.ADR.16#%X,16#%X,8,5?",
          501, indexGroup + axisID, indexOffset);
  status = pC_->writeReadOnErrorDisconnect();
  if (status)
    return status;
  nvals = sscanf(pC_->inString_, "%lf", &res);
  if (nvals != 1) {
    asynPrint(pC_->pasynUserController_, ASYN_TRACE_ERROR|ASYN_TRACEIO_DRIVER,
              "%snvals=%d command=\"%s\" response=\"%s\"\n",
               modNamEMC, nvals, pC_->outString_, pC_->inString_);
    return asynError;
  }
  asynPrint(pC_->pasynUserController_, ASYN_TRACE_INFO,
            "%sout=%s in=%s %s=%g\n",
            modNamEMC, pC_->outString_, pC_->inString_,name, res);
  *value = res;
  return asynSuccess;
}


/** Gets an integer or boolean value from an axis
 * \param[in] name of the variable to be retrieved
 * \param[in] pointer to the integer result
 *
 */
asynStatus EthercatMCAxisEcmc::getValueFromAxis(const char* var, int *value)
{
  asynStatus status;
  int res;
  snprintf(pC_->outString_, sizeof(pC_->outString_),
           pC_->features_ & FEATURE_BITS_GVL
           ? "%sGvl.axes[%d]%s?" : "%sMain.M%d%s?",
           drvlocal.adsport_str, axisNo_, var);
  status = pC_->writeReadOnErrorDisconnect();
  if (status)
    return status;
  if (var[0] == 'b') {
    if (!strcmp(pC_->inString_, "0")) {
      res = 0;
    } else if (!strcmp(pC_->inString_, "1")) {
      res = 1;
    } else {
      asynPrint(pC_->pasynUserController_, ASYN_TRACE_ERROR|ASYN_TRACEIO_DRIVER,
                "%scommand=\"%s\" response=\"%s\"\n",
                modNamEMC, pC_->outString_, pC_->inString_);
      return asynError;
    }
  } else {
    int nvals = sscanf(pC_->inString_, "%d", &res);
    if (nvals != 1) {
      asynPrint(pC_->pasynUserController_, ASYN_TRACE_ERROR|ASYN_TRACEIO_DRIVER,
                "%snvals=%d command=\"%s\" response=\"%s\"\n",
                 modNamEMC, nvals, pC_->outString_, pC_->inString_);
      return asynError;
    }
  }
  asynPrint(pC_->pasynUserController_, ASYN_TRACE_INFO,
            "%sout=%s in=%s status=%s (%d) iValue=%d\n",
            modNamEMC,
            pC_->outString_, pC_->inString_,
            EthercatMCstrStatus(status), (int)status, res);

  *value = res;
  return asynSuccess;
}

/** Gets an integer (or boolean) and a double value from an axis and print
 * \param[in] name of the variable to be retrieved
 * \param[in] pointer to the integer result
 *
 */
asynStatus EthercatMCAxisEcmc::getSAFValuesFromAxisPrint(unsigned iIndexGroup,
                                                     unsigned iIndexOffset,
                                                     const char *iname,
                                                     int *iValue,
                                                     unsigned fIndexGroup,
                                                     unsigned fIndexOffset,
                                                     const char *fname,
                                                     double *fValue)
{
  int iRes;
  int nvals;
  double fRes;
  asynStatus status;
  int axisID = getMotionAxisID();
  if (axisID <= 0) return asynError;
  snprintf(pC_->outString_, sizeof(pC_->outString_), "ADSPORT=%u/.ADR.16#%X,16#%X,2,2?;ADSPORT=%u/.ADR.16#%X,16#%X,8,5?",
          501, iIndexGroup + axisID, iIndexOffset,
          501, fIndexGroup + axisID, fIndexOffset);
  status = pC_->writeReadOnErrorDisconnect();
  if (status)
    return status;
  nvals = sscanf(pC_->inString_, "%d;%lf", &iRes, &fRes);
  if (nvals != 2) {
    asynPrint(pC_->pasynUserController_, ASYN_TRACE_ERROR|ASYN_TRACEIO_DRIVER,
              "%snvals=%d command=\"%s\" response=\"%s\"\n",
               modNamEMC, nvals, pC_->outString_, pC_->inString_);
    return asynError;
  }
  asynPrint(pC_->pasynUserController_, ASYN_TRACE_INFO,
            "%sout=%s in=%s %s=%d %s=%g\n",
            modNamEMC, pC_->outString_, pC_->inString_, iname, iRes, fname, fRes);

  *iValue = iRes;
  *fValue = fRes;
  return asynSuccess;

}

/** Gets a floating point value from an axis
 * \param[in] name of the variable to be retrieved
 * \param[in] pointer to the double result
 *
 */
asynStatus EthercatMCAxisEcmc::getValueFromAxis(const char* var, double *value)
{
  asynStatus status;
  int nvals;
  double res;
  snprintf(pC_->outString_, sizeof(pC_->outString_),
           pC_->features_ & FEATURE_BITS_GVL
           ? "%sGvl.axes[%d]%s?" : "%sMain.M%d%s?",
           drvlocal.adsport_str, axisNo_, var);
  status = pC_->writeReadOnErrorDisconnect();
  if (status)
    return status;
  nvals = sscanf(pC_->inString_, "%lf", &res);
  if (nvals != 1) {
    asynPrint(pC_->pasynUserController_, ASYN_TRACE_ERROR|ASYN_TRACEIO_DRIVER,
              "%snvals=%d command=\"%s\" response=\"%s\"\n",
               modNamEMC, nvals, pC_->outString_, pC_->inString_);
    return asynError;
  }
  *value = res;
  return asynSuccess;
}

/** Gets a string value from an axis
 * \param[in] name of the variable to be retrieved
 * \param[in] pointer to the string result
 *
 */
asynStatus EthercatMCAxisEcmc::getStringFromAxis(const char *var, char *value, size_t maxlen)
{
  value[0] = '\0'; /* Always have a valid string */
  if (!(pC_->features_ & FEATURE_BITS_GVL)) {
    asynStatus status;
    snprintf(pC_->outString_, sizeof(pC_->outString_),
             "%sMain.M%d.%s?", drvlocal.adsport_str, axisNo_, var);
    status = pC_->writeReadOnErrorDisconnect();
    if (status) return status;
    memcpy(value, pC_->inString_, maxlen);
  }
  return asynSuccess;
}

asynStatus EthercatMCAxisEcmc::getValueFromController(const char* var, double *value)
{
  asynStatus status;
  int nvals;
  double res;
  snprintf(pC_->outString_, sizeof(pC_->outString_), "%s?", var);
  status = pC_->writeReadOnErrorDisconnect();
  if (status)
    return status;
  nvals = sscanf(pC_->inString_, "%lf", &res);
  if (nvals != 1) {
    asynPrint(pC_->pasynUserController_, ASYN_TRACE_ERROR|ASYN_TRACEIO_DRIVER,
              "%snvals=%d command=\"%s\" response=\"%s\"\n",
               modNamEMC, nvals, pC_->outString_, pC_->inString_);
    return asynError;
  }
  *value = res;
  return asynSuccess;
}


asynStatus EthercatMCAxisEcmc::setSAFValueOnAxis(unsigned indexGroup,
                                             unsigned indexOffset,
                                             int value)
{
  int axisID = getMotionAxisID();
  if (axisID <= 0) return asynError;
  snprintf(pC_->outString_, sizeof(pC_->outString_), "ADSPORT=%u/.ADR.16#%X,16#%X,2,2=%d",
          501, indexGroup + axisID, indexOffset, value);
  return pC_->writeReadACK(ASYN_TRACE_INFO);
}

asynStatus EthercatMCAxisEcmc::setSAFValueOnAxis(unsigned indexGroup,
                                             unsigned indexOffset,
                                             double value)
{
  int axisID = getMotionAxisID();
  if (axisID <= 0) return asynError;
  snprintf(pC_->outString_, sizeof(pC_->outString_), "ADSPORT=%u/.ADR.16#%X,16#%X,8,5=%g",
          501, indexGroup + axisID, indexOffset, value);
  return pC_->writeReadACK(ASYN_TRACE_INFO);
}


int EthercatMCAxisEcmc::getMotionAxisID(void)
{
  int ret = drvlocal.dirty.nMotionAxisID;
  if (pC_->features_ & FEATURE_BITS_GVL) {
    drvlocal.dirty.nMotionAxisID = 0;
    return 0;
  }
  if (ret == -1) {
    int res = -3;
    asynStatus status;
    static const unsigned adsports[] = {0, 852, 851, 853};
    unsigned adsport_idx;
    ret = -2;
    for (adsport_idx = 0;
         adsport_idx < sizeof(adsports)/sizeof(adsports[0]);
         adsport_idx++) {
      unsigned adsport = adsports[adsport_idx];
      if (!adsport) {
        adsport = drvlocal.adsPort;
      }
      if (adsport) {
        /* Save adsport_str for the poller */
        snprintf(drvlocal.adsport_str, sizeof(drvlocal.adsport_str),
                 "ADSPORT=%u/", adsport);
      }
      snprintf(pC_->outString_, sizeof(pC_->outString_),
               "%sMain.M%d.nMotionAxisID?", drvlocal.adsport_str, axisNo_);
      status = pC_->writeReadOnErrorDisconnect();
      if (status) {
        return -1;
      }
      int nvals = sscanf(pC_->inString_, "%d", &res);
      if (nvals != 1) {
        asynPrint(pC_->pasynUserController_, ASYN_TRACE_ERROR|ASYN_TRACEIO_DRIVER,
                  "%soldret=%d nvals=%d command=\"%s\" response=\"%s\" res=%d\n",
                  modNamEMC, ret, nvals, pC_->outString_, pC_->inString_, res);
        continue;
      }
      ret = res;
      break;
    }
    if (ret != -1) drvlocal.dirty.nMotionAxisID = ret;
  }
  return ret;
}


/** Sets an integer or boolean value on an axis, read it back and retry if needed
 * the values in the controller must be updated
 * \param[in] name of the variable to be updated
 * \param[in] name of the variable where we can read back
 * \param[in] value the (integer) variable to be updated
 * \param[in] number of retries
 */
asynStatus EthercatMCAxisEcmc::setValueOnAxisVerify(const char *var, const char *rbvar,
                                                int value, unsigned int retryCount)
{
  asynStatus status = asynSuccess;
  unsigned int counter = 0;
  int rbvalue = 0 - value;
  while (counter <= retryCount) {
    snprintf(pC_->outString_, sizeof(pC_->outString_),
             pC_->features_ & FEATURE_BITS_GVL
             ? "%sGvl.axes[%d].%s=%d;%sGvl.axes[%d].%s?" : "%sMain.M%d.%s=%d;%sMain.M%d.%s?",
             drvlocal.adsport_str, axisNo_, var, value,
             drvlocal.adsport_str, axisNo_, rbvar);
    status = pC_->writeReadOnErrorDisconnect();
    asynPrint(pC_->pasynUserController_, ASYN_TRACE_INFO,
              "%ssetValueOnAxisVerify(%d) out=%s in=%s status=%s (%d)\n",
              modNamEMC, axisNo_,pC_->outString_, pC_->inString_,
              EthercatMCstrStatus(status), (int)status);
    if (status) {
      return status;
    } else {
      int nvals = sscanf(pC_->inString_, "OK;%d", &rbvalue);
      if (nvals != 1) {
        asynPrint(pC_->pasynUserController_, ASYN_TRACE_ERROR|ASYN_TRACEIO_DRIVER,
                  "%snvals=%d command=\"%s\" response=\"%s\"\n",
                  modNamEMC, nvals, pC_->outString_, pC_->inString_);
        return asynError;
      }
      if (status) break;
      if (rbvalue == value) break;
      counter++;
      epicsThreadSleep(.1);
    }
  }
  /* Verification failed.
     Store the error (unless there was an error before) */
  if ((rbvalue != value) && !drvlocal.cmdErrorMessage[0]) {
      asynPrint(pC_->pasynUserController_, ASYN_TRACE_ERROR|ASYN_TRACEIO_DRIVER,
                "%ssetValueOnAxisV(%d) var=%s value=%d rbvalue=%d",
                modNamEMC, axisNo_,var, value, rbvalue);
      snprintf(drvlocal.cmdErrorMessage, sizeof(drvlocal.cmdErrorMessage)-1,
               "E: setValueOnAxisV(%s) value=%d rbvalue=%d",
               var, value, rbvalue);

      /* The poller co-ordinates the writing into the parameter library */
  }
  return status;
}

asynStatus EthercatMCAxisEcmc::setSAFValueOnAxisVerify(unsigned indexGroup,
                                                   unsigned indexOffset,
                                                   double value,
                                                   unsigned int retryCount)
{
  asynStatus status = asynSuccess;
  unsigned int counter = 0;
  double rbvalue = 0 - value;
  while (counter < retryCount) {
    status = getSAFValueFromAxisPrint(indexGroup, indexOffset, "value", &rbvalue);
    if (status) break;
    if (rbvalue == value) break;
    status = setSAFValueOnAxis(indexGroup, indexOffset, value);
    counter++;
    if (status) break;
    epicsThreadSleep(.1);
  }
  return status;
}

/** Sets an integer or boolean value on an axis
 * the values in the controller must be updated
 * \param[in] name of the variable to be updated
 * \param[in] value the (integer) variable to be updated
 *
 */
asynStatus EthercatMCAxisEcmc::setValueOnAxis(const char* var, int value)
{
  snprintf(pC_->outString_, sizeof(pC_->outString_),
           pC_->features_ & FEATURE_BITS_GVL ? "%sGvl.axes[%d].%s=%d" : "%sMain.M%d.%s=%d",
           drvlocal.adsport_str, axisNo_, var, value);
  return pC_->writeReadACK(ASYN_TRACE_INFO);
}

/** 
 * Connect to ECMC axis over SyncIO interface
 * 
 *  All read / write parameters are connected:
 *  1. "T_SMP_MS=%d/TYPE=asynInt32/ax%d.status?"
 *  2. "T_SMP_MS=%d/TYPE=asynInt8ArrayIn/ax%d.diagnostic?"
 *  3. "T_SMP_MS=%d/TYPE=asynInt8ArrayIn/ax%d.diagnosticbin?"
 *  4. "T_SMP_MS=%d/TYPE=asynInt32/ax%d.control="
 *  5. "T_SMP_MS=%d/TYPE=asynFloat64/ax%d.targetpos="
 *  6. "T_SMP_MS=%d/TYPE=asynFloat64/ax%d.targetvel="
 *  7. "T_SMP_MS=%d/TYPE=asynFloat64/ax%d.targetacc="
 *  8. "T_SMP_MS=%d/TYPE=asynFloat64/ax%d.soflimbwd="
 *  9. "T_SMP_MS=%d/TYPE=asynFloat64/ax%d.soflimfwd="
 * 
*/
asynStatus EthercatMCAxisEcmc::connectEcmcAxis() {

  char buffer[ECMC_MAX_ASYN_DRVINFO_STR_LEN];
  int movingPollPeriodMs = (int)(pC_->movingPollPeriod_*1000);
  char *name = &buffer[0];
  unsigned int charCount = 0;
  asynStatus status;

  // Status Word
  charCount = snprintf(name,
                       sizeof(buffer),
                       ECMC_ASYN_AXIS_STAT_STRING,
                       movingPollPeriodMs,
                       axisId_);
  if (charCount >= sizeof(buffer) - 1) {    
    asynPrint(pC_->pasynUserSelf, ASYN_TRACE_ERROR,
              "%s/%s:%d: ERROR (axis %d): Failed to generate drvInfo for %s on asynport %s.\n",
      __FILE__,
      __FUNCTION__,
      __LINE__,
      axisId_,
      ECMC_ASYN_AXIS_STAT_STRING,
      pC_->mcuPortName_);
    return asynError;
  }

  status = pasynInt32SyncIO->connect(pC_->mcuPortName_, 0, &asynUserStatWd_, name);
  if (status!=asynSuccess) {
    asynPrint(pC_->pasynUserSelf, ASYN_TRACE_ERROR,
              "%s/%s:%d: ERROR (axis %d): Failed to connect drvInfo to ECMC for parameter %s.\n",
      __FILE__,
      __FUNCTION__,
      __LINE__,
      axisId_,
      name);
  }
  
  // Control Word
  charCount = snprintf(name,
                       sizeof(buffer),
                       ECMC_ASYN_AXIS_CONT_STRING,
                       movingPollPeriodMs,
                       axisId_);
  if (charCount >= sizeof(buffer) - 1) {    
    asynPrint(pC_->pasynUserSelf, ASYN_TRACE_ERROR,
              "%s/%s:%d: ERROR (axis %d): Failed to generate drvInfo for %s on asynport %s.\n",
      __FILE__,
      __FUNCTION__,
      __LINE__,
      axisId_,
      ECMC_ASYN_AXIS_CONT_STRING,
      pC_->mcuPortName_);
    return asynError;
  }

  status = pasynInt32SyncIO->connect(pC_->mcuPortName_, 0, &asynUserCntrlWd_, name);
  if (status!=asynSuccess) {
    asynPrint(pC_->pasynUserSelf, ASYN_TRACE_ERROR,
              "%s/%s:%d: ERROR (axis %d): Failed to connect drvInfo to ECMC for parameter %s.\n",
      __FILE__,
      __FUNCTION__,
      __LINE__,
      axisId_,
      name);
  }
  
  // Diag string
  charCount = snprintf(name,
                       sizeof(buffer),
                       ECMC_ASYN_AXIS_DIAG_STRING,
                       movingPollPeriodMs,
                       axisId_);
  if (charCount >= sizeof(buffer) - 1) {    
    asynPrint(pC_->pasynUserSelf, ASYN_TRACE_ERROR,
              "%s/%s:%d: ERROR (axis %d): Failed to generate drvInfo for %s on asynport %s.\n",
      __FILE__,
      __FUNCTION__,
      __LINE__,
      axisId_,
      ECMC_ASYN_AXIS_DIAG_STRING,
      pC_->mcuPortName_);
    return asynError;
  }
           
  status = pasynInt8ArraySyncIO->connect(pC_->mcuPortName_, 0, &asynUserDiagStr_, name);
  if (status!=asynSuccess) {
    asynPrint(pC_->pasynUserSelf, ASYN_TRACE_ERROR,
              "%s/%s:%d: ERROR (axis %d): Failed to connect drvInfo to ECMC for parameter %s.\n",
      __FILE__,
      __FUNCTION__,
      __LINE__,
      axisId_,
      name);
  }

  // Target position
  charCount = snprintf(name,
                       sizeof(buffer),
                       ECMC_ASYN_AXIS_TARG_POS_STRING,
                       movingPollPeriodMs,
                       axisId_);
  if (charCount >= sizeof(buffer) - 1) {    
    asynPrint(pC_->pasynUserSelf, ASYN_TRACE_ERROR,
              "%s/%s:%d: ERROR (axis %d): Failed to generate drvInfo for %s on asynport %s.\n",
      __FILE__,
      __FUNCTION__,
      __LINE__,
      axisId_,
      ECMC_ASYN_AXIS_TARG_POS_STRING,
      pC_->mcuPortName_);
    return asynError;
  }

  status = pasynFloat64SyncIO->connect(pC_->mcuPortName_, 0, &asynUserTargPos_, name);
  if (status!=asynSuccess) {
    asynPrint(pC_->pasynUserSelf, ASYN_TRACE_ERROR,
              "%s/%s:%d: ERROR (axis %d): Failed to connect drvInfo to ECMC for parameter %s.\n",
      __FILE__,
      __FUNCTION__,
      __LINE__,
      axisId_,
      name);
  }

  // Target velo
  charCount = snprintf(name,
                       sizeof(buffer),
                       ECMC_ASYN_AXIS_TARG_VEL_STRING,
                       movingPollPeriodMs,
                       axisId_);
  if (charCount >= sizeof(buffer) - 1) {    
    asynPrint(pC_->pasynUserSelf, ASYN_TRACE_ERROR,
              "%s/%s:%d: ERROR (axis %d): Failed to generate drvInfo for %s on asynport %s.\n",
      __FILE__,
      __FUNCTION__,
      __LINE__,
      axisId_,
      ECMC_ASYN_AXIS_TARG_VEL_STRING,
      pC_->mcuPortName_);
    return asynError;
  }

  status = pasynFloat64SyncIO->connect(pC_->mcuPortName_, 0, &asynUserTargVel_, name);
  if (status!=asynSuccess) {
    asynPrint(pC_->pasynUserSelf, ASYN_TRACE_ERROR,
              "%s/%s:%d: ERROR (axis %d): Failed to connect drvInfo to ECMC for parameter %s.\n",
      __FILE__,
      __FUNCTION__,
      __LINE__,
      axisId_,
      name);
  }

  // Target acc
  charCount = snprintf(name,
                       sizeof(buffer),
                       ECMC_ASYN_AXIS_TARG_ACC_STRING,
                       movingPollPeriodMs,
                       axisId_);
  if (charCount >= sizeof(buffer) - 1) {    
    asynPrint(pC_->pasynUserSelf, ASYN_TRACE_ERROR,
              "%s/%s:%d: ERROR (axis %d): Failed to generate drvInfo for %s on asynport %s.\n",
      __FILE__,
      __FUNCTION__,
      __LINE__,
      axisId_,
      ECMC_ASYN_AXIS_TARG_ACC_STRING,
      pC_->mcuPortName_);
    return asynError;
  }

  status = pasynFloat64SyncIO->connect(pC_->mcuPortName_, 0, &asynUserTargAcc_, name);
  if (status!=asynSuccess) {
    asynPrint(pC_->pasynUserSelf, ASYN_TRACE_ERROR,
              "%s/%s:%d: ERROR (axis %d): Failed to connect drvInfo to ECMC for parameter %s.\n",
      __FILE__,
      __FUNCTION__,
      __LINE__,
      axisId_,
      name);
  }

  // Soft limit bwd
  charCount = snprintf(name,
                       sizeof(buffer),
                       ECMC_ASYN_AXIS_SOFT_LIM_BWD_STRING,
                       movingPollPeriodMs,
                       axisId_);
  if (charCount >= sizeof(buffer) - 1) {    
    asynPrint(pC_->pasynUserSelf, ASYN_TRACE_ERROR,
              "%s/%s:%d: ERROR (axis %d): Failed to generate drvInfo for %s on asynport %s.\n",
      __FILE__,
      __FUNCTION__,
      __LINE__,
      axisId_,
      ECMC_ASYN_AXIS_SOFT_LIM_BWD_STRING,
      pC_->mcuPortName_);
    return asynError;
  }

  status = pasynFloat64SyncIO->connect(pC_->mcuPortName_, 0, &asynUserSoftLimBwd_, name);
  if (status!=asynSuccess) {
    asynPrint(pC_->pasynUserSelf, ASYN_TRACE_ERROR,
              "%s/%s:%d: ERROR (axis %d): Failed to connect drvInfo to ECMC for parameter %s.\n",
      __FILE__,
      __FUNCTION__,
      __LINE__,
      axisId_,
      name);
  }

  // Soft limit fwd
  charCount = snprintf(name,
                       sizeof(buffer),
                       ECMC_ASYN_AXIS_SOFT_LIM_FWD_STRING,
                       movingPollPeriodMs,
                       axisId_);
  if (charCount >= sizeof(buffer) - 1) {    
    asynPrint(pC_->pasynUserSelf, ASYN_TRACE_ERROR,
              "%s/%s:%d: ERROR (axis %d): Failed to generate drvInfo for %s on asynport %s.\n",
      __FILE__,
      __FUNCTION__,
      __LINE__,
      axisId_,
      ECMC_ASYN_AXIS_SOFT_LIM_FWD_STRING,
      pC_->mcuPortName_);
    return asynError;
  }

  status = pasynFloat64SyncIO->connect(pC_->mcuPortName_, 0, &asynUserSoftLimFwd_, name);
  if (status!=asynSuccess) {
    asynPrint(pC_->pasynUserSelf, ASYN_TRACE_ERROR,
              "%s/%s:%d: ERROR (axis %d): Failed to connect drvInfo to ECMC for parameter %s.\n",
      __FILE__,
      __FUNCTION__,
      __LINE__,
      axisId_,
      name);
  }

  // Diag binary struct data (over asynInt8Array interface)
  charCount = snprintf(name,
                       sizeof(buffer),
                       ECMC_ASYN_AXIS_DIAG_BIN_STRING,
                       movingPollPeriodMs,
                       axisId_);
  if (charCount >= sizeof(buffer) - 1) {    
    asynPrint(pC_->pasynUserSelf, ASYN_TRACE_ERROR,
              "%s/%s:%d: ERROR (axis %d): Failed to generate drvInfo for %s on asynport %s.\n",
      __FILE__,
      __FUNCTION__,
      __LINE__,
      axisId_,
      ECMC_ASYN_AXIS_DIAG_BIN_STRING,
      pC_->mcuPortName_);
    return asynError;
  }
           
  status = pasynInt8ArraySyncIO->connect(pC_->mcuPortName_, 0, &asynUserDiagBin_, name);
  if (status!=asynSuccess) {
    asynPrint(pC_->pasynUserSelf, ASYN_TRACE_ERROR,
              "%s/%s:%d: ERROR (axis %d): Failed to connect drvInfo to ECMC for parameter %s.\n",
      __FILE__,
      __FUNCTION__,
      __LINE__,
      axisId_,
      name);
  }

  // try connect interrupt.. Use name from before as drvInfo
  printf("try connect interrupt.. 1\n");
  //asynStatus status;
  asynInterface *pinterface;
  asynDrvUser *pDrvUser;

  asynUserDiagBinIntr_ = pasynManager->createAsynUser(0,0);
  status = pasynManager->connectDevice(asynUserDiagBinIntr_, pC_->mcuPortName_, 0);
  if (status) {
    //throw std::runtime_error(std::string("connectDevice failed:").append(asynUserDiagBinIntr_->errorMessage));
    printf("onnectDevice failed....\n");
    return asynError;
  }
  printf("try connect interrupt.. 2\n");
  pasynIFDiagBinIntr_ = pasynManager->findInterface(asynUserDiagBinIntr_, "asynInt8Array", 1);
  if (!pasynIFDiagBinIntr_) {
    //throw std::runtime_error(std::string("findInterface failed:").append(asynInterfaceType));
    printf("findInterface failed....\n");
    return asynError;
  }
  printf("try connect interrupt.. 3\n");
  if (!name) return asynError;
  pinterface = pasynManager->findInterface(asynUserDiagBinIntr_, asynDrvUserType, 1);
  if (!pinterface) return asynError;
  pDrvUser = (asynDrvUser *)pinterface->pinterface;
  status = pDrvUser->create(pinterface->drvPvt, asynUserDiagBinIntr_, name, 0, 0);
  if (status) {
    //throw std::runtime_error(std::string("drvUser->create failed:"));
    printf("drvUser->create failed....\n");
    return asynError;
  }
  printf("try connect interrupt.. 4\n");
  pIFDiagBinIntr_ = (asynInt8Array *)pasynIFDiagBinIntr_->pinterface;
  //virtual asynStatus registerInterruptUser(interruptCallbackInt32 pCallback, void *userPvt=0) { 
  if(interruptDiagBinPvt_!=NULL) return asynError;
  //if (!userPvt) userPvt=this;
  status = pIFDiagBinIntr_->registerInterruptUser(pasynIFDiagBinIntr_->drvPvt, asynUserDiagBinIntr_,
                                                    diagBinCallback, this, &interruptDiagBinPvt_);
  if (status) {      
    printf("registerInterruptUser failed....\n");
    return asynError;
  }
  printf("try connect interrupt.. 5\n");
   
  readAllStatus();
  return asynSuccess;
}

asynStatus EthercatMCAxisEcmc::readStatusWd() {
  
  ecmcAxisStatusWordType statusWdTemp;
  asynStatus status = pasynInt32SyncIO->read(asynUserStatWd_,(epicsInt32*)&statusWdTemp,DEFAULT_CONTROLLER_TIMEOUT);
  if (status!=asynSuccess) {
    asynPrint(pC_->pasynUserSelf, ASYN_TRACE_ERROR,
              "%s/%s:%d: ERROR (axis %d): Failed to read status word from ecmc.\n",
      __FILE__,
      __FUNCTION__,
      __LINE__,
      axisId_);
    return asynError;
  }
  statusWd_ = statusWdTemp;
  return asynSuccess;
}

asynStatus EthercatMCAxisEcmc::readDiagStr() {
  
  size_t inBytes = 0;
  asynStatus status = pasynInt8ArraySyncIO->read(asynUserDiagStr_,
                                            (epicsInt8*)diagStringBuffer_,
                                            ECMC_MAX_ASYN_DIAG_STR_LEN,
                                            &inBytes,
                                            DEFAULT_CONTROLLER_TIMEOUT);  
  
  if (status!=asynSuccess) {
    asynPrint(pC_->pasynUserSelf, ASYN_TRACE_ERROR,
              "%s/%s:%d: ERROR (axis %d): Failed to read diagnostic string from ecmc.\n",
      __FILE__,
      __FUNCTION__,
      __LINE__,
      axisId_);
    return asynError;
  }

  //diagStringBuffer_[inBytes]='\0';
  ecmcDiagStringData  diagDataTemp;
  int nvals = 0;  

  nvals = sscanf(diagStringBuffer_,
                 "%d,%lf,%lf,%lf,%lf,%lf,%d,%lf,%lf,%lf,%lf," // 11
                 "%d,%d,%d,%d,%d,%d,%d,%d,%d,%d,%d,%d,%d,%d,%d,%d,%d,%d,%d", //19
                  &diagDataTemp.axId,            // 1
                  &diagDataTemp.setpos,          // 2
                  &diagDataTemp.actpos,          // 3
                  &diagDataTemp.poserr,          // 4
                  &diagDataTemp.targpos,         // 5
                  &diagDataTemp.targposerr,      // 6
                  &diagDataTemp.rawpos,          // 7
                  &diagDataTemp.cntrlout,        // 8
                  &diagDataTemp.setvel,          // 9
                  &diagDataTemp.actvel,          // 10
                  &diagDataTemp.rawvelff,        // 11
                  &diagDataTemp.rawvel,          // 12
                  &diagDataTemp.cyclecnt,        // 13
                  &diagDataTemp.error,           // 14
                  &diagDataTemp.cmd,             // 15
                  &diagDataTemp.cmddata,         // 16
                  &diagDataTemp.seqstate,        // 17
                  &diagDataTemp.ilock,           // 18
                  &diagDataTemp.ilocklastactive, // 19
                  &diagDataTemp.trajsource,      // 20
                  &diagDataTemp.encsource,       // 21
                  &diagDataTemp.enable,          // 22
                  &diagDataTemp.enabled,         // 23
                  &diagDataTemp.execute,         // 24
                  &diagDataTemp.busy,            // 25
                  &diagDataTemp.attarget,        // 26
                  &diagDataTemp.homed,           // 27
                  &diagDataTemp.lowlim,          // 28
                  &diagDataTemp.highlim,         // 29
                  &diagDataTemp.homesensor);     // 30

  if (nvals != 30) {
    asynPrint(pC_->pasynUserSelf, ASYN_TRACE_ERROR,
             "%s/%s:%d: ERROR (axis %d): Failed to parse diaganostic string from ecmc.\n",
      __FILE__,
      __FUNCTION__,
      __LINE__,
      axisId_);
    return asynError;
  }

  memcpy(&diagData_,&diagDataTemp,sizeof(ecmcDiagStringData));  
  return asynSuccess;
}

asynStatus EthercatMCAxisEcmc::readDiagBin() {
  
  size_t inBytes = 0;
  ecmcAxisStatusType diagBinDataTemp;
  asynStatus status = pasynInt8ArraySyncIO->read(asynUserDiagBin_,
                                            (epicsInt8*)&diagBinDataTemp,
                                            sizeof(ecmcAxisStatusType),
                                            &inBytes,
                                            DEFAULT_CONTROLLER_TIMEOUT);    

  if (status!=asynSuccess || inBytes != sizeof(ecmcAxisStatusType)) {
    asynPrint(pC_->pasynUserSelf, ASYN_TRACE_ERROR,
              "%s/%s:%d: ERROR (axis %d): Failed to read diag binary data from ecmc.\n",
      __FILE__,
      __FUNCTION__,
      __LINE__,
      axisId_);
    return asynError;
  }

  diagBinData_=diagBinDataTemp;
  return asynSuccess;
}

asynStatus EthercatMCAxisEcmc::readControlWd(ecmcAxisControlWordType *controlWd) {
  
  ecmcAxisControlWordType controlWdTemp;
  asynStatus status = pasynInt32SyncIO->read(asynUserCntrlWd_,(epicsInt32*)&controlWdTemp,DEFAULT_CONTROLLER_TIMEOUT);
  if (status!=asynSuccess) {
    asynPrint(pC_->pasynUserSelf, ASYN_TRACE_ERROR,
              "%s/%s:%d: ERROR (axis %d): Failed to read control word from ecmc.\n",
      __FILE__,
      __FUNCTION__,
      __LINE__,
      axisId_);
    return asynError;
  }

  *controlWd = controlWdTemp;

  return asynSuccess;
}

asynStatus EthercatMCAxisEcmc::writeControlWd(ecmcAxisControlWordType controlWd) {
  epicsInt32 *pCntWd = (epicsInt32 *)&controlWd;
  asynStatus status = pasynInt32SyncIO->write(asynUserCntrlWd_,*pCntWd,DEFAULT_CONTROLLER_TIMEOUT);
  if (status!=asynSuccess) {
    asynPrint(pC_->pasynUserSelf, ASYN_TRACE_ERROR,
              "%s/%s:%d: ERROR (axis %d): Failed to write control word to ecmc.\n",
      __FILE__,
      __FUNCTION__,
      __LINE__,
      axisId_);
    return asynError;
  }  
  return asynSuccess;
}

asynStatus EthercatMCAxisEcmc::writeTargetPos(double pos) {
  
  asynStatus status = pasynFloat64SyncIO->write(asynUserTargPos_,(epicsFloat64)pos,DEFAULT_CONTROLLER_TIMEOUT);
  if (status!=asynSuccess) {
    asynPrint(pC_->pasynUserSelf, ASYN_TRACE_ERROR,
              "%s/%s:%d: ERROR (axis %d): Failed to write target position to ecmc.\n",
      __FILE__,
      __FUNCTION__,
      __LINE__,
      axisId_);
    return asynError;
  }  
  return asynSuccess;
}

asynStatus EthercatMCAxisEcmc::writeTargetVel(double vel) {
  
  asynStatus status = pasynFloat64SyncIO->write(asynUserTargVel_,(epicsFloat64)vel,DEFAULT_CONTROLLER_TIMEOUT);
  if (status!=asynSuccess) {
    asynPrint(pC_->pasynUserSelf, ASYN_TRACE_ERROR,
              "%s/%s:%d: ERROR (axis %d): Failed to write target velocity to ecmc.\n",
      __FILE__,
      __FUNCTION__,
      __LINE__,
      axisId_);
    return asynError;
  }  
  return asynSuccess;
}

asynStatus EthercatMCAxisEcmc::writeTargetAcc(double acc) {
  
  asynStatus status = pasynFloat64SyncIO->write(asynUserTargAcc_,(epicsFloat64)acc,DEFAULT_CONTROLLER_TIMEOUT);
  if (status!=asynSuccess) {
    asynPrint(pC_->pasynUserSelf, ASYN_TRACE_ERROR,
              "%s/%s:%d: ERROR (axis %d): Failed to write target acceleration to ecmc.\n",
      __FILE__,
      __FUNCTION__,
      __LINE__,
      axisId_);
    return asynError;
  }  
  return asynSuccess;
}

asynStatus EthercatMCAxisEcmc::writeSoftLimBwd(double softlimbwd) {
  
  asynStatus status = pasynFloat64SyncIO->write(asynUserSoftLimBwd_,(epicsFloat64)softlimbwd,DEFAULT_CONTROLLER_TIMEOUT);
  if (status!=asynSuccess) {
    asynPrint(pC_->pasynUserSelf, ASYN_TRACE_ERROR,
              "%s/%s:%d: ERROR (axis %d): Failed to write bwd softlimit to ecmc.\n",
      __FILE__,
      __FUNCTION__,
      __LINE__,
      axisId_);
    return asynError;
  }  
  return asynSuccess;
}

asynStatus EthercatMCAxisEcmc::writeSoftLimFwd(double softlimfwd) {
  
  asynStatus status = pasynFloat64SyncIO->write(asynUserSoftLimFwd_,(epicsFloat64)softlimfwd,DEFAULT_CONTROLLER_TIMEOUT);
  if (status!=asynSuccess) {
    asynPrint(pC_->pasynUserSelf, ASYN_TRACE_ERROR,
              "%s/%s:%d: ERROR (axis %d): Failed to write fwd softlimit to ecmc.\n",
      __FILE__,
      __FUNCTION__,
      __LINE__,
      axisId_);
    return asynError;
  }  
  return asynSuccess;
}

asynStatus EthercatMCAxisEcmc::readAllStatus() {
  
  asynStatus status;

  // Only needed for soflimit enable and plc enable and so on.. 
  // TODO: Merge to one struct only instead of two like now..
  status = readStatusWd();
  if(status) {
    return status;
  }

  // Use readDiagBin instead because more effichent..
  /*status =readDiagStr();
  if(status) {
    return status;
  }*/

  status =readDiagBin();
  if(status) {
    return status;
  }

  //printDiagBinData();
  /*int32_t *temp = (int32_t*) &statusWd_;
  printf("##########\n");
  printf("Status word (%d): %d\n",axisId_,*temp);*/
  // printf("Act pos (%d)    : %lf\n",axisId_,actPos_);

  return asynSuccess;
}

//Just for debug
asynStatus EthercatMCAxisEcmc::printDiagBinData() {
  int asynLevel=ASYN_TRACE_ERROR;
  asynPrint(pC_->pasynUserSelf, asynLevel,"  diagBinData_.axisID = %d\n",diagBinData_.axisID);
  asynPrint(pC_->pasynUserSelf, asynLevel,"  diagBinData_.cycleCounter = %d\n",diagBinData_.cycleCounter);
  asynPrint(pC_->pasynUserSelf, asynLevel,"  diagBinData_.acceleration = %lf\n",diagBinData_.acceleration);
  asynPrint(pC_->pasynUserSelf, asynLevel,"  diagBinData_.deceleration = %lf\n",diagBinData_.deceleration);
  asynPrint(pC_->pasynUserSelf, asynLevel,"  diagBinData_.reset = %d\n",diagBinData_.reset);
  asynPrint(pC_->pasynUserSelf, asynLevel,"  diagBinData_.moving = %d\n",diagBinData_.moving);
  asynPrint(pC_->pasynUserSelf, asynLevel,"  diagBinData_.stall = %d\n",diagBinData_.stall);
  asynPrint(pC_->pasynUserSelf, asynLevel,"  diagBinData_.onChangeData.positionSetpoint = %lf\n",diagBinData_.onChangeData.positionSetpoint);
  asynPrint(pC_->pasynUserSelf, asynLevel,"  diagBinData_.onChangeData.positionActual = %lf\n",diagBinData_.onChangeData.positionActual);
  asynPrint(pC_->pasynUserSelf, asynLevel,"  diagBinData_.onChangeData.positionError = %lf\n",diagBinData_.onChangeData.positionError);
  asynPrint(pC_->pasynUserSelf, asynLevel,"  diagBinData_.onChangeData.positionTarget = %lf\n",diagBinData_.onChangeData.positionTarget);
  asynPrint(pC_->pasynUserSelf, asynLevel,"  diagBinData_.onChangeData.cntrlError = %lf\n",diagBinData_.onChangeData.cntrlError);
  asynPrint(pC_->pasynUserSelf, asynLevel,"  diagBinData_.onChangeData.cntrlOutput = %lf\n",diagBinData_.onChangeData.cntrlOutput);
  asynPrint(pC_->pasynUserSelf, asynLevel,"  diagBinData_.onChangeData.velocityActual = %lf\n",diagBinData_.onChangeData.velocityActual);
  asynPrint(pC_->pasynUserSelf, asynLevel,"  diagBinData_.onChangeData.velocitySetpoint = %lf\n",diagBinData_.onChangeData.velocitySetpoint);
  asynPrint(pC_->pasynUserSelf, asynLevel,"  diagBinData_.onChangeData.velocityFFRaw = %lf\n",diagBinData_.onChangeData.velocityFFRaw);
  asynPrint(pC_->pasynUserSelf, asynLevel,"  diagBinData_.onChangeData.positionRaw = %ld\n",diagBinData_.onChangeData.positionRaw);
  asynPrint(pC_->pasynUserSelf, asynLevel,"  diagBinData_.onChangeData.error = %d\n",diagBinData_.onChangeData.error);
  asynPrint(pC_->pasynUserSelf, asynLevel,"  diagBinData_.onChangeData.velocitySetpointRaw = %d\n",diagBinData_.onChangeData.velocitySetpointRaw);
  asynPrint(pC_->pasynUserSelf, asynLevel,"  diagBinData_.onChangeData.seqState = %d\n",diagBinData_.onChangeData.seqState);
  asynPrint(pC_->pasynUserSelf, asynLevel,"  diagBinData_.onChangeData.cmdData = %d\n",diagBinData_.onChangeData.cmdData);
  asynPrint(pC_->pasynUserSelf, asynLevel,"  diagBinData_.onChangeData.command = %d\n",(int)diagBinData_.onChangeData.command);
  asynPrint(pC_->pasynUserSelf, asynLevel,"  diagBinData_.onChangeData.trajInterlock = %d\n",(int)diagBinData_.onChangeData.trajInterlock);
  asynPrint(pC_->pasynUserSelf, asynLevel,"  diagBinData_.onChangeData.lastActiveInterlock = %d\n",(int)diagBinData_.onChangeData.lastActiveInterlock);
  asynPrint(pC_->pasynUserSelf, asynLevel,"  diagBinData_.onChangeData.trajSource = %d\n",(int)diagBinData_.onChangeData.trajSource);
  asynPrint(pC_->pasynUserSelf, asynLevel,"  diagBinData_.onChangeData.encSource = %d\n",(int)diagBinData_.onChangeData.encSource);
  asynPrint(pC_->pasynUserSelf, asynLevel,"  diagBinData_.onChangeData.enable = %d\n",diagBinData_.onChangeData.enable);
  asynPrint(pC_->pasynUserSelf, asynLevel,"  diagBinData_.onChangeData.enabled = %d\n",diagBinData_.onChangeData.enabled);
  asynPrint(pC_->pasynUserSelf, asynLevel,"  diagBinData_.onChangeData.execute = %d\n",diagBinData_.onChangeData.execute);
  asynPrint(pC_->pasynUserSelf, asynLevel,"  diagBinData_.onChangeData.busy = %d\n",diagBinData_.onChangeData.busy);
  asynPrint(pC_->pasynUserSelf, asynLevel,"  diagBinData_.onChangeData.atTarget = %d\n",diagBinData_.onChangeData.atTarget);
  asynPrint(pC_->pasynUserSelf, asynLevel,"  diagBinData_.onChangeData.homed = %d\n",diagBinData_.onChangeData.homed);
  asynPrint(pC_->pasynUserSelf, asynLevel,"  diagBinData_.onChangeData.limitFwd = %d\n",diagBinData_.onChangeData.limitFwd);
  asynPrint(pC_->pasynUserSelf, asynLevel,"  diagBinData_.onChangeData.limitBwd = %d\n",diagBinData_.onChangeData.limitBwd);
  asynPrint(pC_->pasynUserSelf, asynLevel,"  diagBinData_.onChangeData.homeSwitch = %d\n",diagBinData_.onChangeData.homeSwitch);
  asynPrint(pC_->pasynUserSelf, asynLevel,"  diagBinData_.onChangeData.sumIlockFwd = %d\n",diagBinData_.onChangeData.sumIlockFwd);
  asynPrint(pC_->pasynUserSelf, asynLevel,"  diagBinData_.onChangeData.sumIlockBwd = %d\n",diagBinData_.onChangeData.sumIlockBwd);
  return asynSuccess;
}

asynStatus EthercatMCAxisEcmc::uglyConvertFunc(ecmcAxisStatusType*in ,st_axis_status_type *out) {
  //just to test new interface.. Get all data from diagBinData_ instead    
  out->bEnable              = in->onChangeData.enable;
  out->bExecute             = in->onChangeData.execute;
  out->nCommand             = in->onChangeData.command;
  out->nCmdData             = in->onChangeData.cmdData;
  out->fVelocity            = in->onChangeData.velocitySetpoint;
  out->fPosition            = in->onChangeData.positionTarget;
  out->fAcceleration        = in->acceleration;
  out->fDecceleration       = in->deceleration;  
  out->bHomeSensor          = in->onChangeData.homeSwitch;
  out->bEnabled             = in->onChangeData.enabled;
  out->bError               = in->onChangeData.error>0;
  out->nErrorId             = in->onChangeData.error;
  out->fActVelocity         = in->onChangeData.velocityActual;
  out->fActPosition         = in->onChangeData.positionActual;
  out->fActDiff             = in->onChangeData.positionError;
  out->bHomed               = in->onChangeData.homed;
  out->bBusy                = in->onChangeData.busy;
  out->encoderRaw           = in->onChangeData.positionRaw;
  out->atTarget             = in->onChangeData.atTarget;
  out->bLimitBwd            = in->onChangeData.limitBwd;
  out->bLimitFwd            = in->onChangeData.limitFwd;
  
  // Data derveid from struct
  out->mvnNRdyNex           = in->onChangeData.busy || !in->onChangeData.atTarget;
  out->motorStatusDirection = in->onChangeData.positionActual > 
                                              oldPositionAct_ ? 1:0;
  out->motorDiffPostion     = 1; /*Always set to 1?? why */

  
  //TODO not accesible for ecmc.. neeeded? Delete later
  out->bJogFwd              = 0;
  out->bJogBwd              = 0;
  out->bReset               = 0;
  out->fOverride            = 100;

  oldPositionAct_ =  in->onChangeData.positionActual;


  printDiagBinData();
  return asynSuccess;
}

ecmcAxisStatusType *EthercatMCAxisEcmc::getDiagBinDataPtr(){
  return &diagBinData_;
}
