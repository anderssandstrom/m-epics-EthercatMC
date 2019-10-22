require EthercatMC,USER

epicsEnvSet("ECM_NUMAXES",   "2")
epicsEnvSet("MOTOR_PORT",    "$(SM_MOTOR_PORT=MCU1)")

epicsEnvSet("IPADDR",        "$(SM_IPADDR=192.168.88.78)")
epicsEnvSet("IPPORT",        "$(SM_IPPORT=5000)")
epicsEnvSet("ASYN_PORT",     "$(SM_ASYN_PORT=MC_CPU1)")
epicsEnvSet("PREFIX",        "$(SM_PREFIX=IOC:)")
epicsEnvSet("P",             "$(SM_PREFIX=IOC:)")
epicsEnvSet("EGU",           "$(SM_EGU=mm)")
epicsEnvSet("PREC",          "$(SM_PREC=3)")
epicsEnvSet("ADSPORT",       "852")

epicsEnvSet("ECM_OPTIONS",          "")
# Controller
< EthercatMCController.iocsh


# Axis 1
epicsEnvSet("MOTOR_NAME",    "$(SM_MOTOR_NAME=m1)")
epicsEnvSet("AXIS_NO",       "$(SM_AXIS_NO=1)")
epicsEnvSet("DESC",          "$(SM_DESC=Axis1)")
epicsEnvSet("AXISCONFIG",    "adsPort=$(ADSPORT)")

< EthercatMCGvlAxis.iocsh
< EthercatMCAxisdebug.iocsh
#< EthercatMCAxishome.iocsh

# Axis 2
epicsEnvSet("MOTOR_NAME",    "$(SM_MOTOR_NAME=m2)")
epicsEnvSet("AXIS_NO",       "$(SM_AXIS_NO=2)")
epicsEnvSet("DESC",          "$(SM_DESC=Axis2)")
epicsEnvSet("AXISCONFIG",    "adsPort=$(ADSPORT)")

< EthercatMCGvlAxis.iocsh
< EthercatMCAxisdebug.iocsh
