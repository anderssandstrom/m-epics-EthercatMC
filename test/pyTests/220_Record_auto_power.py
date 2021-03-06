#!/usr/bin/env python

# EPICS Single Motion application test script
#
# http://cars9.uchicago.edu/software/python/pyepics3/
#

import unittest
import os
import sys
import math
import time
from motor_lib import motor_lib
lib = motor_lib()
from motor_globals import motor_globals
globals = motor_globals()
import capv_lib
###

PwrOnDly  =  6.0
PwrOffDly =  3.0


def restorePwrSettings(self, motor, tc_no, pwrAuto, pwrOnDly, pwrOffDly):
    capv_lib.capvput(motor + '-PwrAuto', pwrAuto)
    capv_lib.capvput(motor + '-PwrOnDly', pwrOnDly)
    capv_lib.capvput(motor + '-PwrOffDly', pwrOffDly)

def do_220_autopower(self, motor, tc_no, autopower):
    lib.setCNENandWait(motor, tc_no, 0)
    capv_lib.capvput(motor + '-PwrAuto', autopower, wait=True, timeout=globals.TIMEOUT)
    capv_lib.capvput(motor + '-PwrOnDly', PwrOnDly, wait=True, timeout=globals.TIMEOUT)
    capv_lib.capvput(motor + '-PwrOffDly', PwrOffDly, wait=True, timeout=globals.TIMEOUT)
    print('%s Enable move to LLM +10' % tc_no)
    res1 = lib.move(motor, self.saved_LLM + 10 + 2*autopower, globals.TIMEOUT)

    #Make sure drive is still enabled
    power1 = capv_lib.capvget(motor + '.CNEN', use_monitor=False)
    print('%s Check drive is still enabled power1=%d' % (tc_no, power1))


    time.sleep(PwrOnDly + PwrOffDly + 2.0)
    power2 = capv_lib.capvget(motor + '.CNEN', use_monitor=False)
    print('%s Wait 8s and check drive is now disabled power2=%d' % (tc_no, power2))
    restorePwrSettings(self, motor, tc_no, self.saved_PwrAuto, self.saved_PwrOnDly, self.saved_PwrOffDly)
    assert(res1 == 0)
    assert(power1 == 1)
    assert(power2 == 0)



class Test(unittest.TestCase):
    motor           = os.getenv("TESTEDMOTORAXIS")
    #capv_lib.capvput(motor + '-DbgStrToLOG', "Start of " + os.path.basename(__file__)[0:20])
    saved_LLM       = capv_lib.capvget(motor + '.LLM')
    saved_CNEN      = capv_lib.capvget(motor + '.CNEN')
    saved_PwrAuto   = capv_lib.capvget(motor + '-PwrAuto')
    saved_PwrOnDly  = capv_lib.capvget(motor + '-PwrOnDly')
    saved_PwrOffDly = capv_lib.capvget(motor + '-PwrOffDly')

    def test_TC_2200(self):
        motor = self.motor
        tc_no = "TC-2201-Enable_goto_LLM"

        #Enable power
        print('%s Enable drive and move to LLM' % tc_no)
        capv_lib.capvput(motor + '-PwrAuto', 2,         wait=True, timeout=globals.TIMEOUT)
        capv_lib.capvput(motor + '-PwrOnDly', PwrOnDly, wait=True, timeout=globals.TIMEOUT)
        lib.setCNENandWait(motor, tc_no, 1)
        res = lib.move(motor, self.saved_LLM, globals.TIMEOUT)
        restorePwrSettings(self, motor, tc_no, self.saved_PwrAuto, self.saved_PwrOnDly, self.saved_PwrOffDly)
        assert(res == 0)


    def test_TC_2201(self):
        motor = self.motor
        tc_no = "TC-2201-Auto_power_mode_1"
        print('%s autopower ' % tc_no)
        do_220_autopower(self, motor, tc_no, 1)

    def test_TC_2202(self):
        motor = self.motor
        tc_no = "TC-2202-Auto_power_mode_2"
        print('%s autopower ' % tc_no)
        do_220_autopower(self, motor, tc_no, 2)
        lib.setCNENandWait(motor, tc_no, self.saved_CNEN)
