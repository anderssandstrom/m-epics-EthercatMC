#!/bin/bash
#
# Wrapper scipt to run tests
# Usage:
# ./runTests.sh <PV> [test.py]
#
# Examples:
#
# run all tests on a motor PV
# ./runtests.sh IOC:m1
#
# run some tests on a motor PV
# ./runtests.sh IOC:m1 100*.py 12*.py
#
# run specif test on a motor PV a couple of times
# ./runTests.sh IOC:m1 100_Record-HOMF.py 4

. ~/.bash_profile

# First of all, check for whitespace damage (TAB, trailing WS
../checkws.sh || {
  echo >&2   ../checkws.sh failed
  exit 1
}

##############################################################################
# functions
#
#
checkAndInstallSystemPackage()
{
  BINARYNAME=$1
  PACKAGENAME=$2
  if ! which $BINARYNAME; then
    if which yum >/dev/null 2>&1; then
      sudo yum install $PACKAGENAME || {
        echo >&2 failed: sudo yum install $PACKAGENAME
        exit 1
      }
    fi
    if which apt-get >/dev/null 2>&1; then
      sudo apt-get install $PACKAGENAME || {
        echo >&2 failed: sudo apt-get install $PACKAGENAME
        exit 1
      }
    fi
  fi
}

########################################
checkAndInstallCondaPythonPackage()
{
  IMPORTNAME=$1

  if ! python -c "import $IMPORTNAME" >/dev/null 2>&1; then
    while test $# -gt 1; do
      shift
      PACKAGEINSTALL=$1
      echo failed: $PYTHON -c "import $IMPORTNAME"
      $PACKAGEINSTALL && return 0
    done
    echo >&1  $PACKAGEINSTALL failed
    exit 1
  fi
}
##############################################################################
if ! which conda >/dev/null 2>&1; then
  # This is Mac OS
  #URL=https://repo.anaconda.com/miniconda/Miniconda3-latest-MacOSX-x86_64.sh
  #wget $URL || {
    #echo >&2 wget $URL failed
    #exit 1
  #}
  checkAndInstallSystemPackage conda anaconda
fi


##############################################################################
conda activate pyepicsPytestPVApy || {
  echo >&2 conda activate pyepicsPytestPVApy failed
  conda create -n  pyepicsPytestPVApy
  exit 1
}

checkAndInstallCondaPythonPackage pytest "conda install -c conda-forge pyTest"
checkAndInstallCondaPythonPackage epics  "conda install -c GSECARS  pyepics" "conda install pyepics"
checkAndInstallCondaPythonPackage p4p    "pip install p4p"

#conda create -n  pyepicsPytestPVApy


# See if we have a local EPICS installation
uname_s=$(uname -s 2>/dev/null || echo unknown)
uname_m=$(uname -m 2>/dev/null || echo unknown)
INSTALLED_EPICS=../../../.epics.$(hostname).$uname_s.$uname_m
if test -r $INSTALLED_EPICS; then
  echo INSTALLED_EPICS=$INSTALLED_EPICS
. $INSTALLED_EPICS
fi

if test -z "$PYEPICS_LIBCA"; then
    MYLIB=$EPICS_BASE/lib/$EPICS_HOST_ARCH/libca.so
    if test -r "$MYLIB"; then
      PYEPICS_LIBCA=$MYLIB
      export PYEPICS_LIBCA
    else
      MYLIB=$EPICS_BASE/lib/$EPICS_HOST_ARCH/libca.dylib
      if test -r "$MYLIB"; then
        PYEPICS_LIBCA=$MYLIB
        export PYEPICS_LIBCA
      fi
    fi
fi &&

echo "$0" "$@"
if test -n "$1"; then
   TESTEDMOTORAXIS=$1
   PREFIX=${1%:*}
   TESTEDMOTORADDR=${1##*:m}
   TESTEDMCUASYN=$PREFIX:MCU1:asyn
   echo TESTEDMOTORAXIS=$TESTEDMOTORAXIS
   echo TESTEDMOTORADDR=$TESTEDMOTORADDR
   echo TESTEDMCUASYN=$TESTEDMCUASYN
   shift 1
else
  echo >&2 "$0 <PV> [numruns] [testfile.py]"
  exit 1
fi


files=""
numruns=1
while test -n "$1" && test -f "$1"; do
    files="$files $1"
    shift 1
done

if test -n "$1" && test "$1" -ne 0; then
    numruns=$1
    shift 1
else
    numruns=1
fi

run_pytest ()
{
  if test "$CONDA_PREFIX"; then
      echo pytest "$@"
      pytest "$@"
  else
    echo $PYTHON $VIRTUALENVDIR/bin/pytest "$@"
    $PYTHON $VIRTUALENVDIR/bin/pytest "$@" || exit 1
  fi
}

#if test -z "$EPICS_CA_ADDR_LIST" && test -z "$EPICS_CA_AUTO_ADDR_LIST"; then
#  if EPICS_CA_ADDR_LIST=127.0.1 EPICS_CA_AUTO_ADDR_LIST=NO caget $TESTEDMOTORAXIS.RBV >/dev/null 2>&1; then
#    EPICS_CA_ADDR_LIST=127.0.1
#    EPICS_CA_AUTO_ADDR_LIST=NO
#    export EPICS_CA_ADDR_LIST EPICS_CA_AUTO_ADDR_LIST
#  fi
#fi

while test $numruns -gt 0; do
  #if ! caget $TESTEDMOTORAXIS.RBV >/dev/null 2>/dev/null; then
  #  echo >&2 caget $TESTEDMOTORAXIS failed
  #  exit 1
  #fi
  export TESTEDMOTORADDR TESTEDMOTORAXIS TESTEDMCUASYN
  if test -n "$files"; then
    files=$(echo $files | sort)
    echo files=$files
    for file in $files; do
      echo file=$file
      run_pytest "$@" $file || exit 1
    done
  else
    py=$(echo *.py | sort)
    echo py=$py
    for p in $py
    do
      run_pytest "$@" $p || exit 1
    done
  fi
  echo Runs left=$numruns
  numruns=$(($numruns - 1))
done
