#!/bin/bash
# should be run from a build folder that is a subdirectory of the source folder

versionMajor=`cat CMakeLists.txt | grep "VC VERSION" | sed 's/.*VERSION \([0-9]*\).*/\1/'`
versionMinor=`cat CMakeLists.txt | grep "VC VERSION" | sed 's/.*VERSION [0-9]*\.\([0-9]*\).*/\1/'`
versionPatch=`cat CMakeLists.txt | grep "VC VERSION" | sed 's/.*VERSION [0-9]*\.[0-9]*.\([0-9]*\).*/\1/'`
commitHash=`git log -1 --format="%h"`
revisioncount=`git log $(git rev-list --tags --max-count=1)..${commitHash} --oneline | wc -l | tr -d ' '`

if [[ ${1} == "--short" ]]; then
    echo "${versionMajor}.${versionMinor}.${versionPatch}"
else
    echo "${versionMajor}.${versionMinor}.${versionPatch}.${commitHash}-$revisioncount"
fi
