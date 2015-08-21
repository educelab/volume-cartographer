#!/bin/bash
revisioncount=`git log --oneline | wc -l | tr -d ' '`
projectversion=`git describe --tags --long --always`
cleanversion=${projectversion%%-*}

echo "$projectversion-$revisioncount"
