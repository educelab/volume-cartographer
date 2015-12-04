#!/bin/bash
# Download dependencies for deployment

# Target SDKs
mkdir -p SDKs
cd SDKs
curl https://7bba5af52f57be309d65f0be55832483b029b4bd.googledrive.com/host/0BxjtEz6no21sSnZtNVItcEFwaTA/MacOSX10.9.sdk.tar.gz | tar xz
cd ..

# Prebuilt dependencies
curl https://41451397b21f587a032f4b6edf651cb72ff6c8c5.googledrive.com/host/0BxjtEz6no21sOENZalA0cll1RFE/vcdeps-10.9+.tar.gz | tar xz
