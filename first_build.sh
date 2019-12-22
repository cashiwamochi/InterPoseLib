#!/bin/bash

echo [LOG] Building Pangolin ... 
cd ./ThirdParty/Pangolin
mkdir build && cd build
cmake ..
make -j4
echo [LOG] Done !

cd ../../..

if [ -e ./build ]; then
  echo [LOG] build folder has already existed.
  echo [LOG] Stopped to build process.
  echo [LOG] If you want to continue, please delete build folder.

  exit 1 
fi

echo [LOG] Started build process ...

mkdir build
cd build

cmake ..
make

cd ..

if [ -e ./build/bin/exampleInterPose ]; then
  echo [LOG] Build Suceeded!
  echo [LOG] Now you can try exampleInterPoseSE3 in build/bin!
else
  echo [LOG] Build Failed!
fi
