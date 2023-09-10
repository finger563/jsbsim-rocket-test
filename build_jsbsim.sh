#!/bin/bash

# save the current directory for later use
CURRENT_DIR=$(pwd)

# go into the jsbsim folder
cd external/jsbsim

# make build folder and cd into it
mkdir -p build
cd build

# build the jsbsim library with cmake
cmake -DBUILD_SHARED_LIBS=ON -DCMAKE_OSX_ARCHITECTURES="arm64;x86_64" -DCMAKE_C_COMPILER=clang -DCMAKE_CXX_COMPILER=clang++ -DCMAKE_CXX_FLAGS="-stdlib=libc++" ..
make -j4

cd ..

# set the lib and include folders for use in the script
INCLUDE_FOLDER=$CURRENT_DIR/include
LIB_FOLDER=$CURRENT_DIR/lib

echo "Copying JSBSim header files to include folder: $INCLUDE_FOLDER"
# make the include folder
rm -rf $INCLUDE_FOLDER
mkdir -p $INCLUDE_FOLDER
# copy the include files (.h,.hxx) from src (and its subdirectories) into
# include folder, keeping the same directory structure as in src. Since we're on
# macos, we use rsync instead of cp to preserve the directory structure

# Copy headers
rsync -avm --include='*.h' --include='*.hpp' --include='*.hxx' -f 'hide,! */' src/ $INCLUDE_FOLDER/

echo "Copying JSBSim library to lib folder: $LIB_FOLDER"
# make the lib folder
rm -rf $LIB_FOLDER
mkdir -p $LIB_FOLDER
# copy the jsbsim library (.a) from the build folder
# thirdparty/jsbsim/lib folder
# cp ./build/src/libJSBSim.a $LIB_FOLDER/.
cp ./build/src/*.dylib $LIB_FOLDER/.

# go back out to the original directory
cd $CURRENT_DIR
