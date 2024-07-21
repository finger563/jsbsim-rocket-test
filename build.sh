# simple build script for the project
# run with `sh build.sh`

# remove the old build
rm -rf build

# create the build directory
mkdir build

# go into the build directory
cd build

# configure the build
cmake ..

# build the project
make
