## Building
libigl is needed to be set up in one directory up or this directory. Then to build, make the directory build in one directory up and run `cmake -DCMAKE_BUILD_TYPE=Release ../src` in that directory, then do `make` and after run the exectutable.

## Data
You will also need to set up a data directory and fill it with the required .ply files as specified in MyContext.cpp, or rewrite the code in order to load your own files.

## Running
Currently the code is set up to do global alignment via the point to plane ICP method.
