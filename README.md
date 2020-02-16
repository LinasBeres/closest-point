# Closest Point

## Building and Running
Requires cmake at 3.1 or higher and make.

To build:
```
mkdir build && cd build
cmake ..
make -j8
```

The program can either be run on the command line or launched in the gui, for the command line the arguments are specified as bellow:
```
./closest-point --mesh ../data/cow.ply --dist 1.2 --point 1.0 0.3 0.6 --cmd
```
Where dist is the maximum distance and point is the query point. The additional arguments `--threaded` or `--kdtree` can be specified to run the program in threaded mode or in kdtree mode.

Then to run the program with the gui, one launches it by:
```
./closest-point --mesh ../data/cow.ply
```
The gui has options where the user can input the query point and the maximum distance, as well as options for searching for the closest point using either brute force, threaded brute force or kdtree methods.
