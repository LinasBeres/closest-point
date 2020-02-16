# Closest Point
Find the closest point on a mesh given a query point and a maximum distance.
![alt text](https://github.com/LinasBeres/closest-point/blob/master/images/cow-closest-point.png "Closest Point on Cow Mesh")

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
The gui has options where the user can input the query point and the maximum distance, as well as options for searching for the closest point using either brute force, threaded brute force or kdtree methods. Before the user can use the KdTree search, the tree must be built.

## Design
The application has three methods for finding the closest point in a mesh, that being brute force, threaded brute force and using a KdTree. The brute force approach simply loops over all points in the mesh, compares distances to the query point and if it is within the distance specified it returns the one that has the minimum distance. The threaded approach does the same, but utilises all cores on the machine. Both these algorithms have a complexity of O(n), with the threaded option being faster given more cores.

The brute force approach is a fine solution if the user is only querying one point on the mesh. However, if there are multiple points that are to be queried then the KdTree approach is preferred since that is only O(log(n)) in the average case, but can be O(n) is the worst case. The downside to using a KdTree is that it takes O(nlog(n)) time to build and uses O(n) space, but if time to build and space is not a concern, then it is the better solution. 

The gui can be used as an interactive version in order to better show the results from the query with the query point being displayed in green and the closest point in blue.

## Assumptions
I am assuming that the mesh is in 3D and for testing purposes I am using .ply files, however this can easily be extended to read any files that libigl supports.

In addition, if the point is already on the mesh, then in that case the program will output a different position on the mesh and if it cannot find the closest point then it will return nothing. 

## Libraries Used
I am using libigl and Eigen for reading, displaying and storing the mesh.
