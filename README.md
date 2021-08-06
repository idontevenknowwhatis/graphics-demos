# graphics-demos

Two projects from Intro to Graphics in 2018. 

## Project 1
The first project implements Bezier and quadratic/quintic spline interpolation for control points which define the path of the animated runner. Velocity and acceleration computed from distances/deltas between control points (the runner is to traverse two adjacent points in constant time irrespective of distance) are used to give the runner a realistic swaying behavior. Inverse kinematics is implemented and used to animate the movement of the arms and legs. The dinosaur runner has a physically animated tail which utilizes repeated LERPs from each conic tail element to the tail element it is connected to's position in a prior frame. 

Runner demo: https://www.youtube.com/watch?v=PCnQWjB5HSg

Dinosaur demo: https://www.youtube.com/watch?v=PCnQWjB5HSg

## Project 2
The second project revolves around pathfinding. Given a series of control points, we create a set of Voronoi regions which constitute the "floor" that our dynamically resizable Roomba must cover while avoiding clipping through the control points. We enforce for efficiency that the Roomba's diameter must be sufficiently large such that it is in contact with two control points at any given time for maximum coverage. Pathfinding for the Roomba to hit each Voronoi region is implemented using depth first search. The implementation allows for dynamic addition and replacement of control points and subsequent updating of the Voronoi regions. The Roomba may also be directed to follow the cursor's position raycasted to the floor, but this implementation is not optimal/doesn't use Dijkstra's and is instead greedy. Raycasting is also implemented so that the Roomba can keep track of visible control points in another objective which is to traverse the floor/Voronoi regions until it sees every control point. 

Mesh editing demo: https://www.youtube.com/watch?v=iUXwSYqjGrw

Pathfinding and raycasting demo: https://www.youtube.com/watch?v=_EbrBCI-Vsw
