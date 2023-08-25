<div align="center">
  <h1>The RRT-Connect Algorithm</h1>
</div>
</br>

## Introduction

A double RRT tree means there will be two RRT trees that each tree explores and expands outward from the departure point and destination point until the two trees meet. The expansion and growing method of a double RRT tree is similar to a single RRT tree, both of which require three steps: random sampling, step restriction, and collision detection. However, the difference part is that each tree of the double RRT tree grows alternately. For example, In the first iteration, the A tree expands outward, and in the second iteration, it switches to the B tree, and so on. 

> RRT-Connect algorithm, which is optimized from a double RRT algorithm, uses the new growth node of the first tree as a lead point of another tree which is demonstrated more efficient than the normal double RRT algorithm.
>
1. First Round - Departure Tree
   1. Departure point as a root, begins to grow branches nodes;
   2. Randomly generate a lead point P in the search space;
   3. Find the closest point to P on the departure tree and mark it as C;
   4. Grow a step size in the direction of point P and mark it as new node N1 if there are no obstacles to collision. If there is an obstacle to collision then repeat the process from steps 2-4;
2. First Round - Destination Tree
   1. Destination point as a root, begins to grow branches nodes;
   2. Find the closest point to node N1 on the destination tree and market it as D;
   3. Grow a step size in the direction of node N1 and mark it as new node N2 if there are no obstacles to collision. If there is an obstacle to collision then go into the second round;
3. Second Round - Destination Tree
   1. Randomly generate a lead point P in the search space;
   2. Find the closest point to P on the destination tree and mark it as C;
   3. Grow a step size in the direction of point P and mark it as new node N1 if there are no obstacles to collision. If there is an obstacle to collision then repeat the process from steps 2-4;
4. Second Round - Departure Tree
   1. Find the closest point to node N1 on the Departure tree and market it as D;
   2. Grow a step size in the direction of node N1 and mark it as new node N2 if there are no obstacles to collision. If there is an obstacle to collision then go into the second round;

