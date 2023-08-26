<div align="center">
  <h1>The RRT-Connect Algorithm</h1>
</div>
</br>

## Introduction

A double RRT tree means there will be two RRT trees that each tree explores and expands outward from the departure point and destination point until the two trees meet. The expansion and growing method of a double RRT tree is similar to a single RRT tree, both of which require three steps: random sampling, step restriction, and collision detection. However, the difference part is that each tree of the double RRT tree grows alternately. For example, In the first iteration, the A tree expands outward, and in the second iteration, it switches to the B tree, and so on. 

> RRT-Connect algorithm, which is optimized from a double RRT algorithm, uses the new growth node of the first tree as a lead point of another tree which is demonstrated more efficient than the normal double RRT algorithm.
```python
self.treeTurns = [self.departureTree, self.destinationTree]
```
```python
for iterate in range(self.maxIterations):
  # 1. Departure point as a root, begins to grow branches nodes;
  # 2. Randomly generate a lead point P in the search space;
  # 3. Find the closest point to P on the departure tree and mark it as C;
  # 4. Grow a step size in the direction of point P and mark it as new node N1 if there are no obstacles to collision. If there is an obstacle to collision then repeat the process from steps 2-4;
  self.get_legal_children_point(self.treeTurns[0])

  # 5. Destination point as a root, begins to grow branches nodes;
  # 6. Find the closest point to node N1 on the destination tree and market it as D;
  # 7. Grow a step size in the direction of node N1 and mark it as new node N2 if there are no obstacles to collision. If there is an obstacle to collision then go into the second round;
  leadPoint = N1
  self.generate_nexttree_nodes(self.treeTurns[1], leadPoint, self.stepSize)

  # 8. check whether node N2 can connect leadPoint, and in the next iteration, check whether the new node N2 can either connect to the previous node N2 in the last iteration or     leadPoint in the current iteration.
  # 9. exchange trees, go to next iteration
  self.treeTurns.reverse()

def try_connect(self, leadPoint, previousNewTPoint, newPoint_closest):

  # newPoint_closest is node N2 in the current iteration
  # previousNewTPoint is node N2 in the last iteration

  if Tools.is_legal_point(leadPoint, newPoint_closest, self.obstacles, self.safeRadius) is True and Tools.getDistance(leadPoint, newPoint_closest) <= self.targetRadius:
    self.connection = True
  elif Tools.is_legal_point(previousNewTPoint, newPoint_closest, self.obstacles, self.safeRadius) is True and Tools.getDistance(previousNewTPoint, newPoint_closest) <= self.targetRadius:
    self.connection = True

  
```
> RRT-Connect algorithm with a greedy strategy
