<div align="center">
  <h1>The RRT-Connect Algorithm</h1>
</div>
</br>

## Introduction

A double RRT tree means there will be two RRT trees that each tree explores and expands outward from the departure point and destination point until the two trees meet. The expansion and growing method of a double RRT tree is similar to a single RRT tree, both of which require three steps: random sampling, step restriction, and collision detection. However, the difference part is that each tree of the double RRT tree grows alternately. For example, In the first iteration, the A tree extends outward, and in the second iteration, it switches to the B tree, and so on.
