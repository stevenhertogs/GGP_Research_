# Jump Point Search #

## Whats Is Jump Point Search? ##
Jump Point Search (JPS) is an improvement on the A* algorithm for grids with uniform costs.
Instead of adding a new node to the openlist every iteration, we can expand our search until we find nodes that are "interesting".
JPS searches potentially larger areas then A* but spends much less time updating the open and closed lists.

The following example shows how in a uniform grid there can be a lot of differents best paths with the same cost and A* has to add each node of these different paths to the  openList.

![xd.png](https://user-images.githubusercontent.com/36840551/104440916-402a4e80-5593-11eb-8593-2a7bf8fd80c6.png)

## Algorithm #


#### References
*https://zerowidth.com/2013/a-visual-explanation-of-jump-point-search.html
*https://www.gamedev.net/tutorials/programming/artificial-intelligence/jump-point-search-fast-a-pathfinding-for-uniform-cost-grids-r4220/
