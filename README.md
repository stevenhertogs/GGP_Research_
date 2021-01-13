# Jump Point Search #

## Whats Is Jump Point Search? ##
Jump Point Search (JPS) is an improvement on the A* algorithm for grids with uniform costs.
Instead of adding a new node to the openlist every iteration, we can expand our search until we find nodes that are "interesting".
JPS searches potentially larger areas then A* but spends much less time updating the open and closed lists.

The following example shows how in a uniform grid there can be a lot of differents best paths with the same cost and A* has to add each node of these different paths to the  openList.

![xd.png](https://user-images.githubusercontent.com/36840551/104440916-402a4e80-5593-11eb-8593-2a7bf8fd80c6.png)

## Algorithm ##
* The algorithm is largely the same as A*. The main difference is in finding points to add to the open list
* When getting the node with the lowest estimated cost from the open list, we do a scan along its direction to find and add jumpPoints.
* Our NodeStructs have a direction to know in what direction to scan. We can have multiple same nodes on the openlist, but with different directions. They also have a parent node (node where they jumped from) in order to backtrack our path
* Once end goal is reached, use parent nodes to backtrack and find path
  
  
## Horizontal Scan

![Horizontal Scan](https://user-images.githubusercontent.com/36840551/104443228-70272100-5596-11eb-9995-988bd779f33e.png)

Example for Scanning to the right:
* Startnode = b1
* Nextnode = b2
* if(b2 != valid node) return //no jump points found
* if(a2 != valid node && a3 == valid node) add b2 to openList with direction a3
* if(c2 != valid node && c3 == valid node) add b2 to openList with direction c3
* Repeat

//Vertical scan is similar to horizontal scan

## Diagonal Scan

* Do a horizontal and a vertical scan
* Go to next diagonal node along direction
* repeat (until next diagonal node != valid node)

## Example
![example.png](https://user-images.githubusercontent.com/36840551/104445307-415e7a00-5599-11eb-9c6c-0cc5a39e5f91.png)

## Application Exampe
![app_example](https://user-images.githubusercontent.com/36840551/104467364-893fca00-55b6-11eb-9bed-0664abd15b16.png)


#### References
*https://zerowidth.com/2013/a-visual-explanation-of-jump-point-search.html
*https://www.gamedev.net/tutorials/programming/artificial-intelligence/jump-point-search-fast-a-pathfinding-for-uniform-cost-grids-r4220/
