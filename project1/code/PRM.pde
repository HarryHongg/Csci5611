 //You will only be turning in this file
//Your solution will be graded based on it's runtime (smaller is better), 
//the optimality of the path you return (shorter is better), and the
//number of collisions along the path (it should be 0 in all cases).

//You must provide a function with the following prototype:
// ArrayList<Integer> planPath(Vec2 startPos, Vec2 goalPos, Vec2[] centers, float[] radii, int numObstacles, Vec2[] nodePos, int numNodes);
// Where: 
//    -startPos and goalPos are 2D start and goal positions
//    -centers and radii are arrays specifying the center and radius of obstacles
//    -numObstacles specifies the number of obstacles
//    -nodePos is an array specifying the 2D position of roadmap nodes
//    -numNodes specifies the number of nodes in the PRM
// The function should return an ArrayList of node IDs (indexes into the nodePos array).
// This should provide a collision-free chain of direct paths from the start position
// to the position of each node, and finally to the goal position.
// If there is no collision-free path between the start and goal, return an ArrayList with
// the 0'th element of "-1".

// Your code can safely make the following assumptions:
//   - The function connectNeighbors() will always be called before planPath()
//   - The variable maxNumNodes has been defined as a large static int, and it will
//     always be bigger than the numNodes variable passed into planPath()
//   - None of the positions in the nodePos array will ever be inside an obstacle
//   - The start and the goal position will never be inside an obstacle

// There are many useful functions in CollisionLibrary.pde and Vec2.pde
// which you can draw on in your implementation. Please add any additional 
// functionality you need to this file (PRM.pde) for compatabilty reasons.

// Here we provide a simple PRM implementation to get you started.
// Be warned, this version has several important limitations.
// For example, it uses BFS which will not provide the shortest path.
// Also, it (wrongly) assumes the nodes closest to the start and goal
// are the best nodes to start/end on your path on. Be sure to fix 
// these and other issues as you work on this assignment. This file is
// intended to illustrate the basic set-up for the assignmtent, don't assume 
// this example funcationality is correct and end up copying it's mistakes!).



//Here, we represent our graph structure as a neighbor list
//You can use any graph representation you like
ArrayList<Integer>[] neighbors = new ArrayList[maxNumNodes];  //A list of neighbors can be reached from a given node
//We also want some help arrays to keep track of some information about nodes we've visited
Boolean[] visited = new Boolean[maxNumNodes]; //A list which store if a given node has been visited
int[] parent = new int[maxNumNodes]; //A list which stores the best previous node on the optimal path to reach this node
int originNumNodes = numNodes;
//Set which nodes are connected to which neighbors (graph edges) based on PRM rules


void addNode(Vec2 point){
  nodePos[numNodes] = point.times(1);
  numNodes += 1;
}

void deleteHelperNodes(){
  numNodes = originNumNodes;
}


void connectNeighbors(Vec2[] centers, float[] radii, int numObstacles, Vec2[] nodePos, int numNodes, agent[] agentList){
  
  for (int i = 0; i < numAgents; i ++){

    addNode(agentList[i].pos);
    nodePos[numNodes] = agentList[i].pos.times(1);
    numNodes += 1;
    addNode(agentList[i].goal);
    nodePos[numNodes] = agentList[i].goal.times(1);
    numNodes += 1;
  }
  
  
  
  
  for (int i = 0; i < numNodes; i++){
    neighbors[i] = new ArrayList<Integer>();  //Clear neighbors list
    for (int j = 0; j < numNodes; j++){
      if (i == j) continue; //don't connect to myself 
      Vec2 dir = nodePos[j].minus(nodePos[i]).normalized();
      float distBetween = nodePos[i].distanceTo(nodePos[j]);
      hitInfo circleListCheck = rayCircleListIntersect(centers, radii, numObstacles, nodePos[i], dir, distBetween, agentList[0].rad);
      if (!circleListCheck.hit){
        neighbors[i].add(j);
      }
    }
  }


}

//This is probably a bad idea and you shouldn't use it...
int closestNode(Vec2 point, Vec2[] nodePos, int numNodes){
  int closestID = -1;
  float minDist = 999999;
  for (int i = 0; i < numNodes; i++){
    float dist = nodePos[i].distanceTo(point);
    if (dist < minDist){
      closestID = i;
      minDist = dist;
    }
  }
  return closestID;
}



ArrayList<Integer> planPath(agent myAgent, Vec2[] centers, float[] radii, int numObstacles, Vec2[] nodePos, int numNodes){
  ArrayList<Integer> path = new ArrayList();
  
  int startID = closestNode(myAgent.pos, nodePos, numNodes);
  int goalID = closestNode(myAgent.goal, nodePos, numNodes);
  
  //path = runBFS(nodePos, numNodes, startID, goalID);
  path = runAstar(nodePos, numNodes, startID, goalID, myAgent);
  
  deleteHelperNodes();
  numNodes = originNumNodes;
  return path;
}


float estimateCost (int startID, int goalID){
  return nodePos[startID].distanceTo(nodePos[goalID]);
}

ArrayList<Integer> runAstar(Vec2[] nodePos, int numNodes, int startID, int goalID, agent myAgent){
  ArrayList<Integer> fringe = new ArrayList();  //New empty fringe
  ArrayList<Integer> path = new ArrayList();
  ArrayList<Float> priority = new ArrayList();  //New empty fringe
  float[] cost = new float[maxNumNodes];
  

  float rad = myAgent.rad;
  
  for (int i = 0; i < numNodes; i++) { //Clear visit tags and parent pointers
    visited[i] = false;
    parent[i] = -1; //No parent yet
    cost[i] = 999999;  //large enough
  }
  cost[startID] = 0;
  fringe.add(startID);
  priority.add(nodePos[startID].distanceTo(nodePos[goalID]));
  
  
  
  while (fringe.size() != 0){
    float min = 999999;
    int index = -1;
    for (int i = 0; i < fringe.size(); i++){  //get node with the lowest cost from fringe
      if (priority.get(i) < min){
        index = fringe.get(i);
        min = priority.get(i);
      }
    }
    int current = index;
    
    priority.remove(fringe.indexOf(current));
    fringe.remove(fringe.indexOf(current));
    
    if (current == goalID){
      break;
    }
    
    
    for (int i = 0; i < neighbors[current].size(); i++){
      int neighborNode = neighbors[current].get(i);
        float newCost = cost[current] + nodePos[current].distanceTo(nodePos[neighborNode]);
        if (cost[neighborNode] - 999999 > 0.01 || newCost < cost[neighborNode]){  //if(cost[neighborNode] != 999999 || ...)
          float pathLength = nodePos[current].distanceTo(nodePos[neighborNode]);
          Vec2 dir = nodePos[neighborNode].minus(nodePos[current]).normalized();
          hitInfo hit = rayCircleListIntersect(circlePos, circleRad, numObstacles, nodePos[current], dir, pathLength, rad);
          if (!hit.hit){
            cost[neighborNode] = newCost;
            fringe.add(neighborNode);
            priority.add(newCost + nodePos[neighborNode].distanceTo(nodePos[goalID]));
            parent[neighborNode] = current;
          }
        }
    }
  }
  
  if (fringe.size() == 0){
    //println("No Path");
    path.add(0,-1);
    return path;
  }
  

  
  //print("\nReverse path: ");
  int prevNode = parent[goalID];
  path.add(0,goalID);
  //print(goalID, " ");
  while (prevNode >= 0){
    //print(prevNode," ");
    path.add(0,prevNode);
    prevNode = parent[prevNode];
  }
  //print("\n");
  
  
  return path;
}
