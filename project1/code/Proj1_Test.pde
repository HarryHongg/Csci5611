//CSCI 5611 - Graph Search & Planning
//PRM Sample Code [Proj 1]
//Instructor: Stephen J. Guy <sjguy@umn.edu>

//This is a test harness designed to help you test & debug your PRM.

//USAGE:
// On start-up your PRM will be tested on a random scene and the results printed
// Left clicking will set a red goal, right clicking the blue start
// The arrow keys will move the circular obstacle with the heavy outline
// Pressing 'r' will randomize the obstacles and re-run the tests
PImage img;

float k_goal = 10;
float k_avoid = 100;

//Change the below parameters to change the scenario/roadmap size
int numObstacles = 100;
int numNodes  = 100;
int maxNumAgents = 5;
int numAgents = 0;
int maxNumParticles = 1000;
int numParticles = 0;
float genRate = 20;
  
class agent{
  public int id;
  public Vec2 pos;
  public Vec2 vel = new Vec2 (0,0);
  public Vec2 acc;
  public Vec2 goal;
  public Vec2 curGoal;
  public ArrayList<Integer> path;
  public int posInPath = 0;
  public Vec2 tempGoal;
  boolean colliding = false;
  float rad = 10;
  boolean reachingGoal = false;
}


agent agentList[] = new agent[maxNumAgents];
float goalSpeed = 100;


Vec2 parPos[] = new Vec2[maxNumParticles];
Vec2 parVel[] = new Vec2[maxNumParticles];
float parLife[] = new float[maxNumParticles];
boolean ifGen[] = new boolean[maxNumAgents];



//A list of circle obstacles
static int maxNumObstacles = 1000;
Vec2 circlePos[] = new Vec2[maxNumObstacles]; //Circle positions
float circleRad[] = new float[maxNumObstacles];  //Circle radii




static int maxNumNodes = 1000;
Vec2[] nodePos = new Vec2[maxNumNodes];


//Generate non-colliding PRM nodes
void generateRandomNodes(int numNodes, Vec2[] circleCenters, float[] circleRadii, float rad){
  for (int i = 0; i < numNodes; i++){
    Vec2 randPos = new Vec2(random(width),random(height));
    boolean insideAnyCircle = pointInCircleListWithRad(circleCenters,circleRadii,numObstacles,randPos,2,rad);
    //boolean insideBox = pointInBox(boxTopLeft, boxW, boxH, randPos);
    while (insideAnyCircle){
      randPos = new Vec2(random(width),random(height));
      insideAnyCircle = pointInCircleListWithRad(circleCenters,circleRadii,numObstacles,randPos,2,rad);
      //insideBox = pointInBox(boxTopLeft, boxW, boxH, randPos);
    }
    nodePos[i] = randPos;
  }
}

void placeRandomObstacles(int numObstacles){
  //Initial obstacle position
  for (int i = 0; i < numObstacles; i++){
    circlePos[i] = new Vec2(random(50,950),random(50,700));
    circleRad[i] = (10+40*pow(random(1),3));
  }
  circleRad[0] = 30; //Make the first obstacle big
}



int strokeWidth = 2;
void setup(){
  size(1024,768,P3D);
  lights();
  camera(1024/2, 1100, 500.0, 1024/2, 768/2, 0.0, 0.0, 0.0, -1.0);

  
  placeRandomObstacles(numObstacles);
  for (int i = 0; i < 5; i ++){
    agentList[i] = sampleFreeAgent(i);
    numAgents += 1;
  }
  
  

  generateRandomNodes(numNodes, circlePos, circleRad, agentList[0].rad);
  connectNeighbors(circlePos, circleRad, numObstacles, nodePos, numNodes, agentList);
   
  while (true){
    for (int i = 0; i < numAgents; i ++){
      agentList[i].path = planPath(agentList[i], circlePos, circleRad, numObstacles, nodePos, numNodes);
      if( agentList[i].path.get(0) != -1){
        agentList[i].curGoal = nodePos[agentList[i].path.get(0)];
      }
      else{
        println(agentList[i].id, "has no path");
        generateRandomNodes(numNodes, circlePos, circleRad, agentList[0].rad);
        connectNeighbors(circlePos, circleRad, numObstacles, nodePos, numNodes, agentList);
        continue;
      }
    }
    break;
  }
}



agent sampleFreeAgent(int id){
  agent myAgent = new agent();
  myAgent.id = id;
  myAgent.pos = new Vec2(random(width),random(height));
  myAgent.goal = new Vec2(random(width),random(height));
  boolean insideAnyCircle = pointInCircleListWithRad(circlePos,circleRad,numObstacles,myAgent.pos,2,myAgent.rad);
  while (insideAnyCircle){
    myAgent.pos = new Vec2(random(width),random(height));
    insideAnyCircle = pointInCircleListWithRad(circlePos,circleRad,numObstacles,myAgent.pos,2,myAgent.rad);
  }
  insideAnyCircle = pointInCircleListWithRad(circlePos,circleRad,numObstacles,myAgent.goal,2,myAgent.rad);
  while (insideAnyCircle){
    myAgent.goal = new Vec2(random(width),random(height));
    insideAnyCircle = pointInCircleListWithRad(circlePos,circleRad,numObstacles,myAgent.goal,2,myAgent.rad);
  }
  return myAgent;
}
//
//
//
//
//
//
//
Vec2 computeAgentForces(int id){
  agent thisAgent = agentList[id];
  Vec2 acc = new Vec2(0,0);
  if (thisAgent.path.get(0) == -1){
    thisAgent.curGoal = thisAgent.pos;
  }
  else{
    thisAgent.curGoal = nodePos[thisAgent.path.get(thisAgent.posInPath)];
  }

  Vec2 goalVel = (thisAgent.curGoal).minus(thisAgent.pos);

  if (thisAgent.posInPath < thisAgent.path.size() - 1){    //If in intermediate node
    if (goalVel.length() < 1){                           //Turning to other node
      thisAgent.posInPath += 1;
      thisAgent.curGoal = nodePos[thisAgent.path.get(thisAgent.posInPath)];
      goalVel = (thisAgent.curGoal).minus(thisAgent.pos);
    }
    else if (goalVel.length() < 40){                  //a minimum speed to avoid hesitation
      goalVel.setToLength(40);
    }
  }
  else if (thisAgent.path.get(0) != -1){                                          //Moveing toward final goal
    thisAgent.curGoal = thisAgent.goal;
    goalVel = (thisAgent.curGoal).minus(thisAgent.pos);
    if (goalVel.length()< 0.1){
      ifGen[id] = true;
    }
  }
  
  if (goalVel.length() > goalSpeed){    //a maximum speed to avoid large changes in speed
    goalVel.setToLength(goalSpeed);
  }



  
  Vec2 goalForce = goalVel.minus(agentList[id].vel);
  goalForce.mul(k_goal);
  acc.add(goalForce);
  
  if (goalVel.length() < 5) return acc;
  
  for(int i=0; i<numAgents; i++){
    if(id != i){
      float ttc = computeTTC(thisAgent.pos, thisAgent.vel, thisAgent.rad, agentList[i].pos, agentList[i].vel, agentList[i].rad);
      if(ttc > 0){
        Vec2 future_A = thisAgent.pos.plus(thisAgent.vel.times(ttc));
        Vec2 future_B = agentList[i].pos.plus(agentList[i].vel.times(ttc));
        Vec2 relative_future_direction = (future_A.minus(future_B)); 
        relative_future_direction.normalize();
        acc.add(relative_future_direction.times(1/ttc).times(k_avoid));
      }
    }
  }

  return acc;
}



void moveAgent(float dt){
  //Compute accelerations for every agents
  for (int i = 0; i < numAgents; i++){
    agentList[i].acc = computeAgentForces(i);
  }
  //Update position and velocity using (Eulerian) numerical integration
  for (int i = 0; i < numAgents; i++){
    agentList[i].vel.add(agentList[i].acc.times(dt));
    agentList[i].pos.add(agentList[i].vel.times(dt));
  }
  
  
  //particles
  float toGen_float = genRate * dt;
  int toGen = int(toGen_float);
  float fractPart = toGen_float - toGen;
  if (random(1) < fractPart) toGen += 1;
  
  float r = random(20);
  float theta = random(2*PI);
  
  for (int p = 0; p < maxNumAgents; p ++){
    if (ifGen[p]){
      for (int i = 0; i < toGen; i++){
        if (numParticles >= maxNumParticles) break;
        parPos[numParticles] = new Vec2(agentList[p].pos.x+r*sin(theta), agentList[p].pos.y+r*cos(theta));
        parVel[numParticles] = new Vec2(agentList[p].pos.x+r*sin(theta), agentList[p].pos.y+r*cos(theta));  
        numParticles += 1;
      }
      for (int i = 0; i <  numParticles; i++){
        Vec2 acc = agentList[p].pos.minus(parPos[i]);
        parPos[i].add(parVel[i].times(dt));
        parVel[i].add(acc.times(dt));
        parLife[i] += dt;
      }
    }
  }
}

//void generatePar(float dt){
//  float toGen_float = genRate * dt;
//  int toGen = int(toGen_float);
//  float fractPart = toGen_float - toGen;
//  if (random(1) < fractPart) toGen += 1;
  
//  float r = random(20);
//  float theta = random(2*PI);
  
//  for (int p = 0; p < maxNumAgents; p ++){
//    if (ifGen[p]){
//      for (int i = 0; i < toGen; i++){
//        if (numParticles >= maxNumParticles) break;
//        parPos[numParticles] = new Vec2(agentList[p].pos.x+r*sin(theta), agentList[p].pos.y+r*cos(theta));
//        parVel[numParticles] = new Vec2(agentList[p].pos.x+r*sin(theta), agentList[p].pos.y+r*cos(theta));  
//        numParticles += 1;
//      }
//      for (int i = 0; i <  numParticles; i++){
      
//        Vec2 acc = agentList[p].pos.minus(parPos[i]);
//        parPos[i].add(parVel[i].times(dt));
//        parVel[i].add(acc.times(dt));
//        parLife[i] += dt;
//      }
//    }
//  }
//}


boolean paused = true;
void draw(){
  
  
  
  
  //println("FrameRate:",frameRate);
  strokeWeight(1);
  background(200   ); //Grey background
  stroke(0,0,0);
  fill(255,255,255);
  
  //draw graph
  stroke(100,100,100);
  strokeWeight(1);
  for (int i = 0; i < numNodes; i++){
    for (int j : neighbors[i]){
      line(nodePos[i].x,nodePos[i].y,nodePos[j].x,nodePos[j].y);
    }
  }
  
  
  //Draw the circle obstacles
  for (int i = 0; i < numObstacles; i++){
    Vec2 c = circlePos[i];
    float r = circleRad[i];
    //circle(c.x,c.y,r*2);
    drawCylinder(10, c.x, c.y, r, 30);
  }
  
  //Draw the first circle a little special b/c the user controls it
  fill(240);
  strokeWeight(2);
  circle(circlePos[0].x,circlePos[0].y,circleRad[0]*2);
  strokeWeight(1);
  
  
  if (!paused){
    moveAgent(1.0/frameRate);
  }

  
  //Draw agents
  for (int i = 0; i < numAgents; i++){
    fill(255,255,0);

    drawAgent( agentList[i].pos.x, agentList[i].pos.y, agentList[i].rad*2, 30);

    //circle(agentList[i].pos.x,agentList[i].pos.y,agentList[i].rad*2);
  }
  
  
  
  //Draw goal
  for (int i = 0; i < numAgents; i++){
    fill(250,30,50);
    drawCylinder(10, agentList[i].goal.x, agentList[i].goal.y, 20, 35);
    //circle(agentList[i].goal.x,agentList[i].goal.y,20);
  }
  
  //Draw particles
  for (int i = 0; i < numParticles; i++){
    fill(234, 221, 202);
    circle(parPos[i].x, parPos[i].y, 3);

  }
  
  //if (myAgent.path.size() >0 && curPath.get(0) == -1) return; //No path found

}

void drawAgent(float xPos, float yPos, float r, float h)
{
    beginShape();
     vertex(xPos, yPos-r, 0);
     vertex(xPos-r/2, yPos+r, 0);
     vertex(xPos+r/2, yPos+r, 0);
    endShape(CLOSE);
    
    
    beginShape();
     vertex(xPos, yPos-r, h/2);
     vertex(xPos-r/2, yPos+r, h/2);
     vertex(xPos+r/2, yPos+r, h/2);
    endShape(CLOSE);
    

    beginShape();
     vertex(xPos, yPos-r, 0);
     vertex(xPos, yPos-r, h/2);
     vertex(xPos+r/2, yPos+r, h/2);
     vertex(xPos+r/2, yPos+r, 0);
    endShape(CLOSE);
    

    beginShape();
     vertex(xPos, yPos-r, 0);
     vertex(xPos, yPos-r, h/2);
     vertex(xPos-r/2, yPos+r, h/2);
     vertex(xPos-r/2, yPos+r, 0);
    endShape(CLOSE);

    beginShape();
     vertex(xPos-r/2, yPos+r, 0);
     vertex(xPos-r/2, yPos+r, h/2);
     vertex(xPos+r/2, yPos+r, h/2);
     vertex(xPos+r/2, yPos+r, 0);
    endShape(CLOSE);
}

void drawCylinder(int sides, float xPos, float yPos, float r, float h)
{
    float angle = 360 / sides;
    // top
    beginShape();
    for (int i = 0; i < sides; i++) {
        float x = cos( radians( i * angle ) ) * r;
        float y = sin( radians( i * angle ) ) * r;
        vertex( x+xPos, y+yPos, 0);

    }
    endShape(CLOSE);
    // bot
    beginShape();
    for (int i = 0; i < sides; i++) {
        float x = cos( radians( i * angle ) ) * r;
        float y = sin( radians( i * angle ) ) * r;
        vertex( x+xPos, y+yPos, h);
    }
    endShape(CLOSE);

    // draw body
    beginShape(TRIANGLE_STRIP);
    
    for (int i = 0; i < sides + 1; i++) {
        float x1 = cos( radians( i * angle ) ) * r;
        float y1 = sin( radians( i * angle ) ) * r;
        float x2 = cos( radians( i * angle ) ) * r;
        float y2 = sin( radians( i * angle ) ) * r;
        vertex( x1+xPos, y1+yPos, 0);
        vertex( x2+xPos, y2+yPos, h);
    }
    endShape(CLOSE);
}


boolean shiftDown = false;
void keyPressed(){
  if (key == ' ') paused = !paused;
  

  
  if (keyCode == SHIFT){
    shiftDown = true;
  }
  
  float speed = 10;
  if (shiftDown) speed = 30;
  if (keyCode == RIGHT){
    circlePos[0].x += speed;
  }
  if (keyCode == LEFT){
    circlePos[0].x -= speed;
  }
  if (keyCode == UP){
    circlePos[0].y -= speed;
  }
  if (keyCode == DOWN){
    circlePos[0].y += speed;
  }
  //connectNeighbors(circlePos, circleRad, numObstacles, nodePos, numNodes);
  //curPath = planPath(startPos, goalPos, circlePos, circleRad, numObstacles, nodePos, numNodes);
}

void keyReleased(){
  if (keyCode == SHIFT){
    shiftDown = false;
  }
}

//Pause/unpause the simulation


void mousePressed(){
  Vec2 point = new Vec2(mouseX, mouseY);
  if (pointInCircle(circlePos[0], circleRad[0], point, 2)){
    
  }
  if (mouseButton == RIGHT){
    agentList[0].pos = new Vec2(mouseX, mouseY);
    connectNeighbors(circlePos, circleRad, numObstacles, nodePos, numNodes, agentList);
    agentList[0].path = planPath(agentList[0], circlePos, circleRad, numObstacles, nodePos, numNodes);
    agentList[0].posInPath = 0;
    while (agentList[0].path.get(0) == -1){
      generateRandomNodes(numNodes, circlePos, circleRad, agentList[0].rad);
      connectNeighbors(circlePos, circleRad, numObstacles, nodePos, numNodes, agentList);
      agentList[0].path = planPath(agentList[0], circlePos, circleRad, numObstacles, nodePos, numNodes);
    }
    println(agentList[0].path);

  }
    //println("New Start is",startPos.x, startPos.y);

  else{
    circlePos[0] = new Vec2(mouseX, mouseY);
    connectNeighbors(circlePos, circleRad, numObstacles, nodePos, numNodes, agentList);
    for (int i = 0; i < numAgents; i ++){
      agentList[i].path = planPath(agentList[i], circlePos, circleRad, numObstacles, nodePos, numNodes);
      agentList[i].posInPath = 0;
    }

    //println("New Goal is",goalPos.x, goalPos.y);
  }
}
  
