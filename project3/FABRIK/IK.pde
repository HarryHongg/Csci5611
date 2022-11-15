float bodyWidth = 100.0;
float bodyHeight = 190.0;
Vec2 velocity = new Vec2 (0,0);
float speed = 40;
Vec2 start_body = new Vec2(400,260);

float rWalk = 0.5;
float lWalk = 0;
boolean moveL = true;

Vec2 goal = new Vec2(mouseX, mouseY);
Vec2 rLegGoal = new Vec2(400+bodyWidth/2-5+2*rWalk,260+bodyHeight/2+130+110+stepH(rWalk)-10);
Vec2 lLegGoal = new Vec2(400-bodyWidth/2+5+2*lWalk,260+bodyHeight/2+130+110+stepH(lWalk)-10);
FABRIK rArm = new FABRIK(4, new Vec2(start_body.x+bodyWidth/2,start_body.y-bodyHeight/2), goal);
FABRIK lArm = new FABRIK(4, new Vec2(start_body.x-bodyWidth/2,start_body.y-bodyHeight/2), goal);
FABRIK rLeg = new FABRIK(3, new Vec2(start_body.x+bodyWidth/2-10,start_body.y+bodyHeight/2), rLegGoal);
FABRIK lLeg = new FABRIK(3, new Vec2(start_body.x-bodyWidth/2+10,start_body.y+bodyHeight/2), lLegGoal);
void setup()
{
  size(1080, 660);
  rArm.setLength(0, 100);
  rArm.setLength(1, 90);
  rArm.setLength(2, 20);
  rArm.updatePoint();
  
  lArm.setLength(0, 100);
  lArm.setLength(1, 90);
  lArm.setLength(2, 20);
  lArm.updatePoint();
  
  rLeg.setLength(0, 130);
  rLeg.setLength(1, 110);
  rLeg.updatePoint();
  
  lLeg.setLength(0, 130);
  lLeg.setLength(1, 110);
  lLeg.updatePoint();
}

void draw()
{
  background(250,250,250);
  float dt = 1.0/frameRate;
  move(dt);
  // set goal to mouse pos
  goal = new Vec2(mouseX, mouseY);

  // move points
  rArm.fit(1);
  rArm.setGoal(goal);
  lArm.fit(1);
  lArm.setGoal(goal);
  rLeg.fit(1);
  rLeg.setGoal(rLegGoal);
  lLeg.fit(1);
  lLeg.setGoal(lLegGoal);
  // Draw arm
  rArm.drawArm();
  lArm.drawArm();
  rLeg.drawLeg();
  lLeg.drawLeg();
  
  pushMatrix();
  translate(start_body.x,start_body.y);
  rotate(0);
  rect(-bodyWidth/2, -bodyHeight/2, bodyWidth, bodyHeight);
  popMatrix();
  
  pushMatrix();
  translate(start_body.x,start_body.y);
  rotate(0);
  circle(0, -bodyHeight/2-30, 80);
  popMatrix();
}

void move(float dt){
  velocity = new Vec2(0,0);
  if (leftPressed) {
    velocity.add(new Vec2(-speed,0));
  }
  if (rightPressed) {
    velocity.add(new Vec2(speed,0));
  }
  if (velocity.x != 0  && velocity.y != 0){
    velocity.normalize();
    velocity.mul(speed);
  }
  start_body.add(velocity.times(dt));
  if (lWalk - rWalk < 90){
    
  }
  if (lWalk - rWalk < 0.001 || lWalk - rWalk > 90){
    moveL = !moveL;
  }

  if (moveL){
    lWalk += velocity.times(dt).x; 
  }
  else{
    rWalk += velocity.times(dt).x; 
  }
  rArm.setRoot(new Vec2(start_body.x+bodyWidth/2,start_body.y-bodyHeight/2));
  lArm.setRoot(new Vec2(start_body.x-bodyWidth/2,start_body.y-bodyHeight/2));
  rLeg.setRoot(new Vec2(start_body.x+bodyWidth/2-10,start_body.y+bodyHeight/2));
  lLeg.setRoot(new Vec2(start_body.x-bodyWidth/2+10,start_body.y+bodyHeight/2));
  rLeg.setGoal(new Vec2(400+bodyWidth/2-5+2*rWalk,260+bodyHeight/2+130+110+stepH(rWalk)-10));
  lLeg.setGoal(new Vec2(400-bodyWidth/2+5+2*lWalk,260+bodyHeight/2+130+110+stepH(lWalk)-10));
  println(lWalk,rWalk);
}

boolean leftPressed, rightPressed;
void keyPressed(){
  if (keyCode == LEFT) leftPressed = true;
  if (keyCode == RIGHT) rightPressed = true;
}

void keyReleased(){
  if (keyCode == LEFT) leftPressed = false;
  if (keyCode == RIGHT) rightPressed = false;
}

float stepH(float walk){
  float w = abs(walk);
  return -45*sin(((w%90)/90)*PI);
}
