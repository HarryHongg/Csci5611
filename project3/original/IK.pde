void setup(){
  size(1080,660);
  surface.setTitle("Inverse Kinematics");
}
//Body
float bodyWidth = 100.0;
float bodyHeight = 190.0;
Vec2 start_body = new Vec2(400,260);

//Right Arm Root
Vec2 rArmRoot = new Vec2(start_body.x+bodyWidth/2,start_body.y-bodyHeight/2);

float ra0 = 100; 
float raa0 = 0.3; //Shoulder joint

float ra1 = 90;
float raa1 = 0.3; //Elbow joint

float ra2 = 20;
float raa2 = 0.3; //Wrist joint

//Left Arm
Vec2 lArmRoot = new Vec2(start_body.x-bodyWidth/2,start_body.y-bodyHeight/2);

float la0 = 100; 
float laa0 = 0.3;

float la1 = 90; 
float laa1 = 0.3;

float la2 = 20; 
float laa2 = 0.3;



//Right Leg
Vec2 rLegRoot = new Vec2(start_body.x+bodyWidth/2-10,start_body.y+bodyHeight/2);

float rl0 = 130; 
float rla0 = 0.3;

float rl1 = 110; 
float rla1 = 0.3;

//Left Leg
Vec2 lLegRoot = new Vec2(start_body.x-bodyWidth/2+10,start_body.y+bodyHeight/2);

float ll0 = 130; 
float lla0 = 0.3;

float ll1 = 110; 
float lla1 = 0.3;

float lWalk = 0.1;
float rWalk = 0.0;
boolean moveL = true;
Vec2 velocity = new Vec2 (0,0);
float speed = 40;



Vec2 start_ra1,start_ra2,endPoint_ra;
Vec2 start_la1,start_la2,endPoint_la;
Vec2 start_rl1,endPoint_rl;
Vec2 start_ll1,endPoint_ll;

void solve(float dt){
  Vec2 goal = new Vec2(mouseX, mouseY);
  
  Vec2 startToGoal, startToEndEffector;
  float dotProd, angleDiff;
  
  //Update wrist joint
  startToGoal = goal.minus(start_ra2);
  startToEndEffector = endPoint_ra.minus(start_ra2);
  dotProd = dot(startToGoal.normalized(),startToEndEffector.normalized());
  dotProd = clamp(dotProd,-1,1);
  angleDiff = acos(dotProd);
  if (cross(startToGoal,startToEndEffector) < 0)
    raa2 += angleDiff;
  else
    raa2 -= angleDiff;
   
  if (raa2 > PI/2){
    raa2 = PI/2;
  }
  else if (raa2 < -PI/2){
    raa2 = -PI/2;
  }
  fk(dt);
  
  //Update elbow joint
  startToGoal = goal.minus(start_ra1);
  startToEndEffector = endPoint_ra.minus(start_ra1);
  dotProd = dot(startToGoal.normalized(),startToEndEffector.normalized());
  dotProd = clamp(dotProd,-1,1);
  angleDiff = acos(dotProd);
  if (cross(startToGoal,startToEndEffector) < 0)
    raa1 += angleDiff;
  else
    raa1 -= angleDiff;
  fk(dt);
  
  
  //Update shoulder joint
  startToGoal = goal.minus(rArmRoot);
  if (startToGoal.length() < .0001) return;
  startToEndEffector = endPoint_ra.minus(rArmRoot);
  dotProd = dot(startToGoal.normalized(),startToEndEffector.normalized());
  dotProd = clamp(dotProd,-1,1);
  angleDiff = acos(dotProd);
  if (cross(startToGoal,startToEndEffector) < 0)
    raa0 += angleDiff;
  else
    raa0 -= angleDiff;
  if (raa0 > PI/2){
    raa0 = PI/2;
  }
  else if (raa0 < -radians(80)){
    raa0 = -radians(80);
  }
  fk(dt);
  
  
  //Left Arm
  
  startToGoal = goal.minus(start_la2);
  startToEndEffector = endPoint_la.minus(start_la2);
  dotProd = dot(startToGoal.normalized(),startToEndEffector.normalized());
  dotProd = clamp(dotProd,-1,1);
  angleDiff = acos(dotProd);
  if (cross(startToGoal,startToEndEffector) < 0)
    laa2 += angleDiff;
  else
    laa2 -= angleDiff;
   
  if (laa2 > PI/2){
    laa2 = PI/2;
  }
  else if (laa2 < -PI/2){
    laa2 = -PI/2;
  }
  fk(dt);
  

  startToGoal = goal.minus(start_la1);
  startToEndEffector = endPoint_la.minus(start_la1);
  dotProd = dot(startToGoal.normalized(),startToEndEffector.normalized());
  dotProd = clamp(dotProd,-1,1);
  angleDiff = acos(dotProd);
  if (cross(startToGoal,startToEndEffector) < 0)
    laa1 += angleDiff;
  else
    laa1 -= angleDiff;
  fk(dt);
  
  

  startToGoal = goal.minus(lArmRoot);
  if (startToGoal.length() < .0001) return;
  startToEndEffector = endPoint_la.minus(lArmRoot);
  dotProd = dot(startToGoal.normalized(),startToEndEffector.normalized());
  dotProd = clamp(dotProd,-1,1);
  angleDiff = acos(dotProd);
  if (cross(startToGoal,startToEndEffector) < 0)
    laa0 += angleDiff;
  else
    laa0 -= angleDiff;
  if (laa0 > radians(260)){
    laa0 = radians(260);
  }
  else if (laa0 < PI/2){
    laa0 = PI/2;
  }
  fk(dt);
 
  //Right leg
  Vec2 rlGoal = new Vec2(400+bodyWidth/2-5+2*rWalk,260+bodyHeight/2+rl0+rl1+stepH(rWalk)-10);
  startToGoal = rlGoal.minus(start_rl1);
  startToEndEffector = endPoint_rl.minus(start_rl1);
  dotProd = dot(startToGoal.normalized(),startToEndEffector.normalized());
  dotProd = clamp(dotProd,-1,1);
  angleDiff = acos(dotProd);
  if (cross(startToGoal,startToEndEffector) < 0)
    rla1 += angleDiff;
  else
    rla1 -= angleDiff;
  if (rla1 > radians(160)){
    rla1 = radians(160);
  }
  else if (rla1 < 0){
    rla1 = 0;
  }
  fk(dt);
  

  startToGoal = rlGoal.minus(rLegRoot);
  if (startToGoal.length() < .0001) return;
  startToEndEffector = endPoint_rl.minus(rLegRoot);
  dotProd = dot(startToGoal.normalized(),startToEndEffector.normalized());
  dotProd = clamp(dotProd,-1,1);
  angleDiff = acos(dotProd);
  if (cross(startToGoal,startToEndEffector) < 0)
    rla0 += angleDiff;
  else
    rla0 -= angleDiff;
  if (rla0 > radians(130)){
    rla0 = radians(130);
  }
  else if (rla0 < -radians(80)){
    rla0 = -radians(80);
  }
  fk(dt);
  
  //Left leg
  Vec2 llGoal = new Vec2(400-bodyWidth/2+5+2*lWalk,260+bodyHeight/2+ll0+ll1+stepH(lWalk)-10);
  startToGoal = llGoal.minus(start_ll1);
  startToEndEffector = endPoint_ll.minus(start_ll1);
  dotProd = dot(startToGoal.normalized(),startToEndEffector.normalized());
  dotProd = clamp(dotProd,-1,1);
  angleDiff = acos(dotProd);
  if (cross(startToGoal,startToEndEffector) < 0)
    lla1 += angleDiff;
  else
    lla1 -= angleDiff;
  if (lla1 > radians(160)){
    lla1 = radians(160);
  }
  else if (lla1 < 0){
    lla1 = 0;
  }
  fk(dt);
  

  startToGoal = llGoal.minus(lLegRoot);
  if (startToGoal.length() < .0001) return;
  startToEndEffector = endPoint_ll.minus(lLegRoot);
  dotProd = dot(startToGoal.normalized(),startToEndEffector.normalized());
  dotProd = clamp(dotProd,-1,1);
  angleDiff = acos(dotProd);
  if (cross(startToGoal,startToEndEffector) < 0)
    lla0 += angleDiff;
  else
    lla0 -= angleDiff;
  if (lla0 > radians(130)){
    lla0 = radians(130);
  }
  else if (lla0 < -radians(80)){
    lla0 = -radians(80);
  }
  fk(dt);
  println(llGoal,rlGoal);
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
  if (lWalk - rWalk < 0.0001 || lWalk - rWalk > 90){
    moveL = !moveL;
  }

  if (moveL){
    lWalk += velocity.times(dt).x; 
  }
  else{
    rWalk += velocity.times(dt).x; 
  }
  rArmRoot = new Vec2(start_body.x+bodyWidth/2,start_body.y-bodyHeight/2);
  lArmRoot = new Vec2(start_body.x-bodyWidth/2,start_body.y-bodyHeight/2);
  rLegRoot = new Vec2(start_body.x+bodyWidth/2-10,start_body.y+bodyHeight/2);
  lLegRoot = new Vec2(start_body.x-bodyWidth/2+10,start_body.y+bodyHeight/2);
}

void fk(float dt){
  start_ra1 = new Vec2(cos(raa0)*ra0,sin(raa0)*ra0).plus(rArmRoot);
  start_ra2 = new Vec2(cos(raa0+raa1)*ra1,sin(raa0+raa1)*ra1).plus(start_ra1);
  endPoint_ra = new Vec2(cos(raa0+raa1+raa2)*ra2,sin(raa0+raa1+raa2)*ra2).plus(start_ra2);
  
  start_la1 = new Vec2(cos(laa0)*la0,sin(laa0)*la0).plus(lArmRoot);
  start_la2 = new Vec2(cos(laa0+laa1)*la1,sin(laa0+laa1)*la1).plus(start_la1);
  endPoint_la = new Vec2(cos(laa0+laa1+laa2)*la2,sin(laa0+laa1+laa2)*la2).plus(start_la2);
  
  start_rl1 = new Vec2(cos(rla0)*rl0,sin(rla0)*rl0).plus(rLegRoot);
  endPoint_rl = new Vec2(cos(rla0+rla1)*rl1,sin(rla0+rla1)*rl1).plus(start_rl1);
  
  start_ll1 = new Vec2(cos(lla0)*ll0,sin(lla0)*ll0).plus(lLegRoot);
  endPoint_ll = new Vec2(cos(lla0+lla1)*ll1,sin(lla0+lla1)*ll1).plus(start_ll1);
}

float armW = 20;
float legW1 = 35;
float legW2 = 30;
void draw(){
  float dt = 1.0/frameRate;
  move(dt);
  fk(dt);
  solve(dt);
  
  background(250,250,250);
  

  fill(252,224,203);
  //Right Arm
  pushMatrix();
  translate(rArmRoot.x,rArmRoot.y);
  rotate(raa0);
  rect(0, -armW/2, ra0, armW);
  popMatrix();
  
  pushMatrix();
  translate(start_ra1.x,start_ra1.y);
  rotate(raa0+raa1);
  rect(0, -armW/2, ra1, armW);
  popMatrix();
  
  pushMatrix();
  translate(start_ra2.x,start_ra2.y);
  rotate(raa0+raa1+raa2);
  rect(0, -armW/2, ra2, armW);
  popMatrix();
  
  //Left Arm
  pushMatrix();
  translate(lArmRoot.x,lArmRoot.y);
  rotate(laa0);
  rect(0, -armW/2, la0, armW);
  popMatrix();
  
  pushMatrix();
  translate(start_la1.x,start_la1.y);
  rotate(laa0+laa1);
  rect(0, -armW/2, la1, armW);
  popMatrix();
  
  pushMatrix();
  translate(start_la2.x,start_la2.y);
  rotate(laa0+laa1+laa2);
  rect(0, -armW/2, la2, armW);
  popMatrix();
  
  //Body
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
  
  //Right leg
  pushMatrix();
  translate(rLegRoot.x,rLegRoot.y);
  rotate(rla0);
  rect(0, -legW1/2, rl0, legW1);
  popMatrix();
  
  pushMatrix();
  translate(start_rl1.x,start_rl1.y);
  rotate(rla0+rla1);
  rect(0, -legW2/2, rl1, legW2);
  popMatrix();
  
  //Leg leg
  pushMatrix();
  translate(lLegRoot.x,lLegRoot.y);
  rotate(lla0);
  rect(0, -legW1/2, ll0, legW1);
  popMatrix();
  
  pushMatrix();
  translate(start_ll1.x,start_ll1.y);
  rotate(lla0+lla1);
  rect(0, -legW2/2, ll1, legW2);
  popMatrix();
  
  
  //pushMatrix();
  //translate(start_rl1.x,start_rl1.y);
  //circle(0, 0, 10);
  //popMatrix();
  
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


//-----------------
// Vector Library
//-----------------

//Vector Library
//CSCI 5611 Vector 2 Library [Example]
// Stephen J. Guy <sjguy@umn.edu>

public class Vec2 {
  public float x, y;
  
  public Vec2(float x, float y){
    this.x = x;
    this.y = y;
  }
  
  public String toString(){
    return "(" + x+ "," + y +")";
  }
  
  public float length(){
    return sqrt(x*x+y*y);
  }
  
  public Vec2 plus(Vec2 rhs){
    return new Vec2(x+rhs.x, y+rhs.y);
  }
  
  public void add(Vec2 rhs){
    x += rhs.x;
    y += rhs.y;
  }
  
  public Vec2 minus(Vec2 rhs){
    return new Vec2(x-rhs.x, y-rhs.y);
  }
  
  public void subtract(Vec2 rhs){
    x -= rhs.x;
    y -= rhs.y;
  }
  
  public Vec2 times(float rhs){
    return new Vec2(x*rhs, y*rhs);
  }
  
  public void mul(float rhs){
    x *= rhs;
    y *= rhs;
  }
  
  public void clampToLength(float maxL){
    float magnitude = sqrt(x*x + y*y);
    if (magnitude > maxL){
      x *= maxL/magnitude;
      y *= maxL/magnitude;
    }
  }
  
  public void setToLength(float newL){
    float magnitude = sqrt(x*x + y*y);
    x *= newL/magnitude;
    y *= newL/magnitude;
  }
  
  public void normalize(){
    float magnitude = sqrt(x*x + y*y);
    x /= magnitude;
    y /= magnitude;
  }
  
  public Vec2 normalized(){
    float magnitude = sqrt(x*x + y*y);
    return new Vec2(x/magnitude, y/magnitude);
  }
  
  public float distanceTo(Vec2 rhs){
    float dx = rhs.x - x;
    float dy = rhs.y - y;
    return sqrt(dx*dx + dy*dy);
  }
}

Vec2 interpolate(Vec2 a, Vec2 b, float t){
  return a.plus((b.minus(a)).times(t));
}

float interpolate(float a, float b, float t){
  return a + ((b-a)*t);
}

float dot(Vec2 a, Vec2 b){
  return a.x*b.x + a.y*b.y;
}

float cross(Vec2 a, Vec2 b){
  return a.x*b.y - a.y*b.x;
}


Vec2 projAB(Vec2 a, Vec2 b){
  return b.times(a.x*b.x + a.y*b.y);
}

float clamp(float f, float min, float max){
  if (f < min) return min;
  if (f > max) return max;
  return f;
}
