int windows_x=1200;
int windows_y=800;

void setup(){
  size(1200,800);
}

//Set inital conditions
float w = 50;
float h = 200;
float w1= 50;
float h1 = 200;
float box_bounce = 0.8; //Coef. of restitution

float mass = 1;                         //Resistance to change in momentum/velocity
float rot_inertia = mass*(w*w+h*h)/12;  //Resistance to change in angular momentum/angular velocity
float mass1 = 0.8;
float rot_inertia1 = mass1*(w1*w1+h1*h1)/12;


Vec2 momentum = new Vec2(0,0);          //Speed the box is translating (derivative of position)
Vec2 momentum1 = new Vec2(0,0);
float angular_momentum = 0;             //Speed the box is rotating (derivative of angle)
float angular_momentum1 = 0;

Vec2 center = new Vec2(200,200);        //Current position of center of mass
float angle = 0.4; /*radians*/          //Current rotation amount (orientation)
Vec2 center1 = new Vec2(800,400);
float angle1 = 0.4;
float radius = 104;

Vec2 total_force = new Vec2(0,0);       //Forces change position (center of mass)
float total_torque = 0;                 //Torques change orientation (angle)
Vec2 total_force1 = new Vec2(0,0);
float total_torque1 = 0;

Vec2 p1,p2,p3,p4;                       //4 corners of the box -- computed in updateCornerPositions()
Vec2 p5,p6,p7,p8;

int arrow_dir = 0;

//----------
// Physics Functions
void apply_force(Vec2 force, Vec2 applied_position){
  total_force.add(force);
  //TODO: also update total_torque
  //displacement = force's position - object's center of mass
  //torque = cross(displacement,force)
  Vec2 displacement = applied_position.minus(center);
  total_torque = cross(displacement, force);
}

//void apply_force1(Vec2 force, Vec2 applied_position){
//  total_force1.add(force);
//  //TODO: also update total_torque
//  //displacement = force's position - object's center of mass
//  //torque = cross(displacement,force)
//  Vec2 displacement = applied_position.minus(center1);
//  total_torque1 = cross(displacement, force);
//}

void update_physics(float dt){
  //Update center of mass
  momentum.add(total_force.times(dt));     //Linear Momentum = Force * time
  Vec2 box_vel = momentum.times(1.0/mass); //Velocity = Momentum / mass
  
  center.add(box_vel.times(dt));           //Position += Vel * time
  
   angular_momentum += total_torque * dt;
   float box_avel = angular_momentum/rot_inertia;
   angle += box_avel*dt;

  //Reset forces and torques
  total_force = new Vec2(0,9.8*3); //Set forces to 0 after they've been applied
  /*TODO*/ //Set torques to 0 after the forces have been applied
  total_torque = 0;
}

void update_physics1(float dt){
  //Update center of mass
  momentum1.add(total_force1.times(dt));     //Linear Momentum = Force * time
  Vec2 box_vel1 = momentum1.times(1.0/mass1); //Velocity = Momentum / mass
  
  center1.add(box_vel1.times(dt));           //Position += Vel * time
  
   angular_momentum1 += total_torque1 * dt;
   float box_avel1 = angular_momentum1/rot_inertia1;
   angle1 += box_avel1*dt;

  //Reset forces and torques
  total_force1 = new Vec2(0,9.8*3); //Set forces to 0 after they've been applied
  /*TODO*/ //Set torques to 0 after the forces have been applied
  total_torque1 = 0;
}


class ColideInfo{
  public boolean hit = false;
  public Vec2 hitPoint = new Vec2(0,0);
  public Vec2 objectNormal =  new Vec2(0,0);
}


void updateCornerPositions(){
  Vec2 right = new Vec2(cos(angle),sin(angle)).times(w/2);
  Vec2 up = new Vec2(-sin(angle),cos(angle)).times(-h/2);
  p1 = center.plus(right).plus(up);
  p2 = center.plus(right).minus(up);
  p3 = center.minus(right).plus(up);
  p4 = center.minus(right).minus(up);
  right = new Vec2(cos(angle1),sin(angle1)).times(w1/2);
  up = new Vec2(-sin(angle1),cos(angle1)).times(-h1/2);
  p5 = center1.plus(right).plus(up);
  p6 = center1.plus(right).minus(up);
  p7 = center1.minus(right).plus(up);
  p8 = center1.minus(right).minus(up);
}

ColideInfo collisionTest(){
  updateCornerPositions(); //Compute the 4 corners: p1,p2,p3,p4
  //We only check if the corners collide
  
  ColideInfo info = new ColideInfo();
  
  if (p1.x > windows_x){
    info.hitPoint = p1;
    info.hit = true;
    info.objectNormal = new Vec2(-1,0);
  }
  if (p2.x > windows_x){
    info.hitPoint = p2;
    info.hit = true;
    info.objectNormal = new Vec2(-1,0);
  }
  if (p3.x > windows_x){
    info.hitPoint = p3;
    info.hit = true;
    info.objectNormal = new Vec2(-1,0);
  }
  if (p4.x > windows_x){
    info.hitPoint = p4;
    info.hit = true;
    info.objectNormal = new Vec2(-1,0);
  }
  //TODO: Test the 4 corners against the left wall
   if (p1.x < 0){
    info.hitPoint = p1;
    info.hit = true;
    info.objectNormal = new Vec2(-1,0);
  }
  if (p2.x < 0){
    info.hitPoint = p2;
    info.hit = true;
    info.objectNormal = new Vec2(-1,0);
  }
  if (p3.x < 0){
    info.hitPoint = p3;
    info.hit = true;
    info.objectNormal = new Vec2(-1,0);
  }
  if (p4.x < 0){
    info.hitPoint = p4;
    info.hit = true;
    info.objectNormal = new Vec2(-1,0);
  }
  float shift = 0;
  if (p1.y > windows_y){
    shift = p1.y - windows_y + 0.1;
    center.y -= shift;
    info.hitPoint = p1;
    info.hit = true;
    info.objectNormal = new Vec2(0,-1);
  }
  if (p2.y > windows_y){
    shift = p2.y - windows_y + 0.01;
    center.y -= shift;
    info.hitPoint = p2;
    info.hit = true;
    info.objectNormal = new Vec2(0,-1);
  }
  if (p3.y > windows_y){
    shift = p3.y - windows_y + 0.01;
    center.y -= shift;
    info.hitPoint = p3;
    info.hit = true;
    info.objectNormal = new Vec2(0,-1);
  }
  if (p4.y > windows_y){
    shift = p4.y - windows_y + 0.01;
    center.y -= shift;
    info.hitPoint = p4;
    info.hit = true;
    info.objectNormal = new Vec2(0,-1);
  }
  
  return info;
}

ColideInfo collisionTest1(){
  updateCornerPositions(); //Compute the 4 corners: p5,p6,p7,p8
  //We only check if the corners collide
  
  ColideInfo info = new ColideInfo();
  
  if (p5.x > windows_x){
    info.hitPoint = p5;
    info.hit = true;
    info.objectNormal = new Vec2(-1,0);
  }
  if (p6.x > windows_x){
    info.hitPoint = p6;
    info.hit = true;
    info.objectNormal = new Vec2(-1,0);
  }
  if (p7.x > windows_x){
    info.hitPoint = p7;
    info.hit = true;
    info.objectNormal = new Vec2(-1,0);
  }
  if (p8.x > windows_x){
    info.hitPoint = p8;
    info.hit = true;
    info.objectNormal = new Vec2(-1,0);
  }
  //TODO: Test the 4 corners against the left wall
   if (p5.x < 0){
    info.hitPoint = p5;
    info.hit = true;
    info.objectNormal = new Vec2(-1,0);
  }
  if (p6.x < 0){
    info.hitPoint = p6;
    info.hit = true;
    info.objectNormal = new Vec2(-1,0);
  }
  if (p7.x < 0){
    info.hitPoint = p7;
    info.hit = true;
    info.objectNormal = new Vec2(-1,0);
  }
  if (p8.x < 0){
    info.hitPoint = p8;
    info.hit = true;
    info.objectNormal = new Vec2(-1,0);
  }
  float shift = 0;
  if (p5.y > windows_y){
    shift = p5.y - windows_y + 0.1;
    center1.y -= shift;
    info.hitPoint = p5;
    info.hit = true;
    info.objectNormal = new Vec2(0,-1);
  }
  if (p6.y > windows_y){
    shift = p6.y - windows_y + 0.01;
    center1.y -= shift;
    info.hitPoint = p6;
    info.hit = true;
    info.objectNormal = new Vec2(0,-1);
  }
  if (p7.y > windows_y){
    shift = p7.y - windows_y + 0.01;
    center1.y -= shift;
    info.hitPoint = p7;
    info.hit = true;
    info.objectNormal = new Vec2(0,-1);
  }
  if (p8.y > windows_y){
    shift = p8.y - windows_y + 0.01;
    center1.y -= shift;
    info.hitPoint = p8;
    info.hit = true;
    info.objectNormal = new Vec2(0,-1);
  }
  return info;
}

//Updates momentum & angular_momentum based on collision using an impulse based method
//This method assumes you hit an immovable obstacle which simplifies the math
// see Eqn 8-18 of here: https://www.cs.cmu.edu/~baraff/sigcourse/notesd2.pdf
// or Eqn 9 here: http://www.chrishecker.com/images/e/e7/Gdmphys3.pdf
//for obstacle-obstacle collisions.
void resolveCollision(Vec2 hit_point, Vec2 hit_normal, float dt){
  Vec2 r = hit_point.minus(center);
  Vec2 r_perp = perpendicular(r);
  Vec2 object_vel = momentum.times(1/mass);
  float object_angular_speed = angular_momentum/rot_inertia;
  Vec2 point_vel = object_vel.plus(r_perp.times(object_angular_speed));
  float j = -(1+box_bounce)*dot(point_vel,hit_normal);
  j /= (1/mass + pow(dot(r_perp,hit_normal),2)/rot_inertia);
 
  Vec2 impulse = hit_normal.times(j);
  if (impulse.length()>5){
    momentum.add(impulse);
    angular_momentum += dot(r_perp,impulse);
  }
  else{
    momentum=new Vec2(0,0);
    angular_momentum=0;
  }
  update_physics(1.01*dt); //A small hack, better is just to move the object out of collision directly
}

void resolveCollision1(Vec2 hit_point, Vec2 hit_normal, float dt){
  Vec2 r1 = hit_point.minus(center1);
  Vec2 r_perp1 = perpendicular(r1);
  Vec2 object_vel1 = momentum1.times(1/mass1);
  float object_angular_speed1 = angular_momentum1/rot_inertia1;
  Vec2 point_vel = object_vel1.plus(r_perp1.times(object_angular_speed1));
  float j = -(1+box_bounce)*dot(point_vel,hit_normal);
  j /= (1/mass1 + pow(dot(r_perp1,hit_normal),2)/rot_inertia1);
 
  Vec2 impulse1 = hit_normal.times(j);
  if (impulse1.length()>5){
    momentum1.add(impulse1);
    angular_momentum1 += dot(r_perp1,impulse1);
  }
  else{
    momentum1=new Vec2(0,0);
    angular_momentum1=0;
  }
  update_physics1(1.01*dt); //A small hack, better is just to move the object out of collision directly
}

void draw(){
  float dt = 1/frameRate;
  update_physics(dt);
  update_physics1(dt);
  
  boolean clicked_box = mousePressed && point_in_box(new Vec2(mouseX, mouseY),center,w,h,angle);
  Vec2 force = new Vec2(0,0);
  if (clicked_box) {
    if (arrow_dir == 0){
      println("arrow_dir:", arrow_dir);
      force = new Vec2(0,-1).times(100);
    }
    if (arrow_dir == 1){
      force = new Vec2(0,1).times(100);
    }
    if (arrow_dir == 2){
      force = new Vec2(-1,0).times(100);
    }
    if (arrow_dir == 3){
      force = new Vec2(1,0).times(100);
    }
    Vec2 hit_point = new Vec2(mouseX, mouseY);
    apply_force(force, hit_point);
  }
  
  ColideInfo info = collisionTest(); //TODO: Use this result below
  ColideInfo info1 = collisionTest1();
  
  //TODO the these values based on the results of a collision test
  Boolean hit_something = info.hit; //Did I hit something?
  if (hit_something){
    Vec2 hit_point = info.hitPoint;
    Vec2 hit_normal = info.objectNormal;
    
    resolveCollision(hit_point,hit_normal,dt);
  }
  
  Boolean hit_something1 = info1.hit;
  if (hit_something1){
    Vec2 hit_point1 = info1.hitPoint;
    Vec2 hit_normal1 = info1.objectNormal;
    
    resolveCollision1(hit_point1,hit_normal1,dt);
  }
  
  Vec2 box_vel = momentum.times(1/mass);
  float box_speed = box_vel.length();
  float box_agular_velocity = angular_momentum/rot_inertia;
  float linear_kinetic_energy = .5*mass*box_speed*box_speed;
  float rotational_kinetic_energy = .5*rot_inertia*box_agular_velocity*box_agular_velocity;
  float total_kinetic_energy = linear_kinetic_energy+rotational_kinetic_energy;
  println("Box Vel:",box_vel,"ang vel:",box_agular_velocity,"linear KE:",linear_kinetic_energy,"rotation KE:",rotational_kinetic_energy,"Total KE:",total_kinetic_energy);
  
  background(200); //Grey background
  
  fill(255);
  if (clicked_box){
    fill(255,200,200);
  }
  pushMatrix();
  translate(center.x,center.y);
  rotate(angle);
  rect(-w/2, -h/2, w, h);
  popMatrix();
  
  pushMatrix();
  translate(center1.x,center1.y);
  rotate(angle1);
  rect(-w1/2, -h1/2, w1, h1);
  popMatrix();
  
  fill(0);
  circle(center.x, center.y, 6.0);
  circle(center1.x, center1.y, 6.0);
  
  circle(p1.x, p1.y, 4.0);
  circle(p2.x, p2.y, 4.0);
  circle(p3.x, p3.y, 4.0);
  circle(p4.x, p4.y, 4.0);
  
  circle(p5.x, p5.y, 4.0);
  circle(p6.x, p6.y, 4.0);
  circle(p7.x, p7.y, 4.0);
  circle(p8.x, p8.y, 4.0);
  
  drawArrow(mouseX, mouseY, 100, 0);
  
}


void keyPressed(){
  if (key == 'r'){
    center = new Vec2(200,600);
    momentum = new Vec2(0,0);
    angular_momentum = 0;
    angle = radians(90);
    println("Resetting the simulation");
    return;
  }

  if (keyCode == UP){
    arrow_dir = 0;
    return;
  }
  if (keyCode == DOWN){
    arrow_dir = 1;
    return;
  }
  if (keyCode == LEFT){
    arrow_dir = 2;
    return;
  }
  if (keyCode == RIGHT){
    arrow_dir = 3;
    return;
  }
}

//Returns true iff the point 'point' is inside the box
boolean point_in_box(Vec2 point, Vec2 box_center, float box_w, float box_h, float box_angle){
  Vec2 relative_pos = point.minus(box_center);
  Vec2 box_right = new Vec2(cos(box_angle),sin(box_angle));
  Vec2 box_up = new Vec2(sin(box_angle),cos(box_angle));
  float point_right = dot(relative_pos,box_right);
  float point_up = dot(relative_pos,box_up);
  if ((abs(point_right) < box_w/2) && (abs(point_up) < box_h/2))
    return true;
  return false;
}

void drawArrow(int cx, int cy, int len, float angle){
  pushMatrix();
  translate(cx, cy);
  rotate(radians(angle));
  line(-len,0,0, 0);
  line(0, 0,  - 8, -8);
  line(0, 0,  - 8, 8);
  popMatrix();
}


boolean mutualCollisionTest(){
  if (center.minus(center1).length()>104){
    return false;
  }
  
  return false;
}


//---------------
//Vec 2 Library
//---------------

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
  
  public float lengthSqr(){
    return x*x+y*y;
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

//2D cross product is a funny concept
// ...its the 3D cross product but with z = 0
// ... (only the resulting z compontent is not zero so we just store is as a scalar)
float cross(Vec2 a, Vec2 b){
  return a.x*b.y - a.y*b.x;
}

Vec2 projAB(Vec2 a, Vec2 b){
  return b.times(a.x*b.x + a.y*b.y);
}

Vec2 perpendicular(Vec2 a){
  return new Vec2(-a.y,a.x);
}
