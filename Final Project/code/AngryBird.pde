import shiffman.box2d.*;
import org.jbox2d.common.*;
import org.jbox2d.dynamics.joints.*;
import org.jbox2d.collision.shapes.*;
import org.jbox2d.collision.shapes.Shape;
import org.jbox2d.common.*;
import org.jbox2d.dynamics.*;
import org.jbox2d.dynamics.contacts.*;


Box2DProcessing box2d;


ArrayList<Box> Boxes;
ArrayList<Fragment> Fragments;
PImage bird;

Boundary wall;
Ball b;
float fx;
float fy;
float fw;
float fh;
boolean frag = false;
boolean pause = false;
float launchAngle = radians(-30);
float launchMag = 10;
boolean launch = false;
boolean launched = false;

void setup() {
  size(1200,700);
  smooth();
  bird = loadImage("bird.png");

  box2d = new Box2DProcessing(this);
  box2d.createWorld();


  box2d.listenForCollisions();


  Boxes = new ArrayList<Box>();
  Fragments = new ArrayList<Fragment>();
  
  wall = new Boundary(width/2, height-5, width, 10);
  
  
  Boxes.add(new Box(800, 700-75, 100, 150));
  Boxes.add(new Box(800-80, 700-160, 150, 25));
  Boxes.add(new Box(800+80, 700-160, 150, 25));
  Boxes.add(new Box(800, 700-180, 150, 80));
}

void draw() {
  background(255);
  
  //if (random(1) < 0.1) {
  //  float sz1 = random(24, 32);
  //  float sz2 = random(32, 48);
  //  Boxes.add(new Box(random(width), 20, sz1, sz2));
  //}
  
  if (frag){
    createFrag(fx, fy, fw, fh);
    frag = false;
  }
  
  if (!pause){
    box2d.step();
  }
  
  

  for (int i = Boxes.size()-1; i >= 0; i--) {
    Box p = Boxes.get(i);
    p.display();
    if (p.done()) {
      Boxes.remove(i);
    }
  }
  if (Fragments.size()>0){
    for (int i = Fragments.size()-1; i >= 0; i--) {
      Fragment f = Fragments.get(i);
      f.display();
      if (f.done()) {
        Fragments.remove(i);
      }
    }
  }
  wall.display();
  
  pushMatrix();
  translate(340, 500);
  fill(color(175));
  stroke(0);
  strokeWeight(1);
  //ellipse(0, 0, 80, 80);
  imageMode(CENTER);
  image(bird, 0, 0, 80, 80);
  popMatrix();
  
  if (b!=null){
    b.display(bird);
  }
  if (launch){
    if (b!=null){
      b.killBody();
    }
    b =new Ball(340, 500, 40, launchMag, -launchAngle);
    launch = false;
  }
  drawArrow(340, 500, launchMag*4, launchAngle);
  
  
}


void beginContact(Contact cp) {
  

}

void endContact(Contact cp) {
  Fixture f1 = cp.getFixtureA();
  Fixture f2 = cp.getFixtureB();
  Body b1 = f1.getBody();
  Body b2 = f2.getBody();

  Object o1 = b1.getUserData();
  Object o2 = b2.getUserData();

  if (o1.getClass() == Box.class && o2.getClass() == Ball.class) {
    Box p1 = (Box) o1;
    p1.change();
    Vec2 pos = box2d.getBodyPixelCoord(b1);
    frag = true;
    fx = pos.x;
    fy = pos.y;
    fw = p1.w;
    fh = p1.h;
  }
  else if (o1.getClass() == Ball.class && o2.getClass() == Box.class){
    Box p2 = (Box) o2;
    p2.change();
    Vec2 pos = box2d.getBodyPixelCoord(b2);
    frag = true;
    fx = pos.x;
    fy = pos.y;
    fw = p2.w;
    fh = p2.h;
  }
}

void createFrag(float x, float y, float w_, float h_){
  Fragments.add(new Fragment(x-w_/2, y-h_/2, w_/2, h_/2));
  Fragments.add(new Fragment(x-w_/2, y+h_/2, w_/2, h_/2));
  Fragments.add(new Fragment(x+w_/2, y-h_/2, w_/2, h_/2));
  Fragments.add(new Fragment(x+w_/2, y+h_/2, w_/2, h_/2));
}

void keyPressed() {
  if (keyPressed && keyCode == ' ') {
    pause = !pause;
  }
  if (keyPressed && (keyCode == 'l' || keyCode == 'L')) {
    launch = !launch;
  }
  if (keyCode == LEFT) launchAngle -= radians(10);
  if (keyCode == RIGHT) launchAngle += radians(10);
  if (keyCode == UP) launchMag += 1; 
  if (keyCode == DOWN) launchMag -= 1; 

}


void drawArrow(int cx, int cy, float len, float angle){
  pushMatrix();
  translate(cx, cy);
  rotate(angle);
  line(0, 0, len, 0);
  line(len, 0, len - 8, -8);
  line(len, 0, len - 8, 8);
  popMatrix();
}
