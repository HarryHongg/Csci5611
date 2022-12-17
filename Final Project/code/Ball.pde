class Ball {


  Body body;
  float r;

  color col;


  Ball(float x, float y, float r_, float launchMag_, float launchAngle_) {
    r = r_;
    makeBody(x, y, r, launchMag_, launchAngle_);
    body.setUserData(this);
    col = color(175);
  }

  void killBody() {
    box2d.destroyBody(body);
  }

  void change() {
    col = color(255, 0, 0);
  }

  boolean done() {
    Vec2 pos = box2d.getBodyPixelCoord(body);
    if (pos.y > height+r*2) {
      killBody();
      return true;
    }
    return false;
  }
  


  void display(PImage img) {
    
    Vec2 pos = box2d.getBodyPixelCoord(body);
    float a = body.getAngle();
    pushMatrix();
    translate(pos.x, pos.y);
    rotate(-a);
    fill(col);
    stroke(0);
    strokeWeight(1);
    imageMode(CENTER);
    image(img, 0, 0, r*2, r*2);
    //ellipse(0, 0, r*2, r*2);
    line(0, 0, r, 0);
    popMatrix();
  }

  void makeBody(float x, float y, float r, float launchMag_, float launchAngle_) {
    BodyDef bd = new BodyDef();
    bd.position = box2d.coordPixelsToWorld(x, y);
    bd.type = BodyType.DYNAMIC;
    body = box2d.createBody(bd);

    CircleShape cs = new CircleShape();
    cs.m_radius = box2d.scalarPixelsToWorld(r);

    FixtureDef fd = new FixtureDef();
    fd.shape = cs;
    fd.density = 1;
    fd.friction = 0.01;
    fd.restitution = 0.3;

    body.createFixture(fd);
    body.setLinearVelocity(new Vec2(launchMag_*cos(launchAngle_), launchMag_*sin(launchAngle_)));
    println(new Vec2(launchMag_*cos(launchAngle_), launchMag_*sin(launchAngle_)), launchMag_, launchAngle_);
    body.setAngularVelocity(random(-10, 10));
  }
}
