class Fragment {

  Body body;
  float w;
  float h;
  color col;


  Fragment(float x, float y, float w_, float h_) {
    w = w_;
    h = h_;
    makeBody(x, y, w_, h_);
    body.setUserData(this);
    col = color(128,128,128);
  }

  void killBody() {
    box2d.destroyBody(body);
  }



  boolean done() {
    Vec2 pos = box2d.getBodyPixelCoord(body);
    if (pos.y > height+w*h) {
      killBody();
      return true;
    }
    else if (col == color(255, 0, 0)){
      killBody();
      return true;
    }
    return false;
  }


  void display() {
    Vec2 pos = box2d.getBodyPixelCoord(body);
    float a = body.getAngle();
    pushMatrix();
    translate(pos.x, pos.y);
    rotate(-a);
    fill(col);
    stroke(0);
    strokeWeight(1);
    rect(0, 0, w, h);
    //triangle(-w/2,h/2,0,h/2,w/2,-h/2);
    popMatrix();
  }

  void makeBody(float x, float y, float w_, float h_) {
    BodyDef bd = new BodyDef();
    bd.position = box2d.coordPixelsToWorld(x, y);
    bd.type = BodyType.DYNAMIC;
    body = box2d.createBody(bd);

    PolygonShape sd = new PolygonShape();
    float box2dW = box2d.scalarPixelsToWorld(w_/2);
    float box2dH = box2d.scalarPixelsToWorld(h_/2);
    sd.setAsBox(box2dW, box2dH);
    
    //PolygonShape sd = new PolygonShape();
    //Vec2[] vertices = new Vec2[3];
    //vertices[0] = new Vec2(-w_/4,h_/4);
    //vertices[1] = new Vec2(0,h_/4);
    //vertices[2] = new Vec2(w_/4,-h_/4);
    //sd.set(vertices, 3);

    FixtureDef fd = new FixtureDef();
    fd.shape = sd;
    fd.density = 1;
    fd.friction = 0.01;
    fd.restitution = 0.3;

    body.createFixture(fd);

    body.setAngularVelocity(random(-10, 10));
  }
}
