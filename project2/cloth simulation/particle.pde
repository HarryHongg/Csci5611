  class particle {
  float mass = 0.1;
  PVector pos = new PVector(0.0, 0.0, 0.0);
  PVector iniPos = new PVector(0.0, 0.0, 0.0);
  PVector vel = new PVector(0.0, 0.0, 0.0);
  PVector acc = new PVector(0.0, 0.0, 0.0);
  PVector force = new PVector(0.0, 0.0, 0.0);
  boolean fixed = false;
  PVector id = new PVector(0.0, 0.0, 0.0);
  ArrayList<PVector> connected = new ArrayList<PVector>();

  //Constructor
  particle(int row, int col, int interval, boolean fix, PVector id) {
    this.id = id;
    this.iniPos.set((row-1)*interval, (col-1)*interval, 0.0);
    this.pos.set(this.iniPos);
    this.fixed = fix;
    this.force.set(0, 0, 0);
    this.vel.set(0, 0, 0);
  }

  boolean isFixed() {
    return this.fixed;
  }

  void setPos(float x, float y) {
    this.pos.set(x, y);
  }

  //Draw the particle 
  public void show() {
    //noStroke();
    //lights();
    //translate(this.pos.x, this.pos.y, this.pos.z+ trans_Z);
    //sphere(7);
    fill(255);
    ellipse(this.pos.x, this.pos.y, particleR, particleR);
  }

  PVector getPos() {
    return this.pos;
  }
  
}
