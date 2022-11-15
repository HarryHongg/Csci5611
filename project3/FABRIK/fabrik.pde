class FABRIK {
  private int num;
  Vec2 start;
  Vec2 goal;
  float[] lines;
  float[] angles;
  Vec2[] points;
  Vec2[] jointLimits;
  

  float armW = 20;
  float legW1 = 35;
  float legW2 = 30;
  
  public FABRIK(int segments, Vec2 startPoint, Vec2 goalPoint) {
    num = segments;
    start = newV(startPoint);
    goal = newV(goalPoint);
    lines = new float[num - 1];
    angles = new float[num - 1];
    points = new Vec2[num];
    jointLimits = new Vec2[num- 1 ];
    
    for (int i = 0; i < num - 1; ++i){
      lines[i] = 10;
    }
    for (int i = 0; i < num - 1; ++i){
      angles[i] = 0;
    }
    for (int i = 0; i < num - 1; ++i){
      jointLimits[0] = new Vec2(0,0);
    }
  } 
  

  public void updatePoint() {
    points[0] = newV(start);
    for (int i = 1; i < num; ++i) {
      Vec2 dir = new Vec2(cos(angles[i - 1]), sin(angles[i - 1])).times(lines[i - 1]);
      points[i] = newV(points[i-1]).plus(dir);
    }
  }
  

  public void updateAngles() {
    for (int i = 0; i < num - 1; ++i) {
      Vec2 a = newV(points[i]);
      Vec2 b = newV(points[i + 1]);
    
      Vec2 dir = b.minus(a);
      angles[i] = atan2(dir.y, dir.x);
    }
  }

  void backward() {
    points[num - 1] = newV(goal);
    for (int i = num - 1; i > 0; --i) {

      Vec2 a = newV(points[i]);
      Vec2 b = newV(points[i-1]);

      Vec2 dir = newV(b).minus(a);
      dir.normalize();
      dir.mul(lines[i - 1]);
    
      points[i - 1] = a.plus(dir);
    }
  }

  public void forward() {
    points[0] = newV(start);
    for (int i = 1; i < num; ++i) {
      Vec2 a = newV(points[i-1]);
      Vec2 b = newV(points[i]);
      
      Vec2 dir = newV(b).minus(a);
      dir.normalize();
      dir.mul(lines[i - 1]);
      points[i] = a.plus(dir);
    }
  }

  public void solve() { solve(1); }
  public void solve(int times)
  {
    for (int i = 0; i < times; ++i)
    {
      backward();
      forward();
    }
    updateAngles();
  }
  
  public void setLength(int index, float length) {
    if (index < num)
    {
      lines[index] = length;
    }
  }
  
  public void setGoal(Vec2 point) {
    goal = newV(point);
  }
  
  public void setRoot(Vec2 point) {
    start = newV(point);
  }
  
  public void setJointLimit(int i, Vec2 limit){
    jointLimits[i] = newV(limit);
  }
  
  public void drawArm() {
    fill(252,224,203);
    for (int i = 0; i < num - 1; ++i) {
      Vec2 a = newV(points[i]);
      pushMatrix();
      translate(a.x,a.y);
      rotate(angles[i]);
      rect(0, -armW/2, lines[i], armW);
      popMatrix();
    }
  }
  public void drawLeg() {
    fill(252,224,203);
    
    Vec2 a = newV(points[0]);
    pushMatrix();
    translate(a.x,a.y);
    rotate(angles[0]);
    rect(0, -legW1/2, lines[0], legW1);
    popMatrix();
    
    a = newV(points[1]);
    pushMatrix();
    translate(a.x,a.y);
    rotate(angles[1]);
    rect(0, -legW1/2, lines[1], legW1);
    popMatrix();

  }
}
