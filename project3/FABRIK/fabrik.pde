class FABRIK {
  private int num; // Number of points on the arm
  Vec2 start; // Start point of the arm
  Vec2 goal; // Point to go on
  float[] lines; // Length of all shafts connecting points
  float[] angles; // Angles of all shaft (counter-clockwise from the +x axis)
  Vec2[] points; // Points of the arm
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
  
  /*
  Calculate point possition from lengths and angles
  May be used if angles and lengths are known from a servo
  In this example, it is more a way to init points
  */
  public void updatePoint() {
    points[0] = newV(start); // First point at the start
    for (int i = 1; i < num; ++i) {
      /*
      dir is the vector representing the shaft at its angle
      add this vector to the previous position to find the current point
      */
      Vec2 dir = new Vec2(cos(angles[i - 1]), sin(angles[i - 1])).times(lines[i - 1]);
      points[i] = newV(points[i-1]).plus(dir);
    }
  }
  
  /*
  Calculate angles from points position
  May be used to write position of servos
  In this example, it is NOT used
  */
  public void updateAngles() {
    for (int i = 0; i < num - 1; ++i) {
      /*
      Take two consecutive points
      Calculate the vector in between
      Then calculate the angle
      */
      Vec2 a = newV(points[i]);
      Vec2 b = newV(points[i + 1]);
    
      Vec2 dir = b.minus(a);
      angles[i] = atan2(dir.y, dir.x); // atan2 is used to have an angle [0, 2*pi] whereas tan is [-pi/2, pi/2]
    }
  }
  
  /*
  Start from the goal and adjust point position to the start point
  */
  void backward() {
    points[num - 1] = newV(goal); // Set last point on goal
    for (int i = num - 1; i > 0; --i) {
      /*
      Place the previous point on the line between the current point and the current position of the previous point
      At a distance of the length of the shaft
      Repeat for all point from the last one to the first
      */
      Vec2 a = newV(points[i]);
      Vec2 b = newV(points[i-1]);

      Vec2 dir = newV(b).minus(a);
      dir.normalize();
      dir.mul(lines[i - 1]);
    
      points[i - 1] = a.plus(dir);
    }
  }
  /*
  Start from the start and adjust point position to the goal point
  */
  public void forward() {
    /*
    Same thing as backward() but forward
    */
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
  
  /*
  Run backward and forward 'times' times
  */
  public void fit() { fit(1); }
  public void fit(int times)
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
