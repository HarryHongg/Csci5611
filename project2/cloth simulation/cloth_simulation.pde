int height = 1000;
int width = 1200;
int col = 30;
int row = 30;
int FPS = 60;
int trans_X = 0;
int trans_Y = 0;
int trans_Z = 0;
int INTERVAL = 10;
int Area = INTERVAL*INTERVAL;
int particleR = 7;
particle[][] particles = new particle[row][col];

//Constants
float k_string = -1000;
float k_damping = -2;
float k_air = 0.006;
float mass = 0.03;
float ts = 0.001; //timestep
float COR = 0.0000;
float shpereR = 50;
boolean hasSphere = false;
int k_gravity = 2000;
PVector gravity = new PVector(0, -9.8*k_gravity, 0);


PVector selectedP = new PVector(0, 0);
boolean selected = false;

// Wind
int k_wind = 300;
boolean hasWind = true;

//camera
float c_r_x = 0;
float c_r_y = 0;
float c_t_x = 500;
float c_t_y = 400;
float c_t_z = 0;

//Rip
boolean canRip = false;

//Air resistance
boolean hasAir = true;



void settings() {
  size(width, height, P3D);
  createLattice();
}


void setup() {
  frameRate(60);
  drawLattice();
}




void draw() {

  
  beginCamera();
  camera(600, 500, 500 / tan(PI*30.0 / 180.0), 600, 500, 0, 0, 1, 0);
  translate(c_t_x, c_t_y, c_t_z);
  rotateX(c_r_x);
  rotateY(c_r_y);
  endCamera();


  stroke(0);
  background(color(0, 0, 0));
  
  if (hasSphere){
    fill(255);
    ellipse(100, 200, shpereR, shpereR);
  }
  
  //Draw the lattice
  drawLattice();
  updateParticles(); //Update forces acting on particles and positions according to Euler



  

  //stroke(255);
}


void createLattice() {
  
  for (int r=0; r<row; r++) {
    for (int c=0; c<col; c++) {
      PVector id = new PVector(c,r,0);
      if (r == 0) {
        particles[c][r] = new particle(c, r, INTERVAL, true, id);//fixed
      } else {
        particles[c][r] = new particle(c, r, INTERVAL, false, id);
      }
      //top
      if (r > 0) {
        particles[c][r].connected.add(new PVector(c, r-1, 0));
      }

      //bottom
      if (r < row-1) {
        particles[c][r].connected.add(new PVector(c, r+1, 0));
      }
      
      //left
      if (c > 0) {
        particles[c][r].connected.add(new PVector(c-1, r, 0));
      }
      
      
      //right
      if (c < col-1) {
        particles[c][r].connected.add(new PVector(c+1, r, 0));
      }


      //top right
      if (r > 0 && c < col-1) {
        particles[c][r].connected.add(new PVector(c+1, r-1, 0));
      }

      //diagonal, bottom left
      if (r < row-1 && c > 0) {
        particles[c][r].connected.add(new PVector(c-1, r+1, 0));
      }

      //bottom right
      if (r < row-1 && c < col-1) {
        particles[c][r].connected.add(new PVector(c+1, r+1, 0));
      }

      //top left
      if (r > 0 && c > 0) {
         particles[c][r].connected.add(new PVector(c-1, r-1, 0));
      }
    }
  }
}

// Draw lattice
void drawLattice() {
  fill(255);
  strokeWeight(1);
  stroke(255);

  for (int y=0; y<row; y++) {

    for (int x=0; x<col; x++) {
        particles[y][x].show();
     }
    }
  }

 


void updateParticles() {
  PVector fs_bl, fs_b, fs_r, fs_tr, fs_t, fs_l, fs_br, fs_tl, fd_bl, fd_b, fd_r, fd_tr, fd_t, fd_l, fd_br, fd_tl, dis;
  float curL, L; 
  

  float xoff = 0;
  for (int r = 0; r < row; r++) {
    float yoff = 0;
    for (int c = 0; c < col; c++) {
      if (particles[c][r].fixed == true) continue;

      // String
      fs_t = new PVector(0.0, 0.0, 0.0);
      fs_b = new PVector(0.0, 0.0, 0.0);
      fs_l = new PVector(0.0, 0.0, 0.0);
      fs_r = new PVector(0.0, 0.0, 0.0);
      fs_bl = new PVector(0.0, 0.0, 0.0);
      fs_tr = new PVector(0.0, 0.0, 0.0);
      fs_br = new PVector(0.0, 0.0, 0.0);
      fs_tl = new PVector(0.0, 0.0, 0.0);

      // Damping
      fd_t = new PVector(0.0, 0.0, 0.0);
      fd_b = new PVector(0.0, 0.0, 0.0);
      fd_l = new PVector(0.0, 0.0, 0.0);
      fd_r = new PVector(0.0, 0.0, 0.0);
      fd_bl = new PVector(0.0, 0.0, 0.0);
      fd_tr = new PVector(0.0, 0.0, 0.0);
      fd_br = new PVector(0.0, 0.0, 0.0);
      fd_tl = new PVector(0.0, 0.0, 0.0);


      
      //top
      if (r > 0) {
        if (particles[c][r].connected.contains(new PVector(c, r-1, 0))){
          L = particles[c][r].iniPos.dist(particles[c][r-1].iniPos);//rest length
          dis = PVector.sub(particles[c][r].pos, particles[c][r-1].pos);//current length with direction
          curL = dis.mag();//current length
          if (canRip && curL > L*4.5){
            particles[c][r].connected.remove(new PVector(c, r-1, 0));
          }
          float s = curL - L;//current length - rest length
          PVector j = PVector.div(dis, curL);//direction
          fs_t = PVector.mult(j, -k_string*s);
          PVector n = PVector.sub(particles[c][r].vel, particles[c][r-1].vel);
          fd_t = PVector.mult(n, -k_damping);
        }
      }

      //bottom
      if (r < row-1) {
        if (particles[c][r].connected.contains(new PVector(c, r+1, 0))){
          L = particles[c][r].iniPos.dist(particles[c][r+1].iniPos);
          dis = PVector.sub(particles[c][r].pos, particles[c][r+1].pos);
          curL = dis.mag();
          if (canRip && curL > L*4.5){
            particles[c][r].connected.remove(new PVector(c, r+1, 0));
          }
          float s = curL - L;
          PVector j = PVector.div(dis, curL);
          fs_b = PVector.mult(j, -k_string*s);
          PVector n = PVector.sub(particles[c][r].vel, particles[c][r+1].vel);
          fd_b = PVector.mult(n, -k_damping);
        }
      }
      
      //left
      if (c > 0) {
        if (particles[c][r].connected.contains(new PVector(c-1, r, 0))){
          L = particles[c][r].iniPos.dist(particles[c-1][r].iniPos);
          dis = PVector.sub(particles[c][r].pos, particles[c-1][r].pos);
          curL = dis.mag();
          if (canRip && curL > L*4.5){
            particles[c][r].connected.remove(new PVector(c-1, r, 0));
          }
          float s = curL - L;
          PVector j = PVector.div(dis, curL);
          fs_l = PVector.mult(j, -k_string*s);
          PVector n = PVector.sub(particles[c][r].vel, particles[c-1][r].vel);
          fd_l = PVector.mult(n, -k_damping);
        }
      }
      
      
      //right
      if (c < col-1) {
        if (particles[c][r].connected.contains(new PVector(c+1, r, 0))){
          L = PVector.dist(particles[c][r].iniPos, particles[c+1][r].iniPos);
          dis = PVector.sub(particles[c][r].pos, particles[c+1][r].pos);
          curL = dis.mag();
          if (canRip && curL > L*4.5){
            particles[c][r].connected.remove(new PVector(c+1, r, 0));
          }
          float s = curL - L;
          PVector j = PVector.div(dis, curL);
          fs_r = PVector.mult(j, -k_string*s);
          PVector n = PVector.sub(particles[c][r].vel, particles[c+1][r].vel);
          fd_r = PVector.mult(n, -k_damping);
        }
      }


      //top right
      if (r > 0 && c < col-1) {
        if (particles[c][r].connected.contains(new PVector(c+1, r-1, 0))){
          L = particles[c][r].iniPos.dist(particles[c+1][r-1].iniPos);
          dis = PVector.sub(particles[c][r].pos, particles[c+1][r-1].pos);
          curL = dis.mag();
          if (canRip && curL > L*4.5){
            particles[c][r].connected.remove(new PVector(c+1, r-1, 0));
          }
          float s = curL - L;
          PVector j = PVector.div(dis, curL);
          fs_tr = PVector.mult(j, -k_string*s);
          PVector n = PVector.sub(particles[c][r].vel, particles[c+1][r-1].vel);
          fd_tr = PVector.mult(n, -k_damping);
        }
      }

      //diagonal, bottom left
      if (r < row-1 && c > 0) {
        if (particles[c][r].connected.contains(new PVector(c-1, r+1, 0))){
          L = particles[c][r].iniPos.dist(particles[c-1][r+1].iniPos);
          dis = PVector.sub(particles[c][r].pos, particles[c-1][r+1].pos);
          curL = dis.mag();    
          if (canRip && curL > L*4.5){
            particles[c][r].connected.remove(new PVector(c-1, r+1, 0));
          }       
          float s = curL - L;
          PVector j = dis.div(curL);
          fs_bl = PVector.mult(j, -k_string*s);
          PVector n = PVector.sub(particles[c][r].vel, particles[c-1][r+1].vel);
          fd_bl = n.mult(-k_damping);
        }
      }

      //bottom right
      if (r < row-1 && c < col-1) {
        if (particles[c][r].connected.contains(new PVector(c+1, r+1, 0))){
          L = particles[c][r].iniPos.dist(particles[c+1][r+1].iniPos);
          dis = PVector.sub(particles[c][r].pos, particles[c+1][r+1].pos);
          curL = dis.mag();
          if (canRip && curL > L*4.5){
            particles[c][r].connected.remove(new PVector(c+1, r+1, 0));
          }
          float s = curL - L;
          PVector j = PVector.div(dis, curL);
          fs_br = PVector.mult(j, -k_string*s);
          PVector n = PVector.sub(particles[c][r].vel, particles[c+1][r+1].vel);
          fd_br = PVector.mult(n, -k_damping);
        }
      }

      //top left
      if (r > 0 && c > 0) {
        if (particles[c][r].connected.contains(new PVector(c-1, r-1, 0))){
          L = particles[c][r].iniPos.dist(particles[c-1][r-1].iniPos);
          dis = PVector.sub(particles[c][r].pos, particles[c-1][r-1].pos);
          curL = dis.mag();
          if (canRip && curL > L*4.5){
            particles[c][r].connected.remove(new PVector(c-1, r-1, 0));
          }
          float s = curL - L;
          PVector j = PVector.div(dis, curL);
          fs_tl = PVector.mult(j, -k_string*s);
          PVector n = PVector.sub(particles[c][r].vel, particles[c-1][r-1].vel);
          fd_tl = PVector.mult(n, -k_damping);
        }
      }
      
      //drag force
      PVector dragForce =  new PVector(0,0,0);
      if (hasAir){
        dragForce =  dragForce.sub(particles[c][r].vel).mult(k_air*Area);//drag force is relative to the particls's velocity, the area of a piece of cloth and some constant
      }
      
      
      //wind simulation
      PVector wind = new PVector(0, 0, 0);
      if (hasWind){
        float wx = map(noise(yoff, xoff), 0, 1, 0, 10);
        float wy = map(noise(yoff, xoff), -90, 1, -90, 0);
        float wz = map(noise(yoff, xoff), -100, 1, -100, 0);
        wind = new PVector(wx, wy, wz).mult(k_wind); 
      }

      //collision with shpere
      PVector spherePos = new PVector(100, 200, 0);
      if (hasSphere){
        if (pointInCircle(spherePos, (shpereR+particleR)/2, particles[c][r].pos)){
           PVector normal = PVector.sub(particles[c][r].pos, spherePos);
           if (normal.mag()>1){
             normal.normalize();
             particles[c][r].pos.set(spherePos.add(normal.mult(shpereR/2).mult(1.001)));
           }
        }
      }


      PVector f = new PVector(0, 0, 0);
      particles[c][r].force = f.sub(fs_bl).sub(fs_b).sub(fs_r).sub(fs_tr).sub(fs_t).sub(fs_l).sub(fs_br)
        .sub(fs_tl).sub(fd_bl).sub(fd_b).sub(fd_r).sub(fd_tr).sub(fd_t).sub(fd_l).sub(fd_br).sub(fd_tl)
        .sub(PVector.mult(gravity, mass)).add(wind);
      
      particles[c][r].force.add(dragForce);
      xoff += 10;
    }
    yoff += 10;
   }

 
  for (int r = 0; r < row; r++) {      
    for (int c = 0; c < col; c++) {
      if (!particles[c][r].fixed) {  //Upate all nodes except the fixed ones 
        particles[c][r].acc.set(PVector.div(particles[c][r].force, mass));
        particles[c][r].vel.set(euler(particles[c][r].vel.x, particles[c][r].acc.x, ts), euler(particles[c][r].vel.y, particles[c][r].acc.y, ts), 0);
        particles[c][r].pos.set(euler(particles[c][r].pos.x, particles[c][r].vel.x, ts), euler(particles[c][r].pos.y, particles[c][r].vel.y, ts), 0);
      }
    }
  }
  
}

//Intergrater
float euler(float xt, float xtdt, float h) {
  return xt + h*xtdt;
}

float midpoint(float xt, float xtdt, float h) {
  float half = xt + xtdt*0.5*h;
  return xt + h*half;
}


boolean leftPressed, rightPressed, upPressed, downPressed, shiftPressed;

void keyPressed() {
  if (keyPressed && keyCode == 'F' || keyCode == 'f') {
    hasWind = !hasWind;
  }
  if (keyPressed && keyCode == 'W' || keyCode == 'w') {
    k_wind += 50;
  }
  if (keyPressed && keyCode == 'S' || keyCode == 's') {
    k_wind -= 50;
  }
  if (keyPressed && keyCode == 'X' || keyCode == 'x') {
    hasSphere = !hasSphere;
  }
  if (keyPressed && keyCode == 'R' || keyCode == 'r') {
    canRip = !canRip;
  }
  if (keyPressed && keyCode == 'A' || keyCode == 'a') {
    hasAir = !hasAir;
  }
  if (keyCode == LEFT) leftPressed = true;
  if (keyCode == RIGHT) rightPressed = true;
  if (keyCode == UP) upPressed = true; 
  if (keyCode == DOWN) downPressed = true;

}

void keyReleased(){
  if (keyCode == LEFT) leftPressed = false;
  if (keyCode == RIGHT) rightPressed = false;
  if (keyCode == UP) upPressed = false; 
  if (keyCode == DOWN) downPressed = false;
  if (keyCode == SHIFT) shiftPressed = false;
}


void mousePressed() { 
  if (mousePressed && (mouseButton == LEFT)) {
    PVector mousePos = new PVector(mouseX, mouseY);
    for (int r = 0; r < row; r++) {
      for (int c = 0; c < col; c++) {
        if (abs(particles[c][r].pos.x - mousePos.x + c_t_x) < 15 && abs(particles[c][r].pos.y - mousePos.y + c_t_y) < 15) {
          selectedP = new PVector(c, r);
          selected = true;
          return;
        }
      }
    }
  }
}

void mouseReleased() {
  selected = false;
}


void mouseDragged() {
  if (selected) {
    particles[int(selectedP.x)][int(selectedP.y)].setPos(mouseX - c_t_x, mouseY - c_t_y);
  }
}


public void setWind(int kw) {
  k_wind = kw;
}

boolean pointInCircle(PVector center, float r, PVector pointPos){
  float dist = pointPos.dist(center);
  if (dist < r){
    return true;
  }
  return false;
}
