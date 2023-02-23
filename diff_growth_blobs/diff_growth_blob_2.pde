import processing.javafx.*;
import processing.svg.*;
boolean doRecord = false;
// Differential growth algorithm credit to Alberto Giachino with suggestions from Frederik Vanhoutte
// From: http://www.codeplastic.com/2017/07/22/differential-line-growth-with-processing/

ArrayList<createBounds> bounds;

// PARAMETERS
float _maxForce = 0.9; // Maximum steering force
float _maxSpeed = 35; // Maximum speed
float _desiredSeparation = 25;
float _separationCohesionRation = 0.6;
float _maxEdgeLen = 6;

PGraphics pg;
DifferentialLine _diff_line0;
DifferentialLine _diff_line1;
DifferentialLine _diff_line2;
DifferentialLine _diff_line3;

void setup() {
  size(600, 600, FX2D);
  bounds = new ArrayList<createBounds>();

  // Make blob masks
  pg = createGraphics(600, 600);
  pg.beginDraw();
  pg.background(0);
  pg.noStroke();
  pg.fill(255);
  for (int i = 0; i<=6; i++) {
    randomSeed(1000*i + 13);
    noiseSeed(i+13);
    float translateX = random(10, width-10);
    float translateY = random(10, height-10);
    float radius = random(50, 200);
    bounds.add(new createBounds(translateX, translateY, radius));
  }
  pg.endDraw();



  _diff_line0 = new DifferentialLine(_maxForce, _maxSpeed, _desiredSeparation, _separationCohesionRation, _maxEdgeLen);
  _diff_line1 = new DifferentialLine(_maxForce, _maxSpeed, _desiredSeparation, _separationCohesionRation, _maxEdgeLen);
  _diff_line2 = new DifferentialLine(_maxForce, _maxSpeed, _desiredSeparation, _separationCohesionRation, _maxEdgeLen);
  _diff_line3 = new DifferentialLine(_maxForce, _maxSpeed, _desiredSeparation, _separationCohesionRation, _maxEdgeLen);
  
  float nodesStart = 30;
  float angInc = TWO_PI/nodesStart;
  float rayStart = 200;
  for (float a=0; a<TWO_PI; a+=angInc) {
    float x = width/2 + cos(a) * rayStart;
    float y = height/2 + sin(a) * rayStart;
    _diff_line0.addNode(new Node(x, y, _diff_line0.maxForce, _diff_line0.maxSpeed));
    _diff_line1.addNode(new Node(x, y, _diff_line1.maxForce, _diff_line1.maxSpeed));
    _diff_line2.addNode(new Node(x*1.4, y*1.4, _diff_line2.maxForce, _diff_line2.maxSpeed));
    _diff_line3.addNode(new Node(x+30, y-100, _diff_line3.maxForce, _diff_line3.maxSpeed));
  }
}

//Array list of off screen graphics buffer blobs
void draw() {
  background(255);
  //image(pg, 0, 0);

  if (doRecord) {
    beginRecord(SVG, "differential_growth_9.svg");
  }
  stroke(0);//255, 0, 0);
  strokeWeight(1.0);
  noFill();

  _diff_line0.separationCohesionRation = 0.6;
  _diff_line0.maxForce = 0.7;

  _diff_line1.separationCohesionRation = 0.7;
  _diff_line1.maxForce = 0.6;

  _diff_line2.separationCohesionRation = 0.6;
  _diff_line2.maxForce = 0.7; 

  _diff_line3.separationCohesionRation = 0.6; 
  _diff_line3.maxForce = 0.7; 
  //map(sin(millis()/2400.0), -1, 1, 0.1, 0.2); ///map(mouseX,0,width, 0.2,0.7);

  _diff_line0.run();
  _diff_line1.run();
  _diff_line2.run();
  _diff_line3.run();
  _diff_line1.renderLine();


  if (doRecord) {
    endRecord();
    doRecord = false;
  }
}

class createBounds {
  float tX;
  float tY;
  float r;

  createBounds(float translateX, float translateY, float radius) {
    tX = translateX;
    tY = translateY;
    r = radius;

    float yoff = 0.0;

    pg.pushMatrix();
    pg.translate(tX, tY);

    pg.beginShape();
    float xoff = 0;
    for (float a = 0; a < TWO_PI; a += 0.1) {
      float offset = map(noise(xoff, yoff), 0, 1, -30, 30);
      float r = radius + offset;
      float x = r * cos(a);
      float y = r * sin(a);
      pg.vertex(x, y);
      xoff += 0.1;
    }
    pg.endShape();
    pg.popMatrix();

    yoff += 0.01;
  }
}

void keyPressed() {
  if (key == ' ') {
    doRecord = true;
  }
}

class DifferentialLine {
  ArrayList<Node> nodes;
  float maxForce;
  float maxSpeed;
  float desiredSeparation;
  float sq_desiredSeparation;
  float separationCohesionRation;
  float maxEdgeLen;
  DifferentialLine(float mF, float mS, float dS, float sCr, float eL) {
    nodes = new ArrayList<Node>();
    maxSpeed = mF;
    maxForce = mS;
    desiredSeparation = dS;
    sq_desiredSeparation = sq(desiredSeparation);
    separationCohesionRation = sCr;
    maxEdgeLen = eL;
  }
  void addNode(Node n) {
    nodes.add(n);
  }
  void addNodeAt(Node n, int index) {
    nodes.add(index, n);
  }

  void run() {
    differentiate();
    growth();
  }

  void growth() {
    for (int i=0; i<nodes.size()-1; i++) {
      Node n1 = nodes.get(i);
      Node n2 = nodes.get(i+1);
      float d = PVector.dist(n1.position, n2.position);
      if (d>maxEdgeLen) { // Can add more rules for inserting nodes
        int index = nodes.indexOf(n2);
        PVector middleNode = PVector.add(n1.position, n2.position).div(2);
        addNodeAt(new Node(middleNode.x, middleNode.y, maxForce*noise(i), maxSpeed), index);
      }
    }
  }
  void differentiate() {
    PVector[] separationForces = getSeparationForces();
    PVector[] cohesionForces = getEdgeCohesionForces();
    for (int i=0; i<nodes.size(); i++) {

      int nx = (int)round(nodes.get(i).position.x);
      int ny = (int)round(nodes.get(i).position.y);

      PVector separation = separationForces[i];
      PVector cohesion = cohesionForces[i];
      separation.mult(separationCohesionRation);

      color c = pg.get(nx, ny);
      int val = (int) red(c);
      if (val > 0) {
        
        nodes.get(i).applyForce(separation);
        nodes.get(i).applyForce(cohesion);
        nodes.get(i).update();
      }
      
      
    }
  }

  PVector[] getSeparationForces() {
    int n = nodes.size();
    PVector[] separateForces=new PVector[n];
    int[] nearNodes = new int[n];
    Node nodei;
    Node nodej;
    for (int i=0; i<n; i++) {
      separateForces[i]=new PVector();
    }
    for (int i=0; i<n; i++) {
      nodei=nodes.get(i);
      for (int j=i+1; j<n; j++) {
        nodej=nodes.get(j);
        PVector forceij = getSeparationForce(nodei, nodej);
        if (forceij.mag()>0) {
          separateForces[i].add(forceij);
          separateForces[j].sub(forceij);
          nearNodes[i]++;
          nearNodes[j]++;
        }
      }
      if (nearNodes[i]>0) {
        separateForces[i].div((float)nearNodes[i]);
      }
      if (separateForces[i].mag() >0) {
        separateForces[i].setMag(maxSpeed);
        separateForces[i].sub(nodes.get(i).velocity);
        separateForces[i].limit(maxForce);
      }
    }
    return separateForces;
  }
  PVector getSeparationForce(Node n1, Node n2) {
    PVector steer = new PVector(0, 0);
    float sq_d = sq(n2.position.x-n1.position.x)+sq(n2.position.y-n1.position.y);
    if (sq_d>0 && sq_d<sq_desiredSeparation) {
      PVector diff = PVector.sub(n1.position, n2.position);
      diff.normalize();
      diff.div(sqrt(sq_d)); //Weight by distacne
      steer.add(diff);
    }
    return steer;
  }
  PVector[] getEdgeCohesionForces() {
    int n = nodes.size();
    PVector[] cohesionForces=new PVector[n];
    for (int i=0; i<nodes.size(); i++) {
      PVector sum = new PVector(0, 0);
      if (i!=0 && i!=nodes.size()-1) {
        sum.add(nodes.get(i-1).position).add(nodes.get(i+1).position);
      } else if (i == 0) {
        sum.add(nodes.get(nodes.size()-1).position).add(nodes.get(i+1).position);
      } else if (i == nodes.size()-1) {
        sum.add(nodes.get(i-1).position).add(nodes.get(0).position);
      }
      sum.div(2);
      cohesionForces[i] = nodes.get(i).seek(sum);
    }
    return cohesionForces;
  }
  //void renderShape() {
  //  beginShape();
  //  for (int i=0; i<nodes.size(); i++) {
  //    vertex(nodes.get(i).position.x, nodes.get(i).position.y);
  //  }
  //  endShape(CLOSE);
  //}
  void renderLine() {
    beginShape();
    for (int i=0; i<nodes.size(); i++) {
      PVector p1 = nodes.get(i).position;
      curveVertex(p1.x, p1.y);
      //point(p1.x,p1.y);
    }
    endShape(CLOSE);

    //for (int i=0; i<nodes.size()-1; i++) {
    //  PVector p1 = nodes.get(i).position;
    //  PVector p2 = nodes.get(i+1).position;
    //  line(p1.x, p1.y, p2.x, p2.y);
    //  if (i==nodes.size()-2) {
    //    line(p2.x, p2.y, nodes.get(0).position.x, nodes.get(0).position.y);
    //  }
    //}
  }
}
class Node {
  PVector position;
  PVector velocity;
  PVector acceleration;
  float maxForce;
  float maxSpeed;
  Node(float x, float y, float mF, float mS) {
    acceleration = new PVector(0, 0);
    velocity =PVector.random2D();
    position = new PVector(x, y);
    maxSpeed = mF;
    maxForce = mS;
  }
  void applyForce(PVector force) {
    acceleration.add(force);
  }
  void update() {
    velocity.add(acceleration);
    velocity.limit(maxSpeed);
    velocity.limit(maxSpeed);
    position.add(velocity);
    acceleration.mult(0);
    if (position.x == width-200 || position.x == 200) {
      velocity.x = 0;
      maxForce = 0;
    }
    if (position.y == height-200 || position.x == 200) {
      velocity.y = 0;
      maxForce = 0;
    }
  }
  PVector seek(PVector target) {
    PVector desired = PVector.sub(target, position);
    desired.setMag(maxSpeed);
    PVector steer = PVector.sub(desired, velocity);
    steer.limit(maxForce);
    return steer;
  }
}
