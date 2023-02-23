import processing.javafx.*;
import processing.svg.*;
boolean doRecord = false;

float noiseScaleX = 1.0/300.0;
float noiseScaleY = 1.0/300.0;
float mx;
float my;
int count = 24;

// PARAMETERS
//float _maxForce = 0.5; // Maximum steering force
//float _maxSpeed = 15; // Maximum speed
//float _desiredSeparation = 25;
//float _separationCohesionRation = 0.4;
//float _maxEdgeLen = 4;
//float noiseStrength = 0.2;

//slower, wider growths numLines:30
//float _maxForce = 0.5; // Maximum steering force
//float _maxSpeed = 15; // Maximum speed
//float _desiredSeparation = 25;
//float _separationCohesionRation = 0.4;
//float _maxEdgeLen = 4;
//float noiseStrength = 0.2;


float _maxForce = 0.4; // Maximum steering force
float _maxSpeed = 5; // Maximum speed
float _desiredSeparation = 20;
float _separationCohesionRation = 0.3;
float _maxEdgeLen = 5;
float noiseStrength = 0.25;

PGraphics pg;
int num_lines = 24;
DifferentialLine[] diff_lines;


void setup() {
  size(1000, 1000, FX2D);
  diff_lines = new DifferentialLine[num_lines];
  float randX = random(30, width-10);
  float randY = random(30, height-10);
  float nodesStart = 30;
  float angInc = TWO_PI/nodesStart;
  float rayStart = 130;
  
  for (int i = 0; i < num_lines; i++) {
      diff_lines[i] = new DifferentialLine(_maxForce, _maxSpeed, _desiredSeparation, _separationCohesionRation, _maxEdgeLen);
  
    for (float a=0; a<TWO_PI; a+=angInc) {
      float x = randX + cos(a) * rayStart;
      float y = randY + sin(a) * rayStart;
      diff_lines[i].addNode(new Node(x, y, diff_lines[i].maxForce, diff_lines[i].maxSpeed));
    }
    randX = random(20, width);
    randY = random(20, height);
  }
}



void draw() {
  background(250);
  float D = 10;
  stroke(80, 0, 0, 60);
  strokeWeight(2.5);

  if (doRecord) {
    beginRecord(SVG, "flow_field_test_" + count + ".svg");
  }


  //flow field
  stroke(220);
  for (int y = 0; y < height; y+=D) {
    for (int x = 0; x < width; x+=D) {
      float noisex = x * noiseScaleX;
      float noisey = y * noiseScaleY;

      float dx = noise(noisex, noisey, 10) - 0.5;
      float dy = noise(noisex, noisey, 100) - 0.5;
      float angle = atan2(dy, dx);

      float px = x + D * cos(angle);
      float py = y + D * sin(angle);

    }
  }
  
  
  stroke(0);
  noFill();
  
  for (int i = 0; i < diff_lines.length; i++){
   diff_lines[i].run();
   diff_lines[i].renderLine();
  }
  

  if (doRecord) {
    endRecord();
    doRecord = false;
  }
}

void keyPressed() {
  if (key == ' ') {
    doRecord = true;
    count+=1;
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

      float nodex = (nodes.get(i).position.x); // screenspace (large)
      float nodey = (nodes.get(i).position.y);
      
      float noisex = nodex * noiseScaleX; // noise space (small)
      float noisey = nodey * noiseScaleY;

      float dx = noise(noisex, noisey, 10) - 0.5;
      float dy = noise(noisex, noisey, 100) - 0.5;
      float angle = atan2(dy, dx);

      float noisefx = noiseStrength * cos(angle);
      float noisefy = noiseStrength * sin(angle);
      PVector noiseForce = new PVector(noisefx, noisefy); 
      
      PVector separation = separationForces[i];
      PVector cohesion = cohesionForces[i];
      separation.mult(separationCohesionRation);
      
      nodes.get(i).applyForce(noiseForce);
      nodes.get(i).applyForce(separation);
      nodes.get(i).applyForce(cohesion);
      nodes.get(i).update();
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

  void renderLine() {
    beginShape();
    for (int i=0; i<nodes.size(); i++) {
      PVector p1 = nodes.get(i).position;
      curveVertex(p1.x, p1.y);
      //point(p1.x,p1.y);
    }
    endShape(CLOSE);
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
