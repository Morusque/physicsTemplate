
float deltaTime=0;
float lastMillis=0;

ArrayList<Collidable> allCollidables = new ArrayList<Collidable>();

void setup() {
  size(600, 600);
  frameRate(30);

  /* example */
  allCollidables.add(new Line(new PVector(0, 0), new PVector(0, height-1)));
  allCollidables.add(new Line(new PVector(0, 0), new PVector(width-1, 0)));
  allCollidables.add(new Line(new PVector(width-1, 0), new PVector(width-1, height-1)));
  allCollidables.add(new Line(new PVector(0, height-1), new PVector(width-1, height-1)));
  for (int i=0; i<50; i++) allCollidables.add(new Circle(new PVector(random(0.2, 0.8)*width, random(0.2, 0.8)*height), random(5.0f, 15.0f), new PVector(random(-10, 10), random(-10, 10)), allCollidables));
  for (int i=0; i<3; i++) allCollidables.add(new Line(new PVector(random(0.2, 0.8)*width, random(0.2, 0.8)*height), new PVector(random(0.2, 0.8)*width, random(0.2, 0.8)*height)));
  /* example end */
  
}

void draw() {
  deltaTime = ((float)millis()-lastMillis)/1000.0f;
  lastMillis = millis();
  for (Collidable collidable : allCollidables) collidable.update();
  background(0xFF);
  for (Collidable collidable : allCollidables) collidable.draw();
}
