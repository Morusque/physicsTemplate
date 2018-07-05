
interface Collidable {
  void update();
  void draw();
  Bounce getBounceForCircle(PVector pos, PVector dir, float radius);
}

class Bounce {
  PVector collisionPoint; // The point at which the collision occurs 
  PVector nextDir; // The direction of the object after the collision
  boolean bounced = false;// Did the collision occur
  float distance;// Distance of the path to get to that collision
  Collidable concernedCollidable;// The other object on which the collision occured
}

class BouncingPath {
  ArrayList<PVector> intermediates = new ArrayList<PVector>();
  float finalAngle;
}

class BoundingBox {
  PVector p1, p2;
  BoundingBox (PVector p1, PVector p2) {
    this.p1 = p1;
    this.p2 = p2;
  }
  boolean intersectsWith(BoundingBox o) {
    if (min(p1.x, p2.x)>max(o.p1.x, o.p2.x)) return false;
    if (min(p1.y, p2.y)>max(o.p1.y, o.p2.y)) return false;
    if (max(p1.x, p2.x)<min(o.p1.x, o.p2.x)) return false;
    if (max(p1.y, p2.y)<min(o.p1.y, o.p2.y)) return false;
    return true;
  }
}

class Line implements Collidable {
  PVector p1;
  PVector p2;
  Line (PVector p1, PVector p2) {
    this.p1=p1;
    this.p2=p2;
  }
  void update() {
  }
  void draw() {
    line(p1.x, p1.y, p2.x, p2.y);
  }
  Bounce getBounceForCircle(PVector o_pos, PVector o_dir, float o_radius) {
    // check if a ball (o_) will collide on this line during the trajectory

    Bounce bounce = new Bounce();

    // if the bounding boxes don't collide, stop there
    BoundingBox thisBox = new BoundingBox(new PVector(min(p1.x, p2.x), min(p1.y, p2.y)), new PVector(max(p1.x, p2.x), max(p1.y, p2.y)));
    PVector o_finalPos = PVector.add(o_pos, o_dir);
    BoundingBox otherBox = new BoundingBox(new PVector(min(o_pos.x, o_finalPos.x)-o_radius, min(o_pos.y, o_finalPos.y)-o_radius), new PVector(max(o_pos.x, o_finalPos.x)+o_radius, max(o_pos.y, o_finalPos.y)+o_radius));
    if (!thisBox.intersectsWith(otherBox)) return bounce;

    // this part is messy and need comments
    PVector closestPoint = getClosestPointOnSegment(p1, p2, o_pos);
    if (closestPoint==null) return bounce;
    else if (PVector.dist(closestPoint, o_pos)>o_dir.mag()+o_radius) return bounce;
    PVector thisClosestIntersection = closestIntersection(o_pos, o_radius, p1, p2);
    if (thisClosestIntersection!=null) return bounce;// if it's already colliding, ignore it
    PVector nextPos = PVector.add(o_pos, o_dir);
    ArrayList<PVector> intersections = new ArrayList<PVector>();
    PVector end1 = closestIntersection(p1, o_radius, o_pos, nextPos);
    PVector end2 = closestIntersection(p2, o_radius, o_pos, nextPos);
    float normalAngle = atan2(p1.y-p2.y, p1.x-p2.x)+HALF_PI;
    PVector segment1 = lineSegmentIntersection(new PVector(p1.x+cos(normalAngle)*o_radius, p1.y+sin(normalAngle)*o_radius), new PVector(p2.x+cos(normalAngle)*o_radius, p2.y+sin(normalAngle)*o_radius), o_pos, nextPos);
    PVector segment2 = lineSegmentIntersection(new PVector(p1.x+cos(normalAngle+PI)*o_radius, p1.y+sin(normalAngle+PI)*o_radius), new PVector(p2.x+cos(normalAngle+PI)*o_radius, p2.y+sin(normalAngle+PI)*o_radius), o_pos, nextPos);
    if (end1!=null) if (sq(o_dir.mag())>=sqDist(o_pos, end1)) intersections.add(end1);
    if (end2!=null) if (sq(o_dir.mag())>=sqDist(o_pos, end2)) intersections.add(end2);
    if (segment1!=null) intersections.add(segment1);
    if (segment2!=null) intersections.add(segment2);
    PVector bestClosestIntersection = null;
    for (PVector inter : intersections) {
      if (bestClosestIntersection==null) bestClosestIntersection = inter;
      else if (sqDist(o_pos, bestClosestIntersection)>sqDist(o_pos, inter)) bestClosestIntersection = inter;
    }
    if (bestClosestIntersection!=null) {
      PVector closestPointOnSegment = getClosestPointOnSegment(p1, p2, bestClosestIntersection);
      float normalFromSegment = atan2(bestClosestIntersection.y-closestPointOnSegment.y, bestClosestIntersection.x-closestPointOnSegment.x);
      float remainingDist = o_dir.mag()-PVector.dist(o_pos, bestClosestIntersection);
      float incidentAngle = atan2(bestClosestIntersection.y-o_pos.y, bestClosestIntersection.x-o_pos.x);
      float finalAngle = (incidentAngle+PI)+(normalFromSegment-incidentAngle)*2;
      bounce.collisionPoint = bestClosestIntersection;
      bounce.distance = PVector.dist(o_pos, bestClosestIntersection);
      bounce.nextDir = new PVector(cos(finalAngle)*remainingDist, sin(finalAngle)*remainingDist);
      bounce.bounced = true;
      bounce.concernedCollidable = this;
    }

    return bounce;
  }
  String getStrCoords() {
    return p1.x+","+p1.y+","+p2.x+","+p2.y;
  }
}

class Circle implements Collidable {
  PVector pos;
  float radius;
  ArrayList<Collidable> collidables;
  PVector direction;
  Circle(PVector pos, float radius, PVector direction, ArrayList<Collidable> collidables) {
    this.pos = pos;
    this.radius = radius;
    this.direction = direction;
    this.collidables = collidables;
  }
  void update() {

    // ask to go to that direction, get the final path including bounces on other collidables 
    BouncingPath bP = getBouncingPath(collidables, direction);

    // update new position and direction accordingly
    pos = bP.intermediates.get(bP.intermediates.size()-1);
    direction = new PVector(cos(bP.finalAngle)*direction.mag(), sin(bP.finalAngle)*direction.mag());
  }
  BouncingPath getBouncingPath(ArrayList<Collidable> collidables, PVector currentDir) {
    BouncingPath bouncingPath = new BouncingPath();
    bouncingPath.finalAngle = atan2(currentDir.y, currentDir.x);

    PVector currentPos = new PVector(pos.x, pos.y);

    Collidable lastConcernedCollidable = null;

    boolean goOnChecking = true;

    int maxBounces = 500;

    while (goOnChecking) {
      goOnChecking = false;
      bouncingPath.intermediates.add(currentPos);
      ArrayList<Bounce> bounces = new ArrayList<Bounce>();

      // check every collidable
      for (Collidable c : collidables) {
        if (c != lastConcernedCollidable && c != this) {
          Bounce bounce = c.getBounceForCircle(currentPos, currentDir, radius);
          if (bounce.bounced) bounces.add(bounce);
        }
      }

      // bounce against the closest collidable
      Bounce closestBounce = null;
      for (Bounce bounce : bounces) {
        if (closestBounce==null) closestBounce = bounce;
        else if (bounce.distance<closestBounce.distance) closestBounce = bounce;
      }
      if (closestBounce!=null) {
        currentPos = closestBounce.collisionPoint;
        currentDir = closestBounce.nextDir;
        lastConcernedCollidable = closestBounce.concernedCollidable;
        bouncingPath.finalAngle = atan2(currentDir.y, currentDir.x);
        goOnChecking = true;
      }

      // prevent being stucked in a loop
      if (bouncingPath.intermediates.size()>maxBounces) goOnChecking=false;
    }

    bouncingPath.intermediates.add(PVector.add(currentPos, currentDir));

    return bouncingPath;
  }
  void draw() {
    ellipse(pos.x, pos.y, radius*2, radius*2);
  }

  Bounce getBounceForCircle(PVector o_pos, PVector o_dir, float o_radius) {
    // check if another ball (o_) will collide during the trajectory

    Bounce bounce = new Bounce();

    // if the bounding boxes can't be colliding, don't go further
    BoundingBox thisBox = new BoundingBox(new PVector(pos.x-radius, pos.y-radius), new PVector(pos.x+radius, pos.y+radius));
    PVector o_finalPos = PVector.add(o_pos, o_dir);
    BoundingBox otherBox = new BoundingBox(new PVector(min(o_pos.x, o_finalPos.x)-o_radius, min(o_pos.y, o_finalPos.y)-o_radius), new PVector(max(o_pos.x, o_finalPos.x)+o_radius, max(o_pos.y, o_finalPos.y)+o_radius));
    if (!thisBox.intersectsWith(otherBox)) return bounce;

    // if it's already colliding, ignore it (so it will probably pass through)
    if (sqDist(pos, o_pos)<sq(radius+o_radius)) return bounce;

    // check for a possible intersection
    PVector intersection = closestIntersection(pos, radius+o_radius, o_pos, PVector.add(o_pos, o_dir));
    if (intersection!=null) {
      bounce.bounced=true;
      bounce.concernedCollidable = this;
      bounce.collisionPoint = intersection;
      bounce.distance = PVector.dist(o_pos, intersection);
      float normalFromSegment = atan2(intersection.y-pos.y, intersection.x-pos.x);
      float remainingDist = o_dir.mag()-PVector.dist(o_pos, intersection);
      float incidentAngle = atan2(intersection.y-o_pos.y, intersection.x-o_pos.x);
      float finalAngle = (incidentAngle+PI)+(normalFromSegment-incidentAngle)*2;
      bounce.nextDir = new PVector(cos(finalAngle)*remainingDist, sin(finalAngle)*remainingDist);
    }

    return bounce;
  }
}