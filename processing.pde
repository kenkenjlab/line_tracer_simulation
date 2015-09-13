/* @pjs preload="background.png"; */

////////////////////////// CONSTANTS ////////////////////////////////////

final float PIXEL_PER_MILLIMETER = 0.5;
final float TRACK = 80.0 * PIXEL_PER_MILLIMETER;
final float WHEELBASE = 110.0 * PIXEL_PER_MILLIMETER;
final float SENSOR_INTERVAL1 = 7.0 * PIXEL_PER_MILLIMETER;
final float SENSOR_INTERVAL2 = 20.0 * PIXEL_PER_MILLIMETER;
final float ONE_STEP = PI / 375;
final int SENSOR_PREVIEW_WIDTH = 32;
final float MAX_VELOCITY = 200;
final float STEP_VELOCITY = 2;
/*
final float K_P = 0.95;
final float K_I = 0.6;
final float K_D = 0.3;
*/
final float K_P = 0.6;
final float K_I = 0.7;
final float K_D = 0.45;

////////////////////////// CLASSES ////////////////////////////////////

// ------------------------------------------------------------------//
public class Velocity {
  public float left = 0;
  public float right = 0;
  public Velocity() {}
  public Velocity(float l, float r) { left = l; right = r; }
  public boolean areAllZero() { return left == 0 && right == 0; }
}
// ------------------------------------------------------------------//
public class Vector {
  public float x;
  public float y;
  public Vector(float _x, float _y) { x = _x; y = _y; }
  public Vector add(final Vector vector) {
    x += vector.x;
    y += vector.y;
    return this;
  }
  public Vector subtract(final Vector vector) {
    x -= vector.x;
    y -= vector.y;
    return this;
  }
  public Vector duplicate() { return new Vector(x, y); }
  public Vector normalize() {
    float norm = sqrt(x * x + y * y);
    x /= norm;
    y /= norm;
    return this;
  }
  public Vector multiply(float multiplier) { 
    x *= multiplier;
    y *= multiplier;
    return this;
  }
  public Vector rotate(float radian) {
    final Vector oldVector = duplicate();
    final float sine = sin(radian);
    final float cosine = cos(radian);
    x = oldVector.x * cosine - oldVector.y * sine;
    y = oldVector.x * sine + oldVector.y * cosine;
    return this;
  }
};
// ------------------------------------------------------------------//
public class Wheel {
  private Vector position_;
  private boolean isLeft_;
  private boolean isForwarding_ = true;
  
  public Wheel(final Vector position, boolean isLeft) {
    position_ = position.duplicate();
    isLeft_ = isLeft;
  }
  
  public Vector getPosition() { return position_; }
  public boolean isLeft() { return isLeft_; }
  public boolean isForwarding() { return isForwarding_; }
  public void setForwardDirection(boolean forward) { isForwarding_ = forward; }
  
  public void update(final Wheel wheel) {
    float radian = ONE_STEP;
    if(!isLeft()) { radian *= -1; }
    if(!isForwarding()) { radian *= -1; }
    position_.subtract(wheel.getPosition());
    position_.rotate(radian);
    position_.add(wheel.getPosition());
  }
  
  public void draw() {
    noStroke();
    if(isLeft()) { fill(255, 255, 0); } else { fill(0, 255, 255); }
    ellipse(position_.x, position_.y, 4, 4);
  }
}
// ------------------------------------------------------------------//
public class Sensor {
  private ArrayList<Vector> positions_ = new ArrayList<Vector>();  // From left to right
  private boolean isBlacks_[] = new boolean[4];
  
  public boolean isBlack(int i) { return isBlacks_[i]; }
  public boolean isWhite(int i ) { return !isBlack(i); }
  public byte getByte() {
    byte value = 0x00;
    for(int i = 0; i < 4; i++) {
      value <<= 1;
      value |= isBlack(i) ? 1 : 0;
    }
    return value;
  }
  
  public void update(final Vector leftVector, final Vector rightVector) {
    // Update position
    Vector centerWheelsPosition = leftVector.duplicate().add(rightVector).multiply(0.5);
    Vector horizontalVector = leftVector.duplicate().subtract(rightVector).normalize();
    Vector verticalVector = horizontalVector.duplicate().rotate(PI / 2).multiply(WHEELBASE);
    Vector centerSensorPosition = centerWheelsPosition.duplicate().add(verticalVector);
    Vector directionVector1 = horizontalVector.duplicate().multiply(SENSOR_INTERVAL1);
    Vector directionVector2 = horizontalVector.duplicate().multiply(SENSOR_INTERVAL2);
    positions_.clear();
    positions_.add(centerSensorPosition.duplicate().add(directionVector2));
    positions_.add(centerSensorPosition.duplicate().add(directionVector1));
    positions_.add(centerSensorPosition.duplicate().subtract(directionVector1));   
    positions_.add(centerSensorPosition.duplicate().subtract(directionVector2));
  }
  
  public void capture() {
    // Update sensor values
    for(int i = 0; i < 4; i++) { isBlacks_[i] = read_(positions_.get(i)); }
  }
  
  private boolean read_(final Vector position) {
    int x = round(position.x);
    int y = round(position.y);
    if(x < 0 || y < 0 || x >= width || y >= height) { return true; }
    color capturedColor = pixels[round(position.x) + round(position.y) * width];
    return brightness(capturedColor) < 128;  // is Black?
  }
  
  public void draw() {
    noFill();
    stroke(255, 128, 0);
    for(Vector p : positions_) {
      ellipse(p.x, p.y, 4, 4);
    }
    
    stroke(128, 128, 128);
    for(int i = 0; i < 4; i++) {
      if(isBlacks_[i]) { fill(0, 0, 0); } else { fill(255, 255, 255); }
      rect(SENSOR_PREVIEW_WIDTH * i, 0, SENSOR_PREVIEW_WIDTH, SENSOR_PREVIEW_WIDTH);
    }
  }
}
// ------------------------------------------------------------------//
public interface Callback {
  public void onTime();
}
// ------------------------------------------------------------------//
public class TimeManager {
  private Callback callback_;
  private int prevTime_ = 0;
  private int timeCount_ = 0;
  private int targetTime_ = 2147483647;
  
  public TimeManager(Callback callback) { callback_ = callback; }
  public void setTargetTime(int time) { targetTime_ = time; }
  public void update() {
    int currTime = millis();
    timeCount_ += currTime - prevTime_;
    prevTime_ = currTime;
  }
  public void invoke(boolean resetTime) {
    if(!hasTimeCome()) { return; }
    if(resetTime) { timeCount_ = 0; }
    else { timeCount_ -= targetTime_; }
    callback_.onTime();
  }
  public boolean hasTimeCome() { return targetTime_ < timeCount_ && isTargetTimeValid(); }
  public int getTimeCount() { return timeCount_; }
  public boolean isTargetTimeValid() { return targetTime_ > 0; }
}
// ------------------------------------------------------------------//
public class MotorController {
  private Velocity velocity_ = new Velocity();
  private PIDController pid_ = new PIDController();
  
  public boolean isStopped() { return velocity_.areAllZero(); }
  public void computeVelocity(byte sensorValue) {
    // Analyze sensor value
    float displacement = PIXEL_PER_MILLIMETER;
    switch(sensorValue) {
        case 0x0f:
          // STOP
          break;
        case 0x03:
          displacement *= -13.5;
          break;
        case 0x07:
          displacement *= -20.0;
          break;
        case 0x0b:
          displacement *= -7.0;
          break;
        case 0x0c:
          displacement *= 13.5;
          break;
        case 0x0e:
          displacement *= 20.0;
          break;
        case 0x0d:
          displacement *= 7.0;
          break;
        default:
          displacement = 0.0;
          break;
      }
    
    // PID Control (Input: desired displacement)
    float mv = pid_.computeMV(displacement);
    float realMV = displacement - mv;
    
    Velocity velocityTarget = new Velocity(0.0, 0.0);
    if(sensorValue != 0x0f) {
      // Compuate desired velocity from desired displacement  
      velocityTarget = computeVelocityFromDisplacement_(realMV);
    }
    
    // Trapezoidal Control
    velocity_.left = smoothVelocity_(velocityTarget.left, velocity_.left);
    velocity_.right = smoothVelocity_(velocityTarget.right, velocity_.right);
}
  
  public int getPulseIntervalLeft() { return getPulseInterval_(velocity_.left); }
  public int getPulseIntervalRight() { return getPulseInterval_(velocity_.right); }
  private int getPulseInterval_(float velocity) {
    if(velocity <= 0.0) { return -1; }
    return ceil(1000.0 / (float)velocity);
  }
  
  private Velocity computeVelocityFromDisplacement_(float displacement) {
    Velocity velocity = new Velocity(MAX_VELOCITY, MAX_VELOCITY);
    if(displacement == 0.0) { return velocity; }
    float R = (pow(WHEELBASE, 2) + pow(displacement, 2)) / 2 / abs(displacement);
    float ratio = (R - TRACK / 2) / (R + TRACK / 2);
    if(displacement > 0) {
      velocity.right *= ratio;
    } else {
      velocity.left *= ratio;
    }
    return velocity;
  }
  
  private float smoothVelocity_(float tgtVelocity, float currVelocity) {
    float nextVelocity = tgtVelocity;
    if(tgtVelocity > currVelocity) {
      nextVelocity = min(currVelocity + STEP_VELOCITY, tgtVelocity);
    } else if(tgtVelocity < currVelocity) {
      nextVelocity = max(currVelocity - STEP_VELOCITY, tgtVelocity);
    }
    return nextVelocity;
  }
}
// ------------------------------------------------------------------//
public class PIDController {
  private float currErrorValue_ = 0.0;
  private float lastErrorValue_ = 0.0;
  private float beforeLastErrorValue_ = 0.0;
  private float computeMV(float actualValue) {
    beforeLastErrorValue_ = lastErrorValue_;
    lastErrorValue_ = currErrorValue_;
    currErrorValue_ = 0.0 - actualValue;    // In this project, desired value is always 0.0.
    
    float currMinusLast = currErrorValue_ - lastErrorValue_;
    float lastMinusBeforeLast = lastErrorValue_ - beforeLastErrorValue_;
    return K_P * currMinusLast + K_I * currErrorValue_ + K_D * (currMinusLast - lastMinusBeforeLast);
  }
}
// ------------------------------------------------------------------//
public class Car {
  private Wheel leftWheel_, rightWheel_;
  private Sensor sensor_;
  private TimeManager tmComputation_, tmLeftWheel_, tmRightWheel_;
  private MotorController controller_ = new MotorController();
  private OnLeftWheelTimeCallback onLeftWheelTimeCallback_ = new OnLeftWheelTimeCallback();
  private OnRightWheelTimeCallback onRightWheelTimeCallback_ = new OnRightWheelTimeCallback();
  private OnComputationCallback onComputationCallback_ = new OnComputationCallback();
  
  public class OnLeftWheelTimeCallback implements Callback {
    public void onTime() {
      sensor_.update(leftWheel_.getPosition(), rightWheel_.getPosition());
      leftWheel_.update(rightWheel_);
    }
  }
  
  public class OnRightWheelTimeCallback implements Callback {
    public void onTime() {
      sensor_.update(leftWheel_.getPosition(), rightWheel_.getPosition());
      rightWheel_.update(leftWheel_);
    }
  }
  
  public class OnComputationCallback implements Callback {
    public void onTime() {
      sensor_.update(leftWheel_.getPosition(), rightWheel_.getPosition());
      sensor_.capture();
      controller_.computeVelocity(sensor_.getByte());
      tmLeftWheel_.setTargetTime(controller_.getPulseIntervalLeft());
      tmRightWheel_.setTargetTime(controller_.getPulseIntervalRight());
    }
  }

  public Car(final Vector forwardPosition, final Vector backwardPosition) {
    Vector centerPosition = forwardPosition.duplicate().add(backwardPosition).multiply(0.5);
    Vector rightVector = forwardPosition.duplicate().subtract(backwardPosition).rotate(PI / 2).normalize().multiply(TRACK / 2);
    leftWheel_ = new Wheel(centerPosition.duplicate().subtract(rightVector), true);
    rightWheel_ = new Wheel(centerPosition.duplicate().add(rightVector), false);
    sensor_ = new Sensor();
    tmLeftWheel_ = new TimeManager(onLeftWheelTimeCallback_);
    tmRightWheel_ = new TimeManager(onRightWheelTimeCallback_);
    tmComputation_ = new TimeManager(onComputationCallback_);
    tmComputation_.setTargetTime(10);
    onComputationCallback_.onTime();
  }
  
  public void draw() {
    leftWheel_.draw();
    rightWheel_.draw();
    sensor_.draw();
  }
  
  public void update() {
    tmComputation_.update();
    tmLeftWheel_.update();
    tmRightWheel_.update();
    
    // Update sensor and velocity when the time has come
    tmComputation_.invoke(true);
    
    // Update wheels when the time has come
    while(true) {
      tmLeftWheel_.invoke(false);
      tmRightWheel_.invoke(false);
      if(!tmLeftWheel_.hasTimeCome() && !tmRightWheel_.hasTimeCome()) { break; } 
    }

  }
}
// ------------------------------------------------------------------//
public class StartPositionAnalyzer {
  private Vector forwardPosition_, backwardPosition_;
  public Vector getForwardPosition() { return forwardPosition_; }
  public Vector getBackwardPosition() { return backwardPosition_; }
  public boolean compute() {
    boolean isForwardPositionFound = false;
    boolean isBackwardPositionFound = false;
    for(int y = 0; y < height; y++) {
      for(int x = 0; x < width; x++) {
        color c = pixels[x + y * width];
        if(red(c) > 224 && green(c) < 32 && blue(c) < 32) {
          forwardPosition_ = new Vector(x, y);
          isForwardPositionFound = true;
        } else if(red(c) < 32 && green(c) > 224 && blue(c) < 32) {
          backwardPosition_ = new Vector(x, y);
          isBackwardPositionFound = true;
        }
      }
    }
    return isForwardPositionFound & isBackwardPositionFound;
  }
}

////////////////////////// GLOBAL VARIABLE ////////////////////////////////////

Car car;
PImage backImg;

////////////////////////// FUNCTIONS ////////////////////////////////////
void setup() {
  // Default settings
  size(640, 480);
  ellipseMode(CENTER);
  rectMode(CORNER);
  colorMode(RGB, 255);
  frameRate(1000);
  
  // Prepare images
  backImg = loadImage("background.png");
  image(backImg, 0, 0);
  loadPixels();
  
  // Analyze start point and direction of car
  StartPositionAnalyzer analyzer = new StartPositionAnalyzer();
  Vector forwardPosition = new Vector(width / 2, height / 2 - 10);
  Vector backwardPosition = new Vector(width / 2, height / 2 + 10);
  if(analyzer.compute()) {
    forwardPosition = analyzer.getForwardPosition();
    backwardPosition = analyzer.getBackwardPosition();
  }
  
  // Create instances
  car = new Car(forwardPosition, backwardPosition);
}

void draw() {
  // Compute
  car.update();
  
  // Draw
  image(backImg, 0, 0);
  car.draw();
}