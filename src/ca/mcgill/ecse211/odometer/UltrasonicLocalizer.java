package ca.mcgill.ecse211.odometer;


import ca.mcgill.ecse211.odometer.Odometer;
import lejos.hardware.Sound;
import lejos.hardware.ev3.LocalEV3;
import lejos.hardware.motor.EV3LargeRegulatedMotor;
import lejos.hardware.port.Port;
import lejos.hardware.sensor.EV3UltrasonicSensor;
import lejos.hardware.sensor.SensorModes;
import lejos.robotics.SampleProvider;

public class UltrasonicLocalizer {

  private static Odometer odometer;

  private EV3LargeRegulatedMotor rightMotor;

  
  private EV3LargeRegulatedMotor leftMotor;

  
  SensorModes usSensor = new EV3UltrasonicSensor(usPort);
  
  SampleProvider usValue = usSensor.getMode("Distance");
  
  
  float[] usData = new float[usValue.sampleSize()];

  private static final Port usPort = LocalEV3.get().getPort("S4");

  
  private int gap = 38;
  
  private int MID = 35;

  private int MD = 41;
  
  public static final double TRACK = 15.25;

  
  public static final double WHEEL_RADIUS = 2.154;

  
  public final static int ROTATION_SPEED = 60;
  
  
  private static int FILTER_OUT = 3;

 
  double anlgez;
  
  
  double anglex;
  
  private int filterControl;

  public UltrasonicLocalizer(Odometer odometer, EV3LargeRegulatedMotor leftMotor,
      EV3LargeRegulatedMotor rightMotor) {
	  
	  
    this.leftMotor = leftMotor;
    
    this.rightMotor = rightMotor;
    
    
    this.odometer = odometer;
    filterControl = 0;
  }

  public void Falling_Edge() {
	  

	 rightMotor.setSpeed(ROTATION_SPEED);

	  
    leftMotor.setSpeed(ROTATION_SPEED);

    while (processUSData() < MD) {

      leftMotor.forward();
      rightMotor.backward();
    }
    
    while (processUSData() > gap) {
    	
      rightMotor.backward();

    	
      leftMotor.forward();
    }
    
    Sound.beep();
    
    
    anlgez = odometer.getXYT() [2];

    while (processUSData() < MD) {

        rightMotor.forward();

    	
      leftMotor.backward();
    }

    while (processUSData() > gap) {
    	
        rightMotor.forward();

      leftMotor.backward();
    }
    Sound.beep();
    rightMotor.stop(true);

    
    leftMotor.stop(true);
    anglex = odometer.getXYT()[2];

   
    if (anlgez > 360) {
    	anlgez = anlgez - 360;
    }
    
    if (anlgez < 0) {
    	anlgez = anlgez + 360;
    }
    
    if (anglex > 360) {
    	anlgez = anlgez - 360;
    }
    
    if (anglex < 0) {
    	anlgez = anlgez + 360;
    }

    double theta2 = (anlgez + anglex) / 2;

    double theta1 = odometer.getXYT()[2];
    
    
    double angular = (theta1 - theta2 - 8.1);
    
    
    leftMotor.rotate(convertAngle(Lab4.WHEEL_RADIUS, Lab4.TRACK, angular), true);
    
    
    rightMotor.rotate(-convertAngle(Lab4.WHEEL_RADIUS, Lab4.TRACK, angular), false);

    
    odometer.setTheta(0.0);

  }


  public void Rising_Edge() {


    rightMotor.setSpeed(ROTATION_SPEED);
    
    leftMotor.setSpeed(ROTATION_SPEED);

    while (processUSData() > MID) {
    	
      rightMotor.backward();


      leftMotor.forward();

    }
    while (processUSData() < gap) {
    	
      rightMotor.backward();

    	
      leftMotor.forward();
    }
    
    Sound.beep();
    
    anlgez = odometer.getXYT()[2];

    while (processUSData() > MID) {
    	

      rightMotor.forward();
      
      leftMotor.backward();

    }

    while (processUSData() < gap) {
    	
        rightMotor.forward();


      leftMotor.backward();
    }
    Sound.beep();
    rightMotor.stop(true);

    
    leftMotor.stop(true);
    
    anglex = odometer.getXYT()[2];

 
    if (anlgez > 360) {
    	anlgez = anlgez - 360;
    }
   if (anlgez < 0) {
    	
    	anlgez = anlgez + 360;
    }
   
    if (anglex > 360) {
    	anlgez = anlgez - 360;
    }

 if (anglex < 0) {
    	
    	anlgez = anlgez + 360;
    }
    
    double theta1 = (anlgez + anglex) / 2;
    
    
    double theta2 = odometer.getXYT()[2];

    double angular = (theta2 - theta1 - 255);
    
    
    leftMotor.rotate(convertAngle(Lab4.WHEEL_RADIUS, Lab4.TRACK, angular), true);
    rightMotor.rotate(-convertAngle(Lab4.WHEEL_RADIUS, Lab4.TRACK, angular), false);

    odometer.setTheta(0.0);
  }

  public double processUSData() {
	  
    usSensor.fetchSample(usData, 0);
    
    double distance1 = 0;
    double distance2 = (int) (usData[0] * 100.0);

    
    if (((distance2 >= 50 && filterControl < FILTER_OUT))) {

    	filterControl++;
    } else if (distance2 >= 50) {
     
    	distance1 = distance2;
    } else {
      filterControl = 0;
      distance1 = distance2;
    }
    return distance1;
  }

  // Conversion methods from lab2
  private static int convertDistance(double radius, double distance) {
    return (int) ((180.0 * distance) / (Math.PI * radius));
  }

  private static int convertAngle(double radius, double width, double angle) {
    return convertDistance(radius, Math.PI * width * angle / 360.0);
  }



}