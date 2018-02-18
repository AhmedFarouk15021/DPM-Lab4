package ca.mcgill.ecse211.odometer;


import lejos.hardware.motor.EV3LargeRegulatedMotor;

import lejos.robotics.SampleProvider;


public class LightLocalizer {
	public static int FORWARD_SPEED = 100;
	  
	public static int ACCELERATION = 600;

  public static int ROTATION_SPEED = 60;

  
  
  private static double lightintesity = 0.20;
  
  
  private SampleProvider cs;
  private Odometer odometer;
  private EV3LargeRegulatedMotor leftMotor, rightMotor;
  private float[] color;

  public LightLocalizer(Odometer odometer, SampleProvider cs, float[] color,
		  
		  EV3LargeRegulatedMotor leftMotor, EV3LargeRegulatedMotor rightMotor) {
   
    this.color = color;
    
    this.leftMotor = leftMotor;
    
    this.rightMotor = rightMotor;
    
    this.odometer = odometer;
    this.cs = cs;

    rightMotor.setAcceleration(ACCELERATION);

    
    leftMotor.setAcceleration(ACCELERATION);

  }

  public void localize() {


    leftMotor.setSpeed(FORWARD_SPEED);
    
    rightMotor.setSpeed(FORWARD_SPEED);
    
    leftMotor.forward();
    
    rightMotor.forward();

    while (getColorData() > lightintesity) {
            try {
        Thread.sleep(100);
        
      } catch (InterruptedException e) {
    	  
        e.printStackTrace();
      }
    }
    
    leftMotor.stop(true);
    
    rightMotor.stop(true);
    
    double y = odometer.getXYT()[1];
    leftMotor.rotate(-convertDistance(Lab4.WHEEL_RADIUS, y), true); 
    
    rightMotor.rotate(-convertDistance(Lab4.WHEEL_RADIUS, y), false);
    
    
    leftMotor.setSpeed(ROTATION_SPEED);
    
    rightMotor.setSpeed(ROTATION_SPEED);
    
    leftMotor.rotate(convertAngle(Lab4.WHEEL_RADIUS, Lab4.TRACK, 90.0), true);
    
    rightMotor.rotate(-convertAngle(Lab4.WHEEL_RADIUS, Lab4.TRACK, 90.0), false);

    leftMotor.setSpeed(FORWARD_SPEED);
    
    rightMotor.setSpeed(FORWARD_SPEED);
    
    leftMotor.forward();
    
    rightMotor.forward();
    

    while (getColorData() >  lightintesity) { 
      try {
        Thread.sleep(100);
      } catch (InterruptedException e) {
        e.printStackTrace();
      }
    }
    
    leftMotor.stop(true);
    rightMotor.stop(true);
    
    double x = odometer.getXYT()[0];
    
    leftMotor.rotate(-convertDistance(Lab4.WHEEL_RADIUS, x), true); 
    
    rightMotor.rotate(-convertDistance(Lab4.WHEEL_RADIUS, x), false);

    leftMotor.rotate(-convertAngle(Lab4.WHEEL_RADIUS, Lab4.TRACK, 45.0), true);
    
    rightMotor.rotate(convertAngle(Lab4.WHEEL_RADIUS, Lab4.TRACK, 45.0), false);
    
    double d = Math.sqrt((Math.pow(x, 2) + Math.pow(y, 2)));
    double centre = 6.5;
    leftMotor.rotate(convertDistance(Lab4.WHEEL_RADIUS, d+centre), true); 
    
    rightMotor.rotate(convertDistance(Lab4.WHEEL_RADIUS, d+centre), false);

    leftMotor.rotate(-convertAngle(Lab4.WHEEL_RADIUS, Lab4.TRACK, 45.0), true);
    
    rightMotor.rotate(convertAngle(Lab4.WHEEL_RADIUS, Lab4.TRACK, 45.0), false);
    
    odometer.setY(0);

    odometer.setX(0);
    

  }

  private static int convertAngle(double radius, double width, double angle) {
	  
    return convertDistance(radius, Math.PI * width * angle / 360.0);
  }

  private static int convertDistance(double radius, double distance) {
	  
    return (int) ((180.0 * distance) / (Math.PI * radius));
  }

  
  private float getColorData() {
	  
    cs.fetchSample(color, 0);
    
    float Blevel = (color[0] + color[1] + color[2]);
    
    return Blevel;
  }


}