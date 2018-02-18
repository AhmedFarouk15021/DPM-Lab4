
package ca.mcgill.ecse211.odometer;


import lejos.hardware.lcd.TextLCD;

import lejos.hardware.motor.EV3LargeRegulatedMotor;
import lejos.hardware.port.Port;
import lejos.hardware.sensor.*;
import ca.mcgill.ecse211.odometer.Odometer;
import ca.mcgill.ecse211.odometer.LightLocalizer;
import lejos.hardware.Button;
import lejos.hardware.ev3.LocalEV3;
import lejos.robotics.SampleProvider;
import ca.mcgill.ecse211.odometer.UltrasonicLocalizer;




public class Lab4 {
	
 ;
  private static final EV3LargeRegulatedMotor rightMotor =
		  
      new EV3LargeRegulatedMotor(LocalEV3.get().getPort("D"));
  
  private static final EV3LargeRegulatedMotor leftMotor =
		  
	      new EV3LargeRegulatedMotor(LocalEV3.get().getPort("A"));
  
  
  private static final TextLCD lcd = LocalEV3.get().getTextLCD();
  

  
  public static final double TRACK = 10.5;
  
  public static final double WHEEL_RADIUS = 2.08;

  
  private static final Port colorPort = LocalEV3.get().getPort("S1");


  public static void main(String[] args) throws OdometerExceptions {

    int buttonChoice;

    @SuppressWarnings("resource")
	SensorModes myColor = new EV3ColorSensor(colorPort);
    float[] colorData = new float[3];
    SampleProvider myColorSample = myColor.getMode("Red");

    
    
    Odometer odometer = new Odometer(leftMotor, rightMotor, TRACK ,WHEEL_RADIUS);
    
    Display odometryDisplay = new Display(lcd, odometer);
    
    UltrasonicLocalizer usLocalizer = new UltrasonicLocalizer(odometer, leftMotor, rightMotor);
    
    OdometerData data = new OdometerData();

    LightLocalizer lsLocalizer = new LightLocalizer(odometer, myColorSample, colorData, leftMotor, rightMotor);

    do {
    	lcd.clear();

    	lcd.drawString("< Left | Right >", 0, 0);
    	lcd.drawString("       |        ", 0, 1);
    	lcd.drawString("Falling| Rising ", 0, 2);
    	lcd.drawString("Edge   | Edge	", 0, 3);
    	lcd.drawString("       |  		", 0, 4);

      buttonChoice = Button.waitForAnyPress();
      
    } while (buttonChoice != Button.ID_LEFT && buttonChoice != Button.ID_RIGHT);

    if (buttonChoice == Button.ID_LEFT) {

    	Thread odoThread = new Thread(odometer);
    	
        odoThread.start();
        
      Thread odometryDisplayThread = new Thread(odometryDisplay);
      odometryDisplayThread.start();
      usLocalizer.Falling_Edge();
      data.start();
      
      
      lcd.drawString("Press any", 0, 5);
      
      lcd.drawString("Button", 0, 6);
      
      Button.waitForAnyPress();


      lcd.clear();
      

      lsLocalizer.localize();

    }


    else {

    	Thread odoThread = new Thread(odometer);
    	
        odoThread.start();
        
      Thread odometryDisplayThread = new Thread(odometryDisplay);
      
      odometryDisplayThread.start();
      usLocalizer.Rising_Edge();
      data.start();
      // wait for any before ruining light localizer
      lcd.drawString("Press any", 0, 5);
      lcd.drawString("Button", 0, 6);
      Button.waitForAnyPress();

      // clear display
      lcd.clear();
      
      // Run light localizer
      lsLocalizer.localize();

    }

    while (Button.waitForAnyPress() != Button.ID_ESCAPE);
    System.exit(0);
  }

  }