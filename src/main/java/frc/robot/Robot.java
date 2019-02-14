/*----------------------------------------------------------------------------*/
/* Copyright (c) 2017-2018 FIRST. All Rights Reserved.                        */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot;

import java.util.ArrayList;
import java.util.List;

import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.can.TalonSRX;
import com.ctre.phoenix.motorcontrol.can.WPI_TalonSRX;
import com.kauailabs.navx.frc.AHRS;

import org.opencv.core.Mat;
import org.opencv.core.MatOfPoint;
import org.opencv.videoio.VideoCapture;
import org.opencv.core.Point;
import org.opencv.core.Rect;
import org.opencv.imgproc.Imgproc;

import edu.wpi.cscore.CvSink;
import edu.wpi.cscore.UsbCamera;
import edu.wpi.cscore.VideoSource;
import edu.wpi.first.cameraserver.CameraServer;
import edu.wpi.first.vision.VisionRunner;
import edu.wpi.first.vision.VisionThread;
import edu.wpi.first.wpilibj.AnalogInput;
import edu.wpi.first.wpilibj.Encoder;
import edu.wpi.first.wpilibj.Joystick;
import edu.wpi.first.wpilibj.SPI;
import edu.wpi.first.wpilibj.TimedRobot;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.drive.DifferentialDrive;
import edu.wpi.first.wpilibj.interfaces.Potentiometer;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;

/**
 * This is a demo program showing the use of the RobotDrive class, specifically
 * it contains the code necessary to operate a robot with tank drive.
 */
public class Robot extends TimedRobot {
  AHRS ahrs;
  CameraServer cameraServer;
  WPI_TalonSRX frontLeft;
  WPI_TalonSRX frontRight;
  WPI_TalonSRX rearLeft;
  WPI_TalonSRX rearRight;
  DifferentialDrive drive;
  Joystick leftStick;
  Joystick rightStick;
  UsbCamera camera;
  TapePipe pipe;
  VideoCapture cap;
  Mat mat;
  VisionThread visionThread;
  CvSink sink;
  ArrayList<MatOfPoint> contours;
  AnalogInput potentiometer;
  Encoder left;
  Encoder right;
  Timer timer;
  TapeThread tapeThread;

  boolean autonomous = false;
  double leftPower = 0;
  double rightPower = 0;
  double visionTimeout = 0.5;

  @Override
  public void robotInit() {

    ahrs = new AHRS(SPI.Port.kMXP);

    timer = new Timer();

    frontLeft = new WPI_TalonSRX(1);
    frontRight = new WPI_TalonSRX(4);
    rearLeft = new WPI_TalonSRX(2);
    rearRight = new WPI_TalonSRX(3);

    drive = new DifferentialDrive(frontLeft, frontRight);

    rearLeft.set(ControlMode.Follower, frontLeft.getBaseID());
    rearRight.set(ControlMode.Follower, frontRight.getBaseID());

    leftStick = new Joystick(0);
    rightStick = new Joystick(1);

    left = new Encoder(0, 1);
    right = new Encoder(3, 4);
    camera = CameraServer.getInstance().startAutomaticCapture();
   camera.setResolution(800, 600);

   tapeThread = new TapeThread();
   tapeThread.start();

  }
  @Override
  public void teleopPeriodic() {

    leftPower = equation(deadzoneComp(leftStick.getY()) * -1);
    rightPower = equation(deadzoneComp(rightStick.getY()) * -1);
    compareValsWithin(.05);
    if(!autonomous){
  drive.tankDrive(leftPower, rightPower);
    }
    autonomous = false;

    if(rightStick.getTrigger()){
      autonomous = true;
      visionLogic();
  System.out.println(" teleop" + tapeThread.x);
    }
}

public void robotPeriodic(){
}

public double deadzoneComp(double x){
  if(Math.abs(x) < 0.05){
    return 0;
  }else{
    return x;
  }
}

public void compareValsWithin(double zone){
  if(Math.abs(leftPower - rightPower) < zone){
      double newValue = (leftPower + rightPower)/2;
      leftPower = newValue;
      rightPower = newValue;
  }
}

public static double checkAgainstFloor(double x){
  //Used for ensuring that motor outputs are greater than the force of friction holding the robot still

  double floor = .5;
  if(Math.abs(x) < floor){
    if(x>0){return floor;}else{return -floor;}
  }else{
    return x;
  }
}

public void visionLogic(){
/*
  Vision logic for determining what to do while vision mode is enabled
  1. Compare to see if the center of the rectangle is outside of ideal conditions
  2. Turn the robot at the specified speed for the specified duration 
  3.

  54* across 800 pixels = .0675 degrees/pixel

*/


    if(Math.abs(tapeThread.x) > 40 && timer.get() < visionTimeout){
      drive.arcadeDrive((double)0, checkAgainstFloor((double)tapeThread.x/400));
    }else if(Math.abs()){
      
    }

//   if(Math.abs(tapeThread.x) > 40){
//     timer.start();
//     if(timer.get()<0.5){
// drive.arcadeDrive((double)0, checkAgainstFloor((double)tapeThread.x/400));
//     }else{
//       timer.stop();
//       timer.reset();
//     }
// }else{
// drive.arcadeDrive(0.5, 0.0);
//   System.out.println("xx");
// }

}

public static double equation(double a){
  if(a==0) return 0;
  //runs through function f(x) = (x/sqrt2)^2 + .5
 double x = Math.abs(a);
  x = x /Math.sqrt(2);
  x = x*x;
  x = x + 0.5;
  if(a>=0){return x;}
  else{return -x;}
}

public static CameraServer getCameraServer(){
  return CameraServer.getInstance();
}

}
