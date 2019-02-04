/*----------------------------------------------------------------------------*/
/* Copyright (c) 2017-2018 FIRST. All Rights Reserved.                        */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot;

import com.ctre.phoenix.motorcontrol.can.TalonSRX;
import com.ctre.phoenix.motorcontrol.can.WPI_TalonSRX;

import org.opencv.core.Mat;
import org.opencv.videoio.VideoCapture;

import edu.wpi.cscore.UsbCamera;
import edu.wpi.cscore.VideoSource;
import edu.wpi.first.cameraserver.CameraServer;
import edu.wpi.first.vision.VisionRunner;
import edu.wpi.first.vision.VisionThread;
import edu.wpi.first.wpilibj.Joystick;
import edu.wpi.first.wpilibj.TimedRobot;
import edu.wpi.first.wpilibj.drive.DifferentialDrive;

/**
 * This is a demo program showing the use of the RobotDrive class, specifically
 * it contains the code necessary to operate a robot with tank drive.
 */
public class Robot extends TimedRobot {
  CameraServer cameraServer;
  WPI_TalonSRX one;
  UsbCamera camera;
  TapePipe pipe;
  VideoCapture cap;
  Mat mat;
  VisionThread visionThread;

  @Override
  public void robotInit() {
    Mat mat = new Mat();
    camera = CameraServer.getInstance().startAutomaticCapture();
    pipe = new TapePipe();
    
    visionThread = new VisionThread(camera, new TapePipe(), -> {
      mat = pipeline.rgbThresholdOutput();
    });
    
   // cap.read(mat);
    System.out.println(mat.width());
    one = new WPI_TalonSRX(1);
  }
  @Override
  public void teleopPeriodic() {
    one.set(0.05);
  }
}
