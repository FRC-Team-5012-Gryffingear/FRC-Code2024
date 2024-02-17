// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

import java.util.ArrayList;

import edu.wpi.first.wpilibj.TimedRobot;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.CommandScheduler;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import edu.wpi.first.cameraserver.CameraServer; 
import edu.wpi.first.cscore.CvSink;
import edu.wpi.first.math.geometry.Transform3d;
import edu.wpi.first.math.geometry.Translation3d;
import edu.wpi.first.cscore.CvSource;
import edu.wpi.first.cscore.UsbCamera;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;

import org.opencv.core.Mat;
import org.opencv.core.Point;
import org.opencv.core.Scalar;
import org.opencv.imgproc.Imgproc;

import com.fasterxml.jackson.annotation.JsonTypeInfo.Id;

import java.lang.Math;
import java.util.*;

import javax.xml.crypto.dsig.Transform;

import edu.wpi.first.apriltag.AprilTagDetector;
import edu.wpi.first.apriltag.AprilTagPoseEstimator;
import edu.wpi.first.apriltag.AprilTagDetection;
import frc.robot.OpenC;

public class Split extends SubsystemBase {

  public AprilTagDetector createAprilTagDetector(){
    AprilTagDetector detector = new AprilTagDetector();
    detector.addFamily("tag16h5",0);
    return detector;
  }

  public UsbCamera createCamera(){
    //get the UsbCamera from CameraServer
    UsbCamera camera = CameraServer.startAutomaticCapture();
    //set resolution
    camera.setResolution(640, 480);
    return camera;
  }

  public CvSink createCvSink(){
    CvSink cvsink = CameraServer.getVideo();
    return cvsink;
  }
  
  public CvSource createOutputStream(){
    CvSource outputStream = CameraServer.putVideo("detect", 640, 480);
    return outputStream;
  }

  public Mat createImage(){
    Mat image = new Mat();
    return image;
   
  }
  
  public Mat createGrayImage(){
    Mat grayImage = new Mat();
        return grayImage;
  }

  
  public void makeGrayImage(){
    Imgproc.cvtColor(createGrayImage(), createGrayImage(), Imgproc.COLOR_RGB2GRAY);
  }
 
  
  @Override
  public void periodic() {
    // This method will be called once per scheduler run

  }

  @Override
  public void simulationPeriodic() {
    // This method will be called once per scheduler run during simulation
  }
}
