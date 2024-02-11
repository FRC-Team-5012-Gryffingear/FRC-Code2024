// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

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
import java.lang.Math;
import java.util.*;

import javax.xml.crypto.dsig.Transform;

import edu.wpi.first.apriltag.AprilTagDetector;
import edu.wpi.first.apriltag.AprilTagPoseEstimator;
import edu.wpi.first.apriltag.AprilTagDetection;
import frc.robot.OpenC;

public class VisionSub extends SubsystemBase {
  /** Creates a new VisionSub. */


  
 // private Alert enableVisionUpdatesAlert =
  //  new Alert("Vision updates are temporarily disabled.", AlertType.WARNING);

  public void apriltagVisionThreadProc(){
    AprilTagDetector detector = new AprilTagDetector();
    detector.addFamily("tag16h5",0);

    //get the UsbCamera from CameraServer
    UsbCamera camera = CameraServer.startAutomaticCapture();
    //set resolution
    camera.setResolution(640, 480);

    //Get a CvSink This will capture Mats from the camera
    CvSink cvsink = CameraServer.getVideo();
    //setup a cvsource this will send immages back to the dashboard
    CvSource outputStream = CameraServer.putVideo("detect", 640, 480);

    Mat image = new Mat();
    Mat grayimage = new Mat();
    ArrayList<Integer> tags = new ArrayList<>();
    ArrayList<Double> Xvalues = new ArrayList<>();
    ArrayList<Double> Yvalues = new ArrayList<>();
    ArrayList<Double> Zvalues = new ArrayList<>();
    ArrayList<Double> Pitchvalues = new ArrayList<>();
    ArrayList<Double> Rollvalues = new ArrayList<>();
    ArrayList<Double> Yawvalues = new ArrayList<>();

    var TagSize = 36;
    //Camera Logitech C270 HD Webcam
    var fx = 1377.154;
    var fy = 1387.105;
  

    Scalar outlineColor = new Scalar(0,255,0);
    Scalar xColor = new Scalar(0,0,255);

    while(!Thread.interrupted()){
      if (cvsink.grabFrame(image)==0){
        outputStream.notifyError(cvsink.getError());
        continue;
      }
      Imgproc.cvtColor(image, grayimage, Imgproc.COLOR_RGB2GRAY);

      AprilTagDetection[] detections = detector.detect(grayimage);
      tags.clear();
      Xvalues.clear();
      Yvalues.clear();
      Zvalues.clear();
      Pitchvalues.clear();
      Rollvalues.clear();
      Yawvalues.clear();
      ArrayList<Transform3d> poses = new ArrayList<Transform3d>();



      for (AprilTagDetection detection : detections){
        tags.add(detection.getId());

        for (var i = 0; i <= 3; i++){
          var j = (i + 1) % 4;
          var pt1 = new Point(detection.getCornerX(i), detection.getCornerY(i));
          var pt2 = new Point(detection.getCornerX(j), detection.getCornerY(j));
          Imgproc.line(image, pt1, pt2, outlineColor, 2);
        }

        var cx = detection.getCenterX();
        var cy = detection.getCenterY();
        var num = 10;
        Imgproc.line(image, new Point(cx - num, cy), new Point(cx + num, cy), xColor, 2);
        Imgproc.line(image, new Point(cx, cy - num), new Point(cx, cy + num), xColor, 2);
        Imgproc.putText(image, Integer.toString(detection.getId()), new Point(cx + num, cy), Imgproc.FONT_HERSHEY_SIMPLEX, 1, xColor, 3);
        AprilTagPoseEstimator.Config poseEstConfig = new AprilTagPoseEstimator.Config(TagSize, fx, fy, cx, cy);
        AprilTagPoseEstimator poseEst = new AprilTagPoseEstimator(poseEstConfig);
        Transform3d pose = poseEst.estimate(detection);
        poses.add(pose);
        adapt(tags,pose);


      }

      for (Transform3d _pose : poses){
        Xvalues.add(_pose.getX());
        Yvalues.add(_pose.getY());
        Zvalues.add(_pose.getZ());
        Pitchvalues.add(_pose.getRotation().getX());
        Rollvalues.add(_pose.getRotation().getY());
        Yawvalues.add(_pose.getRotation().getZ());


      }

      SmartDashboard.putString("X values", Xvalues.toString());
      SmartDashboard.putString("Y values", Yvalues.toString());
      SmartDashboard.putString("Z values", Zvalues.toString());
      SmartDashboard.putString("Pitch values", Pitchvalues.toString());
      SmartDashboard.putString("Roll values", Rollvalues.toString());
      SmartDashboard.putString("Yaw values", Yawvalues.toString());
      


      SmartDashboard.putString("tag", tags.toString());
      
      outputStream.putFrame(image);
    }
    detector.close();
    
  }

 public void adapt(ArrayList<Integer> tagIDs,Transform3d poses){
  for(int i : tagIDs){
    if(i == 1){
      System.out.println("Detected 1");
      if(Math.abs(poses.getX()) < 0.05){
        System.out.println("Xvalue reading: " + poses.getX());
        SmartDashboard.putNumber("Xvaluesssss", poses.getX());


      }
    }
  }
 }
  
  /**
   * Example command factory method.
   *
   * @return a command
   */
  public Command exampleMethodCommand() {
    // Inline construction of command goes here.
    // Subsystem::RunOnce implicitly requires `this` subsystem.
    return runOnce(
        () -> {
          /* one-time action goes here */
        });
  }

  public void func(){
     Thread visionThread = new Thread(()-> apriltagVisionThreadProc());
    visionThread.setDaemon(true);
    visionThread.start();
  }
  /**
   * An example method querying a boolean state of the subsystem (for example, a digital sensor).
   *
   * @return value of some boolean subsystem state, such as a digital sensor.
   */
  public boolean exampleCondition() {
    // Query some boolean state, such as a digital sensor.
    return false;
  }

  @Override
  public void periodic() {

  }

  @Override
  public void simulationPeriodic() {
    // This method will be called once per scheduler run during simulation
  }
}

