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

import com.fasterxml.jackson.annotation.JsonTypeInfo.Id;

import java.lang.Math;
import java.util.*;

import javax.xml.crypto.dsig.Transform;

import edu.wpi.first.apriltag.AprilTagDetector;
import edu.wpi.first.apriltag.AprilTagPoseEstimator;
import edu.wpi.first.apriltag.AprilTagDetection;
import frc.robot.OpenC;

public class VisionSub extends SubsystemBase {
  /** Creates a new VisionSub. */


  // ArrayList<Double> xPose = new ArrayList<>();
  // //double xPose;
  // ArrayList<Integer> ID = new ArrayList<>();
 // private Alert enableVisionUpdatesAlert =
  //  new Alert("Vision updates are temporarily disabled.", AlertType.WARNING);

  public void apriltagVisionThreadProc(){
    AprilTagDetector detector = new AprilTagDetector();
    detector.addFamily("tag36h11",0);

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
    ArrayList<Integer> tagsIDs = new ArrayList<>();
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
      tagsIDs.clear();
      Xvalues.clear();
      Yvalues.clear();
      Zvalues.clear();
      Pitchvalues.clear();
      Rollvalues.clear();
      Yawvalues.clear();
     // xPose.clear();
      ArrayList<Transform3d> poses = new ArrayList<Transform3d>();



      for (AprilTagDetection detection : detections){
        tagsIDs.add(detection.getId());
       // ID.add(detection.getId());
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
      //  adapt(tags,pose);


      }
      System.out.println(poses);
      for (Transform3d _pose : poses){
        Xvalues.add(_pose.getX());
        Yvalues.add(_pose.getY());
        Zvalues.add(_pose.getZ());
        Pitchvalues.add(_pose.getRotation().getX());
        Rollvalues.add(_pose.getRotation().getY());
        Yawvalues.add(_pose.getRotation().getZ());

        // for(int i : tagsIDs){
        //     if(i == 1){
        //         // xPose.add(_pose.getX());
        //         // xPose.get(0);
        //         // SmartDashboard.putNumber("Number X: ", xPose.get(0));
        //         double x = Double.parseDouble(SmartDashboard.getString("X values","0"));
        //        //System.out.println("This is the X value within: " + x );

        //     }
        // }

      }
// Get the IDs array, check if num matches, then populate the double with the X value of the array 
// 
      SmartDashboard.putString("X values", Xvalues.toString());
      SmartDashboard.putString("Y values", Yvalues.toString());
      SmartDashboard.putString("Z values", Zvalues.toString());
      SmartDashboard.putString("Pitch values", Pitchvalues.toString());
      SmartDashboard.putString("Roll values", Rollvalues.toString());
      SmartDashboard.putString("Yaw values", Yawvalues.toString());

      //double x = SmartDashboard.getString("X values","0");
       System.out.println("This is the X value within: " + Double.parseDouble(SmartDashboard.getString("X values","0")));
      
     //System.out.println("This is the true X: " + SmartDashboard.getData("X values"));
      


      SmartDashboard.putString("tag", tagsIDs.toString());
      
      outputStream.putFrame(image);
    }
    detector.close();
  }

 public void adapt(ArrayList<Integer> tagIDs,Transform3d poses){
  ArrayList<Double> Storage = new ArrayList<>();
  for(int i : tagIDs){
    if(i == 1){
      System.out.println("Detected 1");
      if(Math.abs(poses.getX()) < 0.05){
        Storage.add(poses.getX());
        System.out.println("Xvalue reading: " + poses.getX());
        SmartDashboard.putNumber("Xvaluesssss", poses.getX());
      }
    }
  }
 }

 public double getXs(){
    //System.out.println("This is the X val: " + xPose.get(0));
    // SmartDashboard.putNumber("This is the X value", xPose.get(0));
    return 0;
 }

  

  @Override
  public void periodic() {
    getXs();
  }

  @Override
  public void simulationPeriodic() {
    // This method will be called once per scheduler run during simulation
  }
}

