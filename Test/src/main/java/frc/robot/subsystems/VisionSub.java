// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import java.util.ArrayList;

import edu.wpi.first.wpilibj.DigitalOutput;
import edu.wpi.first.wpilibj.TimedRobot;
import edu.wpi.first.wpilibj.Timer;
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
import java.util.*;

import java.lang.Math;

import javax.xml.crypto.dsig.Transform;

import edu.wpi.first.apriltag.AprilTagDetector;
import edu.wpi.first.apriltag.AprilTagPoseEstimator;
import edu.wpi.first.apriltag.AprilTagDetection;

public class VisionSub extends SubsystemBase {
  /** Creates a new VisionSub. */
private Thread visionThread;
// public boolean Visioncheck = false;
// public boolean AprilTagCheck = false;

  public VisionSub(){
    visionThread = new Thread(this::apriltagVisionThreadProc);
  }







  // ArrayList<Double> xPose = new ArrayList<>();
  // //double xPose;
  // ArrayList<Integer> ID = new ArrayList<>();
  public void startThread(){
    if(!visionThread.isAlive() || visionThread == null){
      visionThread = new Thread(this::apriltagVisionThreadProc);
      visionThread.start();
      // Visioncheck = true;
      // LED_off();
    }
  }

  public void stopThread(){
    if(visionThread!= null || visionThread.isAlive()){
      visionThread.interrupt();
      try {
        visionThread.join();
      } catch (InterruptedException e){
        Thread.currentThread().interrupt();
        // Visioncheck = false;
        // AprilTagCheck = false;
      }
    }
  }

  public void apriltagVisionThreadProc(){
    AprilTagDetector detector = new AprilTagDetector();
    detector.addFamily("tag36h11",0);

    //get the UsbCamera from CameraServer
    UsbCamera camera = CameraServer.startAutomaticCapture();
    //set resolution
    camera.setResolution(320, 240); //4:3 ratio
    camera.setFPS(15);
    camera.setExposureAuto();
    camera.setWhiteBalanceAuto();
    camera.setBrightness(3);
    // LED_on();

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
    ArrayList<Integer> IDValues = new ArrayList<>();

    
    var TagSize = 36;
    
    
  

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
      IDValues.clear();

     // xPose.clear();
      ArrayList<Transform3d> poses = new ArrayList<Transform3d>();
      //CHECK THESE NEW VALUES
      //FOV is information grabbed from logitech website in degrees.
      //fx and fy is found using a formula found on github.
      //cx and cy is found using the grayimage height/width because grayImage is used to grab apriltags.
      var FOV = 55;
      var fx = (grayimage.height() / 2) / Math.tan((Math.PI * FOV / 180) / 2);
      var fy = fx;
      
      // Values we orginillay used for fx and fy:
      // var fxx = 954.6564522093413;
      // var fyy = 950.3993532691603;

      // double cxs = 334.86944443989194;
      // double cys = 221.42493856582405;
      double cxs = grayimage.width() / 2;
      double cys = grayimage.height() / 2;
      // Values we originally usedin cx and cy:
      // double cxs = 320;
      // double cys = 240;


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
        AprilTagPoseEstimator.Config poseEstConfig = new AprilTagPoseEstimator.Config(TagSize, fx, fy, cxs, cys);
        AprilTagPoseEstimator poseEst = new AprilTagPoseEstimator(poseEstConfig);
        Transform3d pose = poseEst.estimate(detection);
        poses.add(pose);
      //  adapt(tags,pose);
      }
      // System.out.println(poses);
      var i = 0;

      if (poses.isEmpty())
      {
        // AprilTagCheck = false;
          SmartDashboard.putNumber("ID 4 X Value",0);
          SmartDashboard.putNumber("ID 4 Y Value",0);
          SmartDashboard.putNumber( "ID 4 Z Value",0);
          SmartDashboard.putNumber( "ID 4 Pitch Value",0);
          SmartDashboard.putNumber("ID 4 Roll Value",0);
          SmartDashboard.putNumber("ID 4 Yaw Value",0);

          SmartDashboard.putNumber("ID 3 X Value",0);
          SmartDashboard.putNumber("ID 3 Y Value",0);
          SmartDashboard.putNumber( "ID 3 Z Value",0);
          SmartDashboard.putNumber( "ID 3 Pitch Value",0);
          SmartDashboard.putNumber("ID 3 Roll Value",0);
          SmartDashboard.putNumber("ID 3 Yaw Value",0);

          SmartDashboard.putNumber("ID 5 X Value",0);
          SmartDashboard.putNumber("ID 5 Y Value",0);
          SmartDashboard.putNumber( "ID 5 Z Value",0);
          SmartDashboard.putNumber( "ID 5 Pitch Value",0);
          SmartDashboard.putNumber("ID 5 Roll Value",0);
          SmartDashboard.putNumber("ID 5 Yaw Value",0);

          SmartDashboard.putNumber("ID 6 X Value",0);
          SmartDashboard.putNumber("ID 6 Y Value",0);
          SmartDashboard.putNumber( "ID 6 Z Value",0);
          SmartDashboard.putNumber( "ID 6 Pitch Value", 0);
          SmartDashboard.putNumber("ID 6 Roll Value",0);
          SmartDashboard.putNumber("ID 6 Yaw Value",0);

          SmartDashboard.putNumber("ID 7 X Value",0);
          SmartDashboard.putNumber("ID 7 Y Value",0);
          SmartDashboard.putNumber( "ID 7 Z Value",0);
          SmartDashboard.putNumber( "ID 7 Pitch Value",0);
          SmartDashboard.putNumber("ID 7 Roll Value",0);
          SmartDashboard.putNumber("ID 7 Yaw Value",0);

          SmartDashboard.putNumber("ID 8 X Value",0);
          SmartDashboard.putNumber("ID 8 Y Value",0);
          SmartDashboard.putNumber( "ID 8 Z Value",0);
          SmartDashboard.putNumber( "ID 8 Pitch Value",0);
          SmartDashboard.putNumber("ID 8 Roll Value",0);
          SmartDashboard.putNumber("ID 8 Yaw Value",0);
      }

      for (Transform3d _pose : poses){
        if(detections[i].getId() == 4){
          SmartDashboard.putNumber("ID 4 X Value", _pose.getX());
          SmartDashboard.putNumber("ID 4 Y Value", _pose.getY());
          SmartDashboard.putNumber( "ID 4 Z Value", _pose.getZ());
          SmartDashboard.putNumber( "ID 4 Pitch Value", _pose.getRotation().getX());
          SmartDashboard.putNumber("ID 4 Roll Value", _pose.getRotation().getY());
          SmartDashboard.putNumber("ID 4 Yaw Value", _pose.getRotation().getZ());
        }
        else if(detections[i].getId() == 3){
          SmartDashboard.putNumber("ID 3 X Value", _pose.getX());
          SmartDashboard.putNumber("ID 3 Y Value", _pose.getY());
          SmartDashboard.putNumber( "ID 3 Z Value", _pose.getZ());
          SmartDashboard.putNumber( "ID 3 Pitch Value", _pose.getRotation().getX());
          SmartDashboard.putNumber("ID 3 Roll Value", _pose.getRotation().getY());
          SmartDashboard.putNumber("ID 3 Yaw Value", _pose.getRotation().getZ());
        }
        else if(detections[i].getId() == 5){
          SmartDashboard.putNumber("ID 5 X Value", _pose.getX());
          SmartDashboard.putNumber("ID 5 Y Value", _pose.getY());
          SmartDashboard.putNumber( "ID 5 Z Value", _pose.getZ());
          SmartDashboard.putNumber( "ID 5 Pitch Value", _pose.getRotation().getX());
          SmartDashboard.putNumber("ID 5 Roll Value", _pose.getRotation().getY());
          SmartDashboard.putNumber("ID 5 Yaw Value", _pose.getRotation().getZ());
          // LED_blink();
          // AprilTagCheck = true;
        }
        else if (detections[i].getId() == 6){
          SmartDashboard.putNumber("ID 6 X Value", _pose.getX());
          SmartDashboard.putNumber("ID 6 Y Value", _pose.getY());
          SmartDashboard.putNumber( "ID 6 Z Value", _pose.getZ());
          SmartDashboard.putNumber( "ID 6 Pitch Value", Math.toDegrees(_pose.getRotation().getX()));
          SmartDashboard.putNumber("ID 6 Roll Value", _pose.getRotation().getY());
          SmartDashboard.putNumber("ID 6 Yaw Value", _pose.getRotation().getZ());
          // LED_blink();
          // AprilTagCheck = true;
        }
        else if (detections[i].getId() == 7){
          SmartDashboard.putNumber("ID 7 X Value", _pose.getX());
          SmartDashboard.putNumber("ID 7 Y Value", _pose.getY());
          SmartDashboard.putNumber( "ID 7 Z Value", _pose.getZ());
          SmartDashboard.putNumber( "ID 7 Pitch Value", _pose.getRotation().getX());
          SmartDashboard.putNumber("ID 7 Roll Value", _pose.getRotation().getY());
          SmartDashboard.putNumber("ID 7 Yaw Value", _pose.getRotation().getZ());}
        else if (detections[i].getId() == 8){
          SmartDashboard.putNumber("ID 8 X Value", _pose.getX());
          SmartDashboard.putNumber("ID 8 Y Value", _pose.getY());
          SmartDashboard.putNumber( "ID 8 Z Value", _pose.getZ());
          SmartDashboard.putNumber( "ID 8 Pitch Value", _pose.getRotation().getX());
          SmartDashboard.putNumber("ID 8 Roll Value", _pose.getRotation().getY());
          SmartDashboard.putNumber("ID 8 Yaw Value", _pose.getRotation().getZ());
        }
        
        i = i + 1 ;
        // Xvalues.add(_pose.getX());
        // Yvalues.add(_pose.getY());
        // Zvalues.add(_pose.getZ());
        // Pitchvalues.add(_pose.getRotation().getX());
        // Rollvalues.add(_pose.getRotation().getY());
        // Yawvalues.add(_pose.getRotation().getZ());
        // IDValues.add(detections[i].getId());
        // i = i + 1;

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

      double value_id_4_value = SmartDashboard.getNumber("ID 4 X Value", 0);
      SmartDashboard.putNumber("ID 4 X Values", value_id_4_value);

      double value_id_3_value = SmartDashboard.getNumber("ID 3 X Value", 0);
      SmartDashboard.putNumber("ID 3 X Values", value_id_3_value);

      double value_id_5_value = SmartDashboard.getNumber("ID 5 X Value", 0);
      SmartDashboard.putNumber("ID 5 X Values", value_id_5_value);

      double value_id_6_value = SmartDashboard.getNumber("ID 6 X Value", 0);
      SmartDashboard.putNumber("ID 6 X Values", value_id_6_value);

      double value_id_7_value = SmartDashboard.getNumber("ID 7 X Value", 0);
      SmartDashboard.putNumber("ID 7 X Values", value_id_7_value);

      double value_id_8_value = SmartDashboard.getNumber("ID 8 X  Value", 0);
      SmartDashboard.putNumber("ID 8 X Values", value_id_8_value);

      SmartDashboard.putString("X values", Xvalues.toString());
      SmartDashboard.putString("Y values", Yvalues.toString());
      SmartDashboard.putString("Z values", Zvalues.toString());
      SmartDashboard.putString("Pitch values", Pitchvalues.toString());
      SmartDashboard.putString("Roll values", Rollvalues.toString());
      SmartDashboard.putString("Yaw values", Yawvalues.toString());
      SmartDashboard.putString("TagIDs", IDValues.toString());

      //double x = SmartDashboard.getString("X values","0");
      //  System.out.println("This is the X value within: " + Double.parseDouble(SmartDashboard.getString("X values","0")));
      
     //System.out.println("This is the true X: " + SmartDashboard.getData("X values"));
      


      SmartDashboard.putString("tag", tagsIDs.toString());
      
      
      outputStream.putFrame(image);
      System.out.println("ENDDDNNDNDND");
    }
    detector.close();
  }
  public double getXID(int ID){
    String ID_name;
    double x;
  
    ID_name = "ID " + ID + " X Value";
    x = SmartDashboard.getNumber(ID_name, 0);
  
    System.out.println("THE VALUE OF X: " + x);
      return x;
  }
  
  public double getIDroll(int ID){
    String ID_name;
    double roll;
  
    ID_name = "ID " + ID + " Roll Value";
    roll = SmartDashboard.getNumber(ID_name, 0);

    System.out.println("THE VALUE OF ROLL: " + roll);
      return roll;
  }
  
  public double getZID(int ID){
    String ID_name;
    double z;
    ID_name = "ID " + ID + " Z Value";
    z = SmartDashboard.getNumber(ID_name, 0);
    System.out.println("THE VALUE OF Z: " + z);
      return z;
  }
  
 
 
  @Override
  public void periodic() {
  }

  @Override
  public void simulationPeriodic() {
    // This method will be called once per scheduler run during simulation
  }
}