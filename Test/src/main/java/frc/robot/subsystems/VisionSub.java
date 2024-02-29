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


public class VisionSub extends SubsystemBase {
  

  // ArrayList<Double> xPose = new ArrayList<>();
  // //double xPose;
  // ArrayList<Integer> ID = new ArrayList<>();
 // private Alert enableVisionUpdatesAlert =
  //  new Alert("Vision updates are temporarily disabled.", AlertType.WARNING);
  private Thread visionThread;
  
  public VisionSub(){
    visionThread = new Thread(this::apriltagVisionThreadProc);
  }

  //Checks if the vision is not running and if so to make the thread connection to the main function
  //and then starts it using the start() command that the thread has
  public void startVision(){
    if(visionThread == null || !visionThread.isAlive()){
      visionThread = new Thread(this::apriltagVisionThreadProc);
      visionThread.start();
    }
  }
//Stops the vision by checking it is running and if so use the 
//Try - catch method to see if there is an exception to use that exception to run the interrupt command
  public void stopVision(){
    if(visionThread.isAlive() || visionThread != null){
      visionThread.interrupt();
      try{
        visionThread.join();
      } catch(InterruptedException e){
        Thread.currentThread().interrupt();
      }
    }
  }


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
    ArrayList<Integer> IDValues = new ArrayList<>();

    var TagSize = 36;
    //Camera Logitech C270 HD Webcam
    var fx = 954.6564;
    var fy = 950.39935;

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

      double cxs = 320;
      double cys = 240;

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
      System.out.println(poses);
      var i = 0;
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
        }
        else if (detections[i].getId() == 6){
          SmartDashboard.putNumber("ID 6 X Value", _pose.getX());
          SmartDashboard.putNumber("ID 6 Y Value", _pose.getY());
          SmartDashboard.putNumber( "ID 6 Z Value", _pose.getZ());
          SmartDashboard.putNumber( "ID 6 Pitch Value", _pose.getRotation().getX());
          SmartDashboard.putNumber("ID 6 Roll Value", _pose.getRotation().getY());
          SmartDashboard.putNumber("ID 6 Yaw Value", _pose.getRotation().getZ());
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

    //   double value_id_3_value = SmartDashboard.getNumber("ID 3 X Value", 0);
    //   SmartDashboard.putNumber("ID 3 X Values", value_id_3_value);

    //   double value_id_5_value = SmartDashboard.getNumber("ID 5 X Value", 0);
    //   SmartDashboard.putNumber("ID 5 X Values", value_id_5_value);

    //   double value_id_6_value = SmartDashboard.getNumber("ID 6 X Value", 0);
    //   SmartDashboard.putNumber("ID 6 X Values", value_id_6_value);

    //   double value_id_7_value = SmartDashboard.getNumber("ID 7 X Value", 0);
    //   SmartDashboard.putNumber("ID 7 X Values", value_id_7_value);

    //   double value_id_8_value = SmartDashboard.getNumber("ID 8 X Value", 0);
    //   SmartDashboard.putNumber("ID 8 X Values", value_id_8_value);

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
    }
    detector.close();
  }


public double get_ID_Xpose(int ID){
    String ID_name;
    double x = 0;

    ID_name = "ID " + ID + " X Value";
    x = SmartDashboard.getNumber(ID_name, 0);
    return x;
}

public double get_ID_Ypose(int ID){
    String ID_name;
    double y = 0;

    ID_name = "ID " + ID + " Y Value";
    y = SmartDashboard.getNumber(ID_name, 0);
    return y;
}

public double get_ID_Zpose(int ID){
    String ID_name;
    double z = 0;

    ID_name = "ID " + ID + " Z Value";
    z = SmartDashboard.getNumber(ID_name, 0);
    return z;
}
//  public double getX_ID_3(){
//     double x = SmartDashboard.getNumber("ID 3 X Value", 0);
//     return x;
//  }

//  public double getX_ID_4(){
//   double x = SmartDashboard.getNumber("ID 4 X Value", 0);
//   return x;
//  }

//  public double getX_ID_5(){
//   double x = SmartDashboard.getNumber("ID 5 X Value", 0);
//   return x;
//  }

//  public double getX_ID_6(){
//   double x = SmartDashboard.getNumber("ID 6 X Value", 0);
//   return x;
//  }
//   public double getX_ID_7(){
//   double x = SmartDashboard.getNumber("ID 7 X Value", 0);
//   return x;
//  }
//  public double getX_ID_8(){
//   double x = SmartDashboard.getNumber("ID 8 X Value", 0);
//   return x;
//  }
  
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
  public void periodic() {}

  @Override
  public void simulationPeriodic() {
    // This method will be called once per scheduler run during simulation
  }
}