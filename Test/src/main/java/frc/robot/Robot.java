// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import edu.wpi.first.wpilibj.TimedRobot;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.CommandScheduler;
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
/**
 * The VM is configured to automatically run this class, and to call the functions corresponding to
 * each mode, as described in the TimedRobot documentation. If you change the name of this class or
 * the package after creating this project, you must also update the build.gradle file in the
 * project.
 */
public class Robot extends TimedRobot {
  private Command m_autonomousCommand;
  
  private RobotContainer m_robotContainer;

  /**
   * This function is run when the robot is first started up and should be used for any
   * initialization code.
   */
  @Override
  public void robotInit() {
    // Instantiate our RobotContainer.  This will perform all our button bindings, and put our
    // autonomous chooser on the dashboard.

    Thread visionThread = new Thread(() -> apriltagVisionThreadProc());
    visionThread.setDaemon(true);
    visionThread.start();

  //   Code seems to not be necessary
  //   CameraServer.startAutomaticCapture();
  //  CvSink cvsink = CameraServer.getVideo();

  //   Mat image = new Mat();
  //   cvsink.grabFrame(image);
  //   AprilTagDetector april_tag_detector = new AprilTagDetector();

  //  april_tag_detector.addFamily("tag36h11");

  //   AprilTagDetection[] april_tag_detection_array = april_tag_detector.detect(image);
  //   AprilTagPoseEstimator.Config april_tag_pose_estimator_config = new AprilTagPoseEstimator.Config(0.1524, 1377.154, 1387.105, 694.783, 406.604);
  //   AprilTagPoseEstimator april_tag_estimator = new AprilTagPoseEstimator(april_tag_pose_estimator_config);
  //   List<Transform3d> april_tag_poses = new ArrayList<Transform3d>();


  //   if(april_tag_detection_array.length != 0){
  //     for (AprilTagDetection april_tag_detection: april_tag_detection_array) {
  //     Transform3d april_tag_pose = april_tag_estimator.estimate(april_tag_detection);
  //     april_tag_poses.add(april_tag_pose);
  //     }
  //   }


  //   for (Transform3d april_tag_pose: april_tag_poses) {
  //     System.out.println("This is the X: " + april_tag_pose.getX());
  //     System.out.println("This is the Y: " + april_tag_pose.getY());
  //     System.out.println("This is the Z: " + april_tag_pose.getZ());
  //   }
    
    m_robotContainer = new RobotContainer();
  }


  /**
   * This function is called every 20 ms, no matter the mode. Use this for items like diagnostics
   * that you want ran during disabled, autonomous, teleoperated and test.
   *
   * <p>This runs after the mode specific periodic functions, but before LiveWindow and
   * SmartDashboard integrated updating.
   */
  @Override
  public void robotPeriodic() {
    // Runs the Scheduler.  This is responsible for polling buttons, adding newly-scheduled
    // commands, running already-scheduled commands, removing finished or interrupted commands,
    // and running subsystem periodic() methods.  This must be called from the robot's periodic
    // block in order for anything in the Command-based framework to work.
    CommandScheduler.getInstance().run();
  }

  /** This function is called once each time the robot enters Disabled mode. */
  @Override
  public void disabledInit() {}

  @Override
  public void disabledPeriodic() {}

  /** This autonomous runs the autonomous command selected by your {@link RobotContainer} class. */
  @Override
  public void autonomousInit() {
    m_autonomousCommand = m_robotContainer.getAutonomousCommand();

    // schedule the autonomous command (example)
    if (m_autonomousCommand != null) {
      m_autonomousCommand.schedule();
    }
  }

  /** This function is called periodically during autonomous. */
  @Override
  public void autonomousPeriodic() {}

  @Override
  public void teleopInit() {
    // This makes sure that the autonomous stops running when
    // teleop starts running. If you want the autonomous to
    // continue until interrupted by another command, remove
    // this line or comment it out.
    if (m_autonomousCommand != null) {
      m_autonomousCommand.cancel();
    }
  }

  /** This function is called periodically during operator control. */
  @Override
  public void teleopPeriodic() {}

  @Override
  public void testInit() {
    // Cancels all running commands at the start of test mode.
    CommandScheduler.getInstance().cancelAll();
  }

  /** This function is called periodically during test mode. */
  @Override
  public void testPeriodic() {}

  /** This function is called once when the robot is first started up. */
  @Override
  public void simulationInit() {}

  /** This function is called periodically whilst in simulation. */
  @Override
  public void simulationPeriodic() {}

   void apriltagVisionThreadProc(){
    AprilTagDetector detector = new AprilTagDetector();
    detector.addFamily("tag16h5",0);

    // Get the UsbCamera from CameraServer
    UsbCamera camera = CameraServer.startAutomaticCapture();
    //Set the resolution
    camera.setResolution(640, 480); //double check if this works with our camera

    //Get a CvSink. This will caputre Mats from the camera
    CvSink cvSink = CameraServer.getVideo();
    //Setup a CvSource. this will send images back to the Dashboard
    CvSource outputStream = CameraServer.putVideo("detect", 640, 480);
    var tagSize = 36;
    //Camera Logitech C920 Camera
    // var fx = 939.646;
    // var fy = 945.506; 

    //Camera Logitech C270 HD Webcam
    var fx = 1377.154;
    var fy = 1387.105;
    // var _cx = 694.783;
    // var _cy = 406.604;

    // AprilTagPoseEstimator.Config poseEstConfig = new AprilTagPoseEstimator.Config(tagSize, fx, fy, _cx, _cy);
    // AprilTagPoseEstimator estimator = new AprilTagPoseEstimator(poseEstConfig);


    //resuing this Mat
    Mat image = new Mat();
    Mat grayimage = new Mat();
    ArrayList<Integer> tags = new ArrayList<>();
    ArrayList<Double> Xvalues = new ArrayList<>();
    ArrayList<Double> Yvalues = new ArrayList<>();
    ArrayList<Double> Zvalues = new ArrayList<>();
    ArrayList<Double> Pitchvalues = new ArrayList<>();
    ArrayList<Double> Rollvalues = new ArrayList<>();
    ArrayList<Double> Yawvalues = new ArrayList<>();

    Scalar outlineColor = new Scalar(0,255,0); //
    Scalar xColor = new Scalar(0,0,255); 

    while (!Thread.interrupted()){
      //Tell the CvSink to grab a frame and put it in source mat, if error notify the output
      if (cvSink.grabFrame(image)==0){
        //send the output the error
        outputStream.notifyError(cvSink.getError());
        //skip the rest of the current iteration
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

      for(AprilTagDetection detection : detections){
        tags.add(detection.getId());

        for(var i = 0; i <=3; i++){
          var j = (i + 1) % 4;
          var pt1 = new Point(detection.getCornerX(i), detection.getCornerY(i));
          var pt2 = new Point(detection.getCornerX(j), detection.getCornerY(j));
          Imgproc.line(image, pt1, pt2, outlineColor, 2);
        }

        var cx = detection.getCenterX();
        var cy = detection.getCenterY();
        var num = 10;
        Imgproc.line(image, new Point(cx - num, cy), new Point(cx + num, cy), xColor, 2);
        Imgproc.line(image, new Point(cx, cy - num), new Point(cx, cy - num), xColor, 2);
        Imgproc.putText(image, Integer.toString(detection.getId()), new Point(cx + num, cy), Imgproc.FONT_HERSHEY_SIMPLEX, 1, xColor, 3);


        // AprilTagPoseEstimator.Config poseEstConfig = new AprilTagPoseEstimator.Config(tagSize, fx, fy, cx, cy);
        // AprilTagPoseEstimator estimator = new AprilTagPoseEstimator(poseEstConfig);
        // Transform3d pose = estimator.estimate(detection);
        // poses.add(pose);


      
      for(Transform3d _pose : poses){
        Xvalues.add(_pose.getX());
        Yvalues.add(_pose.getY());
        Zvalues.add(_pose.getZ());
        Pitchvalues.add(_pose.getRotation().getX());
        Rollvalues.add(_pose.getRotation().getY());
        Yawvalues.add(_pose.getRotation().getZ());
        //Check if we can use getDistance to our advantage.
              SmartDashboard.putString("X Values", Xvalues.toString());
      SmartDashboard.putString("Y Values", Yvalues.toString());
      SmartDashboard.putString("Z Values", Zvalues.toString());
      SmartDashboard.putString("Pitch Values", Pitchvalues.toString());
      SmartDashboard.putString("Roll Values", Rollvalues.toString());
      SmartDashboard.putString("Yaw Values", Yawvalues.toString());
      //Put Tag ID on Array
     //SmartDashboard.putString("tag", tags.toString());

      //detect Specific April Tag
      // Give the output stream a new image to display
      outputStream.putFrame(image);

        
        // SmartDashboard.putNumber("X: ", _pose.getX());
        // SmartDashboard.putNumber("Y: ", _pose.getY());
        // SmartDashboard.putNumber("Z: ", _pose.getZ());
        // SmartDashboard.putNumber("Pitch: ", _pose.getRotation().getX());
        // SmartDashboard.putNumber("Roll: ", _pose.getRotation().getY());
        // SmartDashboard.putNumber("Yaw: ", _pose.getRotation().getZ());


        // System.out.println("X: " + _pose.getX());
        // System.out.println("Y: " + _pose.getY());
        // System.out.println("Z: " + _pose.getZ());
        // System.out.println("Pitch: " + _pose.getRotation().getX());
        // System.out.println("Roll: " + _pose.getRotation().getY());
        // System.out.println("Yaw: " + _pose.getRotation().getZ());

      }
      SmartDashboard.putString("tag", tags.toString());

      //Put X, Y, Z, Pitch, Roll, Yaw values on SmartDashboard

    }

    detector.close();
  }

  }


  


}
