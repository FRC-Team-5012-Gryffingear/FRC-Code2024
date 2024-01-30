// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

/*
 * Copyright (c) 2009-2023 FIRST and other WPILib contributors All rights reserved.
 */
package frc.robot.subsystems;

import com.ctre.phoenix6.hardware.Pigeon2;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.kinematics.SwerveDriveKinematics;
import edu.wpi.first.math.kinematics.SwerveDriveOdometry;
import edu.wpi.first.math.kinematics.SwerveModulePosition;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import edu.wpi.first.wpilibj.smartdashboard.Field2d;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;


public class SwerveSubsystem extends SubsystemBase {
 


private final SwerveMod frontLeftMod = new SwerveMod(
    Constants.FrontLeftDriveID, Constants.FrontLeftSteerID, Constants.FrontLeftEncoderID, Constants.FrontLeftOffset, Constants.FrontLeftInv, "FL");
private final SwerveMod frontRightMod = new SwerveMod(
    Constants.FrontRightDriveID, Constants.FrontRightSteerID, Constants.FrontRightEncoderID, Constants.FrontRightOffset, Constants.FrontRightInv, "FR");
private final SwerveMod backLeftMod = new SwerveMod(
    Constants.BackLeftDriveID, Constants.BackLeftSteerID, Constants.BackLeftEncoderID, Constants.BackLeftOffset, Constants.BackLeftInv, "BL");
private final SwerveMod backRightMod = new SwerveMod(
    Constants.BackRightDriveID, Constants.BackRightSteerID, Constants.BackRightEncoderID, Constants.BackRightOffset, Constants.BackRightInv, "BR");

private final Pigeon2 pigeon = new Pigeon2(Constants.PigeonID);



private final SwerveDriveOdometry odometry = new SwerveDriveOdometry(Constants.kinematics, getHeading(), new SwerveModulePosition[] {
    frontLeftMod.getModPos(),
    frontRightMod.getModPos(),
    backLeftMod.getModPos(),
    backRightMod.getModPos()
});

//makes a 2d field


private Field2d fieldMaker = new Field2d();

  public SwerveSubsystem() {

  }





  @Override
  public void periodic() {
    // updates odometry
    odometry.update(getHeading(), new SwerveModulePosition[] {
        frontLeftMod.getModPos(),
        frontRightMod.getModPos(),
        backLeftMod.getModPos(),
        backRightMod.getModPos()
    });

    //gets info on drive and steer Motor
    frontLeftMod.Info();
    frontRightMod.Info();
    backLeftMod.Info();
    backRightMod.Info();


    fieldMaker.setRobotPose(getPose());
    SmartDashboard.putData(fieldMaker);
  }



  public Rotation2d getHeading(){
    //Returns the gyro heading might need invert if it is not clockwise positive
    return Rotation2d.fromDegrees(pigeon.getAngle()); // add negative on pigeon angle
  }

  public void resetHeading(){
    //resets Gyro heading on field
    //Change to .setYaw(0)???
    pigeon.reset();
  }

  public Pose2d getPose(){
    //returns the odometry pose
    return new Pose2d(
        new Translation2d(odometry.getPoseMeters().getX(), -odometry.getPoseMeters().getY()),
        odometry.getPoseMeters().getRotation()
    );
  }



  public void resetPose(){
    //resets the odometry pose
    resetPose(new Pose2d());
  }

  public void resetPose(Pose2d pose){
    odometry.resetPosition(
        getHeading(),
        new SwerveModulePosition[] {
            frontLeftMod.getModPos(),
            frontRightMod.getModPos(),
            backLeftMod.getModPos(),
            backRightMod.getModPos()
        }, 
        pose);

  }

  //Gets Mods states 
  private ChassisSpeeds getRobotRelativeSpeeds(){
    return Constants.kinematics.toChassisSpeeds(frontLeftMod.getModState(),frontRightMod.getModState(),backLeftMod.getModState(),backRightMod.getModState());
  }

  private void drive(ChassisSpeeds chassisSpeeds){
    drive(chassisSpeeds.vxMetersPerSecond, chassisSpeeds.vyMetersPerSecond, chassisSpeeds.omegaRadiansPerSecond, false);
  }

  public void drive(double xSpeed, double ySpeed, double rotSpeed){
    drive(xSpeed,ySpeed,rotSpeed,false);
  }

  public void drive(double xSpeed, double ySpeed, double rotSpeed, boolean fieldRelative){
    ChassisSpeeds speeds = new ChassisSpeeds(xSpeed, ySpeed, rotSpeed);

    if(fieldRelative){
        //add negative on getHeading if need to invert
        speeds = ChassisSpeeds.fromFieldRelativeSpeeds(xSpeed, ySpeed, rotSpeed, Rotation2d.fromRadians(getHeading().getRadians()));
    }

    SwerveModuleState[] states = Constants.kinematics.toSwerveModuleStates(speeds);

    setModStates(states);
  }

  
  public void setModStates(SwerveModuleState[] states){
    //Reduce Speeds to attainable values
    //Add MaxVelocity by doing math if needed and place into attainable Max speed
    SwerveDriveKinematics.desaturateWheelSpeeds(states, 1);
    
    //sets module states, MAKE SURE THEY ARE IN ORDER CALLED
    frontLeftMod.setModState(states[0]);
    frontRightMod.setModState(states[1]);
    backLeftMod.setModState(states[2]);
    backRightMod.setModState(states[3]);
  }
  public void stopMods(){
    //stops the modules 
    frontLeftMod.stop();
    frontRightMod.stop();
    backLeftMod.stop();
    backRightMod.stop();
  }





  @Override
  public void simulationPeriodic() {
    // This method will be called once per scheduler run during simulation
  }
}
