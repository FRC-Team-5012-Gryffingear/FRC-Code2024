// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.LimelightHelpers;

public class limey extends SubsystemBase {
  /** Creates a new limey. */
    PIDController Movement = new PIDController(.1, 0, 0);

  public limey() {
    Movement.reset();
  } 

  public double getx() {
    double tx = LimelightHelpers.getTX("");
    SmartDashboard.putNumber("Distance Y ", tx);
    return tx;
  }

  public double estimate3DZ(){
    Pose3d poses = LimelightHelpers.getTargetPose3d_CameraSpace("");
    double distance = poses.getZ();
    double zinOneInch = 39.6153846154;
    double distanceInches = distance * zinOneInch;
    return distanceInches;
  }


  public double gety(){
    double ty = LimelightHelpers.getTY("");
    SmartDashboard.putNumber("Distance Y", ty);
    return ty;
  }
  public double getz(){
    double targetOffsetAngle_Vertical = gety();

    double limelightMountAngleDegrees = 0.0; 

    // distance from the center of the Limelight lens to the floor
    double limelightLensHeightInches = 44.0; 

    // distance from the target to the floor
    double goalHeightInches = 55.0; 

    double angleToGoalDegrees = limelightMountAngleDegrees + targetOffsetAngle_Vertical;
    double angleToGoalRadians = angleToGoalDegrees * (3.14159 / 180.0);

    //calculate distance
    double distanceFromLimelightToGoalInches = (goalHeightInches - limelightLensHeightInches) / Math.tan(angleToGoalRadians);
    SmartDashboard.putNumber("Distance Z Estimated", distanceFromLimelightToGoalInches);

    return distanceFromLimelightToGoalInches;
  }

  // Asks for the getX() value in order to calculate the necessary value to move
  // 
  public double rotationLock(double x_value){ 
    double rot = Movement.calculate(x_value, 0); // setpoint should be changed after testing 
    
    // This is to check if the rotational movement is worth moving
    // Also to prevent microadjustment which would cause clicking 
    // in the motors
    if(Math.abs(rot) < 0.15){
      rot = 0; // setting the rotation value to 0 to prevent unecessary rotation
    }
    return rot;
  }

  public double fwrdLock(double z_value){
    double z_movement = Movement.calculate(z_value,5);
    return z_movement;
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
