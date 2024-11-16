// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.LimelightHelpers;

public class limey extends SubsystemBase {
  /** Creates a new limey. */
  public limey() {} 

  public double getx() {
    double tx = LimelightHelpers.getTX("");
    SmartDashboard.putNumber("Distance Y ", tx);
    return tx;
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


  
  @Override
  public void periodic() {
    // This method will be called once per scheduler run
  }

  @Override
  public void simulationPeriodic() {
    // This method will be called once per scheduler run during simulation
  }
}
