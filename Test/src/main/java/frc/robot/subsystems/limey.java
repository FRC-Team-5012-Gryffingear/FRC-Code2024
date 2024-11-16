// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.fasterxml.jackson.databind.ser.std.StdKeySerializers.Default;
import com.revrobotics.CANSparkMax;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj.DigitalOutput;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.LimelightHelpers;
import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.networktables.NetworkTable;
import edu.wpi.first.networktables.NetworkTableEntry;
import edu.wpi.first.networktables.NetworkTableInstance;

public class limey extends SubsystemBase {
  /** Creates a new limey. */

  public limey() {

  }

  public double getX(){
    return LimelightHelpers.getTX("");
  }

  public double estimate3DZ(){
    Pose3d poses = LimelightHelpers.getTargetPose3d_CameraSpace("");
    double distance = poses.getZ();
    double zinOneInch = 39.6153846154;
    double distanceInches = distance * zinOneInch;
    return distanceInches;
  }
  
  public double getTZ(){
    Pose3d poses = LimelightHelpers.getTargetPose3d_CameraSpace("");
    return poses.getZ();
  }

  public double getEstZ(){
    double ty = LimelightHelpers.getTY("");

    double limelightMountAngleDeg = 0;
    double limelightMountHeight = 44;

    double goalHeightInches = 50.5;

    double anleToGoalDegrees = limelightMountAngleDeg + ty;
    double angletoGoalRad = anleToGoalDegrees * (Math.PI / 180);

    double distanceFromTag = (goalHeightInches - limelightMountHeight) / Math.tan(angletoGoalRad);
    
    return distanceFromTag;
  }

  public double getErrorValue(){
    double April1TrueDistance = 91;
    double April2TrueDistance = 97;
    double April3TrueDistance = 101;

    double id = getId();
    if(id == 1){
      return Math.abs(April1TrueDistance - getEstZ());
    } else if(id == 2){
      return Math.abs(April2TrueDistance - getEstZ());
    } else if(id == 3){
      return Math.abs(April3TrueDistance - getEstZ());
    } else{
      return 0;
    }

  }

  public double getId(){
    return NetworkTableInstance.getDefault().getTable("limelight").getEntry("tid").getDouble(0.0);
  }

  @Override
  public void periodic() { 
    double tx = LimelightHelpers.getTX("");
    SmartDashboard.putNumber("X Value", tx);

  }

  @Override
  public void simulationPeriodic() {
    // This method will be called once per scheduler run during simulation
  }
}
