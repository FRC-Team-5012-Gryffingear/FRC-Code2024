// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.SwerveDriveKinematics;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.units.Unit;

/**
 * The Constants class provides a convenient place for teams to hold robot-wide numerical or boolean
 * constants. This class should not be used for any other purpose. All constants should be declared
 * globally (i.e. public static). Do not put anything functional in this class.
 *
 * <p>It is advised to statically import this class (or one of its inner classes) wherever the
 * constants are needed, to reduce verbosity.
 */
public final class Constants {
//Trackwidth is the distance between front 2 modules
//wheebase is the distance between the front to the back Modules
  public static final double trackWidth = Units.inchesToMeters(16.366);
  public static final double wheelBase = Units.inchesToMeters(22.580);

// Max velo using RPM/Gear Ratio * pi * 4(diameter)/12(feet) * 60(min/seconds?)
  public static final double Max_velo = 6380 / 6.75 * Math.PI * 4 / 12 / 60;
  public static final int Max_Voltage = 12;
  public static final double Max_Angular = Max_velo/ Math.hypot(trackWidth/2, wheelBase/2);

  //IDS FOR MOTORS AND ENCODERS
  public static final int FrontLeftDriveID = 1;
  public static final int FrontRightDriveID = 4;
  public static final int BackLeftDriveID = 7;
  public static final int BackRightDriveID = 10;

  public static final int FrontLeftSteerID = 2;
  public static final int FrontRightSteerID = 5;
  public static final int BackLeftSteerID = 8;
  public static final int BackRightSteerID = 11;

  public static final int FrontLeftEncoderID = 3;
  public static final int FrontRightEncoderID = 6;
  public static final int BackLeftEncoderID = 9;
  public static final int BackRightEncoderID = 12;

//One of these encoders needs to be changed due to new encoder
  public static final double FrontLeftOffset = -0.771728515625;
  public static final double FrontRightOffset = -0.003173828125;
  public static final double BackLeftOffset = -0.408203125;
  public static final double BackRightOffset = -0.017822265625;

  // if true then inverse drive motors
  public static final boolean FrontLeftInv = false;
  public static final boolean FrontRightInv = false;
  public static final boolean BackLeftInv = false;
  public static final boolean BackRightInv = false;

  public static final int PigeonID = 0;

  // gives position of modules on a 2d plane
  public static final SwerveDriveKinematics kinematics = new SwerveDriveKinematics(
    new Translation2d(wheelBase/2, -trackWidth/2), // Front Right
    new Translation2d(wheelBase/2, trackWidth/2),  // FrontLeft
    new Translation2d(-wheelBase/2, -trackWidth/2), // Back Right
    new Translation2d(-wheelBase/2, trackWidth/2) // Rear Left
  );

  
  public static final int Intake1 = 13;
  public static final int Elev1 = 14;
  public static final int Elev2 = 15;
    
   

  public static class ModConstants{
    // same Kp values for all modules, Might need to adjust
    public static final double KP = 0.4;
    public static final double KI = 0;
    public static final double KD = 0;
  }

  public static class OperatorConstants {
    public static final int DriverControllerPort = 0;
    public static final double DriverDeadband = 0.05;

    public static final int OperatorControllerPort = 1;
  }
}
