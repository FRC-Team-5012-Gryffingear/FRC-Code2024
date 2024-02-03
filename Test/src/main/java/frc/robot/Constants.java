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


  public static final double Max_velo = 6380 / 6.75 * Math.PI * 4 / 12 / 60;
  public static final int Max_Voltage = 12;
  public static final double Max_Angular = Max_velo/ Math.hypot(trackWidth/2, wheelBase/2);

  //CHANGE IDS FOR ALL MOTORS AND ENCODERS
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
//-0.4833984375 FL
//FR:  0.252685536875
//BL: -0.153564453125
//BR: 0.273681640625
  public static final double FrontLeftOffset = -0.0263671875;
  public static final double FrontRightOffset = -0.760498046875;
  public static final double BackLeftOffset = -0.658203125;
  public static final double BackRightOffset = -0.7802734375;

  public static final boolean FrontLeftInv = true;
  public static final boolean FrontRightInv = false;
  public static final boolean BackLeftInv = true;
  public static final boolean BackRightInv = false;

  public static final int PigeonID = 0;

  public static final SwerveDriveKinematics kinematics = new SwerveDriveKinematics(
    new Translation2d(wheelBase/2, -trackWidth/2), // Front Right
    new Translation2d(wheelBase/2, trackWidth/2),  // FrontLeft
    new Translation2d(-wheelBase/2, -trackWidth/2), // Back Right
    new Translation2d(-wheelBase/2, trackWidth/2) // Rear Left
  );

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
