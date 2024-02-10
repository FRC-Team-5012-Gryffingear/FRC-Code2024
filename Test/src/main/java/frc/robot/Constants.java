// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

/**
 * The Constants class provides a convenient place for teams to hold robot-wide numerical or boolean
 * constants. This class should not be used for any other purpose. All constants should be declared
 * globally (i.e. public static). Do not put anything functional in this class.
 *
 * <p>It is advised to statically import this class (or one of its inner classes) wherever the
 * constants are needed, to reduce verbosity.
 */
public final class Constants {
//for the elev
  public static final int talon1 = 0;  //inverted(?)
  public static final int talon2 = 0;
//for the intake
  public static final int talon3 = 0; //inverted
  public static final int talon4 = 0;

  public static class OperatorConstants {
    public static final int kDriverControllerPort = 0;
  }
}
