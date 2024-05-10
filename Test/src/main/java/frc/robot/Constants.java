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

  public final static int talon1 = 13; //FL
  public final static int talon2 = 7; //FR
  public final static int talon3 = 12; //BL
  public final static int talon4 = 0; // BR


  public final static int drivercontroller = 0;

  public final static int pigeonport = 1;


  //shooter SS for prototype
   public final static int talonS1 = 2;
   public final static int talonS2 = 4;


  public static class OperatorConstants {
    public static final int kDriverControllerPort = 0;

  }
}
