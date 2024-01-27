package frc.robot.subsystems;

import com.ctre.phoenix6.CANBus;
import com.ctre.phoenix6.configs.Slot0Configs;
import com.ctre.phoenix6.mechanisms.swerve.SwerveDrivetrain;
import com.ctre.phoenix6.mechanisms.swerve.SwerveDrivetrainConstants;
import com.ctre.phoenix6.mechanisms.swerve.SwerveModule;
import com.ctre.phoenix6.mechanisms.swerve.SwerveModuleConstants;
import frc.robot.CommandSwerveDrivetrain;
import com.ctre.phoenix6.mechanisms.swerve.SwerveModuleConstantsFactory;
import com.ctre.phoenix6.mechanisms.swerve.SwerveModule.ClosedLoopOutputType;
import com.ctre.phoenix6.mechanisms.swerve.SwerveModuleConstants.SteerFeedbackType;

import edu.wpi.first.math.trajectory.constraint.MaxVelocityConstraint;
import edu.wpi.first.math.util.Units;

public class SwerveMod {


    private static final String CANbusName = "rio";
/*
 * Might need to Change the Gains, IDs, and offset
 * change track and wheel and Gearatios
 */
private static final double trackWidth = 16;
private static final double wheelbase = 22.727;

    private static final double DriveMotorGR = 6.75;
    private static final double SteerMotorGR = 150/7;

    private static final Slot0Configs SteerGain = new Slot0Configs()
    .withKP(100).withKI(0).withKD(0.2)
    .withKA(0).withKV(1.5).withKS(0);

    private static final Slot0Configs DriveGain = new Slot0Configs()
    .withKP(3)
    .withKI(0)
    .withKD(0)
    .withKA(0)
    .withKS(0)
    .withKV(0);

    private static final double WheelRad = 2;

    // public static final double MaxSpeed = (6380/6.75* Math.PI * 4/12 / 60);
    // public static final double MaxAngularSpeed = MaxSpeed/Math.hypot(trackWidth/2, wheelbase/2);
    public static final double SpeedAt12Volts = 4.4;

    public static final double DriveMaxSpeed = 6; // 6 meters per second desired top speed
    public static final double SteerMaxRate = 1.5; // 3/4 of a rotation per second max velocity
    public static final double MaxAcceleration = 3;
    public static final double MaxAngularAccel = 4;

    private static final ClosedLoopOutputType SteerLoopOutput = ClosedLoopOutputType.Voltage;

    private static final ClosedLoopOutputType DriveLoopOutput = ClosedLoopOutputType.Voltage;

   


    public static final SwerveDrivetrainConstants DriveConstant = new SwerveDrivetrainConstants()
    .withPigeon2Id(0).withCANbusName(CANbusName);

// SwerveDriveConstants factory = Settings

public static final SwerveModuleConstantsFactory factorySettings = new SwerveModuleConstantsFactory()
.withDriveMotorGearRatio(DriveMotorGR)
.withSteerMotorGearRatio(SteerMotorGR)
.withWheelRadius(2)
.withDriveMotorGains(DriveGain)
.withSteerMotorGains(SteerGain)
.withDriveMotorClosedLoopOutput(DriveLoopOutput)
.withSteerMotorClosedLoopOutput(SteerLoopOutput)
.withSpeedAt12VoltsMps(SpeedAt12Volts)
.withFeedbackSource(SteerFeedbackType.RemoteCANcoder)
.withSteerMotorInverted(true);     


// Change the IDs of the motors and offset
private static final int FRightDriveID = 4;
private static final int FRightSteerID = 5;
private static final int FRightSteerEncoderID = 6;
private static final double FRightSteerOffset = 0;

private static final int FLeftDriveID = 1;
private static final int FLeftSteerID = 2;
private static final int FLeftSteerEncoder = 3;
private static final double FLeftSteerOffset = -0.245;

private static final int BRightDriveID = 10;
private static final int BRightSteerId = 11;
private static final int BRightSteerEncoder = 12;
private static final double BRightSteerOffset = 0;

private static final int BLeftDriveID = 7;
private static final int BLeftSteerID = 8;
private static final int BLeftSteerEncoderID = 9;
private static final double BLeftSteerOffset = -0.226;




private static final SwerveModuleConstants FrontRight = factorySettings.createModuleConstants(
    FRightSteerID
    ,FRightDriveID
    ,FRightSteerEncoderID
    ,FRightSteerOffset
    ,Units.inchesToMeters(trackWidth/2)
    ,Units.inchesToMeters(-wheelbase/2)
    ,true);
  
private static final SwerveModuleConstants FrontLeft = factorySettings.createModuleConstants(
    FLeftSteerID,
    FLeftDriveID,
    FLeftSteerEncoder,
    FLeftSteerOffset,
    Units.inchesToMeters(trackWidth/2),
    Units.inchesToMeters(wheelbase/2),
    false
);

private static final SwerveModuleConstants BackLeft = factorySettings.createModuleConstants(
    BLeftSteerID,
    BLeftDriveID,
    BLeftSteerEncoderID,
    BLeftSteerOffset,
    Units.inchesToMeters(-trackWidth/2),
    Units.inchesToMeters(wheelbase/2),
    false
);

private static final SwerveModuleConstants BackRight = factorySettings.createModuleConstants(
    BRightSteerId
    ,BRightDriveID
    ,BRightSteerEncoder
    ,BRightSteerOffset
    ,Units.inchesToMeters(-trackWidth/2)
    ,Units.inchesToMeters(-wheelbase/2)
    ,true);

public static final CommandSwerveDrivetrain train = new CommandSwerveDrivetrain(DriveConstant
, FrontLeft,
FrontRight,
BackLeft,
BackRight);





}
