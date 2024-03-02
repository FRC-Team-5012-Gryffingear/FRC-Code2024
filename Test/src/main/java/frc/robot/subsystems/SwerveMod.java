package frc.robot.subsystems;

import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.InvertType;
import com.ctre.phoenix.motorcontrol.NeutralMode;
import com.ctre.phoenix6.configs.CANcoderConfiguration;
import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.controls.ControlRequest;
import com.ctre.phoenix6.hardware.CANcoder;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.mechanisms.swerve.SwerveModule;
import com.ctre.phoenix6.signals.AbsoluteSensorRangeValue;
import com.ctre.phoenix6.signals.NeutralModeValue;
import com.ctre.phoenix6.signals.SensorDirectionValue;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.filter.SlewRateLimiter;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.SwerveModulePosition;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import edu.wpi.first.wpilibj.RobotBase;
import edu.wpi.first.wpilibj.motorcontrol.Talon;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import frc.robot.Constants;
import frc.robot.Constants.ModConstants;

public class SwerveMod {
    private final TalonFX DriveMotor, SteerMotor;
    private final CANcoder SteerEncoder;

    private final PIDController pidcontrl = new PIDController(ModConstants.KP, ModConstants.KI, ModConstants.KD);

    private final SlewRateLimiter DriveLim = new SlewRateLimiter(1);

    private final PIDController drivecont = new PIDController(ModConstants.KP, 0, 0);
    private final String Modname;

    private SwerveModuleState lastDesiredState = new SwerveModuleState();



    //Creates a new Swerve Mod using talons and cancoders
    public SwerveMod(int DriveID, int SteerID, int encoderID, double offset, boolean DriveInvr, String moduleName){
        DriveMotor = new TalonFX(DriveID);
        SteerMotor = new TalonFX(SteerID);
        SteerEncoder = new CANcoder(encoderID);

        DriveMotor.getConfigurator().apply(new TalonFXConfiguration());
        DriveMotor.setInverted(DriveInvr);

        SteerMotor.getConfigurator().apply(new TalonFXConfiguration());

        DriveMotor.setNeutralMode(NeutralModeValue.Brake);             
        SteerMotor.setNeutralMode(NeutralModeValue.Brake);  
 
        CANcoderConfiguration config = new CANcoderConfiguration();
        config.MagnetSensor.MagnetOffset = offset;
        //Config to full 360 if problems set back to signed half
        config.MagnetSensor.AbsoluteSensorRange = AbsoluteSensorRangeValue.Unsigned_0To1;
        config.MagnetSensor.SensorDirection = SensorDirectionValue.Clockwise_Positive;

        SteerEncoder.getConfigurator().apply(config);

        pidcontrl.enableContinuousInput(-Math.PI, Math.PI);
        
        pidcontrl.reset();
        drivecont.reset();

        Modname = moduleName;

    }

    public void Info(){
        if(RobotBase.isReal()){
            //Gets the Rotation of the steer as well as the power from the Drive motor
            SmartDashboard.putNumber(Modname + " Turn Rotation", getRotation().getDegrees());
            SmartDashboard.putNumber(Modname + " Drive Speed", DriveMotor.get());
        } else {
            SmartDashboard.putNumber(Modname + " Turn Rotation", lastDesiredState.angle.getDegrees());
            SmartDashboard.putNumber(Modname + " Drive Speed", lastDesiredState.speedMetersPerSecond);
        }
    }

    public Rotation2d getRotation(){
        return Rotation2d.fromRotations(SteerEncoder.getAbsolutePosition().getValueAsDouble());
    }
    
    public double getDrivePos(){
        return DriveMotor.getPosition().getValueAsDouble();
    }

    public double getDriveVelo(){
        return DriveMotor.getVelocity().getValueAsDouble();
    }

    public SwerveModuleState getModState(){
        return new SwerveModuleState(getDriveVelo(),getRotation());
    }

    public SwerveModulePosition getModPos(){
        return new SwerveModulePosition(getDrivePos(),getRotation());
    }

    public void setModState(SwerveModuleState desiredState){
         lastDesiredState = desiredState;

        if(Math.abs(lastDesiredState.speedMetersPerSecond) < 0.05){
            stop();
            return;
        }
        SwerveModuleState optimizedState = SwerveModuleState.optimize(lastDesiredState, getRotation());

        DriveMotor.set(drivecont.calculate(optimizedState.speedMetersPerSecond * 8, lastDesiredState.speedMetersPerSecond));

        SteerMotor.set(pidcontrl.calculate(getRotation().getRadians(), optimizedState.angle.getRadians()));

    }

    public void stop(){
        DriveMotor.set(0);
        SteerMotor.set(0);
    }



}
