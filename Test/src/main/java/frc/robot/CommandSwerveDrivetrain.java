package frc.robot;

import java.util.function.Supplier;

import com.ctre.phoenix6.mechanisms.swerve.SwerveDrivetrain;
import com.ctre.phoenix6.mechanisms.swerve.SwerveDrivetrainConstants;
import com.ctre.phoenix6.mechanisms.swerve.SwerveModuleConstants;
import com.ctre.phoenix6.mechanisms.swerve.SwerveRequest;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Subsystem;

public class CommandSwerveDrivetrain extends SwerveDrivetrain implements Subsystem {
    

    public CommandSwerveDrivetrain(SwerveDrivetrainConstants Driveconstants,double OdometryFrequency, SwerveModuleConstants ...modules){
        super(Driveconstants,OdometryFrequency, modules);
    }
    public CommandSwerveDrivetrain(SwerveDrivetrainConstants Driveconstants, SwerveModuleConstants ... modules){
        super(Driveconstants, modules);
    }
    
    public Command applyRequest(Supplier<SwerveRequest> requestSupplier){
        return run(() -> this.setControl(requestSupplier.get()));
    }
}
