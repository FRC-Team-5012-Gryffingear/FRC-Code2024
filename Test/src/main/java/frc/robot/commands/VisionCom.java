package frc.robot.commands;

import frc.robot.Robot;
import frc.robot.subsystems.VisionSub;

import java.util.function.BooleanSupplier;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;

/** An example command that uses an example subsystem. */
public class VisionCom extends Command {
  @SuppressWarnings({"PMD.UnusedPrivateField", "PMD.SingularField"})
  private final VisionSub subsys;
  
  // private final Robot j;
  

  /**
   * Creates a new VisionCom.
   *
   * @param subsystem The subsystem used by this command.
   */
  public VisionCom(VisionSub subsystem) {
    subsys = subsystem;
    // Use addRequirements() here to declare subsystem dependencies.
    addRequirements(subsystem);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {}

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    subsys.apriltagVisionThreadProc();
   // SmartDashboard.putNumber("Getting the X value: ", subsys.getXs());
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {}

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return false;
  }
}
