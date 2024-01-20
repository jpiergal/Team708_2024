package frc.robot.commands.drive;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.wpilibj2.command.PIDCommand;

import frc.robot.subsystems.drive.Drivetrain;

/** A command that will turn the robot to the specified angle. */
public class TurnToCommand extends PIDCommand {
  /**
   * Turns to robot to the specified angle.
   *
   * @param targetAngleDegrees The angle to turn to
   * @param drive The drive subsystem sto use
   */
  

  public TurnToCommand(double targetAngleDegrees, Drivetrain drive) {
    super(
      new PIDController(0.04, 0.01, 0),
      drive::getGyroDegrees,
      targetAngleDegrees,
      output -> drive.drive(0, 0, output, true, true),
      drive);

      getController().enableContinuousInput(-180, 180);
    // getController().setTolerance(1.5, 10);
  }

  @Override
  public boolean isFinished() {
    return getController().atSetpoint();
  }
}
