package frc.robot.commands;

import java.util.function.DoubleSupplier;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsytems.DriveSubsystem;

/**
 * A command to drive the robot with joystick input (passed in as {@link DoubleSupplier}s). Written
 * explicitly for pedagogical purposes - actual code should inline a command this simple with {@link
 * edu.wpi.first.wpilibj2.command.RunCommand}.
 */
public class teleopDrive extends CommandBase {
  private final DriveSubsystem driveSubsystem;
  private final DoubleSupplier translateX;
  private final DoubleSupplier translateY;
  private final DoubleSupplier turn;

  /**
   * Creates a new DefaultDrive.
   *
   * @param subsystem The drive subsystem this command wil run on.
   * @param forward The control input for driving forwards/backwards
   * @param rotation The control input for turning
   */
  public teleopDrive(
    DriveSubsystem subsystem, 
    DoubleSupplier strafe, 
    DoubleSupplier forward, 
    DoubleSupplier rotation) {

    driveSubsystem = subsystem;
    translateX = strafe;
    translateY = forward;
    turn = rotation;
    addRequirements(driveSubsystem);

    SmartDashboard.putBoolean("this constructor runs", true);
  }

  @Override
  public void execute() {
    SmartDashboard.putBoolean("this runs", true);
    SmartDashboard.putNumber("translateX", translateX.getAsDouble());

    driveSubsystem.holdRotationDrive( translateX.getAsDouble(), translateY.getAsDouble(),turn.getAsDouble());
  }

}