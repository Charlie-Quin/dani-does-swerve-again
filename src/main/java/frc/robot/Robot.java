// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;

import edu.wpi.first.wpilibj.Joystick;
import edu.wpi.first.wpilibj.TimedRobot;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj.drive.DifferentialDrive;
import edu.wpi.first.wpilibj.motorcontrol.MotorController;
import edu.wpi.first.wpilibj.motorcontrol.PWMSparkMax;
import frc.robot.subsytems.DriveSubsystem;

/**
 * This is a demo program showing the use of the DifferentialDrive class, specifically it contains
 * the code necessary to operate a robot with tank drive.
 */
public class Robot extends TimedRobot {
  private DifferentialDrive m_myRobot;
  private XboxController controller = new XboxController(0);

  //private final CANSparkMax m_leftMotor = new CANSparkMax(5,MotorType.kBrushless);
  //private final CANSparkMax m_rightMotor = new CANSparkMax(6,MotorType.kBrushless);

  DriveSubsystem driveSub;

  @Override
  public void robotInit() {
    // We need to invert one side of the drivetrain so that positive voltages
    // result in both sides moving forward. Depending on how your robot's
    // gearbox is constructed, you might have to invert the left side instead.
    //m_rightMotor.setInverted(true);

    driveSub = new DriveSubsystem();

    //m_myRobot = new DifferentialDrive(m_leftMotor, m_rightMotor);
    
  }

  boolean flag = false;

  @Override
  public void teleopPeriodic() {

    driveSub.fakePeriodic();

    driveSub.drive(controller.getLeftX(),controller.getLeftY(), controller.getRightX());

    if (driveSub.deadBand(controller.getLeftTriggerAxis()) != 0){
      if (!flag){
        driveSub.tryReZero();
        flag = true;
      }
    }
    else{
      flag = false;
    }
  }
}
