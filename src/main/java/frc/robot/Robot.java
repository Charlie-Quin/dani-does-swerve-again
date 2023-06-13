// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import edu.wpi.first.wpilibj.TimedRobot;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj.drive.DifferentialDrive;
import edu.wpi.first.wpilibj2.command.CommandScheduler;
import frc.robot.subsytems.DriveSubsystem;

/**
 * This is a demo program showing the use of the DifferentialDrive class, specifically it contains
 * the code necessary to operate a robot with tank drive.
 */
public class Robot extends TimedRobot {
  

  //private final CANSparkMax m_leftMotor = new CANSparkMax(5,MotorType.kBrushless);
  //private final CANSparkMax m_rightMotor = new CANSparkMax(6,MotorType.kBrushless);

  RobotContainer robotContainer;

  @Override
  public void robotInit() {

    robotContainer = new RobotContainer();
    
    

  }

  public void robotPeriodic(){
    CommandScheduler.getInstance().run();
  }

  boolean flag = false;

  public void teleopInit(){
    
  }

  @Override
  public void teleopPeriodic() {

    //driveSub.fakePeriodic();

    //driveSub.holdRotationDrive(controller.getLeftX(),controller.getLeftY(), controller.getRightX());
    //driveSub.drive(controller.getLeftX(),controller.getLeftY(), controller.getRightX());

    // if (driveSub.deadBand(controller.getLeftTriggerAxis()) != 0){
    //   if (!flag){
    //     driveSub.tryReZero();
    //     flag = true;
    //   }
    // }
    // else{
    //   flag = false;
    // }
  }

}
