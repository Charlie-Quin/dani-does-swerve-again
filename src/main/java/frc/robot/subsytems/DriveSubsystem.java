package frc.robot.subsytems;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.SwerveModule;
import frc.robot.tools.RobotMap;

public class DriveSubsystem{// extends SubsystemBase{

    public final SwerveModule frontLeft = new SwerveModule(
     RobotMap.kFLDriveMotorPort,
     RobotMap.kFLTurnMotorPort,
     RobotMap.kFLEncoderPort,
     RobotMap.kModuleTurningGains,
     RobotMap.kModuleDrivingGains,
     RobotMap.kFLEncoderOffset,
     "frontLeft");

     public final SwerveModule backLeft = new SwerveModule(
     RobotMap.kBLDriveMotorPort,
     
     RobotMap.kBLTurnMotorPort,
     RobotMap.kBLEncoderPort,
     RobotMap.kModuleTurningGains,
     RobotMap.kModuleDrivingGains,
     RobotMap.kBLEncoderOffset,
     "backLeft");

     public final SwerveModule frontRight = new SwerveModule(
     RobotMap.kFRDriveMotorPort,
     RobotMap.kFRTurnMotorPort,
     RobotMap.kFREncoderPort,
     RobotMap.kModuleTurningGains,
     RobotMap.kModuleDrivingGains,
     RobotMap.kFREncoderOffset,
     "frontRight");

     public final SwerveModule backRight = new SwerveModule(
     RobotMap.kBRDriveMotorPort,
     RobotMap.kBRTurnMotorPort,
     RobotMap.kBREncoderPort,
     RobotMap.kModuleTurningGains,
     RobotMap.kModuleDrivingGains,
     RobotMap.kBREncoderOffset,
     "backRight");


     public void fakePeriodic(){
        
      frontLeft.smartDash();
      frontRight.smartDash();
      backLeft.smartDash();
      backRight.smartDash();
      

     }

     public double deadBand(double num){
        return (Math.abs(num) > 0.1 ? num : 0);
     }

     public void drive(double x, double y,double turn){

        x = deadBand(x);
        y = deadBand(y);
        turn = deadBand(turn);

        if (x == 0 && y == 0 && turn == 0){
            idle();
            return;
        }

        if (x == 0 && y == 0 && turn != 0){
            turnInPlace(turn/2);
            return;
        }

        if (turn == 0){
            translate(x, y);
            return;
        }

        translateTurn(x,y,turn);
     }

     public void idle(){
        frontLeft.setSpeed(0);
        backLeft.setSpeed(0);
        frontRight.setSpeed(0);
        backRight.setSpeed(0);
     }

     public void turnInPlace(double power){
        frontLeft.setTargetRotationDegrees(135.0 - 180);
        backLeft.setTargetRotationDegrees(45.0);
        frontRight.setTargetRotationDegrees(-45.0 - 90);
        backRight.setTargetRotationDegrees(-135.0 - 90);

        frontLeft.setSpeed(power);
        backLeft.setSpeed(power);
        frontRight.setSpeed(power);
        backRight.setSpeed(power);
     }

     public void translate(double x , double y){
        double direction = -Math.atan2(x,y);

        double power = Math.sqrt(x * x + y * y)/2;


        frontLeft.setTargetRotation(direction);
        frontRight.setTargetRotation(direction);
        backLeft.setTargetRotation(direction);
        backRight.setTargetRotation(direction);

        frontLeft.setSpeed(power);
        frontRight.setSpeed(power);
        backLeft.setSpeed(power);
        backRight.setSpeed(power);
     }


     public void translateTurn(double x, double y, double turn){

        double direction = Math.toDegrees(-Math.atan2(x,y));
        double turnAngle = turn * 45.0;
        double power = Math.sqrt(x * x + y * y)/2;

        // if the left front wheel is in the front
        if (angleDifferenceDegrees(direction, 135.0) >= 90.0)
        {
            frontLeft.setTargetRotationDegrees(direction + turnAngle);
        }
        // if it's in the back
        else
        {
            frontLeft.setTargetRotationDegrees(direction - turnAngle);
        }
        // if the left back wheel is in the front
        if (angleDifferenceDegrees(direction, 225.0) > 90.0)
        {
            backLeft.setTargetRotationDegrees(direction + turnAngle);
        }
        // if it's in the back
        else
        {
            backLeft.setTargetRotationDegrees(direction - turnAngle);
        }
        // if the right front wheel is in the front
        if (angleDifferenceDegrees(direction, 45.0) > 90.0)
        {
            frontRight.setTargetRotationDegrees(direction + turnAngle);
        }
        // if it's in the back
        else
        {
            frontRight.setTargetRotationDegrees(direction - turnAngle);
        }
        // if the right back wheel is in the front
        if (angleDifferenceDegrees(direction, 315.0) >= 90.0)
        {
            backRight.setTargetRotationDegrees(direction + turnAngle);
        }
        // if it's in the back
        else
        {
            backRight.setTargetRotationDegrees(direction - turnAngle);
        }
    
        frontLeft.setSpeed(power);
        backLeft.setSpeed(power);
        frontRight.setSpeed(power);
        backRight.setSpeed(power);
     }

     public static double angleDifferenceDegrees(double a, double b){
        return Math.abs(Math.toDegrees(differenceBetweenAngles(Math.toRadians(a), Math.toRadians(b))));
     }

     public static double differenceBetweenAngles(double Angle, double Bngle){
        
        Angle = (Angle % (2 * Math.PI) + 2 * Math.PI) % (2 * Math.PI);
        Bngle = (Bngle % (2 * Math.PI) + 2 * Math.PI) % (2 * Math.PI);
        
        double returnVal = Bngle - Angle;
        
        if (returnVal > Math.PI) returnVal -= 2 * Math.PI;
        else if (returnVal < -Math.PI) returnVal += 2 * Math.PI;
        
        if (isEqualApprox(Angle,Bngle,0.01)) returnVal = 0;
        if (isEqualApprox(Angle,Bngle - (2*Math.PI),0.01)) returnVal = 0;
        
        
        return returnVal;
    }

    public static boolean isEqualApprox(double a, double b, double tolerance){
        if (a + tolerance > b && a - tolerance < b) return true;
        return false;
    }

    
}
