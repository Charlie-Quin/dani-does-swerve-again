package frc.robot.subsytems;

import com.ctre.phoenix.sensors.WPI_Pigeon2;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.SwerveModule;
import frc.robot.tools.RobotMap;

public class DriveSubsystem{// extends SubsystemBase{

    private final WPI_Pigeon2 gyro;

    public DriveSubsystem(){
        gyro = new WPI_Pigeon2(RobotMap.kPigeonPort, RobotMap.kDriveCANBusName);
		gyro.setYaw(0);
    }

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

      SmartDashboard.putString("offsets: ", 
      "public static double kFLEncoderOffset = " + frontLeft.directReadAngleDegrees() + ";\n" +
      "public static double kFREncoderOffset = " + frontRight.directReadAngleDegrees() + ";\n" +
      "public static double kBLEncoderOffset = " + backLeft.directReadAngleDegrees() + ";\n" +
      "public static double kBREncoderOffset = " + backRight.directReadAngleDegrees()+ ";" );
      

     }

     public void tryReZero(){
        frontLeft.turnEncoder.setPosition(frontLeft.readAngle());
        frontRight.turnEncoder.setPosition(frontRight.readAngle());
        backLeft.turnEncoder.setPosition(backLeft.readAngle());
        backRight.turnEncoder.setPosition(backRight.readAngle());
     }

     public double deadBand(double num){
        return (Math.abs(num) > 0.1 ? num : 0);
     }

     public void drive(double x, double y,double turn){

        x = deadBand(x);
        y = deadBand(y);
        turn = deadBand(turn);

        double direction = Math.toDegrees(-Math.atan2(x,y)) - gyro.getAngle();
        double power = Math.sqrt(x * x + y * y)/2;

        if (x == 0 && y == 0 && turn == 0){
            idle();
            return;
        }

        if (x == 0 && y == 0 && turn != 0){
            turnInPlace(turn/2);
            return;
        }

        if (turn == 0){
            translate(direction, power);
            return;
        }
        
        

        translateTurn(direction,power,turn);
     }

     public void idle(){
        frontLeft.setSpeed(0);
        backLeft.setSpeed(0);
        frontRight.setSpeed(0);
        backRight.setSpeed(0);
     }

     public void turnInPlace(double power){
        frontLeft.setTargetRotationDegrees(135.0 - 90);
        backLeft.setTargetRotationDegrees(45.0 - 90);
        frontRight.setTargetRotationDegrees(-45.0 );
        backRight.setTargetRotationDegrees(-135.0 );

        power *= -1;

        frontLeft.setSpeed(power);
        backLeft.setSpeed(power);
        frontRight.setSpeed(power);
        backRight.setSpeed(power);
     }

     public void translate(double direction , double power){
        translateTurn(direction, power, 0);
     }


     public void translateTurn(double direction,double power, double turn){

        
        double turnAngle = turn * 45.0;
        
        /* 
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
        }*/

        frontLeft.setTargetRotationDegrees(direction + turnAngle * (angleDifferenceDegrees(direction, 135.0) >= 90.0 ? -1 : 1));
        frontRight.setTargetRotationDegrees(direction + turnAngle * (angleDifferenceDegrees(direction, 45.0) >= 90.0 ? -1 : 1));

        backLeft.setTargetRotationDegrees(direction + turnAngle * (angleDifferenceDegrees(direction, 225.0) >= 90.0 ? -1 : 1));
        backRight.setTargetRotationDegrees(direction + turnAngle * (angleDifferenceDegrees(direction, 315.0) >= 90.0 ? -1 : 1));
    
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
