package frc.robot;

//import com.revrobotics.AnalogInput;
import com.revrobotics.CANSparkMax;
import com.revrobotics.RelativeEncoder;
import com.revrobotics.CANSparkMax.ControlType;
import com.revrobotics.CANSparkMax.IdleMode;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;
import com.revrobotics.SparkMaxPIDController;

import edu.wpi.first.wpilibj.AnalogInput;
import edu.wpi.first.wpilibj.RobotController;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import frc.robot.tools.PIDValues;
import frc.robot.tools.RobotMap;

import com.ctre.phoenix.sensors.AbsoluteSensorRange;
import com.ctre.phoenix.sensors.CANCoder;
import com.ctre.phoenix.sensors.CANCoderStatusFrame;
import com.ctre.phoenix.sensors.SensorInitializationStrategy;

public class SwerveModule {
    
    private final CANSparkMax driveMotor;
    private final CANSparkMax turnMotor;

    public final RelativeEncoder turnEncoder;
    private final RelativeEncoder driveEncoder;

    private final CANCoder absoluteEncoder;

   

    public double startingRotation;

    public String name = "";

    SparkMaxPIDController driveController;
    SparkMaxPIDController turnController;
    private double encoderOffset;

    boolean zeroed = false;

    boolean backWards = false;

    public SwerveModule(int drivePort,int turnPort,int encoderPort,PIDValues turnGains, PIDValues driveGains,double encoderOffset, String name){
        driveMotor = new CANSparkMax( drivePort,MotorType.kBrushless);
        turnMotor = new CANSparkMax(turnPort,MotorType.kBrushless);

        absoluteEncoder = new CANCoder(encoderPort, RobotMap.kDriveCANBusName);
		absoluteEncoder.configFactoryDefault();
		absoluteEncoder.configSensorInitializationStrategy(SensorInitializationStrategy.BootToAbsolutePosition);
		absoluteEncoder.configAbsoluteSensorRange(AbsoluteSensorRange.Unsigned_0_to_360);
		absoluteEncoder.setStatusFramePeriod(CANCoderStatusFrame.SensorData, 10, 100);
		absoluteEncoder.clearStickyFaults();
        
        driveMotor.restoreFactoryDefaults();
        turnMotor.restoreFactoryDefaults();

        driveMotor.setIdleMode(IdleMode.kBrake);
        turnMotor.setIdleMode(IdleMode.kCoast);
        
        this.encoderOffset = encoderOffset;
        startingRotation = readAngle();
        
        turnEncoder = turnMotor.getEncoder();
        
        turnEncoder.setPositionConversionFactor((2*Math.PI)/(150d/7d));
        turnEncoder.setPosition(startingRotation);

        driveEncoder = driveMotor.getEncoder();
        driveEncoder.setPosition(0);

        driveController = driveMotor.getPIDController();
        
        driveController.setP(driveGains.kP);
        driveController.setI(driveGains.kI);
        driveController.setD(driveGains.kD);

        turnController = turnMotor.getPIDController();
        
        turnController.setP(turnGains.kP);
        turnController.setI(turnGains.kI);
        turnController.setD(turnGains.kD);

        

        this.name = name;


        

    } 


    public void setTargetRotationDegrees(double degrees){
        setTargetRotation(Math.toRadians(degrees));
    }

    //rotation in radians
    public void setTargetRotation(double rotation){
        double realRotation = turnEncoder.getPosition();

        if (Math.abs(differenceBetweenAngles(rotation, realRotation)) > Math.PI/2){
            rotation += Math.PI;
            backWards = true;
        }
        else{
            backWards = false;
        }

        double newRotation = realRotation  - (differenceBetweenAngles(rotation, realRotation) );

        turnController.setReference(newRotation, ControlType.kPosition);

        

    }

    public void setSpeed(double speed){
        driveMotor.set(speed * (backWards?-1 : 1));
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

    public double readAngle() {
        
        double angle = absoluteEncoder.getAbsolutePosition();

        angle -= encoderOffset;

        angle = Math.toRadians(angle);
        
        if (angle < 0) angle += 2 * Math.PI;

        angle = 2* Math.PI - angle;
        



        return angle;
    }

    public double directReadAngleDegrees(){
        return absoluteEncoder.getAbsolutePosition();
    }

    public void smartDash(){
        SmartDashboard.putNumber(name + " motor encoder", Math.toDegrees(turnEncoder.getPosition()%(2*Math.PI)));
        SmartDashboard.putNumber(name + " encoder", Math.toDegrees(readAngle()));
        //SmartDashboard.putString("errors: ", absoluteEncoder.configFactoryDefault().name());

    }


}
