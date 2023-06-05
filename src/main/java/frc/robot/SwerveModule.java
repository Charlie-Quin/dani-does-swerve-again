package frc.robot;

//import com.revrobotics.AnalogInput;
import com.revrobotics.CANSparkMax;
import com.revrobotics.RelativeEncoder;
import com.revrobotics.CANSparkMax.ControlType;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;
import com.revrobotics.SparkMaxPIDController;

import edu.wpi.first.wpilibj.AnalogInput;
import edu.wpi.first.wpilibj.RobotController;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import frc.robot.tools.PIDValues;

public class SwerveModule {
    
    private final CANSparkMax driveMotor;
    private final CANSparkMax turnMotor;

    final RelativeEncoder turnEncoder;
    private final RelativeEncoder driveEncoder;

    private final AnalogInput absoluteEncoder;

   

    public double startingRotation;

    public String name = "";

    SparkMaxPIDController driveController;
    SparkMaxPIDController turnController;
    private double encoderOffset;

    boolean zeroed = false;

    boolean backWards = false;

    public SwerveModule(int drivePort,int turnPort,int encoderPort,PIDValues turnGains, PIDValues driveGains,double encoderOffset, String name){
        driveMotor = new CANSparkMax(drivePort,MotorType.kBrushless);
        turnMotor = new CANSparkMax(turnPort,MotorType.kBrushless);
        absoluteEncoder = new AnalogInput(encoderPort);
        
        driveMotor.restoreFactoryDefaults();
        turnMotor.restoreFactoryDefaults();
        
        this.encoderOffset = encoderOffset;
        startingRotation = readAngle();
        
        turnEncoder = turnMotor.getEncoder();
        turnEncoder.setPosition(startingRotation);
        turnEncoder.setPositionConversionFactor((2*Math.PI)/18);

        driveEncoder = driveMotor.getEncoder();
        driveEncoder.setPosition(0);
        driveEncoder.setPositionConversionFactor((2*Math.PI)/18);

        driveController = driveMotor.getPIDController();
        
        driveController.setP(driveGains.kP);
        driveController.setI(driveGains.kI);
        driveController.setD(driveGains.kD);

        turnController = turnMotor.getPIDController();
        
        turnController.setP(turnGains.kP);
        turnController.setI(turnGains.kI);
        turnController.setD(turnGains.kD);

        

        this.name = name;


        // if (!zeroed && Math.abs(readAngle()) < 0.05 ){
        //     turnEncoder.setPosition(Math.PI/2);
        //     zeroed = true;
        // } 

        
        turnEncoder.setPosition(readAngle() - encoderOffset);// + Math.PI/2);
            

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
        double angle = (1.0 - absoluteEncoder.getVoltage() / RobotController.getVoltage5V()) * 2.0 * Math.PI;
        angle %= 2.0 * Math.PI;
        if (angle < 0.0) {
            angle += 2.0 * Math.PI;
        }

        return angle;
    }

    public void smartDash(){
        SmartDashboard.putNumber(name + " real rot", turnEncoder.getPosition());
        SmartDashboard.putNumber(name + " encoder", Math.toDegrees(readAngle()));

    }


}
