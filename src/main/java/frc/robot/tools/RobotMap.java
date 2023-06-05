package frc.robot.tools;

public class RobotMap {
    
    public static final int kFLDriveMotorPort = 1;
    public static final int kFLTurnMotorPort = 2;
    public static final int kFLEncoderPort = 0;

    public static final int kBLDriveMotorPort = 1000; //it's broken. 5;
    public static final int kBLTurnMotorPort = 6;
    public static final int kBLEncoderPort = 2;

    public static final int kFRDriveMotorPort = 3;
    public static final int kFRTurnMotorPort = 4;
    public static final int kFREncoderPort = 1;

    public static final int kBRDriveMotorPort = 7;
    public static final int kBRTurnMotorPort = 8;
    public static final int kBREncoderPort = 3;

    public static final PIDValues kModuleTurningGains = new PIDValues(1, 0.0002, 0);
    public static final PIDValues kModuleDrivingGains = new PIDValues(0.1, 0, 0);
    
    public static double kFLEncoderOffset = 2.089131847263335; // 2.130102227645993;
    public static double kFREncoderOffset = 4.157538146012147;

    public static double kBREncoderOffset = 3.489212401016271; // good
    public static double kBLEncoderOffset = 3.298719820120973;//2.475036426798523;this was working //4.815034538658552 - 0.198451166256656/2;



}
