package frc.robot.tools;

public class RobotMap {
    
    public static final int kFLDriveMotorPort = 1;
    public static final int kFLTurnMotorPort = 5;
    public static final int kFLEncoderPort = 9;

    public static final int kBLDriveMotorPort = 3; 
    public static final int kBLTurnMotorPort = 7; 
    public static final int kBLEncoderPort = 11; 

    public static final int kFRDriveMotorPort = 2;
    public static final int kFRTurnMotorPort = 6;
    public static final int kFREncoderPort = 10;

    public static final int kBRDriveMotorPort = 4;
    public static final int kBRTurnMotorPort = 8;
    public static final int kBREncoderPort = 12;

    public static final PIDValues kModuleTurningGains = new PIDValues(1, 0.0002, 0);
    public static final PIDValues kModuleDrivingGains = new PIDValues(0.1, 0, 0);
    
    // public static double kFLEncoderOffset = -2.089131832122803; // 2.130102227645993;
    // public static double kFREncoderOffset = -2.048314571380615;

    // public static double kBREncoderOffset = -2.545814275741577; // good
    // public static double kBLEncoderOffset = -3.275344610214233;//2.475036426798523;this was working //4.815034538658552 - 0.198451166256656/2;

    public static double kFLEncoderOffset = 218.14453125;public static double kFREncoderOffset = 297.509765625;public static double kBLEncoderOffset = 327.744140625;public static double kBREncoderOffset = 240.205078125;
    public static int kPigeonPort = 20;

    public static final String kDriveCANBusName = "canivore";


}
