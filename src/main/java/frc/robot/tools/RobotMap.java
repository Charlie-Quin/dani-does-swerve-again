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

    public static boolean fieldRelative = true;
    
    
    //works "ok"
    //public static double kFLEncoderOffset = 218.14453125;public static double kFREncoderOffset = 297.509765625;public static double kBLEncoderOffset = 327.744140625;public static double kBREncoderOffset = 240.205078125;
    
    //backwards zeroes
    //public static double kFLEncoderOffset = 37.177734375;public static double kFREncoderOffset = 121.201171875;public static double kBLEncoderOffset = 148.271484375;public static double kBREncoderOffset = 59.765625;
    
    public static double kFLEncoderOffset = 216.826171875;public static double kFREncoderOffset = 301.025390625;public static double kBLEncoderOffset = 327.919921875;public static double kBREncoderOffset = 240.029296875;

    public static int kPigeonPort = 20;

    public static final String kDriveCANBusName = "canivore";


}
