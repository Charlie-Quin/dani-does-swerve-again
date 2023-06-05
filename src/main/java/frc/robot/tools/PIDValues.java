package frc.robot.tools;

public class PIDValues {
    
    public double kP;
    public double kI;
    public double kD;
    public double kFF;
    public double kIz;

    public PIDValues(double p, double i, double d){
        this.kP = p;
        this.kI = i;
        this.kD = d;
        kFF = 0;
        kIz = 0;
    }

    public PIDValues(double p, double i, double d,double ff, double iz){
        this.kP = p;
        this.kI = i;
        this.kD = d;
        kFF = ff;
        kIz = iz;
    }


}
