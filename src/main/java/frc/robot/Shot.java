package frc.robot;

public class Shot 
{
    double shooterRPM;
    double theta;

    public Shot(double rpm, double theta)
    {
        shooterRPM = rpm;
        this.theta = theta;
    }  

    public double GetRPM()
    {
        return shooterRPM;
    }

    public double GetTheta()
    {
        return theta;
    }
}