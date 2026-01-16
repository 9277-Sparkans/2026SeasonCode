package frc.robot;

public class Constants 
{
    public static class IntakeConstants
    {
        // deployment
        public static final int deploymentID = 0;

        public static final int deploymentKP = 10;
        public static final int deploymentKI = 10;
        public static final int deploymentKD = 10;

        public static final double deploymentMaxVoltage = 4;
        public static final double deploymentMaxAcceleration = 40;
        public static final double deploymentMaxVelocity = 60;

        public static final int deploymentCountsPerRevolution = 2048; // for kraken x60
        public static final int deploymentGearRatio = 1 / (1 / 1);

        public static final int deploymentMaxDeg = 115;
    
        // rollers
        public static final int rollerID = 1;

        public static final int rollerKP = 10;
        public static final int rollerKI = 10;
        public static final int rollerKD = 10;

        public static final double rollerMaxVoltage = 4;
        public static final double rollerMaxAcceleration = 40;
        public static final double rollerMaxVelocity = 60;
    }    
}