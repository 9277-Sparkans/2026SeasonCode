package frc.robot;

public class Constants {
    public static class ShooterConstants {
        public static final int kHoodMotorId = 30;
        public static final int kShooterMotorId = 31;
    }

    public static class IntakeConstants
    {
        // deployment
        public static final int deploymentID = 0;

        public static final double deploymentKS = 0.01;
        public static final double deploymentKP = 10;
        public static final double deploymentKI = 0;
        public static final double deploymentKD = 0.1;

        public static final double deploymentMaxVoltage = 4;
        public static final double deploymentMaxAcceleration = 40;
        public static final double deploymentMaxVelocity = 100; // rps

        public static final int deploymentCountsPerRevolution = 2048; // for kraken x60
        public static final int deploymentGearRatio = 1 / (1 / 1);

        public static final int deploymentMaxDeg = 115;
    
        // rollers
        public static final int rollerID = 1;

        public static final double rollerKS = 0.01;
        public static final double rollerKP = 10;
        public static final double rollerKI = 0;
        public static final double rollerKD = 0.1;

        public static final double rollerMaxVoltage = 4; // can change if not needed
        public static final double rollerMaxAcceleration = 40;
        public static final double rollerMaxVelocity = 100; // rps
    }    
}
