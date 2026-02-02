package frc.robot;

import frc.robot.LimelightHelpers.PoseEstimate;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.DriverStation.Alliance;
import edu.wpi.first.networktables.NetworkTableInstance;

import edu.wpi.first.math.geometry.Translation2d;

public class Limelight  
{
    private static final Translation2d redHub = new Translation2d(11.915521, 4.034536);
    // private static final Translation2d blueHub = new Translation2d(182.105, 158.84);
    private static final Translation2d blueHub = new Translation2d(4.625467, 4.034536);

    static boolean isBlue = false;
    
    public static double GetTx()
    {
        boolean isBlue = DriverStation.getAlliance().orElse(Alliance.Blue) == Alliance.Blue;

        long tid = NetworkTableInstance.getDefault().getTable("limelight-a").getEntry("tid").getInteger(0);
        if (isBlue)
        {
            return (tid == 26 || tid == 25) ? NetworkTableInstance.getDefault().getTable("limelight-a").getEntry("tx").getDouble(1111) : 222222; 
        }
        else
        {
            return (tid == 9 || tid == 10) ? NetworkTableInstance.getDefault().getTable("limelight-a").getEntry("tx").getDouble(0) : 0; 
        }  
    }

    public static boolean getIsBlue()
    {
        return isBlue = DriverStation.getAlliance().orElse(Alliance.Blue) == Alliance.Blue;
    }

    public static PoseEstimate getPose()
    {
        return LimelightHelpers.getBotPoseEstimate_wpiBlue("limelight-a");        
    }

    public static Translation2d getHub(boolean isBlue)
    {
        final Translation2d hub;

        if (isBlue)
        {
            hub = blueHub;
        }
        else 
        {
            hub = redHub;
        }

        return hub;
    }

    public static Translation2d GetDistance()
    {
        boolean isBlue = getIsBlue();
        Translation2d hub = getHub(isBlue);
        PoseEstimate pose = getPose();

        return hub.minus(pose.pose.getTranslation());
    }

    public static double GetAngle()
    {
        boolean isBlue = getIsBlue();
        Translation2d hub = getHub(isBlue);
        PoseEstimate pose = getPose();

        // System.out.println("bot position is " + pose.pose);
        // System.out.println(hub);

        if (pose == null) {
            return 0;
        }

        return hub.minus(pose.pose.getTranslation()).getAngle().getDegrees();
    }
}