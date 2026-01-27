package frc.robot.commands;

import frc.robot.subsystems.Transfer;
import frc.robot.subsystems.Turret;
import frc.robot.subsystems.Shooter;
import frc.robot.subsystems.Hood;

import frc.robot.Constants.TransferConstants;
import frc.robot.Constants.TurretConstants;
import frc.robot.Limelight;
import frc.robot.Shot;
import frc.robot.Constants.ShooterConstants;
import frc.robot.Constants.HoodConstants;
import frc.robot.Constants.AutoShooterConstants;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;

import java.util.ArrayList;
import java.util.List;

public class AutoFire extends Command 
{
    Turret turret;
    Transfer transfer;
    Shooter shooter;
    Hood hood;
    double tgtRPM;
    double tgtAngle;
    
    double lastDistance = 0;

    Translation2d lastOffset;

    public AutoFire(Turret turret, Transfer transfer, Shooter shooter, Hood hood)
    {
        this.turret = turret;
        this.transfer = transfer;
        this.shooter = shooter;
        this.hood = hood;

        addRequirements(shooter, hood);

        tgtRPM = 0;
        tgtAngle = 0.0;
        
    }

    
    @Override
    public void initialize(){
        tgtRPM = shooter.GetCorrectRPS();
        //tgtAngle = hood.GetTargetHoodAngle();
    }

    // hub is 47 inches by 47 inches
    /* 
        trajectory equation

        evaluate values of y between hub position - 23.5 inches  and + 23.5 inches;

        y = x tan theta - (gx^2)/(2u^2cos^theta)
    */ 

    // gets lists of all possible shots
    private ArrayList<Shot> GetPossibleShots()
    {
        ArrayList<Shot> possibleShots = new ArrayList<>();

        Translation2d offset = Limelight.GetOffset(); // gets y x components
        double distance = Limelight.GetDistance(); // computes diagonal dist
        
        // polar of r theta although its speed will use for robot readjustment later
        double speed = (distance - lastDistance) / AutoShooterConstants.kUpdateRate; // rate of change
        double angleTranslation = Math.atan2(offset.getX() - lastOffset.getX(), offset.getY() - lastOffset.getY());

        lastOffset = offset;

        // add 1 for inclusive
        int angleRange = (int)(HoodConstants.kMaximumAngle - HoodConstants.kMinimumAngle) + 1;

        // do not need to calculate for < 1000 bc useless
        int shooterRange = (int)((ShooterConstants.kMaxRPM - 1000) / ShooterConstants.kRpmStagingAuto); 

        boolean hasFoundValidShot = true;
        
        // generate a list of all hood angles
        for (int i = 0; i < angleRange; i++)
        {
            double hoodValue = HoodConstants.kMinimumAngle + i;

            // gets a list of all reasonable shooter values
            for (int j = 1; j < shooterRange; j++)
            {
                double shooterRPM = 1000 + (j * ShooterConstants.kRpmStagingAuto);
                
                // get the initial velocity of the ball based on rad/s * m
                double initialVelocity = shooterRPM * 2 * Math.PI * AutoShooterConstants.kFlywheelRadius / 60; 
                
                for (int k = 0; k < (int)(AutoShooterConstants.kHubSize/AutoShooterConstants.kHubStepSize); k++)
                {
                    // current problem: hood angle != release angle
                    double y = ComputeTrajectory(distance + AutoShooterConstants.kHubStepSize * k, hoodValue, initialVelocity);

                    if ((y - AutoShooterConstants.kHubHeight) < AutoShooterConstants.kExtrapolationLenience)
                    {
                        possibleShots.add(new Shot(shooterRPM, hoodValue));
                        hasFoundValidShot = true;
                    }
                }
            }
            
            if (hasFoundValidShot)
            {
                return possibleShots;
            }
        }
        
        return possibleShots;
    }

    // computes y value for given x of a trajectory
    // x in meters, theta in radians, u in rad*m/s
    public double ComputeTrajectory(double x, double theta, double u)
    {
        return (x * Math.tan(theta)) - ((9.81 * x * x) / 
        (2 * u * u * Math.cos(theta) * Math.cos(theta)));
    }

    // // get lowest angle shot from the list
    // public Shot GetBestShot(ArrayList<Shot> shots)
    // {  
    //     double midRPM = 100000;
    //     int lowestAngleIndex = 0;
    //     for (int i = 0; i < shots.size(); i++)
    //     {
    //         Shot temp = shots.get(i);
            
    //         if (temp.GetTheta() < lowestAngle)
    //         {
    //             lowestAngle = temp.GetTheta();
    //             lowestAngleIndex = i;
    //         }
    //     }
    //     shots.get(0).GetRPM();
    //     return shots.get(lowestAngleIndex);
    // }
    
    // fix find center logic
    public Shot GetBestShot(ArrayList<Shot> shots)
    {  
        int length = shots.size();
        int index = (int)(length/2);
        if (length % 2 == 0)
        {
            index -= 1;
        }
        
        return shots.get(index - 1);
    }

    @Override
    public void execute()
    {
        ArrayList<Shot> possibleShots = GetPossibleShots();

        Shot bestShot = GetBestShot(possibleShots);

        shooter.SetShooterVelocity(bestShot.GetRPM());
        shooter.fireAtRPM();

        // hood.moveHoodToAngle(bestShot.GetTheta());

        if (Math.abs(shooter.GetCorrectRPS() - shooter.GetShooterVelocity()) < ShooterConstants.kRpmLenience)
        {
            transfer.activateTransfer();
        }
        else
        {
            transfer.stop();
        }
    }

    @Override
    public void end(boolean interrupted)
    {
        // hood.stopHood();
        shooter.targetRPM = 0;
        shooter.fireAtRPM();
        transfer.stop();
    }

    @Override
    public boolean isFinished()
    {
        return false;
    }
}