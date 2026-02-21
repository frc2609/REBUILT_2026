package frc.robot.commands;
import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.DriverStation.Alliance;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Constants;
import frc.robot.subsystems.ShooterSubsystem;

public class AimAtHub extends Command {
    private final ShooterSubsystem shooter;
    //private final Pose3d HUB_POSITION;

    public AimAtHub(ShooterSubsystem shooter) {
        this.shooter = shooter;
        // if (DriverStation.getAlliance().get() == Alliance.Red)
        // {
        //     HUB_POSITION = Constants.Field.RED_HUB;
        // }
        // else
        // {
        //     HUB_POSITION = Constants.Field.BLUE_HUB;
        // }
    }

    

    @Override
    public void execute() {
         
    }

    @Override
    public void end(boolean interrupted) {
        
    }

    @Override
    public boolean isFinished() {
        return true;
    }
}
