package frc.robot.commands.movement;


import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.robot.subsystems.Swerve;

public class DriveToFeederLeftCommandGroup extends SequentialCommandGroup {
    public DriveToFeederLeftCommandGroup(Swerve swerve, Pose2d pose2d) {
        // TODO: Add your sequential commands in the super() call, e.g.
        //           super(new OpenClawCommand(), new MoveArmCommand());
        super(swerve.driveToFeederLeft());
    }
}