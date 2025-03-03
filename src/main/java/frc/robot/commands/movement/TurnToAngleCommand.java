package frc.robot.commands.movement;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Constants;
import frc.robot.subsystems.Swerve;


public class TurnToAngleCommand extends Command {
    private final Swerve swerve;
    private final PIDController  omegaController = new PIDController(0.05, 0, 0);

    private double Angle;

    public TurnToAngleCommand(Swerve swerve, double angle) {
        this.swerve = swerve;
        // each subsystem used by the command must be passed into the
        // addRequirements() method (which takes a vararg of Subsystem)
        omegaController.setTolerance(3);
        Angle = angle;
        addRequirements(this.swerve);

    }

    @Override
    public void initialize() {

    }

    @Override
    public void execute() {
        var omegaSpeed = MathUtil.clamp(omegaController.calculate(swerve.getGyroYaw().getDegrees(), Angle), -0.05, 0.05);
        if (omegaController.atSetpoint()) {
            omegaSpeed = 0;
        }
        swerve.drive(new Translation2d(0, 0).times(Constants.Swerve.maxSpeed),omegaSpeed
                ,
                true,
                true);


    }

    @Override
    public boolean isFinished() {
        return omegaController.atSetpoint();
    }

    @Override
    public void end(boolean interrupted) {
        swerve.stop();
    }
}
