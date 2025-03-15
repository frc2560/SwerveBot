package frc.robot.commands.movement;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Constants;
import frc.robot.LimelightHelpers;
import frc.robot.subsystems.Swerve;

import java.util.Dictionary;
import java.util.Hashtable;


public class TurnToAngleCommand extends Command {
    private final Swerve swerve;
    private final PIDController  omegaController = new PIDController(0.05, 0, 0);

    // all coral tags with field-relative angles
    private static final Dictionary<Integer, Double> tagAngles = new Hashtable<>();

    static {
        tagAngles.put(6, 300.0);
        tagAngles.put(7, 0.0); //todo fix
        tagAngles.put(8, 60.0);
        tagAngles.put(9, 120.0);
        tagAngles.put(10, 180.0);
        tagAngles.put(11, 240.0);
        tagAngles.put(17, 240.0);
        tagAngles.put(18, 180.0);
        tagAngles.put(19, 120.0);
        tagAngles.put(20, 60.0);
        tagAngles.put(21, 0.0);
        tagAngles.put(22, 300.0);
    }

    private double targetAngle;

    public TurnToAngleCommand(Swerve swerve) {
        this.swerve = swerve;
        // each subsystem used by the command must be passed into the
        // addRequirements() method (which takes a vararg of Subsystem)
        omegaController.setTolerance(3);
        omegaController.enableContinuousInput(-180, 180);
        // add 180.0 to each tag
        LimelightHelpers.SetFiducialDownscalingOverride(Constants.Sensor.LIMELIGHT, 2.0f);
        int tagNumber = (int)LimelightHelpers.getFiducialID(Constants.Sensor.LIMELIGHT);
        this.targetAngle = tagAngles.get(tagNumber) != null ? tagAngles.get(tagNumber) + 180 : 0.0;
        addRequirements(this.swerve);

    }

    @Override
    public void initialize() {

    }

    @Override
    public void execute() {
        var omegaSpeed = MathUtil.clamp(omegaController.calculate(swerve.getGyroYaw().getDegrees(), targetAngle), -0.05, 0.05);
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
