package frc.robot.commands.movement;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.geometry.*;
import edu.wpi.first.math.trajectory.TrapezoidProfile;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Constants;
import frc.robot.LimelightHelpers;
import frc.robot.subsystems.Swerve;

public class AlignWithTag extends Command {

  //private static final int TAG_TO_CHASE = 2;

  private final Swerve drivetrainSubsystem;
  private final PIDController xController = new PIDController(0.05, 0, 0);
  private final PIDController yController = new PIDController(0.05, 0, 0);
  private final PIDController omegaController = new PIDController(0.05, 0, 0);

  private final double SET_AREA;
  private final double SET_Y;
  private final double SET_OMEGA;

  public AlignWithTag(Swerve drivetrainSubsystem, double area, double y, double omega) {
    this.drivetrainSubsystem = drivetrainSubsystem;

    //x was 1
    //y was 10
    //omega was 10
    SET_AREA = area;
    SET_Y = y;
    SET_OMEGA = omega;

    xController.setTolerance(0.2);
    yController.setTolerance(0.5);
    omegaController.setTolerance(.5);

    addRequirements(drivetrainSubsystem);
  }

  @Override
  public void initialize() {
    //LimelightHelpers.SetFiducialIDFiltersOverride(Constants.Sensor.LIMELIGHT, new int[]{TAG_TO_CHASE});
    LimelightHelpers.SetFiducialDownscalingOverride(Constants.Sensor.LIMELIGHT, 2.0f);
    //xController.setSetpoint(5);
    //yController.setSetpoint(0);
  }

  @Override
  public void execute() {
    //Check if Limelight is seeing a target


    if (!LimelightHelpers.getTV(Constants.Sensor.LIMELIGHT)) {
      // No target has been visible
      drivetrainSubsystem.stop();
    }
    else
    {
      double tx = LimelightHelpers.getTX(Constants.Sensor.LIMELIGHT);  // Horizontal offset from crosshair to target in degrees
      double ty = LimelightHelpers.getTY(Constants.Sensor.LIMELIGHT);  // Vertical offset from crosshair to target in degrees
      double ta = LimelightHelpers.getTA(Constants.Sensor.LIMELIGHT);

      // Drive to the target
      var xSpeed = MathUtil.clamp(xController.calculate(ta, SET_AREA), -0.05, .05);
      if (xController.atSetpoint()) {
        xSpeed = 0;
      }

      var ySpeed =  MathUtil.clamp(yController.calculate(ty, SET_Y), -0.01, .01);
      if (yController.atSetpoint()) {
        ySpeed = 0;
      }

      var omegaSpeed = MathUtil.clamp(omegaController.calculate(tx, SET_OMEGA), -0.05, 0.05);
      if (omegaController.atSetpoint()) {
        omegaSpeed = 0;
      }

      drivetrainSubsystem.drive(new Translation2d(xSpeed, omegaSpeed).times(Constants.Swerve.maxSpeed),0
              ,
              false,
              true);
    }
  }

  @Override
  public boolean isFinished()
  {
    return xController.atSetpoint() && yController.atSetpoint() && omegaController.atSetpoint();
  }

  @Override
  public void end(boolean interrupted) {
    drivetrainSubsystem.stop();
  }

}
