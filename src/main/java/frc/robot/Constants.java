package frc.robot;

import com.ctre.phoenix6.signals.InvertedValue;
import com.ctre.phoenix6.signals.NeutralModeValue;
import com.ctre.phoenix6.signals.SensorDirectionValue;

import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.SwerveDriveKinematics;
import edu.wpi.first.math.trajectory.TrapezoidProfile;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.SPI;
import frc.lib.util.COTSTalonFXSwerveConstants;
import frc.lib.util.SwerveModuleConstants;

public final class Constants {
    public static final String CANivore = "canivore";
    public static final double stickDeadband = 0.1;
    public static class AlignToTag
    {
        //TODO measure with LimeLight Camera
        public static final double CenterReef_AREA_STAGE = 1.75;
        public static final double CenterReef_AREA_REEF = 4.93;
        public static final double CenterReef_Y_STAGE = 9.26;
        public static final double CenterReef_Y_REEF = 19.65;
        public static final double CenterReef_OMEGA_STAGE = 3.28;
        public static final double CenterReef_OMEGA_REEF = -4.13;

        public static final double LeftReef_AREA_STAGE =1.41;
        public static final double LeftReef_AREA_REEF = 1.41;
        public static final double LeftReef_Y_STAGE = 19.77;
        public static final double LeftReef_Y_REEF = 19.77;
        public static final double LeftReef_OMEGA_STAGE = 4.55;
        public static final double LeftReef_OMEGA_REEF = 4.55;

        public static final double RightReef_AREA_STAGE = 1.8;
        public static final double RightReef_AREA_REEF = 4.51;

        public static final double RightReef_Y_STAGE = -0.18;
        public static final double RightReef_Y_REEF = 4.77;

        public static final double RightReef_OMEGA_STAGE = 3.3;
        public static final double RightReef_OMEGA_REEF = -3.44;



    };

    public static class ControllerConstants {
        public static final int DRIVER_PORT = 0;
        public static final int OPERATOR_PORT = 1;

        public static class AxesConstants {
            public static final int translationAxis = 1;
            public static final int strafeAxis = 0;
            public static final int rotationAxis = 2;
            public static final int SLIDER = 3;


        }

        public static class ButtonConstants {

            public static final int Trigger = 1;
            public static final int Button_2 = 2;
            public static final int Button_3 = 3;
            public static final int Button_4 = 4;
            public static final int Button_5 = 5;
            public static final int Button_6 = 6;
            public static final int Button_7 = 7;
            public static final int Button_8 = 8;
            public static final int Button_9 = 9;
            public static final int Button_10 = 10;
            public static final int Button_11 = 11;
            public static final int Button_12 = 12;

            public static final int rightWhiteButton = 13;
            public static final int rightBlackButton = 12;
            public static final int rightRedButton = 11;
            public static final int rightBlueButton = 10;
            public static final int rightYellowButton = 8;
            public static final int rightGreenButton = 9;
            public static final int leftWhiteButton = 5;
            public static final int leftBlackButton = 7;
            public static final int leftRedButton = 3;
            public static final int leftBlueButton = 4;
            public static final int leftYellowButton = 2;
            public static final int leftGreenButton = 1;
        }
    }

    public static class Algae
    {
        //TODO which color was left and right
        public static final int INTAKE_LEFT = 12;
        public static final int INTAKE_RIGHT = 11;
        public static final int ARM_MOTOR = 21;
        public static final int A_UPPER_LIMIT_SWITCH = 0;
        public static final double INTAKE_SPEED = 0.8;
        public static final double ARM_SPEED = 0.3;
        public static double UpperArmPosition = 20;
        //TODO
        public static int hasAlgaeSwitch = 4;
    };

    public static class Coral{
        public static final int IntakeMotor = 23;
        public static final int ArmMotor = 30;

        //TODO make limit switch
        public static final int C_UPPER_LIMIT = 3;
        public static double IntakeSpeed = 0.6;
        public static double OutTakeSpeed = 0.2;
        public static double ArmSpeed = 0.15;
        //TODO
        public static double IntakePosition = 6;
        public static double OuttakeL1L2L3Position = 15;
        public static double OuttakeL4Position = 18;
        public static double ArmUpPosition = 0;
        //TODO
        public static int PhotoSensor = 1;
    }

    public static final class Swerve {

        public static final double SCORE_DISTANCE = 0.5;
        public static boolean invertGyro = true;
        public static SPI.Port navX = SPI.Port.kMXP;

        public static final COTSTalonFXSwerveConstants chosenModule =  //TODO: This must be tuned to specific robot
                COTSTalonFXSwerveConstants.SDS.MK4i.KrakenX60(COTSTalonFXSwerveConstants.SDS.MK4i.driveRatios.L2);

        /* Drivetrain Constants */
        public static final double trackWidth = Units.inchesToMeters(28); //TODO: This must be tuned to specific robot
        public static final double wheelBase = Units.inchesToMeters(28); //TODO: This must be tuned to specific robot
        public static final double wheelCircumference = chosenModule.wheelCircumference;

        /* Swerve Kinematics
         * No need to ever change this unless you are not doing a traditional rectangular/square 4 module swerve */
        public static final SwerveDriveKinematics swerveKinematics = new SwerveDriveKinematics(
                new Translation2d(wheelBase / 2.0, trackWidth / 2.0),
                new Translation2d(wheelBase / 2.0, -trackWidth / 2.0),
                new Translation2d(-wheelBase / 2.0, trackWidth / 2.0),
                new Translation2d(-wheelBase / 2.0, -trackWidth / 2.0));

        /* Module Gear Ratios */
        public static final double driveGearRatio = chosenModule.driveGearRatio;
        public static final double angleGearRatio = chosenModule.angleGearRatio;

        /* Motor Inverts */
        public static final InvertedValue angleMotorInvert = chosenModule.angleMotorInvert;
        public static final InvertedValue driveMotorInvert = chosenModule.driveMotorInvert;

        /* Angle Encoder Invert */
        public static final SensorDirectionValue cancoderInvert = chosenModule.cancoderInvert;

        /* Swerve Current Limiting */
        public static final int angleCurrentLimit = 25;
        public static final int angleCurrentThreshold = 40;
        public static final double angleCurrentThresholdTime = 0.1;
        public static final boolean angleEnableCurrentLimit = true;

        public static final int driveCurrentLimit = 35;
        public static final int driveCurrentThreshold = 60;
        public static final double driveCurrentThresholdTime = 0.1;
        public static final boolean driveEnableCurrentLimit = true;

        /* These values are used by the drive falcon to ramp in open loop and closed loop driving.
         * We found a small open loop ramp (0.25) helps with tread wear, tipping, etc */
        public static final double openLoopRamp = 0.25;
        public static final double closedLoopRamp = 0.0;

        /* Angle Motor PID Values */
        public static final double angleKP = chosenModule.angleKP;
        public static final double angleKI = chosenModule.angleKI;
        public static final double angleKD = chosenModule.angleKD;

        /* Drive Motor PID Values */
        public static final double driveKP = 0.5; //TODO: This must be tuned to specific robot
        public static final double driveKI = 0.0;
        public static final double driveKD = 0.0;
        public static final double driveKF = 0.0;

        /* Drive Motor Characterization Values From SYSID */
        public static final double driveKS = 0.32; //TODO: This must be tuned to specific robot
        public static final double driveKV = 1.51;
        public static final double driveKA = 0.27;

        /* Swerve Profiling Values */
        /** Meters per Second */
        public static final double maxSpeed = 1; //TODO: This must be tuned to specific robot
        /** Radians per Second */
        public static final double maxAngularVelocity = 2; //TODO: This must be tuned to specific robot

        /* Neutral Modes */
        public static final NeutralModeValue angleNeutralMode = NeutralModeValue.Coast;
        public static final NeutralModeValue driveNeutralMode = NeutralModeValue.Brake;

        /* Module Specific Constants */
        /* Front Left Module - Module 0 */
        public static final class Mod0 { //TODO: This must be tuned to specific robot
            public static final int driveMotorID = 2;
            public static final int angleMotorID = 1;
            public static final int canCoderID = 1;
            public static final Rotation2d angleOffset = Rotation2d.fromDegrees(-119.79);
            public static final SwerveModuleConstants constants =
                    new SwerveModuleConstants(driveMotorID, angleMotorID, canCoderID, angleOffset);
        }

        /* Front Right Module - Module 1 */
        public static final class Mod1 { //TODO: This must be tuned to specific robot
            public static final int driveMotorID = 3;
            public static final int angleMotorID = 4;
            public static final int canCoderID = 2;
            public static final Rotation2d angleOffset = Rotation2d.fromDegrees(140);
            public static final SwerveModuleConstants constants =
                    new SwerveModuleConstants(driveMotorID, angleMotorID, canCoderID, angleOffset);
        }

        /* Back Left Module - Module 2 */
        public static final class Mod2 { //TODO: This must be tuned to specific robot
            public static final int driveMotorID = 5;
            public static final int angleMotorID = 6;
            public static final int canCoderID = 3;
            public static final Rotation2d angleOffset = Rotation2d.fromDegrees(161.15);
            public static final SwerveModuleConstants constants =
                    new SwerveModuleConstants(driveMotorID, angleMotorID, canCoderID, angleOffset);
        }

        /* Back Right Module - Module 3 */
        public static final class Mod3 { //TODO: This must be tuned to specific robot
            public static final int driveMotorID = 7;
            public static final int angleMotorID = 8;
            public static final int canCoderID = 4;
            public static final Rotation2d angleOffset = Rotation2d.fromDegrees(147.1);
            public static final SwerveModuleConstants constants =
                    new SwerveModuleConstants(driveMotorID, angleMotorID, canCoderID, angleOffset);
        }

    }


    public static final class AutoConstants { //TODO: The below constants are used in the example auto, and must be tuned to specific robot
        public static final double kMaxSpeedMetersPerSecond = 10;
        public static final double kMaxAccelerationMetersPerSecondSquared = 10;
        public static final double kMaxAngularSpeedRadiansPerSecond = Math.PI;
        public static final double kMaxAngularSpeedRadiansPerSecondSquared = Math.PI;

        public static final double kPXController = 0.5;
        public static final double kPYController = 1;
        public static final double kPThetaController = 0.5;

        /* Constraint for the motion profilied robot angle controller */
        public static final TrapezoidProfile.Constraints kThetaControllerConstraints =
                new TrapezoidProfile.Constraints(
                        kMaxAngularSpeedRadiansPerSecond, kMaxAngularSpeedRadiansPerSecondSquared);
    }
    public static final class Sensor{
        public static String LIMELIGHT = "limelight";
    }
    public static class ElevatorConstants {
        // Have to wire this all to get ID, set to 0 for now
        public static final int ELEVATOR_MOTOR = 20;
        public static final int ELEVATOR_BOTTOM_SWITCH = 2;

        public static final double UPSPEED = 0.4;
        public static final double DOWNSPEED = 0.1;
        public static double L1AlgaePosition = 25;
        public static double L2AlgaePosition = 35;
        public static double L1Position = 15;
        public static double L2Position = 20;
        public static double L3Position = 42;
        public static double L4Position = 85;
    }
}