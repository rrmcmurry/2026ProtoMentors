package frc.robot.commands;

import java.util.function.BooleanSupplier;
import java.util.function.DoubleSupplier;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.field.AllianceUtil;
import frc.robot.subsystems.DriveSubsystem;
import frc.robot.subsystems.Launcher;

public class AimAtTargetCommand extends Command {
    private final DriveSubsystem drive;
    private final Launcher launcher;
    private final DoubleSupplier fwd;
    private final DoubleSupplier strafe;
    private final BooleanSupplier fieldRelative;
    private final PIDController thetaPid = new PIDController(4.0, 0.0, 0.2);

    private int stableLoops = 0;

    public AimAtTargetCommand(
            DriveSubsystem drive,
            Launcher launcher,
            DoubleSupplier fwd,
            DoubleSupplier strafe,
            BooleanSupplier fieldRelative) {
        this.drive = drive;
        this.launcher = launcher;
        this.fwd = fwd;
        this.strafe = strafe;
        this.fieldRelative = fieldRelative;
        

        addRequirements(drive);

        thetaPid.enableContinuousInput(-Math.PI, Math.PI);
        thetaPid.setTolerance(Math.toRadians(1.0));
    }

    @Override
    public void initialize() {
        thetaPid.reset();
    }


    private Translation2d getTarget(Pose2d launcherPose) {
        Translation2d pos = launcherPose.getTranslation();

        // If in our Alliance Zone, target the Hub
        if (AllianceUtil.isInAllianceZone(pos)) {
            return AllianceUtil.getAllianceHubCenter();
        }

        // If in the neutral zone, choose one of the alliance-zone shot locations
        if (AllianceUtil.isInNeutralZone(pos)) {
            return AllianceUtil.getBestAllianceZoneShotTarget(pos);
        }

        // Fallback: hub
        return AllianceUtil.getAllianceHubCenter();
    }

    @Override
    public void execute() {
        // Current position and orientation of launcher
        Pose2d LauncherPose = drive.getLauncherPose();

        // Target position
        Translation2d target = getTarget(LauncherPose);

        ChassisSpeeds robotSpeeds = drive.getRobotRelativeSpeeds();

        // Vector from launch point to target
        Translation2d toTarget = target.minus(LauncherPose.getTranslation());

        // Calculate distance, lookup voltage, and set voltage 
        double distance = toTarget.getNorm();
        if (launcher.isUsingRpmControl()) {
            double rpm = lookupRpm(distance);
            launcher.setTargetRpm(rpm);
        } else {
            double voltage = lookupVoltage(distance);
            launcher.setTargetVoltage(voltage);
        }

        // Lookup Time of Flight 
        double timeOfFlight = lookupTimeOfFlight(distance);

        // Get Current speed and heading
        Translation2d robotFieldVelocity = new Translation2d(
            robotSpeeds.vxMetersPerSecond,
            robotSpeeds.vyMetersPerSecond
        );

        // Compensate target based on current velocity
        Translation2d compensatedTarget = target.minus(robotFieldVelocity.times(-timeOfFlight));

        // Recompute Aim vector using compensated target
        Translation2d compensatedToTarget = compensatedTarget.minus(LauncherPose.getTranslation());

        // Desired field-facing angle for the Launcher/robot
        Rotation2d desiredFieldHeading = compensatedToTarget.getAngle();

        // Current Launcher heading is same as robot heading here
        Rotation2d currentFieldHeading = LauncherPose.getRotation();

        // Calculate rotation command value in radians per second using PID 
        double rotCmdRadPerSec = thetaPid.calculate(
            currentFieldHeading.getRadians(),
            desiredFieldHeading.getRadians()
        );

        drive.drive(
            fwd.getAsDouble(),
            strafe.getAsDouble(),
            -rotCmdRadPerSec / DriveSubsystem.kMaxAngularSpeed,
            fieldRelative.getAsBoolean()
        );
    }

    @Override
    public void end(boolean interrupted) {
        drive.drive(fwd.getAsDouble(), strafe.getAsDouble(), 0.0, fieldRelative.getAsBoolean());
    }

    @Override
    public boolean isFinished() {
        return false;
    }

    public boolean atGoal() {
      if (thetaPid.atSetpoint()) {
          stableLoops++;
      } else {
          stableLoops = 0;
      }
      return stableLoops > 5; // ~100 ms at 20ms loop
    }

    // Lookup Tables by Distance:
    private static final double[] Distance = { 6, 8, 18 };
    private static final double[] Voltage = { 5.35, 5.4, 7};
    private static final double[] TimeOfFlight = {0.4, 0.6, 0.8};
    private static final double[] Rpm = { 3200, 3400, 4300 };

    public static double lookupVoltage(double meters) {
        // Convert meters to feet
        double feet = Units.metersToFeet(meters);

        // If less than 6 feet, return 5.35 volts
        if (feet <= Distance[0]) return Voltage[0];

        // Otherwise interpolate from mapped values
        for (int i = 0; i < Distance.length - 1; i++) {
        if (feet <= Distance[i + 1]) {
            double t = (feet - Distance[i]) / (Distance[i + 1] - Distance[i]);
            return Voltage[i] + t * (Voltage[i + 1] - Voltage[i]);
        }
        }
        return Voltage[Voltage.length - 1];
    }

    public static double lookupTimeOfFlight(double meters) {
        // Convert meters to feet
        double feet = Units.metersToFeet(meters);

        // If less than 6 feet, return .25 seconds 
        if (feet <= Distance[0]) return TimeOfFlight[0];

        // Otherwise interpolate from mapped values
        for (int i = 0; i < Distance.length - 1; i++) {
        if (feet <= Distance[i + 1]) {
            double t = (feet - Distance[i]) / (Distance[i + 1] - Distance[i]);
            return TimeOfFlight[i] + t * (TimeOfFlight[i + 1] - TimeOfFlight[i]);
        }
        }
        return TimeOfFlight[TimeOfFlight.length - 1];
    }


    public static double lookupRpm(double meters) {
        double feet = Units.metersToFeet(meters);

        if (feet <= Distance[0]) return Rpm[0];

        for (int i = 0; i < Distance.length - 1; i++) {
            if (feet <= Distance[i + 1]) {
                double t = (feet - Distance[i]) / (Distance[i + 1] - Distance[i]);
                return Rpm[i] + t * (Rpm[i + 1] - Rpm[i]);
            }
        }
        return Rpm[Rpm.length - 1];
    }

}