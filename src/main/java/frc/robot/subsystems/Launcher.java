package frc.robot.subsystems;

import java.util.function.DoubleSupplier;

import com.revrobotics.spark.ClosedLoopSlot;
import com.revrobotics.spark.SparkMax;
import com.revrobotics.spark.config.SparkBaseConfig.IdleMode;
import com.revrobotics.spark.config.SparkMaxConfig;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

import com.revrobotics.spark.SparkBase.ControlType;
import com.revrobotics.spark.SparkBase.PersistMode;
import com.revrobotics.spark.SparkBase.ResetMode;
import com.revrobotics.spark.SparkClosedLoopController;
import com.revrobotics.spark.SparkLowLevel.MotorType;

@SuppressWarnings("removal")
public class Launcher extends SubsystemBase{

    private static SparkMaxConfig DefaultConfig = new SparkMaxConfig();    
    private SparkMax LaunchMotor; 
    private SparkMax PreLaunchMotor;    
    private SparkMax HopperMotor;
    public static final int kHopperMotorCanID = 12;
    public static final int kPreLaunchMotorCanID = 10;
    public static final int kLaunchMotorCanID = 9;

    private double targetVoltage = 5.8;
    private double hopperVoltage = 4.5;
    private double prelaunchVoltage = 5.5;
    private DoubleSupplier targetVoltageSupplier;
    
    private SparkClosedLoopController launchClosedLoopController;
    private boolean useRpmControl = false;
    private double targetRpm = 4000.0;
    private DoubleSupplier targetRpmSupplier;

    static {
        DefaultConfig.smartCurrentLimit(50);
        DefaultConfig.idleMode(IdleMode.kCoast);
        DefaultConfig.openLoopRampRate(1.5);
        DefaultConfig.inverted(true);
        DefaultConfig.voltageCompensation(12);
        DefaultConfig.closedLoop.allowedClosedLoopError(100.0, ClosedLoopSlot.kSlot0);        
        DefaultConfig.closedLoop
            .p(0.0002)
            .i(0.0)
            .d(0.0)
            .velocityFF(0.00017);
    }

    public Launcher() {
        LaunchMotor = new SparkMax(kLaunchMotorCanID, MotorType.kBrushless);
        LaunchMotor.configure(DefaultConfig, ResetMode.kNoResetSafeParameters, PersistMode.kNoPersistParameters);
        PreLaunchMotor = new SparkMax(kPreLaunchMotorCanID, MotorType.kBrushless);
        PreLaunchMotor.configure(DefaultConfig, ResetMode.kNoResetSafeParameters, PersistMode.kNoPersistParameters);
        HopperMotor = new SparkMax(kHopperMotorCanID, MotorType.kBrushless);
        HopperMotor.configure(DefaultConfig, ResetMode.kNoResetSafeParameters, PersistMode.kNoPersistParameters);
        targetVoltageSupplier = ()-> targetVoltage;
        launchClosedLoopController = LaunchMotor.getClosedLoopController();
        targetRpmSupplier = () -> targetRpm;
    }

    @Override
    public void periodic() {        
        SmartDashboard.putNumber("Shooter/Voltage", targetVoltage);        
    }

    public double getTargetRpm() {
        return targetRpm;
    }

    public void setTargetVoltage(double v) {
        targetVoltage = MathUtil.clamp(v, 4.0, 8.0);
    }

    public Command increaseLaunchVoltage() {
        return Commands.runOnce(() -> targetVoltage = Math.min(targetVoltage + .1, 8.0));
    }

    public Command decreaseLaunchVoltage() {
        return Commands.runOnce(() -> targetVoltage = Math.max(targetVoltage - .1, 4.0));
    }

    public void stopAll() {
        HopperMotor.stopMotor();
        PreLaunchMotor.stopMotor();
        LaunchMotor.stopMotor();
    }

    public Command run() {
        return this.runOnce(this::startLaunchMotor)
            .andThen(Commands.waitSeconds(1.3))
            .andThen(this.runOnce(() -> PreLaunchMotor.setVoltage(prelaunchVoltage)))
            .andThen(this.runOnce(() -> HopperMotor.setVoltage(hopperVoltage)));
    }

    public Command dump() {
        return Commands.sequence(
            this.runOnce(() -> LaunchMotor.setVoltage(-4)),
            this.runOnce(() -> PreLaunchMotor.setVoltage(-4)),            
            this.run(() -> HopperMotor.setVoltage(-4)))
            .finallyDo(() -> {
                HopperMotor.stopMotor(); 
                PreLaunchMotor.stopMotor();
                LaunchMotor.stopMotor();
            });
    }

    public Command stop() {
        return this.runOnce(() -> {
            HopperMotor.stopMotor(); 
            PreLaunchMotor.stopMotor();
            LaunchMotor.stopMotor();
        });
    }


    public Command fire() {
        return Commands.sequence(
            this.runOnce(this::startLaunchMotor),
            Commands.waitSeconds(1.3),
            this.run(() -> {
                PreLaunchMotor.setVoltage(prelaunchVoltage);
                HopperMotor.setVoltage(hopperVoltage);
            })
        ).finallyDo(() -> {
            LaunchMotor.stopMotor();
            PreLaunchMotor.stopMotor();
            HopperMotor.stopMotor();
        });
    }

    public void setUseRpmControl(boolean enabled) {
        useRpmControl = enabled;
    }

    public boolean isUsingRpmControl() {
        return useRpmControl;
    }

    public void setTargetRpm(double rpm) {
        targetRpm = MathUtil.clamp(rpm, 2500.0, 6000.0);
    }

    private void startLaunchMotor() {
        if (useRpmControl) {
            launchClosedLoopController.setReference(
                targetRpmSupplier.getAsDouble(),
                ControlType.kVelocity
            );
        } else {
            LaunchMotor.setVoltage(targetVoltageSupplier.getAsDouble());
        }
    }

}