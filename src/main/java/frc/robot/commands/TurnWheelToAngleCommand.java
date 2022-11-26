package frc.robot.commands;

import java.util.function.DoubleSupplier;

import com.swervedrivespecialties.swervelib.SwerveModule;

import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.Constants.WheelPositions;
import frc.robot.subsystems.DrivetrainSubsystem;

public class TurnWheelToAngleCommand extends CommandBase  {

    private final DrivetrainSubsystem m_drivetrainSubsystem;
    private WheelPositions m_wheelPosition;
    private double m_angle;
    private SwerveModule m_module;

    public TurnWheelToAngleCommand(DrivetrainSubsystem drivetrainSubsystem, WheelPositions wheel,
                               double angle) {
        this.m_drivetrainSubsystem = drivetrainSubsystem;
        this.m_angle = angle;
        this.m_wheelPosition = wheel;
        this.m_module = m_drivetrainSubsystem.getModule(m_wheelPosition);
        addRequirements(drivetrainSubsystem);
    }

    @Override 
    public void initialize() {
             
    }

    @Override
    public void execute() {
        m_module.set(0.0, m_angle);  
    }

    @Override public boolean isFinished() {
        if(Math.abs(m_module.getSteerAngle() - m_angle) < Rotation2d.fromDegrees(3.0).getRadians()) {
            return true;
        } else {
            return false;
        }
    }

    @Override
    public void end(boolean interrupted) {

    }
}