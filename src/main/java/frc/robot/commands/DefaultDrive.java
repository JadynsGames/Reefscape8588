package frc.robot.commands;

import edu.wpi.first.math.filter.SlewRateLimiter;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import frc.robot.Constants;
import frc.robot.RobotContainer;
import frc.robot.Constants.DriveConstants;
import frc.robot.subsystems.SwerveSubsystem;


public class DefaultDrive extends Command {
    CommandXboxController joystick;
    SwerveSubsystem m_Swerb;

    SlewRateLimiter xRateLimiter = new SlewRateLimiter(DriveConstants.kXYSlewRate);
    SlewRateLimiter yRateLimiter = new SlewRateLimiter(DriveConstants.kXYSlewRate);
    SlewRateLimiter wRateLimiter = new SlewRateLimiter(DriveConstants.kRotationalSlewRate);
    
    public DefaultDrive() {
        addRequirements(RobotContainer.m_Swerb);
        this.m_Swerb = RobotContainer.m_Swerb;
        joystick = RobotContainer.driverController;
    }
    
    @Override
    public void execute() {
        double maxSpeed = Constants.DriveConstants.kMaxLinearSpeed;
        double vx = -joystick.getLeftY() * maxSpeed;
        double vy = -joystick.getLeftX() * maxSpeed;
        double vw = -joystick.getRightX() * .1;
        
        // Apply slew rate limits
        vx = xRateLimiter.calculate(vx);
        vy = yRateLimiter.calculate(vy);
        vw = wRateLimiter.calculate(vw);
        
        ChassisSpeeds speeds = ChassisSpeeds.fromFieldRelativeSpeeds(vx, vy, vw, Rotation2d.fromDegrees(-m_Swerb.getYaw()));
        //ChassisSpeeds speeds = new ChassisSpeeds(vx, vy, vw);
        m_Swerb.drive(speeds);
    }
    
    /*
    * CONTROLLER CHANNELS
    * Analog:
    * 0 - Left X
    * 1 - Left Y
    * 4 - Right X
    * 
    * Digital:
    * 4 - Y Button
    */
}
