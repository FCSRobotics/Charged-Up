package frc.robot.commands.swervedrive2.auto

import com.pathplanner.lib.PathPlannerTrajectory
import com.pathplanner.lib.commands.PPSwerveControllerCommand
import edu.wpi.first.math.kinematics.ChassisSpeeds
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup
import frc.robot.Constants.Auton
import frc.robot.subsystems.swervedrive2.SwerveSubsystem

class FollowTrajectory(drivebase: SwerveSubsystem, trajectory: PathPlannerTrajectory, resetOdometry: Boolean) : SequentialCommandGroup() {
    init {
        addRequirements(drivebase)
        if (resetOdometry) {
            drivebase.resetOdometry(trajectory.initialHolonomicPose)
        }
        addCommands(
                PPSwerveControllerCommand(
                        trajectory,
                        drivebase::pose,
                        Auton.xAutoPID.createPIDController(),
                        Auton.yAutoPID.createPIDController(),
                        Auton.angleAutoPID.createPIDController(), { chassisSpeeds: ChassisSpeeds? -> drivebase.setChassisSpeeds(chassisSpeeds) },
                        drivebase)
        )
    }
}
