package org.firstinspires.ftc.teamcode.swerve;

import com.arcrobotics.ftclib.geometry.Translation2d;
import com.arcrobotics.ftclib.geometry.Vector2d;
import com.arcrobotics.ftclib.geometry.Rotation2d;
import com.arcrobotics.ftclib.kinematics.wpilibkinematics.ChassisSpeeds;
import com.arcrobotics.ftclib.kinematics.wpilibkinematics.SwerveDriveKinematics;
import com.arcrobotics.ftclib.kinematics.wpilibkinematics.SwerveModuleState;

public class SwerveKinematics {

    private final Translation2d frontRightPos;
    private final Translation2d backLeftPos;
    private final SwerveDriveKinematics kinematics;

    public SwerveKinematics(double wheelDistance) {
        frontRightPos = new Translation2d(wheelDistance, wheelDistance);
        backLeftPos = new Translation2d(-wheelDistance, -wheelDistance);
        kinematics = new SwerveDriveKinematics(frontRightPos, backLeftPos);
    }

    public SwerveModuleState[] calculate(Vector2d translation, double rotation, boolean fieldCentric, Rotation2d heading) {
        if (fieldCentric) {
            translation = translation.rotateBy(-heading.getRadians());
        }

        ChassisSpeeds speeds = new ChassisSpeeds(
                translation.getX(),
                translation.getY(),
                rotation
        );

        SwerveModuleState[] states = kinematics.toSwerveModuleStates(speeds);
        SwerveDriveKinematics.normalizeWheelSpeeds(states, 1.0);

        return states;
    }
}
