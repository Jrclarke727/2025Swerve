package org.firstinspires.ftc.teamcode.swerve;

import com.qualcomm.hardware.rev.RevHubOrientationOnRobot;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.IMU;
import com.qualcomm.robotcore.hardware.ImuOrientationOnRobot;
import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.YawPitchRollAngles;

import com.arcrobotics.ftclib.kinematics.wpilibkinematics.SwerveModuleState;
import com.arcrobotics.ftclib.geometry.Vector2d;
import com.arcrobotics.ftclib.geometry.Rotation2d;

@TeleOp(name = "Swerve TeleOp")
public class SwerveTeleOp extends LinearOpMode {


    private IMU imu;
    private SwerveKinematics kinematics;
    private SwerveModule module1, module2;

    @Override
    public void runOpMode() {
        // Initialize the IMU with orientation
        imu = hardwareMap.get(IMU.class, "imu");

        IMU.Parameters parameters = new IMU.Parameters(
                new RevHubOrientationOnRobot(
                        RevHubOrientationOnRobot.LogoFacingDirection.UP,
                        RevHubOrientationOnRobot.UsbFacingDirection.FORWARD
                )
        );
        imu.initialize(parameters);

        telemetry.addLine("IMU Initialization Started");
        telemetry.update();

        // Wait for start
        waitForStart();

        // Proceed after start
        telemetry.addLine("IMU Initialized (BHI260AP does not block for calibration)");
        telemetry.update();

        // Initialize swerve modules and kinematics
        module1 = new SwerveModule(hardwareMap, "motor1", "pivot1", "encoder1", 0);
        module2 = new SwerveModule(hardwareMap, "motor2", "pivot2", "encoder2", 0);
        kinematics = new SwerveKinematics(5.33); // replace with your actual wheel offset

        while (opModeIsActive()) {
            YawPitchRollAngles orientation = imu.getRobotYawPitchRollAngles();
            double headingRadians = orientation.getYaw();
            Rotation2d heading = new Rotation2d(headingRadians);

            Vector2d translation = new Vector2d(
                    -gamepad1.left_stick_y,
                    -gamepad1.left_stick_x
            );
            double rotation = -gamepad1.right_stick_x;

            SwerveModuleState[] states = kinematics.calculate(
                    translation, rotation, true, heading
            );



            // Apply target states
            module1.setTargetState(states[0].speedMetersPerSecond, states[0].angle.getDegrees());
            module2.setTargetState(states[1].speedMetersPerSecond, states[1].angle.getDegrees());

            // Telemetry for Module 1
            telemetry.addData("Module 1 - Current Angle", "%.2f°", module1.getCurrentAngle());
            telemetry.addData("Module 1 - Target Speed", "%.2f", states[0].speedMetersPerSecond);
            telemetry.addData("Module 1 - Target Angle", "%.2f°", states[0].angle.getDegrees());
            telemetry.addData("Module1 Angle (deg)", "%.2f", module1.getCurrentAngle());

            // Telemetry for Module 2
            telemetry.addData("Module 2 - Current Angle", "%.2f°", module2.getCurrentAngle());
            telemetry.addData("Module 2 - Target Speed", "%.2f", states[1].speedMetersPerSecond);
            telemetry.addData("Module 2 - Target Angle", "%.2f°", states[1].angle.getDegrees());
            telemetry.addData("Module2 Angle (deg)", "%.2f", module2.getCurrentAngle());

            // Also show robot heading
            telemetry.addData("Robot Heading (rad)", "%.3f", headingRadians);

            telemetry.update();
        }

    }
}