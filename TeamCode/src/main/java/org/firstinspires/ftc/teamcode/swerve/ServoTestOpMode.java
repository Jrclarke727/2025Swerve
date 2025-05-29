package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.Servo;

@TeleOp(name = "Servo Test", group = "Test")
public class ServoTestOpMode extends LinearOpMode {

    private Servo testServo;

    // You can adjust these positions based on your servo's range and usage
    private final double POSITION_MIN = 0.0;
    private final double POSITION_MAX = 1.0;

    @Override
    public void runOpMode() {
        // Change "servoName" to match the name of your servo in the configuration
        testServo = hardwareMap.get(Servo.class, "servoName");

        telemetry.addData("Status", "Initialized");
        telemetry.update();

        waitForStart();

        while (opModeIsActive()) {
            if (gamepad1.a) {
                testServo.setPosition(POSITION_MIN);
            } else if (gamepad1.b) {
                testServo.setPosition(POSITION_MAX);
            }

            telemetry.addData("Servo Position", testServo.getPosition());
            telemetry.addData("Press A", "to move to MIN (%.2f)", POSITION_MIN);
            telemetry.addData("Press B", "to move to MAX (%.2f)", POSITION_MAX);
            telemetry.update();
        }
    }
}
