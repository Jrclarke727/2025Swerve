package org.firstinspires.ftc.teamcode.swerve;

import com.arcrobotics.ftclib.hardware.motors.MotorEx;
import com.qualcomm.robotcore.hardware.AnalogInput;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.CRServoImpl;
import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.telemetry.TelemetryPacket;

@Config
public class SwerveModule {

    private final MotorEx driveMotor;
    private final CRServoImpl pivotServo;  // CR servo
    private final AnalogInput encoder;
    private final double encoderOffset;

    // PID constants — tune these in FTC Dashboard
    public static double kP = 0.01;
    public static double kI = 0.005;
    public static double kD = 0.00025;
    public static double DEADZONE = 0.1;

    // Telemetry variables
    public static double currentError = 0.0;
    public static double targetAngle = 0.0;
    public static double actualAngle = 0.0;
    public static double servoPowerOutput = 0.0;

    // PID state
    private double integral = 0.0;
    private double lastError = 0.0;
    private long lastTime = System.nanoTime();

    private final FtcDashboard dashboard = FtcDashboard.getInstance();

    public SwerveModule(HardwareMap hardwareMap, String motorName, String servoName, String encoderName, double offset) {
        driveMotor = new MotorEx(hardwareMap, motorName);
        pivotServo = hardwareMap.get(CRServoImpl.class, servoName);
        encoder = hardwareMap.get(AnalogInput.class, encoderName);
        encoderOffset = offset;

        driveMotor.setRunMode(MotorEx.RunMode.VelocityControl);
    }

    public double getCurrentAngle() {
        double voltage = encoder.getVoltage();
        double angle = ((voltage / 3.3) * 360.0) - encoderOffset;
        return normalizeAngle(angle);
    }

    private double normalizeAngle(double angle) {
        return ((angle % 360) + 360) % 360;
    }

    private double normalizeDelta(double delta) {
        delta = normalizeAngle(delta);
        if (delta > 180) delta -= 360;
        return delta;
    }

    public void setTargetState(double speed, double angle) {
        double currentAngle = getCurrentAngle();
        double delta = normalizeDelta(angle - currentAngle);

        // Reverse wheel direction if needed
        if (Math.abs(delta) > 90) {
            angle = normalizeAngle(angle + 180);
            speed *= -1;
            delta = normalizeDelta(angle - currentAngle);
        }

        // PID calculations
        long currentTime = System.nanoTime();
        double dt = (currentTime - lastTime) / 1e9;  // seconds
        lastTime = currentTime;

        double error = delta;
        integral += error * dt;
        double derivative = (error - lastError) / dt;
        lastError = error;

        double servoPower = (kP * error) + (kI * integral) + (kD * derivative);

        // Clamp output
        servoPower = Math.max(-1.0, Math.min(1.0, servoPower));

        if (Math.abs(servoPower) < DEADZONE) {
            servoPower = 0;
        }

        // Dashboard telemetry
        targetAngle = angle;
        actualAngle = currentAngle;
        currentError = error;
        servoPowerOutput = servoPower;

        TelemetryPacket packet = new TelemetryPacket();
        packet.put("Target Angle", targetAngle);
        packet.put("Actual Angle", actualAngle);
        packet.put("Error", currentError);
        packet.put("Servo Power", servoPowerOutput);
        dashboard.sendTelemetryPacket(packet);

        pivotServo.setPower(servoPower);
        driveMotor.set(speed);
    }
}
