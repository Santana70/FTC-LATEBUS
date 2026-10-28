
package org.firstinspires.ftc.teamcode.TestOpModes;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;

@TeleOp(name = "ButtonRPM", group = "TeleOp")
public class ButtonRPM extends LinearOpMode {

    private static final String SHOOTER_NAME = "shooter";
    private static final String KICKER_SERVO_NAME = "kicker";

    private static final double TARGET_RPM = 2200.0;

    // 28 motor encoder ticks/rev, with 3 motor revs
    // for every 4 flywheel revolutions.
    private static final double TICKS_PER_FLYWHEEL_REV =
            28.0 * (3.0 / 4.0);

    @Override
    public void runOpMode() {

        // Make sure these names and device types match the Robot
        // Configuration in the Driver Station.
        DcMotorEx shooter =
                hardwareMap.get(DcMotorEx.class, SHOOTER_NAME);

        CRServo kickerServo =
                hardwareMap.get(CRServo.class, KICKER_SERVO_NAME);

        shooter.setDirection(DcMotor.Direction.FORWARD);
        shooter.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.FLOAT);
        shooter.setMode(DcMotor.RunMode.RUN_USING_ENCODER);

        // IMPORTANT:
        // Do not apply custom PIDF coefficients during this test.
        // Use the motor controller's existing velocity PIDF settings.

        shooter.setVelocity(0.0);
        kickerServo.setPower(0.0);

        telemetry.addLine("ButtonRPM ready");
        telemetry.addLine("Press A to start shooter");
        telemetry.addLine("Press START to stop shooter");
        telemetry.addLine("Press B to activate kicker");
        telemetry.update();

        waitForStart();

        boolean shooterEnabled = false;
        boolean previousA = false;
        boolean previousStart = false;
        boolean previousB = false;

        while (opModeIsActive()) {

            boolean a = gamepad1.a;
            boolean start = gamepad1.start;
            boolean b = gamepad1.b;

            // A toggles the shooter ON.
            if (a && !previousA) {
                shooterEnabled = true;
            }

            // START stops the shooter and kicker.
            if (start && !previousStart) {
                shooterEnabled = false;
                kickerServo.setPower(0.0);
            }
            // B activates the kicker.
            if (b && !previousB) {
                kickerServo.setPower(-1.0);
            }

            previousA = a;
            previousStart = start;
            previousB = b;

            double targetTicksPerSecond =
                    TARGET_RPM * TICKS_PER_FLYWHEEL_REV / 60.0;

            if (shooterEnabled) {
                shooter.setVelocity(targetTicksPerSecond);
            } else {
                shooter.setVelocity(0.0);
            }

            // Convert measured encoder velocity back to flywheel RPM.
            double measuredRPM =
                    shooter.getVelocity() * 60.0
                            / TICKS_PER_FLYWHEEL_REV;

            telemetry.addData("Shooter enabled", shooterEnabled);
            telemetry.addData("Target RPM", "%.0f", TARGET_RPM);
            telemetry.addData("Measured RPM", "%.0f", measuredRPM);
            telemetry.addData(
                    "RPM error", "%.0f", TARGET_RPM - measuredRPM);
            telemetry.addData(
                    "Encoder ticks/sec", "%.1f", shooter.getVelocity());
            telemetry.update();

            idle();
        }

        shooter.setVelocity(0.0);
        kickerServo.setPower(0.0);
    }
}
