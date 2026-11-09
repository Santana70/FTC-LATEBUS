
package org.firstinspires.ftc.teamcode.TestOpModes;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
// testing PIDF control
@TeleOp(name = "Shooter PIDF Test", group = "Test")
public class NectarPIDFTuner extends LinearOpMode {

    private static final String SHOOTER_NAME = "shooter";

    private static final double TARGET_RPM = 2200.0;
    private static final double RPM_STEP = 100.0;
    private static final double RPM_TOLERANCE = 50.0;

    // Encoder resolution at the motor shaft.
    private static final double MOTOR_TICKS_PER_REV = 28.0;

    // 3 motor revolutions produce 4 flywheel revolutions.
    private static final double MOTOR_REVS_PER_FLYWHEEL_REV = 3.0 / 4.0;

    private static final double TICKS_PER_FLYWHEEL_REV =
            MOTOR_TICKS_PER_REV * MOTOR_REVS_PER_FLYWHEEL_REV;

    /*
     * Start with the hub's existing PIDF coefficients.
     * Tune using measured results before enabling custom coefficients.
     */
    private static final boolean USE_CUSTOM_PIDF = true;
    // test P and F values first before applying D, but main focus on P.

    private static final double PID_P = 0.0015; //.009
    private static final double PID_I = 0.0;
    private static final double PID_D = 0.0; //.00001
    private static final double PID_F = 0.005; //.0002

    @Override
    public void runOpMode() {
        DcMotorEx shooter =
                hardwareMap.get(DcMotorEx.class, SHOOTER_NAME);

        shooter.setDirection(DcMotor.Direction.REVERSE);
        shooter.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.FLOAT);
        shooter.setMode(DcMotor.RunMode.RUN_USING_ENCODER);

        if (USE_CUSTOM_PIDF) {
            shooter.setVelocityPIDFCoefficients(
                    PID_P, PID_I, PID_D, PID_F
            );
        }

        double targetRpm = 0.0;

        telemetry.addLine("Shooter PIDF Test");
        telemetry.addLine("A: Start/stop shooter");
        telemetry.addLine("D-pad up/down: Adjust target RPM");
        telemetry.addLine("B: Set target to 3000 RPM");
        telemetry.addLine("X: Stop shooter");
        telemetry.addData("Custom PIDF enabled", USE_CUSTOM_PIDF);
        telemetry.update();

        waitForStart();

        boolean shooterEnabled = false;
        boolean previousA = false;
        boolean previousB = false;
        boolean previousX = false;
        boolean previousUp = false;
        boolean previousDown = false;

        while (opModeIsActive()) {
            // Read button states.
            boolean a = gamepad1.a;
            boolean b = gamepad1.b;
            boolean x = gamepad1.x;
            boolean up = gamepad1.dpad_up;
            boolean down = gamepad1.dpad_down;

            // Toggle shooter with A.
            if (a && !previousA) {
                shooterEnabled = !shooterEnabled;
                if (!shooterEnabled) {
                    targetRpm = 0.0;
                    shooter.setVelocity(0.0);
                }
            }

            // Set target to 2200 RPM with B.
            if (b && !previousB) {
                targetRpm = TARGET_RPM;
                shooterEnabled = true;
            }

            // Stop immediately with X.
            if (x && !previousX) {
                targetRpm = 0.0;
                shooterEnabled = false;
                shooter.setVelocity(0.0);
            }

            // Adjust target in 100 RPM steps.
            if (up && !previousUp) {
                targetRpm = Math.min(5000.0, targetRpm + RPM_STEP);
                shooterEnabled = true;
            }

            if (down && !previousDown) {
                targetRpm = Math.max(0.0, targetRpm - RPM_STEP);
                if (targetRpm == 0.0) {
                    shooterEnabled = false;
                }
            }

            previousA = a;
            previousB = b;
            previousX = x;
            previousUp = up;
            previousDown = down;

            if (shooterEnabled && targetRpm > 0.0) {
                double targetTicksPerSecond =
                        targetRpm * TICKS_PER_FLYWHEEL_REV / 60.0;

                shooter.setVelocity(targetTicksPerSecond);
            } else {
                shooter.setVelocity(0.0);
            }

            double measuredRpm =
                    shooter.getVelocity() * 60.0
                            / TICKS_PER_FLYWHEEL_REV;

            double errorRpm = targetRpm - measuredRpm;

            telemetry.addData("Shooter enabled", shooterEnabled);
            telemetry.addData("Target RPM", "%.0f", targetRpm);
            telemetry.addData("Measured RPM", "%.0f", measuredRpm);
            telemetry.addData("RPM error", "%.0f", errorRpm);
            telemetry.addData(
                    "At target speed",
                    shooterEnabled
                            && Math.abs(errorRpm) <= RPM_TOLERANCE
            );
            telemetry.addData(
                    "Motor velocity (ticks/sec)",
                    "%.1f", shooter.getVelocity()
            );
            telemetry.addLine(
                    "A: Toggle | B: 3000 RPM | X: Stop"
            );
            telemetry.addLine("D-pad up/down: +/- 100 RPM");
            telemetry.update();

            idle();
        }

        shooter.setVelocity(0.0);
    }
}
