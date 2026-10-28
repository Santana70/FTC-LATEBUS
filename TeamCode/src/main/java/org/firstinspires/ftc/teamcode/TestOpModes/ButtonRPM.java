
package org.firstinspires.ftc.teamcode.TestOpModes;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;

@TeleOp(name = "ButtonRPM", group = "TeleOp")
public class ButtonRPM extends LinearOpMode {

    private static final String SHOOTER_NAME = "shooter";
    private static final String KICKER_SERVO_NAME = "kicker";

    private static final double TARGET_RPM = 2200.0;

    // 28 motor encoder ticks/rev, with 3 motor revs
    // for every 4 flywheel revolutions.
    private static final double TICKS_PER_FLYWHEEL_REV =
            28.0 * (3.0 / 4.0);

    // PIDF values from ShooterPIDFTuner
    private static final double PID_P = 0.0015;
    private static final double PID_I = 0.0;
    private static final double PID_D = 0.0;
    private static final double PID_F = 0.005;

    @Override
    public void runOpMode() {

        DcMotorEx shooter =
                hardwareMap.get(DcMotorEx.class, SHOOTER_NAME);

        CRServo kickerServo =
                hardwareMap.get(CRServo.class, KICKER_SERVO_NAME);

        shooter.setDirection(DcMotor.Direction.FORWARD);
        shooter.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.FLOAT);
        shooter.setMode(DcMotor.RunMode.RUN_USING_ENCODER);

        // Apply custom velocity PIDF coefficients.
        // Temporarily use the hub's existing velocity PIDF.
        shooter.setMode(DcMotor.RunMode.RUN_USING_ENCODER);

        shooter.setVelocity(0.0);
        kickerServo.setPower(0.0);

        telemetry.addLine("ButtonRPM ready");
        telemetry.addLine("A: Start shooter");
        telemetry.addLine("START: Stop shooter");
        telemetry.addLine("B: Activate kicker");
        telemetry.addData(
                "PIDF",
                "P %.4f | I %.4f | D %.4f | F %.4f",
                PID_P, PID_I, PID_D, PID_F
        );
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

            // A toggles the shooter on.
            if (a && !previousA) {
                shooterEnabled = !shooterEnabled;
            }

            // START stops the shooter and kicker.
            if (start && !previousStart) {
                shooterEnabled = false;
                kickerServo.setPower(0.0);
            }

            // B activates the continuous rotation kicker servo.
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

            double measuredRPM =
                    shooter.getVelocity() * 60.0
                            / TICKS_PER_FLYWHEEL_REV;

            telemetry.addData("Shooter enabled", shooterEnabled);
            telemetry.addData(
                    "Target RPM", "%.0f",
                    shooterEnabled ? TARGET_RPM : 0.0
            );
            telemetry.addData("Measured RPM", "%.0f", measuredRPM);
            telemetry.addData(
                    "RPM error", "%.0f",
                    (shooterEnabled ? TARGET_RPM : 0.0) - measuredRPM
            );
            telemetry.addData(
                    "Encoder ticks/sec", "%.1f",
                    shooter.getVelocity()
            );

            telemetry.update();
            idle();
        }

        shooter.setVelocity(0.0);
        kickerServo.setPower(0.0);
    }
}
