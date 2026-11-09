package org.firstinspires.ftc.teamcode.TestOpModes;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;

@TeleOp(name = "Pollen PIDF Tuner", group = "Test")
public class PollenPIDFTuner extends LinearOpMode {
    // Configure this name to match the pollen shooter motor in the Robot Controller.
    private static final String SHOOTER_NAME = "Pollen";

    private static final double PRESET_TARGET_RPM = 3000.0;
    private static final double RPM_STEP = 100.0;
    private static final double RPM_TOLERANCE = 50.0;
    private static final double MAX_TARGET_RPM = 3000.0;

    // HD Hex encoder: 28 ticks per motor-shaft revolution and 20:1 internal gearbox.
    // The external motor-to-flywheel gear ratio is 1:1.
    // Confirm this against your motor configuration and encoder readings.
    private static final double MOTOR_TICKS_PER_SHAFT_REV = 28.0;
    private static final double MOTOR_GEAR_REDUCTION = 20.0;
    private static final double MOTOR_REVS_PER_FLYWHEEL_REV = 1.0;
    private static final double TICKS_PER_FLYWHEEL_REV =
            MOTOR_TICKS_PER_SHAFT_REV * MOTOR_GEAR_REDUCTION
                    / MOTOR_REVS_PER_FLYWHEEL_REV;

    // Tune these for the pollen shooter motor and mechanism.
    private static final boolean USE_CUSTOM_PIDF = true;
    // test P and F values first before applying D, but main focus on P.
    private static final double PID_P = 0.0015; //.009
    private static final double PID_I = 0.0;
    private static final double PID_D = 0.0; //.00001
    private static final double PID_F = 0.005; //.0002

    @Override
    public void runOpMode() {
        DcMotorEx shooter = hardwareMap.get(DcMotorEx.class, SHOOTER_NAME);
        shooter.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.FLOAT);
        shooter.setMode(DcMotor.RunMode.RUN_USING_ENCODER);

        if (USE_CUSTOM_PIDF) {
            shooter.setVelocityPIDFCoefficients(PID_P, PID_I, PID_D, PID_F);
        }

        double targetRpm = 0.0;
        telemetry.addLine("Pollen Shooter PIDF Tuner");
        telemetry.addLine("A: Toggle shooter | B: Set preset RPM");
        telemetry.addLine("D-pad up/down: adjust by 100 RPM | START: stop and reset");
        telemetry.addData("Preset target RPM", "%.0f", PRESET_TARGET_RPM);
        telemetry.addData("Ticks per flywheel revolution", "%.1f", TICKS_PER_FLYWHEEL_REV);
        telemetry.addData("Custom PIDF enabled", USE_CUSTOM_PIDF);
        telemetry.update();

        waitForStart();
        boolean shooterEnabled = false;
        boolean previousA = false;
        boolean previousB = false;
        boolean previousStart = false;
        boolean previousUp = false;
        boolean previousDown = false;

        while (opModeIsActive()) {
            boolean a = gamepad1.a;
            boolean b = gamepad1.b;
            boolean start = gamepad1.start;
            boolean up = gamepad1.dpad_up;
            boolean down = gamepad1.dpad_down;

            if (a && !previousA) {
                shooterEnabled = !shooterEnabled;
                if (shooterEnabled && targetRpm == 0.0) {
                    targetRpm = PRESET_TARGET_RPM;
                } else if (!shooterEnabled) {
                    targetRpm = 0.0;
                }
            }
            if (b && !previousB) {
                targetRpm = PRESET_TARGET_RPM;
                shooterEnabled = true;
            }
            if (up && !previousUp) {
                targetRpm = Math.min(MAX_TARGET_RPM, targetRpm + RPM_STEP);
                shooterEnabled = targetRpm > 0.0;
            }
            if (down && !previousDown) {
                targetRpm = Math.max(0.0, targetRpm - RPM_STEP);
                if (targetRpm == 0.0) shooterEnabled = false;
            }
            if (start && !previousStart) {
                targetRpm = 0.0;
                shooterEnabled = false;
            }

            previousA = a;
            previousB = b;
            previousStart = start;
            previousUp = up;
            previousDown = down;

            double targetTicksPerSecond = targetRpm * TICKS_PER_FLYWHEEL_REV / 60.0;
            shooter.setVelocity(shooterEnabled ? targetTicksPerSecond : 0.0);

            double measuredRpm = shooter.getVelocity() * 60.0 / TICKS_PER_FLYWHEEL_REV;
            double errorRpm = targetRpm - measuredRpm;
            telemetry.addData("Shooter enabled", shooterEnabled);
            telemetry.addData("Target RPM", "%.0f", shooterEnabled ? targetRpm : 0.0);
            telemetry.addData("Measured RPM", "%.0f", measuredRpm);
            telemetry.addData("RPM error", "%.0f", errorRpm);
            telemetry.addData("At target speed", shooterEnabled && Math.abs(errorRpm) <= RPM_TOLERANCE);
            telemetry.addData("Motor velocity (ticks/sec)", "%.1f", shooter.getVelocity());
            telemetry.addLine("A: Toggle | B: Preset RPM | START: Stop/reset");
            telemetry.addLine("D-pad up/down: +/- 100 RPM");
            telemetry.update();
            idle();
        }

        shooter.setVelocity(0.0);
    }
}


