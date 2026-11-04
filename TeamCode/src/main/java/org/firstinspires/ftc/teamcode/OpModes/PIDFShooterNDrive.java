package org.firstinspires.ftc.teamcode.OpModes;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
// only can be used with expansion hub
@TeleOp(name = "CompOpMode", group = "TeleOp")
public class PIDFShooterNDrive extends LinearOpMode {
    private static final String SHOOTER_NAME = "shooter";
    private static final double TARGET_RPM = 2200.0;

    // 28 motor ticks/rev × 3/4 motor rev per flywheel rev.
    private static final double TICKS_PER_FLYWHEEL_REV = 28.0 * (3.0 / 4.0);

    @Override
    public void runOpMode() {
        DcMotor frontLeftMotor = hardwareMap.get(DcMotor.class, "frontLeft");
        DcMotor frontRightMotor = hardwareMap.get(DcMotor.class, "frontRight");
        DcMotor backLeftMotor = hardwareMap.get(DcMotor.class, "backLeft");
        DcMotor backRightMotor = hardwareMap.get(DcMotor.class, "backRight");
        DcMotorEx shooter = hardwareMap.get(DcMotorEx.class, SHOOTER_NAME);

        frontLeftMotor.setDirection(DcMotor.Direction.REVERSE);
        backLeftMotor.setDirection(DcMotor.Direction.REVERSE);

        shooter.setDirection(DcMotor.Direction.REVERSE);
        shooter.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.FLOAT);
        shooter.setMode(DcMotor.RunMode.RUN_USING_ENCODER);

        telemetry.addLine("Left stick: drive; right stick: turn/strafe.");
        telemetry.addLine("A starts the shooter; gamepad Start stops it.");
        telemetry.update();

        waitForStart();

        boolean shooterEnabled = false;
        boolean previousA = false;
        boolean previousStart = false;

        while (opModeIsActive()) {
            // Mecanum drive controls.
            double y = -gamepad1.left_stick_y;
            double x = gamepad1.left_stick_x;
            double rx = gamepad1.right_stick_x;

            double frontLeftPower = y + x + rx;
            double frontRightPower = y - x - rx;
            double backLeftPower = y - x + rx;
            double backRightPower = y + x - rx;

            double max = Math.max(1.0, Math.max(
                    Math.max(Math.abs(frontLeftPower), Math.abs(backLeftPower)),
                    Math.max(Math.abs(frontRightPower), Math.abs(backRightPower))));

            frontLeftMotor.setPower(frontLeftPower / max);
            frontRightMotor.setPower(frontRightPower / max);
            backLeftMotor.setPower(backLeftPower / max);
            backRightMotor.setPower(backRightPower / max);

            // A latches the shooter on; Start latches it off.
            boolean a = gamepad1.a;
            boolean start = gamepad1.start;
            if (a && !previousA) {
                shooterEnabled = true;
            }
            if (start && !previousStart) {
                shooterEnabled = false;
            }
            previousA = a;
            previousStart = start;

            double targetRpm = shooterEnabled ? TARGET_RPM : 0.0;
            double targetTicksPerSecond = targetRpm * TICKS_PER_FLYWHEEL_REV / 60.0;
            shooter.setVelocity(targetTicksPerSecond);

            telemetry.addData("Shooter enabled", shooterEnabled);
            telemetry.addData("Target RPM", "%.0f", targetRpm);
            telemetry.update();

            idle();
        }

        // Leave the robot in a stopped state when the OpMode exits.
        frontLeftMotor.setPower(0.0);
        frontRightMotor.setPower(0.0);
        backLeftMotor.setPower(0.0);
        backRightMotor.setPower(0.0);
        shooter.setVelocity(0.0);
    }
}
