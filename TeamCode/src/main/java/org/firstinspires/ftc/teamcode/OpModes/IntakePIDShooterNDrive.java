package org.firstinspires.ftc.teamcode.OpModes;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;

@TeleOp(name = " IntakePIDShooterNDrive", group = "TeleOp")
public class IntakePIDShooterNDrive extends LinearOpMode {
    private static final String SHOOTER_NAME = "shooter";
    private static final String KICKER_SERVO_NAME = "kicker";
    private static final String INTAKE_NAME = "Intake";

    private static final double TARGET_RPM = 2200.0;
    // 28 motor encoder ticks/rev, with 3 motor revs for every 4 flywheel revs.
    private static final double TICKS_PER_FLYWHEEL_REV = 28.0 * (3.0 / 4.0);
    private static final double PID_P = 0.0018;
    private static final double PID_I = 0.0;
    private static final double PID_D = 0.0;
    private static final double PID_F = 0.005;

    @Override
    public void runOpMode() {
        DcMotor frontLeftMotor = hardwareMap.get(DcMotor.class, "frontLeft");
        DcMotor frontRightMotor = hardwareMap.get(DcMotor.class, "frontRight");
        DcMotor backLeftMotor = hardwareMap.get(DcMotor.class, "backLeft");
        DcMotor backRightMotor = hardwareMap.get(DcMotor.class, "backRight");
        DcMotorEx shooter = hardwareMap.get(DcMotorEx.class, SHOOTER_NAME);
        CRServo kicker = hardwareMap.get(CRServo.class, KICKER_SERVO_NAME);
        DcMotor intake = hardwareMap.get(DcMotor.class, INTAKE_NAME);

        frontLeftMotor.setDirection(DcMotor.Direction.REVERSE);
        backLeftMotor.setDirection(DcMotor.Direction.REVERSE);

        shooter.setDirection(DcMotor.Direction.FORWARD);
        shooter.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.FLOAT);
        shooter.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        shooter.setVelocityPIDFCoefficients(PID_P, PID_I, PID_D, PID_F);
        shooter.setVelocity(0.0);

        intake.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        kicker.setPower(0.0);

        telemetry.addLine("Left stick: drive; right stick: strafe/turn.");
        telemetry.addLine("A toggles shooter; gamepad Start stops shooter and kicker.");
        telemetry.addLine("B activates kicker; triggers run intake forward/reverse.");
        telemetry.addData("Shooter PIDF", "%.4f / %.4f / %.4f / %.4f",
                PID_P, PID_I, PID_D, PID_F);
        telemetry.update();

        waitForStart();

        boolean shooterEnabled = false;
        boolean kickerEnabled = false;
        boolean previousA = false;
        boolean previousStart = false;
        boolean previousB = false;

        while (opModeIsActive()) {
            // Mecanum drive.
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

            boolean a = gamepad1.a;
            boolean start = gamepad1.start;
            boolean b = gamepad1.b;

            if (a && !previousA) {
                shooterEnabled = !shooterEnabled;
            }
            if (start && !previousStart) {
                shooterEnabled = false;
                kickerEnabled = false;
            }
            if (b && !previousB) {
                kickerEnabled = true;
            }
            previousA = a;
            previousStart = start;
            previousB = b;

            double targetRpm = shooterEnabled ? TARGET_RPM : 0.0;
            double targetTicksPerSecond = targetRpm * TICKS_PER_FLYWHEEL_REV / 60.0;
            shooter.setVelocity(targetTicksPerSecond);
            kicker.setPower(kickerEnabled ? -1.0 : 0.0);

            String intakeState;
            double intakePower;
            if (gamepad1.right_trigger > 0.1) {
                intakePower = 1.0;
                intakeState = "Forward";
            } else if (gamepad1.left_trigger > 0.1) {
                intakePower = -1.0;
                intakeState = "Reverse";
            } else {
                intakePower = 0.0;
                intakeState = "Stopped";
            }
            intake.setPower(intakePower);

            double measuredRpm = shooter.getVelocity() * 60.0 / TICKS_PER_FLYWHEEL_REV;
            telemetry.addData("Shooter", shooterEnabled ? "On" : "Off");
            telemetry.addData("Target RPM", "%.0f", targetRpm);
            telemetry.addData("Measured RPM", "%.0f", measuredRpm);
            telemetry.addData("RPM error", "%.0f", targetRpm - measuredRpm);
            telemetry.addData("Kicker", kickerEnabled ? "On" : "Off");
            telemetry.addData("Intake", intakeState);
            telemetry.addData("Intake power", "%.2f", intakePower);
            telemetry.update();
            idle();
        }

        // Stop all mechanisms when the OpMode exits.
        frontLeftMotor.setPower(0.0);
        frontRightMotor.setPower(0.0);
        backLeftMotor.setPower(0.0);
        backRightMotor.setPower(0.0);
        shooter.setVelocity(0.0);
        kicker.setPower(0.0);
        intake.setPower(0.0);
    }
}
