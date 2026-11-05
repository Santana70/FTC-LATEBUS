package org.firstinspires.ftc.teamcode.Autonomous;

import com.qualcomm.hardware.rev.RevHubOrientationOnRobot;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.IMU;

import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;

@Autonomous(name = "Encoder IMU Auto", group = "Auto")
public class EncoderImuAuto extends LinearOpMode {

    // Core Hex encoder: 288 counts per motor-shaft revolution.
    // A 2:1 speed gearbox makes the wheel turn twice per motor revolution.
    private static final double COUNTS_PER_MOTOR_REV = 288.0;
    private static final double DRIVE_GEAR_REDUCTION = 0.5;
    private static final double WHEEL_DIAMETER_INCHES = 4.0; // Change to your measured wheel diameter.
    private static final double COUNTS_PER_INCH =
            COUNTS_PER_MOTOR_REV * DRIVE_GEAR_REDUCTION
                    / (Math.PI * WHEEL_DIAMETER_INCHES);

    private static final double HEADING_KP = 0.015;

    private DcMotorEx frontLeft;
    private DcMotorEx frontRight;
    private DcMotorEx backLeft;
    private DcMotorEx backRight;
    private IMU imu;

    @Override
    public void runOpMode() {
        frontLeft = hardwareMap.get(DcMotorEx.class, "frontLeft");
        frontRight = hardwareMap.get(DcMotorEx.class, "frontRight");
        backLeft = hardwareMap.get(DcMotorEx.class, "backLeft");
        backRight = hardwareMap.get(DcMotorEx.class, "backRight");

        // Match the motor directions in your TeleOp.
        frontLeft.setDirection(DcMotor.Direction.REVERSE);
        backLeft.setDirection(DcMotor.Direction.REVERSE);

        imu = hardwareMap.get(IMU.class, "imu");

        // Assumes the Control Hub is flat, logo facing the robot's front,
        // and USB port facing the robot's back.
        RevHubOrientationOnRobot orientation = new RevHubOrientationOnRobot(
                RevHubOrientationOnRobot.LogoFacingDirection.FORWARD,
                RevHubOrientationOnRobot.UsbFacingDirection.FORWARD);
        imu.initialize(new IMU.Parameters(orientation));

        DcMotorEx[] motors = {frontLeft, frontRight, backLeft, backRight};

        for (DcMotorEx motor : motors) {
            motor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
            motor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
            motor.setTargetPosition(0);
            motor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        }

        telemetry.addLine("Ready");
        telemetry.addData("Counts per inch", COUNTS_PER_INCH);
        telemetry.update();

        waitForStart();
        if (isStopRequested()) return;

        imu.resetYaw();

        // Example: drive forward 24 inches, holding the starting heading.
        driveForward(24.0, 0.30, 0.0);

        stopDrive();

        // Leave motors in the usual open-loop mode when the auto movement ends.
        for (DcMotorEx motor : motors) {
            motor.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        }
    }

    private void driveForward(double inches, double maxPower, double targetHeading) {
        DcMotorEx[] motors = {frontLeft, frontRight, backLeft, backRight};
        int moveCounts = (int) (inches * COUNTS_PER_INCH);

        for (DcMotorEx motor : motors) {
            motor.setTargetPosition(motor.getCurrentPosition() + moveCounts);
            motor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        }

        double power = Math.abs(maxPower);
        double timeoutSeconds = Math.max(2.0, Math.abs(inches) / 4.0 + 2.0);
        double startTime = getRuntime();

        while (opModeIsActive()
                && (frontLeft.isBusy() || frontRight.isBusy()
                || backLeft.isBusy() || backRight.isBusy())
                && getRuntime() - startTime < timeoutSeconds) {

            double heading = imu.getRobotYawPitchRollAngles()
                    .getYaw(AngleUnit.DEGREES);
            double error = wrapDegrees(targetHeading - heading);
            double correction = error * HEADING_KP;

            // RUN_TO_POSITION uses positive power; heading correction adjusts
            // the left and right sides while encoders control distance.
            double leftPower = clip(power - correction, 0.0, 1.0);
            double rightPower = clip(power + correction, 0.0, 1.0);

            frontLeft.setPower(leftPower);
            backLeft.setPower(leftPower);
            frontRight.setPower(rightPower);
            backRight.setPower(rightPower);

            telemetry.addData("Heading", "%.1f", heading);
            telemetry.addData("Heading error", "%.1f", error);
            telemetry.addData("FL encoder", frontLeft.getCurrentPosition());
            telemetry.addData("FR encoder", frontRight.getCurrentPosition());
            telemetry.update();
        }

        stopDrive();
    }

    private double wrapDegrees(double degrees) {
        while (degrees > 180.0) degrees -= 360.0;
        while (degrees < -180.0) degrees += 360.0;
        return degrees;
    }

    private double clip(double value, double min, double max) {
        return Math.max(min, Math.min(max, value));
    }

    private void stopDrive() {
        frontLeft.setPower(0);
        frontRight.setPower(0);
        backLeft.setPower(0);
        backRight.setPower(0);
    }
}