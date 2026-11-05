package org.firstinspires.ftc.teamcode.Autonomous;

import com.qualcomm.hardware.rev.RevHubOrientationOnRobot;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.IMU;

import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;

@Autonomous(name = "EncoderAutoFNT", group = "Auto")
public class EncoderAutoFNT extends LinearOpMode {

    // Core Hex encoder and 2:1 speed gearbox:
    // 288 motor counts per revolution; wheel turns twice per motor turn.
    private static final double COUNTS_PER_MOTOR_REV = 288.0;
    private static final double DRIVE_GEAR_REDUCTION = 0.5;
    private static final double WHEEL_DIAMETER_INCHES = 4.0; // Set to your measured wheel diameter.
    private static final double COUNTS_PER_INCH =
            COUNTS_PER_MOTOR_REV * DRIVE_GEAR_REDUCTION
                    / (Math.PI * WHEEL_DIAMETER_INCHES);

    private static final double HEADING_KP = 0.015;
    private static final double TURN_KP = 0.012;
    private static final double TURN_TOLERANCE_DEGREES = 2.0;

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

        // Same direction setup as your TeleOp.
        frontLeft.setDirection(DcMotor.Direction.REVERSE);
        backLeft.setDirection(DcMotor.Direction.FORWARD);

        imu = hardwareMap.get(IMU.class, "imu");

        // Hub flat, logo facing robot front, USB facing robot back.
        RevHubOrientationOnRobot hubOrientation = new RevHubOrientationOnRobot(
                RevHubOrientationOnRobot.LogoFacingDirection.UP,
                RevHubOrientationOnRobot.UsbFacingDirection.FORWARD);
        imu.initialize(new IMU.Parameters(hubOrientation));

        DcMotorEx[] motors = {frontLeft, frontRight, backLeft, backRight};
        for (DcMotorEx motor : motors) {
            motor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
            motor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
            motor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        }

        telemetry.addLine("Ready");
        telemetry.addData("Counts per inch", COUNTS_PER_INCH);
        telemetry.update();

        waitForStart();
        if (isStopRequested()) return;

        imu.resetYaw();

        // Sequence, similar to your timed example:
        driveForward(24.0, 0.90, 0.0);  // Forward 24 inches
        sleep(300);

        turnToHeading(90.0, 0.90);      // Turn clockwise to 90 degrees
        sleep(300);

        driveForward(24.0, 0.90, 0.0); // Forward 24 inches at the new heading

        stopDrive();
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
        double timeout = Math.max(2.0, Math.abs(inches) / 4.0 + 2.0);
        double startTime = getRuntime();

        while (opModeIsActive()
                && anyMotorBusy()
                && getRuntime() - startTime < timeout) {

            double heading = getHeading();
            double error = wrapDegrees(targetHeading - heading);
            double correction = error * HEADING_KP;

            // RUN_TO_POSITION takes positive power; adjust each side to correct heading.
            double leftPower = clip(power - correction, 0.05, 1.0);
            double rightPower = clip(power + correction, 0.05, 1.0);

            frontLeft.setPower(leftPower);
            backLeft.setPower(leftPower);
            frontRight.setPower(rightPower);
            backRight.setPower(rightPower);

            telemetry.addData("Drive heading", "%.1f", heading);
            telemetry.addData("Heading error", "%.1f", error);
            telemetry.update();
            idle();
        }

        stopDrive();
    }

    private void turnToHeading(double targetHeading, double maxPower) {
        double startTime = getRuntime();
        double timeout = 5.0;

        while (opModeIsActive() && getRuntime() - startTime < timeout) {
            double error = wrapDegrees(targetHeading - getHeading());

            if (Math.abs(error) <= TURN_TOLERANCE_DEGREES) {
                break;
            }

            double turnPower = clip(Math.abs(error) * TURN_KP, 0.10, maxPower);

            // Positive power on left and negative on right turns clockwise
            // with the motor directions used above. Reverse these signs if needed.
            double direction = Math.signum(error);
            frontLeft.setPower(direction * turnPower);
            backLeft.setPower(direction * turnPower);
            frontRight.setPower(-direction * turnPower);
            backRight.setPower(-direction * turnPower);

            telemetry.addData("Turn heading", "%.1f", getHeading());
            telemetry.addData("Target heading", "%.1f", targetHeading);
            telemetry.addData("Turn error", "%.1f", error);
            telemetry.update();
            idle();
        }

        stopDrive();
    }

    private boolean anyMotorBusy() {
        return frontLeft.isBusy() || frontRight.isBusy()
                || backLeft.isBusy() || backRight.isBusy();
    }

    private double getHeading() {
        return imu.getRobotYawPitchRollAngles().getYaw(AngleUnit.DEGREES);
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