package org.firstinspires.ftc.teamcode.OpModes;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
// only can be used with expansion hub
@TeleOp(name = "ShootNDrive", group = "CompOpModes")
public class PercentShootNDrive extends LinearOpMode {
// not in use
    @Override
    public void runOpMode() {
        DcMotor frontLeftMotor = hardwareMap.get(DcMotor.class, "frontLeft");
        DcMotor frontRightMotor = hardwareMap.get(DcMotor.class, "frontRight");
        DcMotor backLeftMotor = hardwareMap.get(DcMotor.class, "backLeft");
        DcMotor backRightMotor = hardwareMap.get(DcMotor.class, "backRight");
        DcMotor shooterMotor = hardwareMap.get(DcMotor.class, "shooter");

        frontLeftMotor.setDirection(DcMotor.Direction.REVERSE);
        backLeftMotor.setDirection(DcMotor.Direction.REVERSE);

        shooterMotor.setDirection(DcMotor.Direction.REVERSE);
        shooterMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

        telemetry.addData("Status", "Initialized");
        telemetry.update();

        waitForStart();

        while (opModeIsActive()) {
            // Mecanum drive
            double y = -gamepad1.left_stick_y;
            double x = gamepad1.left_stick_x;
            double rx = gamepad1.right_stick_x;

            double frontLeftPower = y + x + rx;
            double frontRightPower = y - x - rx;
            double backLeftPower = y - x + rx;
            double backRightPower = y + x - rx;

            double max = Math.max(
                    1.0,
                    Math.max(
                            Math.max(Math.abs(frontLeftPower), Math.abs(backLeftPower)),
                            Math.max(Math.abs(frontRightPower), Math.abs(backRightPower))
                    )
            );

            frontLeftMotor.setPower(frontLeftPower / max);
            frontRightMotor.setPower(frontRightPower / max);
            backLeftMotor.setPower(backLeftPower / max);
            backRightMotor.setPower(backRightPower / max);

            // press either A or B to run at different speeds, press start to stop it
            if (gamepad1.a) {
                shooterMotor.setPower(0.85);
                if (gamepad1.b) {
                    shooterMotor.setPower(0.65);
                }
            } else if (gamepad1.start) {
                shooterMotor.setPower(0.0);
            }

            telemetry.addData("Shooter Power", shooterMotor.getPower());
            telemetry.update();
        }
    }
}