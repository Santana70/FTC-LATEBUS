package org.firstinspires.ftc.teamcode.Autonomous;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.util.ElapsedTime;

@Autonomous(name = "ForwardDriveNTurn", group = "Auto")
public class ForwardDriveNTurn extends LinearOpMode {
// not as precise and needs exact postitioning to be used
    @Override
    public void runOpMode() {
        DcMotor frontLeft = hardwareMap.get(DcMotor.class, "frontLeft");
        DcMotor frontRight = hardwareMap.get(DcMotor.class, "frontRight");
        DcMotor backLeft = hardwareMap.get(DcMotor.class, "backLeft");
        DcMotor backRight = hardwareMap.get(DcMotor.class, "backRight");

        frontLeft.setDirection(DcMotor.Direction.REVERSE);
        backLeft.setDirection(DcMotor.Direction.REVERSE);

        waitForStart();

        if (opModeIsActive()) {
            ElapsedTime timer = new ElapsedTime();

            // Drive forward for 5 seconds
            frontLeft.setPower(0.65);
            frontRight.setPower(0.65);
            backLeft.setPower(0.65);
            backRight.setPower(0.65);

            timer.reset();
            while (opModeIsActive() && timer.seconds() < 1) {
                idle();
            }

            // Stop briefly before rotating
            stopMotors(frontLeft, frontRight, backLeft, backRight);
            sleep(500);

            // Rotate for 1 second
            if (opModeIsActive()) {
                frontLeft.setPower(0.65);
                frontRight.setPower(0.65);
                backLeft.setPower(-0.65);
                backRight.setPower(-0.65);

                timer.reset();
                while (opModeIsActive() && timer.seconds() < 1) {
                    idle();
                }
            }

            // Stop briefly after rotating
            stopMotors(frontLeft, frontRight, backLeft, backRight);
            sleep(500);

            // Drive forward again for 5 seconds
            if (opModeIsActive()) {
                frontLeft.setPower(0.65);
                frontRight.setPower(0.65);
                backLeft.setPower(0.65);
                backRight.setPower(0.65);

                timer.reset();
                while (opModeIsActive() && timer.seconds() < 1) {
                    idle();
                }
            }

            stopMotors(frontLeft, frontRight, backLeft, backRight);
        }
    }

    private void stopMotors(
            DcMotor frontLeft,
            DcMotor frontRight,
            DcMotor backLeft,
            DcMotor backRight) {
        frontLeft.setPower(0);
        frontRight.setPower(0);
        backLeft.setPower(0);
        backRight.setPower(0);
    }
}