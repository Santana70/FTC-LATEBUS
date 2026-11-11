package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.hardware.DcMotorEx;

public class MecanumDrive {

    private final DcMotorEx frontLeft;
    private final DcMotorEx frontRight;
    private final DcMotorEx backLeft;
    private final DcMotorEx backRight;

    public MecanumDrive(RobotHardware hardware) {

        frontLeft = hardware.frontLeft;
        frontRight = hardware.frontRight;
        backLeft = hardware.backLeft;
        backRight = hardware.backRight;
    }

    public void drive(
            double forward,
            double strafe,
            double turn,
            double speed
    ) {

        double frontLeftPower =
                forward + strafe + turn;

        double frontRightPower =
                forward - strafe - turn;

        double backLeftPower =
                forward - strafe + turn;

        double backRightPower =
                forward + strafe - turn;


        // Normalize powers so none exceed 1.0
        double max = Math.max(
                1.0,
                Math.max(
                        Math.max(
                                Math.abs(frontLeftPower),
                                Math.abs(frontRightPower)
                        ),
                        Math.max(
                                Math.abs(backLeftPower),
                                Math.abs(backRightPower)
                        )
                )
        );

        frontLeftPower /= max;
        frontRightPower /= max;
        backLeftPower /= max;
        backRightPower /= max;


        // Apply speed multiplier
        frontLeft.setPower(frontLeftPower * speed);
        frontRight.setPower(frontRightPower * speed);
        backLeft.setPower(backLeftPower * speed);
        backRight.setPower(backRightPower * speed);
    }


    public void stop() {

        frontLeft.setPower(0);
        frontRight.setPower(0);
        backLeft.setPower(0);
        backRight.setPower(0);
    }
}