package org.firstinspires.ftc.teamcode.TestOpModes;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;

//will rewrite soon with PIDF control
@TeleOp(name = "NectarPercentageShooter", group = "Test")
public class NectarPercentageShooter extends LinearOpMode {

    private DcMotor shooterMotor;

    @Override
    public void runOpMode() {
        shooterMotor = hardwareMap.get(DcMotor.class, "Nectar");

        shooterMotor.setDirection(DcMotor.Direction.FORWARD);

        telemetry.addData("Status", "Initialized");
        telemetry.update();

        waitForStart();

        while (opModeIsActive()) {
            if (gamepad1.start) {
                shooterMotor.setPower(0.0);
            } else if (gamepad1.a) {
                shooterMotor.setPower(.95);
            } else if (gamepad1.b) {
                shooterMotor.setPower(0.75);
            }

            // Show data on the Driver Station
            telemetry.addData("Shooter Power", shooterMotor.getPower());
            telemetry.update();
        }
    }
}


