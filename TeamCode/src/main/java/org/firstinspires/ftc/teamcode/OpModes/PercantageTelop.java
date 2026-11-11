package org.firstinspires.ftc.teamcode.OpModes;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.DcMotor;

import org.firstinspires.ftc.teamcode.Subsystems.Intake;

// Gamepad Y toggles the pollen shooter.
@TeleOp(name = "Percentage Telop", group = "MAINOPMODES")
public class PercantageTelop extends LinearOpMode {
    private static final String INTAKE_MOTOR_NAME = "Intake";
    private static final String KICKER_SERVO_NAME = "Kicker";
    private static final String POLLEN_MOTOR_NAME = "Pollen";
    private static final double POLLEN_POWER = -0.90;
    private static final double KICKER_POWER = -1.0;
    private static final double SHOOTER_HIGH_POWER = 0.99;
    private static final double SHOOTER_LOW_POWER = -0.85;

    @Override
    public void runOpMode() {
        DcMotor frontLeftMotor = hardwareMap.get(DcMotor.class, "frontLeft");
        DcMotor frontRightMotor = hardwareMap.get(DcMotor.class, "frontRight");
        DcMotor backLeftMotor = hardwareMap.get(DcMotor.class, "backLeft");
        DcMotor backRightMotor = hardwareMap.get(DcMotor.class, "backRight");
        DcMotor shooterMotor = hardwareMap.get(DcMotor.class, "Nectar");
        DcMotor pollenMotor = hardwareMap.get(DcMotor.class, POLLEN_MOTOR_NAME);
        Intake intake = new Intake(hardwareMap, INTAKE_MOTOR_NAME);
        CRServo kickerServo = hardwareMap.get(CRServo.class, KICKER_SERVO_NAME);

        frontLeftMotor.setDirection(DcMotor.Direction.REVERSE);
        backLeftMotor.setDirection(DcMotor.Direction.REVERSE);
        shooterMotor.setDirection(DcMotor.Direction.REVERSE);
        shooterMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        pollenMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        pollenMotor.setPower(0.0);
        kickerServo.setPower(0.0);

        telemetry.addLine("Left stick: drive; right stick: strafe/turn.");
        telemetry.addLine("A: shooter at 90%; X: shooter at 85%; START: stop.");
        telemetry.addLine("B: toggle kicker; Y: toggle pollen shooter at 95%.");
        telemetry.addLine("Triggers: intake forward/reverse.");
        telemetry.addData("Intake speed", intake.getSpeed());
        telemetry.update();

        waitForStart();
        boolean shooterRunning = false;
        boolean kickerRunning = false;
        boolean pollenRunning = false;
        boolean previousB = false;
        boolean previousY = false;
        boolean previousStart = false;

        while (opModeIsActive()) {
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

            if (gamepad1.right_trigger > 0.1) {
                intake.run();
            } else if (gamepad1.left_trigger > 0.1) {
                intake.reverse();
            } else {
                intake.stop();
            }

            if (gamepad1.a) {
                shooterRunning = true;
                shooterMotor.setPower(SHOOTER_HIGH_POWER);
            } else if (gamepad1.x) {
                shooterRunning = true;
                shooterMotor.setPower(SHOOTER_LOW_POWER);
            }

            boolean b = gamepad1.b;
            boolean start = gamepad1.start;
            boolean yButton = gamepad1.y;
            if (b && !previousB) {
                kickerRunning = !kickerRunning;
            }
            if (yButton && !previousY) {
                pollenRunning = !pollenRunning;
            }
            if (start && !previousStart) {
                shooterRunning = false;
                pollenRunning = false;
                kickerRunning = false;
                shooterMotor.setPower(0.0);
            }
            pollenMotor.setPower(pollenRunning ? POLLEN_POWER : 0.0);
            previousB = b;
            previousY = yButton;
            previousStart = start;

            kickerServo.setPower(kickerRunning ? KICKER_POWER : 0.0);

            String intakeState = gamepad1.right_trigger > 0.1 ? "Forward"
                    : gamepad1.left_trigger > 0.1 ? "Reverse" : "Stopped";
            telemetry.addData("Shooter power", shooterMotor.getPower());
            telemetry.addData("Shooter", shooterRunning ? "Running" : "Stopped");
            telemetry.addData("Intake", intakeState);
            telemetry.addData("Kicker", kickerRunning ? "Running" : "Stopped");
            telemetry.addData("Pollen shooter", pollenRunning ? "Running at 95%" : "Stopped");
            telemetry.update();
            idle();
        }

        intake.stop();
        shooterMotor.setPower(0.0);
        pollenMotor.setPower(0.0);
        kickerServo.setPower(0.0);
        frontLeftMotor.setPower(0.0);
        frontRightMotor.setPower(0.0);
        backLeftMotor.setPower(0.0);
        backRightMotor.setPower(0.0);
    }
}




