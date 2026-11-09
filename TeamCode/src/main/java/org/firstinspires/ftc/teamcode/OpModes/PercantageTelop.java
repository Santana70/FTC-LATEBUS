package org.firstinspires.ftc.teamcode.OpModes;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.DcMotor;

import org.firstinspires.ftc.teamcode.Subsystems.Intake;
// will add pollen shooter soon
@TeleOp(name = "Percentage Telop", group = "MAINOPMODES")
public class PercantageTelop extends LinearOpMode {
    private static final String INTAKE_MOTOR_NAME = "Intake";
    private static final String KICKER_SERVO_NAME = "kicker";
    private static final long KICKER_DELAY_MS = 1000;
    private static final double KICKER_POWER = -1.0;
    private static final double SHOOTER_HIGH_POWER = 0.85;
    private static final double SHOOTER_LOW_POWER = 0.65;

    @Override
    public void runOpMode() {
        DcMotor frontLeftMotor = hardwareMap.get(DcMotor.class, "frontLeft");
        DcMotor frontRightMotor = hardwareMap.get(DcMotor.class, "frontRight");
        DcMotor backLeftMotor = hardwareMap.get(DcMotor.class, "backLeft");
        DcMotor backRightMotor = hardwareMap.get(DcMotor.class, "backRight");
        DcMotor shooterMotor = hardwareMap.get(DcMotor.class, "Nectar");
        Intake intake = new Intake(hardwareMap, INTAKE_MOTOR_NAME);
        CRServo kickerServo = hardwareMap.get(CRServo.class, KICKER_SERVO_NAME);

        frontLeftMotor.setDirection(DcMotor.Direction.REVERSE);
        backLeftMotor.setDirection(DcMotor.Direction.REVERSE);
        shooterMotor.setDirection(DcMotor.Direction.REVERSE);
        shooterMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        kickerServo.setPower(0.0);

        telemetry.addLine("Left stick: drive; right stick: strafe/turn.");
        telemetry.addLine("A: shooter at 85%; B: shooter at 65%; START: stop.");
        telemetry.addLine("Triggers: intake forward/reverse; kicker starts after 2 seconds.");
        telemetry.addData("Intake speed", intake.getSpeed());
        telemetry.update();

        waitForStart();
        long shooterStartTime = 0;
        boolean shooterRunning = false;
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
                if (!shooterRunning) {
                    shooterRunning = true;
                    shooterStartTime = System.currentTimeMillis();
                }
                shooterMotor.setPower(SHOOTER_HIGH_POWER);
            } else if (gamepad1.b) {
                if (!shooterRunning) {
                    shooterRunning = true;
                    shooterStartTime = System.currentTimeMillis();
                }
                shooterMotor.setPower(SHOOTER_LOW_POWER);
            }

            boolean start = gamepad1.start;
            if (start && !previousStart) {
                shooterRunning = false;
                shooterMotor.setPower(0.0);
                kickerServo.setPower(0.0);
            }
            previousStart = start;

            boolean kickerRunning = shooterRunning
                    && System.currentTimeMillis() - shooterStartTime >= KICKER_DELAY_MS;
            kickerServo.setPower(kickerRunning ? KICKER_POWER : 0.0);

            String intakeState = gamepad1.right_trigger > 0.1 ? "Forward"
                    : gamepad1.left_trigger > 0.1 ? "Reverse" : "Stopped";
            telemetry.addData("Shooter power", shooterMotor.getPower());
            telemetry.addData("Shooter", shooterRunning ? "Running" : "Stopped");
            telemetry.addData("Intake", intakeState);
            telemetry.addData("Kicker", kickerRunning ? "Running" : "Waiting / Stopped");
            telemetry.update();
            idle();
        }

        intake.stop();
        shooterMotor.setPower(0.0);
        kickerServo.setPower(0.0);
        frontLeftMotor.setPower(0.0);
        frontRightMotor.setPower(0.0);
        backLeftMotor.setPower(0.0);
        backRightMotor.setPower(0.0);
    }
}
