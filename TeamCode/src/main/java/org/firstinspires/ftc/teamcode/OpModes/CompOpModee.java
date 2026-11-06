package org.firstinspires.ftc.teamcode.OpModes;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.PIDFCoefficients;
import com.qualcomm.robotcore.hardware.VoltageSensor;

import org.firstinspires.ftc.teamcode.Subsystems.Intake;
//can only be used with expansion hub
@TeleOp(name = "Competition OpMode", group = "ATeleOp")
public class CompOpModee extends LinearOpMode {
    private static final String INTAKE_MOTOR_NAME = "Intake";
    private static final String SHOOTER_NAME = "Shooter";
    private static final String KICKER_SERVO_NAME = "kicker";

    private static final double TARGET_RPM = 2200.0;
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

        frontLeftMotor.setDirection(DcMotor.Direction.REVERSE);
        backLeftMotor.setDirection(DcMotor.Direction.FORWARD);
        frontLeftMotor.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        frontRightMotor.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        backLeftMotor.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        backRightMotor.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);

        Intake intake = new Intake(hardwareMap, INTAKE_MOTOR_NAME);
        DcMotorEx shooter = hardwareMap.get(DcMotorEx.class, SHOOTER_NAME);
        CRServo kickerServo = hardwareMap.get(CRServo.class, KICKER_SERVO_NAME);
        VoltageSensor battery = hardwareMap.voltageSensor.iterator().next();

        shooter.setDirection(DcMotor.Direction.FORWARD);
        shooter.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.FLOAT);
        shooter.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        shooter.setVelocityPIDFCoefficients(PID_P, PID_I, PID_D, PID_F);
        shooter.setVelocity(0.0);
        kickerServo.setPower(0.0);

        telemetry.addLine("Comp ready");
        telemetry.addLine(" Left Stick For Movement, Right Stick For Rotation");
        telemetry.addLine("Hold right trigger forward, Left trigger reverse");
        telemetry.addLine("Shooter: A To Run Shooter At 2200 RPM, B To Run Kicker, START To Stop All");
        telemetry.addData("Intake speed", intake.getSpeed());
        telemetry.addData("Configured PIDF", "%.4f / %.4f / %.4f / %.4f",
                PID_P, PID_I, PID_D, PID_F);
        telemetry.update();

        waitForStart();

        boolean shooterEnabled = false;
        boolean previousA = false;
        boolean previousStart = false;
        boolean previousB = false;

        while (opModeIsActive()) {
            // Mecanum drive
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

            // Intake
            if (gamepad1.right_trigger > 0.1) {
                intake.run();
            } else if (gamepad1.left_trigger > 0.1) {
                intake.reverse();
            } else {
                intake.stop();
            }

            // Shooter and kicker controls use rising edges so a held button acts once.
            boolean a = gamepad1.a;
            boolean start = gamepad1.start;
            boolean b = gamepad1.b;

            if (a && !previousA) {
                shooterEnabled = !shooterEnabled;
            }
            if (start && !previousStart) {
                shooterEnabled = false;
                kickerServo.setPower(0.0);
            }
            if (b && !previousB) {
                kickerServo.setPower(-1.0);
            }

            previousA = a;
            previousStart = start;
            previousB = b;

            double targetTicksPerSecond = TARGET_RPM * TICKS_PER_FLYWHEEL_REV / 60.0;
            shooter.setVelocity(shooterEnabled ? targetTicksPerSecond : 0.0);

            double measuredRPM = shooter.getVelocity() * 60.0 / TICKS_PER_FLYWHEEL_REV;
            PIDFCoefficients activePIDF = shooter.getPIDFCoefficients(
                    DcMotor.RunMode.RUN_USING_ENCODER);


            telemetry.addLine("---- INTAKE ----");
            String intakeState = gamepad1.right_trigger > 0.1 ? "Forward"
                    : gamepad1.left_trigger > 0.1 ? "Reverse" : "Stopped";
            double intakePower = gamepad1.right_trigger > 0.1 ? intake.getSpeed()
                    : gamepad1.left_trigger > 0.1 ? -intake.getSpeed() : 0.0;
            telemetry.addData("State", intakeState);
            telemetry.addData("Power", intakePower);

            telemetry.addLine("---- SHOOTER ----");
            telemetry.addData("Target RPM", "%.0f", shooterEnabled ? TARGET_RPM : 0.0);
            telemetry.addData("Measured RPM", "%.0f", measuredRPM);
            telemetry.addData("RPM Error", "%.0f",
                    (shooterEnabled ? TARGET_RPM : 0.0) - measuredRPM);
            telemetry.addData("Shooter power", "%.3f", shooter.getPower());
            telemetry.addData("Battery voltage", "%.2f V", battery.getVoltage());
            telemetry.addData("Encoder ticks/sec", "%.1f", shooter.getVelocity());
            telemetry.addData("Active PIDF", "%.6f / %.6f / %.6f / %.6f",
                    activePIDF.p, activePIDF.i, activePIDF.d, activePIDF.f);
            telemetry.update();
            idle();
        }

        intake.stop();
        shooter.setVelocity(0.0);
        kickerServo.setPower(0.0);
        frontLeftMotor.setPower(0.0);
        frontRightMotor.setPower(0.0);
        backLeftMotor.setPower(0.0);
        backRightMotor.setPower(0.0);
    }
}
