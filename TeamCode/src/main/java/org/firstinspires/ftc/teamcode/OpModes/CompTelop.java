package org.firstinspires.ftc.teamcode.OpModes;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.PIDFCoefficients;
import com.qualcomm.robotcore.hardware.VoltageSensor;

import org.firstinspires.ftc.teamcode.Subsystems.Intake;

/**  Can only be used with expansion hub
 * Intake is assigned to port 0 on the expansion hub, Shooter to port 1.
 */
@TeleOp(name = "CompTelop", group = "MAINOPMODES")
public class CompTelop extends LinearOpMode {
    private static final String INTAKE_MOTOR_NAME = "Intake";
    private static final String SHOOTER_NAME = "Nectar";
    private static final String KICKER_SERVO_NAME = "kicker";

    private static final long KICKER_DELAY_MS = 2500;
    private static final double KICKER_POWER = -1.0;

    private static final double PID_P = 0.0018; //.009
    private static final double PID_I = 0.0;
    private static final double PID_D = 0.0; //.00001
    private static final double PID_F = 0.005; //.0002

    private static final double WALL_SHOT = 2200.0;
    private static final double TICKS_PER_FLYWHEEL_REV = 28.0 * (3.0 / 4.0);


    @Override
    public void runOpMode() {
        // installization
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

        telemetry.addLine("Left Stick For Movement, Right Stick For Rotation");
        telemetry.addLine("Hold right trigger forward, Left trigger reverse");
        telemetry.addLine("A: Start shooter; kicker starts after 2 seconds");
        telemetry.addLine("START: Stop shooter and kicker");
        telemetry.addData("Intake speed", intake.getSpeed());
        telemetry.addData("Configured PIDF", "%.4f / %.4f / %.4f / %.4f",
                PID_P, PID_I, PID_D, PID_F);
        telemetry.update();

        waitForStart();
        boolean shooterEnabled = false;
        boolean previousA = false;
        boolean previousStart = false;
        long shooterStartTime = 0;

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

            // A starts the shooter process. START stops both mechanisms.
            boolean a = gamepad1.a;
            boolean start = gamepad1.start;

            if (a && !previousA && !shooterEnabled) {
                shooterEnabled = true;
                shooterStartTime = System.currentTimeMillis();
                kickerServo.setPower(0.0);
            }

            if (start && !previousStart) {
                shooterEnabled = false;
                kickerServo.setPower(0.0);
                shooter.setVelocity(0.0);
            }

            previousA = a;
            previousStart = start;

            double targetTicksPerSecond =
                    WALL_SHOT * TICKS_PER_FLYWHEEL_REV / 60.0;
            shooter.setVelocity(shooterEnabled ? targetTicksPerSecond : 0.0);

            // Runs the kicker after the shooter has been running for two seconds
            if (shooterEnabled
                    && System.currentTimeMillis() - shooterStartTime >= KICKER_DELAY_MS) {
                kickerServo.setPower(KICKER_POWER);
            }

            double measuredRPM =
                    shooter.getVelocity() * 60.0 / TICKS_PER_FLYWHEEL_REV;
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
            telemetry.addData("Process", shooterEnabled ? "Running" : "Stopped");
            telemetry.addData("Target RPM", "%.0f",
                    shooterEnabled ? WALL_SHOT : 0.0);
            telemetry.addData("Measured RPM", "%.0f", measuredRPM);
            telemetry.addData("RPM Error", "%.0f",
                    (shooterEnabled ? WALL_SHOT : 0.0) - measuredRPM);
            telemetry.addData("Kicker", shooterEnabled
                    && System.currentTimeMillis() - shooterStartTime >= KICKER_DELAY_MS
                    ? "Running" : "Waiting / Stopped");
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