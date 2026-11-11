package org.firstinspires.ftc.teamcode.OpModes;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.PIDFCoefficients;
import com.qualcomm.robotcore.hardware.VoltageSensor;

import org.firstinspires.ftc.teamcode.Subsystems.Intake;

// Can only be used with expansion hub
@TeleOp(name = "CompTelop", group = "MAINOPMODES")
public class CompTelop extends LinearOpMode {
    private static final String INTAKE_MOTOR_NAME = "Intake";

    private static final String SHOOTER_NAME = "Nectar";
    private static final String POLLEN_SHOOTER_NAME = "Pollen";
    private static final String KICKER_SERVO_NAME = "Kicker";

    private static final double WALL_SHOT = 2200.0;
    private static final double TICKS_PER_FLYWHEEL_REV = 28.0 * (3.0 / 4.0);

    // Pollen shooter settings from PollenPIDFTuner.
    private static final double POLLEN_TARGET_RPM = 2000.0;
    // Bare HD Hex encoder at 28 ticks/motor revolution; motor-to-flywheel ratio is 1:1.
    private static final double POLLEN_TICKS_PER_FLYWHEEL_REV = 28.0;
    // Custom velocity PIDF coefficients; adjust each shooter independently while tuning
    private static final double NECTAR_PID_P = 0.0015;
    private static final double NECTAR_PID_I = 0.0;
    private static final double NECTAR_PID_D =  0.0005;
    private static final double NECTAR_PID_F = 32767.0 / (WALL_SHOT * TICKS_PER_FLYWHEEL_REV / 60.0);

    private static final double POLLEN_PID_P = 0.0015;
    private static final double POLLEN_PID_I = 0.0;
    private static final double POLLEN_PID_D = 0.0005;
    private static final double POLLEN_PID_F = 32767.0 / (POLLEN_TARGET_RPM * POLLEN_TICKS_PER_FLYWHEEL_REV / 60.0);
    private static final double KICKER_POWER = -1.0;

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
        DcMotorEx pollenShooter = hardwareMap.get(DcMotorEx.class, POLLEN_SHOOTER_NAME);
        CRServo kickerServo = hardwareMap.get(CRServo.class, KICKER_SERVO_NAME);
        VoltageSensor battery = hardwareMap.voltageSensor.iterator().next();

        shooter.setDirection(DcMotor.Direction.FORWARD);
        shooter.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.FLOAT);
        shooter.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        shooter.setVelocityPIDFCoefficients(
                NECTAR_PID_P, NECTAR_PID_I, NECTAR_PID_D, NECTAR_PID_F);
        shooter.setVelocity(0.0);

        pollenShooter.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.FLOAT);
        pollenShooter.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        pollenShooter.setVelocityPIDFCoefficients(
                POLLEN_PID_P, POLLEN_PID_I, POLLEN_PID_D, POLLEN_PID_F);
        pollenShooter.setVelocity(0.0);
        kickerServo.setPower(0.0);

        telemetry.addLine("Left stick: movement; right stick: rotation.");
        telemetry.addLine("Right trigger: intake forward; left trigger: reverse.");
        telemetry.addLine("A: Nectar shooter; Y: toggle Pollen shooter at 3000 RPM.");
        telemetry.addLine("B: Toggle kicker; START: stop both shooters and kicker.");
        telemetry.addData("Intake speed", intake.getSpeed());
        telemetry.update();

        waitForStart();
        boolean nectarEnabled = false;
        boolean pollenEnabled = false;
        boolean previousA = false;
        boolean previousY = false;
        boolean kickerEnabled = false;
        boolean previousB = false;
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

            boolean a = gamepad1.a;
            boolean yButton = gamepad1.y;
            boolean b = gamepad1.b;
            boolean start = gamepad1.start;

            if (a && !previousA && !nectarEnabled) {
                nectarEnabled = true;
            }
            if (yButton && !previousY) {
                pollenEnabled = !pollenEnabled;
            }
            if (b && !previousB) {
                kickerEnabled = !kickerEnabled;
            }
            if (start && !previousStart) {
                nectarEnabled = false;
                pollenEnabled = false;
                kickerEnabled = false;
            }
            previousA = a;
            previousY = yButton;
            previousB = b;
            previousStart = start;

            double nectarTicksPerSecond = WALL_SHOT * TICKS_PER_FLYWHEEL_REV / 60.0;
            shooter.setVelocity(nectarEnabled ? nectarTicksPerSecond : 0.0);
            double pollenTicksPerSecond = POLLEN_TARGET_RPM
                    * POLLEN_TICKS_PER_FLYWHEEL_REV / 60.0;
            pollenShooter.setVelocity(pollenEnabled ? pollenTicksPerSecond : 0.0);

            kickerServo.setPower(kickerEnabled ? KICKER_POWER : 0.0);

            double nectarMeasuredRpm = shooter.getVelocity() * 60.0 / TICKS_PER_FLYWHEEL_REV;
            double pollenMeasuredRpm = pollenShooter.getVelocity()
                    * 60.0 / POLLEN_TICKS_PER_FLYWHEEL_REV;
            PIDFCoefficients activePIDF = shooter.getPIDFCoefficients(
                    DcMotor.RunMode.RUN_USING_ENCODER);

            telemetry.addData("Nectar", nectarEnabled ? "Running" : "Stopped");
            telemetry.addData("Nectar target RPM", "%.0f", nectarEnabled ? WALL_SHOT : 0.0);
            telemetry.addData("Nectar measured RPM", "%.0f", nectarMeasuredRpm);
            telemetry.addData("Kicker", kickerEnabled ? "Running" : "Stopped");
            telemetry.addData("Pollen", pollenEnabled ? "Running" : "Stopped");
            telemetry.addData("Pollen target RPM", "%.0f", pollenEnabled ? POLLEN_TARGET_RPM : 0.0);
            telemetry.addData("Pollen measured RPM", "%.0f", pollenMeasuredRpm);
            telemetry.addData("Intake", gamepad1.right_trigger > 0.1 ? "Forward"
                    : gamepad1.left_trigger > 0.1 ? "Reverse" : "Stopped");
            telemetry.addData("Battery voltage", "%.2f V", battery.getVoltage());
            PIDFCoefficients pollenPIDF = pollenShooter.getPIDFCoefficients(DcMotor.RunMode.RUN_USING_ENCODER);
            telemetry.addData("Nectar PIDF active", "%.6f / %.6f / %.6f / %.6f",
                    activePIDF.p, activePIDF.i, activePIDF.d, activePIDF.f);
            telemetry.addData("Pollen PIDF active", "%.6f / %.6f / %.6f / %.6f",
                    pollenPIDF.p, pollenPIDF.i, pollenPIDF.d, pollenPIDF.f);
            telemetry.update();
            idle();
        }

        intake.stop();
        shooter.setVelocity(0.0);
        pollenShooter.setVelocity(0.0);
        kickerServo.setPower(0.0);
        frontLeftMotor.setPower(0.0);
        frontRightMotor.setPower(0.0);
        backLeftMotor.setPower(0.0);
        backRightMotor.setPower(0.0);
    }
}








