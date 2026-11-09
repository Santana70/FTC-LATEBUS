package org.firstinspires.ftc.teamcode.TestOpModes;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.PIDFCoefficients;
import com.qualcomm.robotcore.hardware.VoltageSensor;

import org.firstinspires.ftc.teamcode.Subsystems.Intake;
// only to test the intake and shooter without drive input
@TeleOp(name = "IntakeNNectarShoot", group = "ATeleOp")
public class IntakeNNectarShoot extends LinearOpMode {
    private static final String INTAKE_MOTOR_NAME = "Intake";
    private static final String SHOOTER_NAME = "Nectar";
    private static final String KICKER_SERVO_NAME = "kicker";

    private static final double TARGET_RPM = 2200.0;
    private static final double TICKS_PER_FLYWHEEL_REV = 28.0 * (3.0 / 4.0);

    private static final double PID_P = 0.0018;
    private static final double PID_I = 0.0;
    private static final double PID_D = 0.0;
    private static final double PID_F = 0.005;

    @Override
    public void runOpMode() {
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

        telemetry.addLine("Intake and shooter ready");
        telemetry.addLine("Right trigger: intake forward; left trigger: reverse");
        telemetry.addLine("A: Toggle shooter; START: stop shooter and kicker; B: activate kicker");
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
            // Intake control
            if (gamepad1.right_trigger > 0.1) {
                intake.run();
            } else if (gamepad1.left_trigger > 0.1) {
                intake.reverse();
            } else {
                intake.stop();
            }

            // Shooter and kicker controls
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
            double shooterPower = shooter.getPower();
            double batteryVoltage = battery.getVoltage();
            PIDFCoefficients activePIDF = shooter.getPIDFCoefficients(
                    DcMotor.RunMode.RUN_USING_ENCODER);

            String intakeState = gamepad1.right_trigger > 0.1 ? "Forward"
                    : gamepad1.left_trigger > 0.1 ? "Reverse" : "Stopped";
            double intakePower = gamepad1.right_trigger > 0.1 ? intake.getSpeed()
                    : gamepad1.left_trigger > 0.1 ? -intake.getSpeed() : 0.0;

            telemetry.addLine("---- INTAKE ----");
            telemetry.addData("State", intakeState);
            telemetry.addData("Power", intakePower);

            telemetry.addLine("---- SHOOTER ----");
            telemetry.addData("Enabled", shooterEnabled);
            telemetry.addData("Target RPM", "%.0f", shooterEnabled ? TARGET_RPM : 0.0);
            telemetry.addData("Measured RPM", "%.0f", measuredRPM);
            telemetry.addData("Kicker Power", "%.1f", kickerServo.getPower());
            telemetry.addData("Active PIDF", "%.6f / %.6f / %.6f / %.6f",
                    activePIDF.p, activePIDF.i, activePIDF.d, activePIDF.f);

            telemetry.update();
            idle();
        }

        intake.stop();
        shooter.setVelocity(0.0);
        kickerServo.setPower(0.0);
    }
}
