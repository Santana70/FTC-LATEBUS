
package org.firstinspires.ftc.teamcode.TestOpModes;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.VoltageSensor;
import com.qualcomm.robotcore.hardware.PIDFCoefficients;

@TeleOp(name = "ButtonRPM", group = "ATeleOp")
public class ButtonRPM extends LinearOpMode {

    private static final String SHOOTER_NAME = "shooter";
    private static final String KICKER_SERVO_NAME = "kicker";

    private static final double TARGET_RPM = 2200.0;

    private static final double TICKS_PER_FLYWHEEL_REV =
            28.0 * (3.0 / 4.0);

    // Custom PIDF values
    private static final double PID_P = 0.0018;
    private static final double PID_I = 0.0;
    private static final double PID_D = 0.0;
    private static final double PID_F = 0.005;

    @Override
    public void runOpMode() {

        DcMotorEx shooter =
                hardwareMap.get(DcMotorEx.class, SHOOTER_NAME);

        CRServo kickerServo =
                hardwareMap.get(CRServo.class, KICKER_SERVO_NAME);

        VoltageSensor battery =
                hardwareMap.voltageSensor.iterator().next();

        shooter.setDirection(DcMotor.Direction.FORWARD);
        shooter.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.FLOAT);
        shooter.setMode(DcMotor.RunMode.RUN_USING_ENCODER);

        // Apply custom PIDF values.
        shooter.setVelocityPIDFCoefficients(
                PID_P, PID_I, PID_D, PID_F
        );

        shooter.setVelocity(0.0);
        kickerServo.setPower(0.0);

        telemetry.addLine("ButtonRPM ready");
        telemetry.addLine("A: Toggle shooter");
        telemetry.addLine("START: Stop shooter and kicker");
        telemetry.addLine("B: Activate kicker");
        telemetry.addData("Configured PIDF",
                "%.4f / %.4f / %.4f / %.4f",
                PID_P, PID_I, PID_D, PID_F);
        telemetry.update();

        waitForStart();

        boolean shooterEnabled = false;
        boolean previousA = false;
        boolean previousStart = false;
        boolean previousB = false;

        while (opModeIsActive()) {

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

            double targetTicksPerSecond =
                    TARGET_RPM * TICKS_PER_FLYWHEEL_REV / 60.0;

            if (shooterEnabled) {
                shooter.setVelocity(targetTicksPerSecond);
            } else {
                shooter.setVelocity(0.0);
            }

            double measuredRPM =
                    shooter.getVelocity() * 60.0
                            / TICKS_PER_FLYWHEEL_REV;

            double shooterPower = shooter.getPower();
            double batteryVoltage = battery.getVoltage();

            // Read the actual active controller coefficients.
            PIDFCoefficients activePIDF =
                    shooter.getPIDFCoefficients(
                            DcMotor.RunMode.RUN_USING_ENCODER
                    );

            telemetry.addLine("---- SHOOTER ----");
            telemetry.addData("Enabled", shooterEnabled);
            telemetry.addData("Target RPM", "%.0f",
                    shooterEnabled ? TARGET_RPM : 0.0);
            telemetry.addData("Measured RPM", "%.0f", measuredRPM);
            telemetry.addData("RPM Error", "%.0f",
                    (shooterEnabled ? TARGET_RPM : 0.0) - measuredRPM);

            telemetry.addLine("---- MOTOR OUTPUT ----");
            telemetry.addData("Shooter Power", "%.3f", shooterPower);
            telemetry.addData("Battery Voltage", "%.2f V",
                    batteryVoltage);
            telemetry.addData("Encoder ticks/sec", "%.1f",
                    shooter.getVelocity());

            telemetry.addLine("---- ACTIVE PIDF ----");
            telemetry.addData("Active P", "%.6f", activePIDF.p);
            telemetry.addData("Active I", "%.6f", activePIDF.i);
            telemetry.addData("Active D", "%.6f", activePIDF.d);
            telemetry.addData("Active F", "%.6f", activePIDF.f);

            telemetry.update();
            idle();
        }

        shooter.setVelocity(0.0);
        kickerServo.setPower(0.0);
    }
}
