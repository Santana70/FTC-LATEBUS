package org.firstinspires.ftc.teamcode.TestOpModes;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.teamcode.Subsystems.Intake;

@TeleOp(name = "IntakeTest", group = "Test")
public class IntakeTest extends LinearOpMode {
    // only for testing the intake by itself
    private static final String INTAKE_MOTOR_NAME = "Intake";

    @Override
    public void runOpMode() {
        Intake intake = new Intake(hardwareMap, INTAKE_MOTOR_NAME);

        telemetry.addLine("Hold right trigger for forward; hold left trigger for reverse.");
        telemetry.addData("Intake speed", intake.getSpeed());
        telemetry.update();

        waitForStart();
        while (opModeIsActive()) {
            if (gamepad1.right_trigger > 0.1) {
                intake.run();
            } else if (gamepad1.left_trigger > 0.1) {
                intake.reverse();
            } else {
                intake.stop();
            }

            String intakeState = gamepad1.right_trigger > 0.1
                    ? "Forward"
                    : gamepad1.left_trigger > 0.1 ? "Reverse" : "Stopped";
            double appliedPower = gamepad1.right_trigger > 0.1
                    ? intake.getSpeed()
                    : gamepad1.left_trigger > 0.1 ? -intake.getSpeed() : 0.0;
            telemetry.addData("Intake", intakeState);
            telemetry.addData("Power", appliedPower);
            telemetry.update();
            idle();
        }
        intake.stop();
    }
}
