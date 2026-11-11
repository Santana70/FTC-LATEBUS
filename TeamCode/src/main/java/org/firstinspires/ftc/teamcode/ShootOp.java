package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

@TeleOp(name = "ShootOp", group = "Competition")
public class ShootOp extends OpMode {

    private RobotHardware hardware;
    private Shooter shooter;

    private boolean shooterEnabled = false;


    @Override
    public void init() {

        hardware = new RobotHardware();
        hardware.init(hardwareMap);

        shooter = new Shooter(hardware.shooter);

        telemetry.addLine("COMPETITION TELEOP READY");
        telemetry.update();
    }


    @Override
    public void loop() {



        // ==========================================
        // DRIVER 2 - SHOOTER
        // ==========================================

        // X = shooter ON
        if (gamepad1.x) {

            shooterEnabled = true;

            shooter.setTargetRPM(
                    Constants.SHOOTER_TARGET_RPM
            );
        }


        // Y = shooter OFF
        if (gamepad1.y) {

            shooterEnabled = false;

            shooter.stop();
        }


        // Keep shooter at target RPM
        if (shooterEnabled) {

            shooter.update();
        }


        // ==========================================
        // TELEMETRY
        // ==========================================

        telemetry.addData(
                "Shooter Target RPM",
                "%.0f",
                shooter.getTargetRPM()
        );

        telemetry.addData(
                "Shooter RPM",
                "%.0f",
                shooter.getRPM()
        );

        telemetry.addData(
                "Shooter Power",
                "%.2f",
                shooter.getPower()
        );

        telemetry.addData(
                "Shooter At Speed",
                shooter.atSpeed(50)
        );


        telemetry.update();
    }


    @Override
    public void stop() {

        hardware.stopAll();
    }
}