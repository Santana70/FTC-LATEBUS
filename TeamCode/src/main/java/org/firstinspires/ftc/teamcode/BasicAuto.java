
package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.util.ElapsedTime;

@Autonomous(name = "Basic Auto", group = "Competition")
public class BasicAuto extends LinearOpMode {

    private RobotHardware hardware;
    private MecanumDrive drive;
    private Shooter shooter;
    private Intake intake;
    private Indexer indexer;

    @Override
    public void runOpMode() {

        // Initialize hardware
        hardware = new RobotHardware();
        hardware.init(hardwareMap);

        drive = new MecanumDrive(hardware);
        shooter = new Shooter(hardware.shooter);
        intake = new Intake(hardware.intake);
        indexer = new Indexer(hardware.indexer);

        telemetry.addLine("BASIC AUTO READY");
        telemetry.update();

        waitForStart();

        if (isStopRequested()) return;

        ElapsedTime timer = new ElapsedTime();

        // =================================
        // 1. SHOOTER ON + INDEXER FOR 10 SEC
        // =================================

        shooter.setTargetRPM(Constants.SHOOTER_TARGET_RPM);

        timer.reset();

        while (opModeIsActive() && timer.seconds() < 10.0) {

            shooter.update();
            indexer.forward();

            telemetry.addData("Shooter RPM", shooter.getRPM());
            telemetry.addData("Shooter Power", shooter.getPower());
            telemetry.update();

            idle();
        }

        // =================================
        // 2. TURN EVERYTHING OFF
        // =================================

        shooter.stop();
        indexer.stop();
        intake.stop();

        // =================================
        // 3. TURN LEFT FOR 0.25 SECONDS
        // =================================

        timer.reset();

        while (opModeIsActive() && timer.seconds() < 0.25) {

            drive.drive(0, 0, 0.3, 0.3);

            idle();
        }

        drive.drive(0, 0, 0, 0);

        // =================================
        // 4. DRIVE STRAIGHT FOR 5 SECONDS
        // =================================

        timer.reset();

        while (opModeIsActive() && timer.seconds() < 5.0) {

            drive.drive(0.4, 0, 0, 0.4);

            idle();
        }

        // =================================
        // 5. STOP EVERYTHING
        // =================================

        hardware.stopAll();
        drive.drive(0, 0, 0, 0);

        telemetry.addLine("AUTO COMPLETE");
        telemetry.update();
    }
}
