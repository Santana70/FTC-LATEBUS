package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.CRServo;
public class RobotHardware {

    // =========================
    // DRIVE MOTORS
    // =========================

    public DcMotorEx frontLeft;
    public DcMotorEx frontRight;
    public DcMotorEx backLeft;
    public DcMotorEx backRight;

    // =========================
    // MECHANISMS
    // =========================

    public DcMotorEx shooter;
    public DcMotorEx intake;

    public CRServo indexer;

    public void init(HardwareMap hardwareMap) {

        // Drive
        frontLeft = hardwareMap.get(DcMotorEx.class, "frontLeft");
        frontRight = hardwareMap.get(DcMotorEx.class, "frontRight");
        backLeft = hardwareMap.get(DcMotorEx.class, "backLeft");
        backRight = hardwareMap.get(DcMotorEx.class, "backRight");

        // Mechanisms
        shooter = hardwareMap.get(DcMotorEx.class, "shooter");
        intake = hardwareMap.get(DcMotorEx.class, "intake");

        indexer = hardwareMap.get(CRServo.class, "indexer");

        // =========================
        // MOTOR DIRECTIONS
        // =========================

        frontLeft.setDirection(DcMotor.Direction.FORWARD);
        backLeft.setDirection(DcMotor.Direction.FORWARD);

        frontRight.setDirection(DcMotor.Direction.REVERSE);
        backRight.setDirection(DcMotor.Direction.FORWARD);

        shooter.setDirection(DcMotor.Direction.FORWARD);
        intake.setDirection(DcMotor.Direction.FORWARD);


        // =========================
        // BRAKE MODE
        // =========================
//
//        setBrake(frontLeft);
//        setBrake(frontRight);
//        setBrake(backLeft);
//        setBrake(backRight);
//        setBrake(shooter);
//        setBrake(intake);


        // =========================
        // ENCODERS
        // =========================

        shooter.setMode(DcMotor.RunMode.RUN_USING_ENCODER);

        frontLeft.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        frontRight.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        backLeft.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        backRight.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
    }


    private void setBrake(DcMotorEx motor) {
        motor.setZeroPowerBehavior(
                DcMotor.ZeroPowerBehavior.BRAKE
        );
    }


    public void stopAll() {

        frontLeft.setPower(0);
        frontRight.setPower(0);
        backLeft.setPower(0);
        backRight.setPower(0);

        shooter.setPower(0);
        intake.setPower(0);
    }
}