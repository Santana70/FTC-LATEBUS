package org.firstinspires.ftc.teamcode.Subsystems;

import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.HardwareMap;

// controls intake motor speed
public class Intake {
    private final DcMotor motor;
    private double speed = -.90;

    public Intake(HardwareMap hardwareMap, String Intake) {
        motor = hardwareMap.get(DcMotor.class, Intake);
        motor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
    }

    /** Sets intake speed as a fraction from 0.0 to 1.0. */
    public void setSpeed(double speed) {
        this.speed = Math.max(0.0, Math.min(1.0, speed));
    }

    public double getSpeed() {
        return speed;
    }

    public void run() {
        motor.setPower(-speed);
    }

    public void reverse() {
        motor.setPower(speed);
    }

    public void stop() {
        motor.setPower(0.0);
    }
}