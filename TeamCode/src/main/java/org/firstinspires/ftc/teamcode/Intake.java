package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.hardware.DcMotorEx;

public class Intake {

    private final DcMotorEx motor;

    public Intake(DcMotorEx motor) {
        this.motor = motor;
    }

    // Intake at 30% power
    public void forward() {
        motor.setPower(Constants.INTAKE_POWER);
    }

    // Reverse intake at 30% power
    public void reverse() {
        motor.setPower(-Constants.INTAKE_POWER);
    }

    // Stop intake
    public void stop() {
        motor.setPower(0);
    }

    public double getPower() {
        return motor.getPower();
    }
}