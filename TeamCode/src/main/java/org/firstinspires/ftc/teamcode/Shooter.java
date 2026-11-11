package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.hardware.DcMotorEx;

public class Shooter {

    private final DcMotorEx motor;

    private double targetRPM = 0;

    private double integral = 0;
    private double lastError = 0;

    private long lastTimeNanos = 0;


    public Shooter(DcMotorEx motor) {
        this.motor = motor;
    }


    // =========================
    // TARGET RPM
    // =========================

    public void setTargetRPM(double rpm) {
        targetRPM = Math.max(0, rpm);
    }

    public double getTargetRPM() {
        return targetRPM;
    }


    // =========================
    // RPM
    // =========================

    public double getMotorRPM() {

        double ticksPerSecond = motor.getVelocity();

        return (ticksPerSecond
                / Constants.SHOOTER_TICKS_PER_MOTOR_REV)
                * 60.0;
    }


    public double getRPM() {

        // Account for the 5:6 speed gearing.
        return getMotorRPM()
                / Constants.SHOOTER_GEAR_RATIO;
    }


    // =========================
    // POWER
    // =========================

    public double getPower() {
        return motor.getPower();
    }


    // =========================
    // UPDATE CONTROLLER
    // =========================

    public void update() {

        long now = System.nanoTime();

        if (lastTimeNanos == 0) {
            lastTimeNanos = now;
            lastError = 0;
        }

        double dt =
                (now - lastTimeNanos)
                        / 1_000_000_000.0;

        if (dt <= 0 || dt > 0.25) {
            dt = 0.02;
        }

        lastTimeNanos = now;


        // Shooter OFF
        if (targetRPM <= 0) {

            motor.setPower(0);

            integral = 0;
            lastError = 0;

            return;
        }


        double currentRPM = getRPM();

        double error =
                targetRPM - currentRPM;


        // Integral
        integral += error * dt;

        integral = clamp(
                integral,
                -1000,
                1000
        );


        // Derivative
        double derivative =
                (error - lastError) / dt;

        lastError = error;


        // PID + feedforward
        double output =
                Constants.SHOOTER_KF * targetRPM
                        + Constants.SHOOTER_KP * error
                        + Constants.SHOOTER_KI * integral
                        + Constants.SHOOTER_KD * derivative;


        output = clamp(
                output,
                -1.0,
                1.0
        );


        motor.setPower(output);
    }


    // =========================
    // AT SPEED
    // =========================

    public boolean atSpeed(double toleranceRPM) {

        return targetRPM > 0
                && Math.abs(
                targetRPM - getRPM()
        ) <= toleranceRPM;
    }


    // =========================
    // STOP
    // =========================

    public void stop() {

        targetRPM = 0;

        integral = 0;
        lastError = 0;

        motor.setPower(0);
    }


    private double clamp(
            double value,
            double min,
            double max
    ) {

        return Math.max(
                min,
                Math.min(max, value)
        );
    }
}