
package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.hardware.CRServo;

public class Indexer {
    private final CRServo servo;

    public Indexer(CRServo servo) {
        this.servo = servo;
    }

    public void forward() {
        servo.setPower(1.0);
    }

    public void reverse() {
        servo.setPower(-1.0);
    }

    public void stop() {
        servo.setPower(0.0);
    }
}
