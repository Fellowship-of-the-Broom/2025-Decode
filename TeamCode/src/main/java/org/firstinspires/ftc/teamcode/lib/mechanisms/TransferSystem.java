package org.firstinspires.ftc.teamcode.lib.mechanisms;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.Servo;

public class TransferSystem {
    public static final double TRANSFER_WHEEL_SPEED = .85;
    private final double STOP_SPEED = 0.5;
    private LinearOpMode opMode = null;



    private Servo transferWheel;

    public TransferSystem(LinearOpMode OpMode) {
        opMode = OpMode;

        this.transferWheel = opMode.hardwareMap.get(Servo.class, "transferWheel");
    }
    public void run() {
        if (opMode.gamepad2.a){
            this.forward(TRANSFER_WHEEL_SPEED);
        }
        if (opMode.gamepad2.left_bumper) {
            this.reject(TRANSFER_WHEEL_SPEED);
        }
    }

    public void forward(double speed) {
        transferWheel.setDirection(Servo.Direction.FORWARD);
        transferWheel.setPosition(speed);
    }

    public void reject(double speed) {
        transferWheel.setDirection(Servo.Direction.REVERSE);
        transferWheel.setPosition(speed);
    }



    public void stop() {
        transferWheel.setPosition(STOP_SPEED);
    }
}


