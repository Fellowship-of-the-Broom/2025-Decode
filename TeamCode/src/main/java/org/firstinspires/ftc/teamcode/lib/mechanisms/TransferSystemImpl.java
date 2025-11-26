package org.firstinspires.ftc.teamcode.lib.mechanisms;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.Servo;

public class TransferSystemImpl implements TransferSystem {
    public static final double BLOCK_ANGLE = .85;
    public static final double CLEAR_ANGLE = .85;
    private final double STOP_SPEED = 0.5;
    private LinearOpMode opMode = null;



    private Servo transferFinger;

    public TransferSystemImpl(LinearOpMode OpMode) {
        this.opMode = OpMode;

        this.transferFinger = opMode.hardwareMap.get(Servo.class, "transferWheel");
    }
    @Override
    public void run() {
       if(opmode.gamepad2.a);
    }
    public void stop() {
        transferFinger.setPosition(STOP_SPEED);
    }
}


