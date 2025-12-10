package org.firstinspires.ftc.teamcode.lib.mechanisms;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.ElapsedTime;

public class TransferSystemImpl implements TransferSystem {
    public static final double BLOCK_ANGLE = .5;
    public static final double CLEAR_ANGLE = .04;
    private final double STOP_SPEED = 0.5;
    private LinearOpMode opMode = null;
    private Servo transferGate;
    private GateStates gateState;
    private ElapsedTime timeElapsed = new ElapsedTime(ElapsedTime.Resolution.MILLISECONDS);
    private final double TIME_TO_CLOSE = 250;

    public TransferSystemImpl(LinearOpMode OpMode) {
        this.opMode = OpMode;


        this.transferGate = opMode.hardwareMap.get(Servo.class, "transferWheel");

        this.close();

    }
    @Override
    public void run() {

        transferGate.setDirection(Servo.Direction.FORWARD);

        if(opMode.gamepad2.a){
            this.open();
        }
        else if (timeElapsed.time() >= TIME_TO_CLOSE){
            this.close();
        }
    }

    public void stop() {
        transferGate.setPosition(STOP_SPEED);
    }

    private void close(){
        this.gateState = GateStates.CLOSED;
        this.transferGate.setPosition(BLOCK_ANGLE);
    }

    private void open(){
        this.gateState = GateStates.OPEN;
        this.transferGate.setPosition(CLEAR_ANGLE);
        this.timeElapsed.reset();
        this.timeElapsed.time();
    }
    private static enum GateStates{
        OPEN,
        CLOSED
    }
}