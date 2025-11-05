package org.firstinspires.ftc.teamcode.lib.mechanisms;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;

public class Intake implements Runnable {
    private LinearOpMode opMode = null;
    private DcMotor motor = null;
    private final static double POWER = 0.75;
    public Intake(LinearOpMode OpMode) {
        opMode = OpMode;
        //this.motor = opMode.hardwareMap.get(DcMotor.class, "Jimmy");
        //Jimmy is a placeholder

    }

//    public void start() {
//        Thread thread = new Thread(this);
//        thread.start();
//    }

    @Override
    public void run() {

       // while (this.opMode.opModeIsActive()) {
            //1. read inputs
            //2. process
            //3. set outputs
           if (this.opMode.gamepad2.right_bumper) {
                this.motor.setDirection(DcMotorSimple.Direction.FORWARD);
                this.motor.setPower(POWER);

            } else if(this.opMode.gamepad2.left_bumper) {
                this.motor.setDirection(DcMotorSimple.Direction.REVERSE);
                this.motor.setPower(POWER);

            } else {
                this.motor.setPower(0);
            }
            try {
                Thread.sleep(100);
            } catch (InterruptedException e) {
               // throw new RuntimeException(e);
            }
        }
    }
//}
