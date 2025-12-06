package org.firstinspires.ftc.teamcode.lib.mechanisms;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.Servo;

import org.firstinspires.ftc.robotcore.external.Telemetry;

public class LauncherImpl implements Runnable, Launcher {
    private static final double MANUAL_SPEED_HIGH = 1;
    private static final double MANUAL_SPEED_LOW = 0.6;
    private static final double ANGLE_3FT = 0.5;
    private static final double SPEED_3FT = 1;
    private static final double manualIncrementQuotient = 165; //Divides
    private final Telemetry telemetry;
    private LinearOpMode opMode = null;
    private DcMotor flywheelMotor = null;
    private Servo hoodServo = null;
    private double power = 0;
    private double hoodAngle = 0;
    private double triggerThreshold = 0.5;

    public LauncherImpl(LinearOpMode opMode) {

        this.opMode = opMode;
        this.telemetry = opMode.telemetry;

        this.flywheelMotor = opMode.hardwareMap.get(DcMotor.class, "flywheelMotor");
        this.hoodServo = opMode.hardwareMap.get(Servo.class, "hoodServo");

    }

//    public void start() {
//        Thread thread = new Thread(this);
//        thread.start();
//    }

    @Override
    public void run() {
        // while (this.opMode.opModeIsActive()) {

        // Flywheel motor
        // Not sure about the trigger and how this is going to be used on the gamepad
        this.flywheelMotor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        this.flywheelMotor.setDirection(DcMotorSimple.Direction.FORWARD);

        // MANUAL LOGIC

        if (this.opMode.gamepad2.right_trigger > 0 && this.opMode.gamepad2.right_trigger < triggerThreshold) {
            power = MANUAL_SPEED_LOW;
        } else if (this.opMode.gamepad2.right_trigger > 0 && this.opMode.gamepad2.right_trigger > triggerThreshold) {
            power = MANUAL_SPEED_HIGH;
        } else {
            power = 0;
        }

        if (opMode.gamepad2.left_stick_y != 0) {
            hoodAngle = hoodServo.getPosition() + (opMode.gamepad2.left_stick_y / manualIncrementQuotient);
            hoodAngle = Math.max(hoodAngle, 0);
            hoodAngle = Math.min(hoodAngle, 1);


            telemetry.addData("hoodAngle", hoodAngle);
            telemetry.addData("hoodservo", hoodServo.getPosition());
        }

        //AUTO LOGIC

        if (opMode.gamepad1.left_bumper || opMode.gamepad1.right_bumper) {
            //Set power to zero when the right trigger 2 is not pressed down
            power = 0;
            hoodAngle = ANGLE_3FT;
            if (this.opMode.gamepad2.right_trigger > 0) {
                power = SPEED_3FT;
            }
        }

        this.flywheelMotor.setPower(power);
        hoodServo.setPosition(hoodAngle);

        //yayayayaya good yes code josh waz here

//            try {
//                Thread.sleep(100);
//            } catch (InterruptedException e) {
//                throw new RuntimeException(e);
//            }


    }
}
//}
