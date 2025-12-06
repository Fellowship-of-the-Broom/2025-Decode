package org.firstinspires.ftc.teamcode.production;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.HardwareMap;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.teamcode.lib.mechanisms.AprilTag;
import org.firstinspires.ftc.teamcode.lib.mechanisms.Chassis;
import org.firstinspires.ftc.teamcode.lib.mechanisms.FakeIntakeImpl;
import org.firstinspires.ftc.teamcode.lib.mechanisms.FakeLauncherImpl;
import org.firstinspires.ftc.teamcode.lib.mechanisms.FakeTransferSystemImpl;
import org.firstinspires.ftc.teamcode.lib.mechanisms.Intake;
import org.firstinspires.ftc.teamcode.lib.mechanisms.IntakeImpl;
import org.firstinspires.ftc.teamcode.lib.mechanisms.Launcher;
import org.firstinspires.ftc.teamcode.lib.mechanisms.LauncherImpl;
import org.firstinspires.ftc.teamcode.lib.mechanisms.TransferSystem;
import org.firstinspires.ftc.teamcode.lib.mechanisms.TransferSystemImpl;

public class Robot {

    private final LinearOpMode opMode;
    private final Chassis chassis;

    private final AprilTag aprilTag;
    private final HardwareMap hardwareMap;
    private final Telemetry telemetry;
    private final Launcher launcher;
    private final Intake intake;
    private final TransferSystem transferSystem;


    public Robot(LinearOpMode opMode, boolean useReal){
        this.hardwareMap = opMode.hardwareMap;
        this.telemetry = opMode.telemetry;
        this.opMode = opMode;

        aprilTag = new AprilTag (opMode);
        chassis = new Chassis (opMode, aprilTag);
        if (useReal) {
            intake = new IntakeImpl(opMode);
            launcher = new LauncherImpl(opMode);
            transferSystem = new TransferSystemImpl(opMode);
        }
        else {
            intake = new FakeIntakeImpl();
            launcher = new FakeLauncherImpl();
            transferSystem = new FakeTransferSystemImpl();
        }



    }
    public void init(AllianceColor allianceColor){
        chassis.init();
        aprilTag.init(allianceColor);

    }
    public void start(){
//        launcher.start();
//        intake.start();
//       // aprilTag.start();
//        chassis.start():
        while (this.opMode.opModeIsActive()){
            intake.run();
            chassis.run();
            launcher.run();
            transferSystem.run();

            try {
                Thread.sleep(50);
            } catch (InterruptedException e) {
                //throw new RuntimeException(e);
            }
        }

    }

    public void rollout() {
        this.chassis.moveRobot(0, .5,0);
        try {
            Thread.sleep(1000);
        } catch (InterruptedException e) {
            //throw new RuntimeException(e);
        }
        this.chassis.moveRobot(0,0,0);
    }
}
