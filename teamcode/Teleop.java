package org.firstinspires.ftc.teamcode;

import com.pedropathing.Drivetrain;
import com.pedropathing.follower.Follower;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.Servo;

import org.firstinspires.ftc.teamcode.pedroPathing.Constants;
import org.firstinspires.ftc.teamcode.pedroPathing.TransferInfo;

@TeleOp(name="Main Teleop", group = "TYWLS26")
public class Teleop extends LinearOpMode {

    private DcMotorEx fl, fr, bl, br;
    private DcMotorEx shooter1, shooter2;
    private DcMotorEx intake;
    private DcMotor indexer;
    private Servo Hood1;
    private Servo Hood2;
    private Servo Turret;

    private Follower follower;

    @Override
    public void runOpMode() throws InterruptedException {

        shooter1 = hardwareMap.get(DcMotorEx.class, "shooter1");
        shooter2 = hardwareMap.get(DcMotorEx.class, "shooter2");
        shooter2.setDirection(DcMotorSimple.Direction.REVERSE);

        follower = Constants.createFollower(hardwareMap);
        follower.startTeleopDrive();

        waitForStart();

        while(opModeIsActive()){

            if(gamepad1.right_bumper){
                shooter1.setVelocity(1000);
                shooter2.setVelocity(1000);
            } else {
                shooter1.setVelocity(0);
                shooter2.setVelocity(0);
            }


            follower.setTeleOpDrive(-gamepad1.left_stick_y, -gamepad1.left_stick_x, -gamepad1.right_stick_x, 180);
            telemetry.addData("WORKED", TransferInfo.worked);
            telemetry.update();
        }
    }


}
