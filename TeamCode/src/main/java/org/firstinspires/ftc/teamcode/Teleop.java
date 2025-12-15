package org.firstinspires.ftc.teamcode;

import com.pedropathing.Drivetrain;
import com.pedropathing.follower.Follower;
import com.pedropathing.geometry.Pose;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.hardware.IMU;

import org.firstinspires.ftc.teamcode.pedroPathing.Constants;
import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;
import org.firstinspires.ftc.robotcore.external.navigation.Pose2D;




@TeleOp(name="Blue Teleop", group = "TYWLS26")
public class Teleop extends LinearOpMode {

    private DcMotorEx shooter1, shooter2;
    private DcMotorEx intake;
    private DcMotor indexer;
    private Servo Turret;
    public static Follower follower;

    public int velocitySetter = 1000;
    @Override
    public void runOpMode() throws InterruptedException {

        shooter1 = hardwareMap.get(DcMotorEx.class, "shooter1");
        shooter2 = hardwareMap.get(DcMotorEx.class, "shooter2");
        shooter1.setDirection(DcMotorSimple.Direction.REVERSE);

        intake = hardwareMap.get(DcMotorEx.class, "intake");
        indexer = hardwareMap.get(DcMotorEx.class, "indexer");
        indexer.setDirection(DcMotorSimple.Direction.REVERSE);

        Turret = hardwareMap.get(Servo.class, "turret");


        follower = Constants.createFollower(hardwareMap);
        Pose blueStart = new Pose(33, 136, Math.toRadians(0));
        follower.setStartingPose(blueStart);
        follower.update();

        follower.startTeleopDrive();

        waitForStart();

        while(opModeIsActive()){

            if(gamepad1.right_bumper){
                shooter1.setVelocity(velocitySetter);
                shooter2.setVelocity(velocitySetter);
            } else {
                shooter1.setVelocity(0);
                shooter2.setVelocity(0);
            }

            if(gamepad1.right_trigger > 0.5){
                intake.setPower(1);
            } else {
                intake.setPower(0);
            }

            if(gamepad1.left_bumper){
                indexer.setPower(1);
            } else {
                indexer.setPower(0);
            }

            if(gamepad1.a){
                velocitySetter += 100;
            }
            if(gamepad1.b){
                velocitySetter -= 100;
            }

            telemetry.addData("Velocity", shooter1.getVelocity());
            follower.setTeleOpDrive(-gamepad1.left_stick_y, -gamepad1.left_stick_x, -gamepad1.right_stick_x, true,-180);
            telemetry.update();
            follower.update();
        }
    }


}
