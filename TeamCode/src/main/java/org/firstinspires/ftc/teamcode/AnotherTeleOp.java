package org.firstinspires.ftc.teamcode;

import com.pedropathing.Drivetrain;
import com.pedropathing.follower.Follower;
import com.pedropathing.geometry.Pose;
import com.qualcomm.hardware.gobilda.GoBildaPinpointDriver;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.AnalogInput;

import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;
import org.firstinspires.ftc.robotcore.external.navigation.Pose2D;
import org.firstinspires.ftc.teamcode.pedroPathing.Constants;
import org.firstinspires.ftc.teamcode.pedroPathing.TransferInfo;


@TeleOp(name="Red Teleop", group = "TYWLS26")
public class AnotherTeleOp extends LinearOpMode {

    private DcMotorEx shooter1, shooter2;
    GoBildaPinpointDriver pinpoint;

    private DcMotorEx intake;
    private DcMotor indexer;
    private CRServo turret;
    private AnalogInput turretPOT;
    public static Follower follower;
    private double turretOffset = 5.5;
    private double rawTurretZero = 193.9;
    private double testingRelativeTurretPosition = 15.0;


    static final double MAX_VOLTAGE = 3.3;
    static final double DEGREES_PER_REV = 360.0;

    private Pose redGoalPs = new Pose (86.2, 134.5, Math.toRadians(0));
    private Pose blueGoalPs = new Pose(34.9, 121.9, Math.toRadians(0));
    Pose blueStart = new Pose(33, 136, Math.toRadians(0));

    private boolean gamepadDown = false;

    double lastAngleDeg = 0.0;
    int turnCount = 0;

    // Call once at init
    void initEncoder(double initialVoltage) {
        lastAngleDeg = voltageToDegrees(initialVoltage);
        turnCount = 0;
    }

    // Call every loop
    public static double mapValue(double originalValue) {
        // Ensure the input value is within the expected bounds (optional, but good practice)
        if (originalValue < 0.0) {
            return 0.0;
        }
        if (originalValue > 3.3) {
            return 1.0;
        }

        // Perform the linear scaling
        return originalValue / 3.3;
    }
    double updateEncoder(double voltage) {
        double currentAngleDeg = voltageToDegrees(voltage);

        double delta = currentAngleDeg - lastAngleDeg;

        // Detect wrap-around
        if (delta > 180.0) {
            // Wrapped backwards (e.g. 5° → 355°)
            turnCount--;
        } else if (delta < -180.0) {
            // Wrapped forwards (e.g. 355° → 5°)
            turnCount++;
        }

        lastAngleDeg = currentAngleDeg;

        // Continuous angle (can grow positive or negative)
        return (turnCount * DEGREES_PER_REV) + currentAngleDeg;
    }
    public double getAbsoluteDeg () {
        return(turretPOT.getVoltage()/MAX_VOLTAGE)*DEGREES_PER_REV;
    }
    public void reset (){
        turnCount = 0;
        lastAngleDeg = getAbsoluteDeg();
    }

    // Utility
    double voltageToDegrees(double voltage) {
        return (voltage / MAX_VOLTAGE) * DEGREES_PER_REV;
    }

    @Override
    public void runOpMode() throws InterruptedException {

        shooter1 = hardwareMap.get(DcMotorEx.class, "shooter1");
        shooter2 = hardwareMap.get(DcMotorEx.class, "shooter2");
        shooter2.setDirection(DcMotorSimple.Direction.REVERSE);

        intake = hardwareMap.get(DcMotorEx.class, "intake");
        indexer = hardwareMap.get(DcMotorEx.class, "indexer");

        turret = hardwareMap.get(CRServo.class, "turret");
        turretPOT = hardwareMap.get(AnalogInput.class, "turretPOT");

        pinpoint = hardwareMap.get(GoBildaPinpointDriver.class, "pinpoint");

        pinpoint.setOffsets(-152.4, 69.85, DistanceUnit.MM); // Set your offsets in mm
        pinpoint.setEncoderResolution(GoBildaPinpointDriver.GoBildaOdometryPods.goBILDA_4_BAR_POD);
        pinpoint.setEncoderDirections(GoBildaPinpointDriver.EncoderDirection.FORWARD,
                GoBildaPinpointDriver.EncoderDirection.REVERSED);

        pinpoint.resetPosAndIMU();


        follower = Constants.createFollower(hardwareMap);
        follower.setStartingPose(blueStart.mirror());
        follower.update();

        follower.startTeleopDrive();

        turret.setPower(0);

        initEncoder(turretPOT.getVoltage());

        reset();

        waitForStart();




        while(opModeIsActive()){

            pinpoint.update();
            Pose2D pos = pinpoint.getPosition();

            double yaw = pos.getHeading(AngleUnit.DEGREES);

            double turretAngle = alignTurret(
                    follower.getPose().getX(),
                    follower.getPose().getY(),
                    yaw,
                    redGoalPs);
            double rawDeg = (turretPOT.getVoltage()/3.3)*360;

            double largeGearCurrentPosition = -(turnCount*52+(mapValue(turretPOT.getVoltage())*52)); // Negative because of CC rotation of servo




            // Get the current pose (includes x, y, and heading)

            // Get yaw in degrees

            // Display yaw
            telemetry.addData("Yaw (degrees)", yaw);
            telemetry.addData("X TICKS", pinpoint.getEncoderX());
            telemetry.addData("Y TICKS", pinpoint.getEncoderY());
            telemetry.addData("shooterAngle", turretAngle);
            telemetry.addData("is shooter working", turret.getPower());
            telemetry.addData("current turret ang", rawDeg);
            telemetry.addData("voltage", turretPOT.getVoltage());
            telemetry.addData("relative deg ", updateEncoder(turretPOT.getVoltage()));
            telemetry.addData("encoder to Large gear 'help me'", largeGearCurrentPosition);



//            if(gamepad1.left_bumper){
//                shooter1.setVelocity(1000);
//                shooter2.setVelocity(1000);
//            } else {
//                shooter1.setVelocity(0);
//                shooter2.setVelocity(0);
//            }
//
//            if(gamepad1.right_trigger > 0.5){
//                intake.setPower(1);
//            } else {
//                intake.setPower(0);
//            }
//
//            if(gamepad1.right_bumper){
//                indexe r.setPower(1);
//            } else {
//                indexer.setPower(0);
//            }
            if(gamepad1.right_bumper){
                gamepadDown = !gamepadDown;
            }

            if (gamepad1.right_bumper){
                if (largeGearCurrentPosition <= 90){
                    turret.setPower(-1);
                } else {
                    turret.setPower(0);
                }
            }

//            follower.setTeleOpDrive(-gamepad1.left_stick_y, -gamepad1.left_stick_x, -gamepad1.right_stick_x, false);
            telemetry.update();
            follower.update();
        }
    }
    public double alignTurret(double x, double y, double headingDeg, Pose target) {
        double dx = target.getX() - x;
        double dy = target.getY() - y;
        // angle from robot to target
        double angleToGoal = Math.toDegrees(Math.atan2(dy, dx));
        // turret angle = angle to goal minus robot heading
        double turretAngle = angleToGoal - headingDeg;
        return  turretAngle + turretOffset;
    }


}