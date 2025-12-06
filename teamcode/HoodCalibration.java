package org.firstinspires.ftc.teamcode;

import com.bylazar.configurables.annotations.Configurable;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.Servo;

@Configurable
@TeleOp
public class HoodCalibration extends OpMode {

    public static double hoodPos = 0;
    private Servo hood1, hood2;

    public static double MK1Offset = 0;
    public static double MK2Offset = 0;

    @Override
    public void init() {
        hood1 = hardwareMap.get(Servo.class, "hood1");
        hood2 = hardwareMap.get(Servo.class, "hood2");
        hood2.setDirection(Servo.Direction.REVERSE);
    }

    @Override
    public void loop() {
        hood1.setPosition(hoodPos + MK1Offset);
        hood2.setPosition(hoodPos + MK2Offset);
        telemetry.addData("Hood 1 POS", hood1.getPosition());
        telemetry.addData("Hood 2 POS", hood2.getPosition() );
        telemetry.update();
    }
}
