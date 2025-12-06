package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

import org.firstinspires.ftc.teamcode.pedroPathing.TransferInfo;

@Autonomous(name = "Main Auton", group = "TYWLS26")
public class TwylsAuton extends LinearOpMode {

    @Override
    public void runOpMode() throws InterruptedException {
        TransferInfo.worked = true;


    }
}
