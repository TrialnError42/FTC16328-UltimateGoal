package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;

@Autonomous
public class ParkOnLaunch extends AutoCommon {

    @Override
    public void runOpMode() {
        super.runOpMode();

        //drive out to the foundation
        //driveOnHeading(5,0.3,0);
       // sleep(2000);
        driveOnHeadingIntake(5, 0.1, 0.3,20,0,5000);
    }
}
