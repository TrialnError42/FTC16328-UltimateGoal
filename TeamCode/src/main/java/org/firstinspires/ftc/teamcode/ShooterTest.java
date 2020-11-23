package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;

@Autonomous
public class ShooterTest extends AutoCommon {

    @Override
    public void runOpMode() {
        super.runOpMode();

        //drive out to the foundation
        moveShooter(500);
    }
}
