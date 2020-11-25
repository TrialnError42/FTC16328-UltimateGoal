 package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.util.Range;

@TeleOp

public class RgSample extends LinearOpMode {
//    private DcMotor motorTest;
//    private DigitalChannel digitalTouch;
//    private DistanceSensor sensorColorRange;
//    private Servo servoTest;

    private RobotHardware robot;

    @Override

    public void runOpMode() {
        telemetry.addData("Status", "Initializing...");
        telemetry.update();

        robot = new RobotHardware(hardwareMap, false);

        telemetry.addData("Status", "Initialized");

        telemetry.update();
        waitForStart();

        telemetry.addData("Status", "Running");
        telemetry.update();

        // run until the end of the match (driver presses STOP)
        while (opModeIsActive()) {
            driveControl();
            shooterControl();
            shooterControl();
            intakeControl();
            storageControl();
            positionControl();
        }
    }

    // intake power increase and decrease
/*
    private boolean lastTriggerPress = true;
    private double motorSpeed = 1;
    private boolean isMotorOn = false;
    private void intakeControl() {
        if (gamepad1.right_bumper) {
            isMotorOn = true;
        } else if (gamepad1.right_trigger > 0.5) {
            isMotorOn = false;
        }

        if (gamepad1.dpad_right && !lastTriggerPress) {
            motorSpeed += 0.05;
        } else if (gamepad1.dpad_left && !lastTriggerPress){
            motorSpeed -= 0.05;
        }
        lastTriggerPress = gamepad1.dpad_left || gamepad1.dpad_right;

        if (motorSpeed < 0 ) {
            robot.motorIntake.setPower(0);
        } else if (motorSpeed > 1) {
            robot.motorIntake.setPower(1);
        }

        if (isMotorOn) {
            robot.motorIntake.setPower(motorSpeed);
        } else  {
            robot.motorIntake.setPower(0);
        }

        telemetry.addData("intakeSpeed", robot.motorIntake.getPower());
        telemetry.update();
    }
*/
    private void intakeControl() {
            if (gamepad1.right_bumper) {
                robot.motorIntake.setPower(1);
            } else if (gamepad1.right_trigger > 0.5) {
                robot.motorIntake.setPower(0);
            }
        }


    private void driveControl() {
        double scale = 0.3;
        if (gamepad1.left_bumper) {
            scale = 1.0;
        } else if (gamepad1.left_trigger > 0.5) {
            scale = 0.1;
        }

        double drive = -gamepad1.left_stick_y; // ?? why is this getting negated
        double strafe = gamepad1.left_stick_x;
        double turn = gamepad1.right_stick_x;
        robot.startMove(drive, strafe, turn, scale);
    }

    private boolean prevAPress = true;
    private boolean isStorageDown = true;
    private void storageControl() {
        if (gamepad1.a && !prevAPress) {
            isStorageDown = !isStorageDown;
            robot.setStorage(isStorageDown);

        }
        prevAPress = gamepad1.a;
}

    private int motorPosOffset = 0;

    private void shooterControl() {
        if (gamepad1.dpad_up) {
            robot.motorShooter.setPower(-1);
        } else if (gamepad1.dpad_down) {
            robot.motorShooter.setPower(0);
        }



    }
    private void positionControl() {
        if (gamepad1.b) {
            robot.startMove(0, 0, 90, 1);
        }
    }
}
