package org.firstinspires.ftc.teamcode.teleop;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.Servo;

@TeleOp(name = "mecanumDriveRed", group = "Competition")
public class mecanumDriveRed extends LinearOpMode {

    int ticks = 0;

    @Override
    public void runOpMode() {
        // Declare our motors
        // Make sure your ID's match your configuration
        DcMotor lF = hardwareMap.dcMotor.get("front_left");
        DcMotor lB = hardwareMap.dcMotor.get("back_left");
        DcMotor rF = hardwareMap.dcMotor.get("front_right");
        DcMotor rB = hardwareMap.dcMotor.get("back_right");
        DcMotor arm1 = hardwareMap.dcMotor.get("arm1");
        //DcMotor duckies = hardwareMap.dcMotor.get("duckies");
        Servo wrist1 = hardwareMap.servo.get("wrist");
        Servo grabber = hardwareMap.servo.get("grabber");


        // Reverse the right side motors
        // Reverse left motors if you are using NeveRests

        rF.setDirection(DcMotor.Direction.REVERSE);
        rB.setDirection(DcMotor.Direction.REVERSE);
        arm1.setDirection(DcMotor.Direction.REVERSE);

        // Set motors to brake when not in use
        lF.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        rF.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        lB.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        rB.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

        //reset encoders
        arm1.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);

        waitForStart();

        if (isStopRequested()) return;

        while (opModeIsActive()) {


            // hello there
            double rx = -gamepad1.right_stick_x; // Remember, this is reversed!
            double x = -gamepad1.left_stick_x * 1.1; // Counteract imperfect strafing
            double y = gamepad1.left_stick_y;


            double frontLeftPower = y + x + rx;
            double backLeftPower = y - x + rx;
            double frontRightPower = y - x - rx;
            double backRightPower = y + x - rx;

            // Put powers in the range of -1 to 1 only if they aren't already
            // Not checking would cause us to always drive at full speed
            if (Math.abs(frontLeftPower) > 1 || Math.abs(backLeftPower) > 1 ||
                    Math.abs(frontRightPower) > 1 || Math.abs(backRightPower) > 1) {
                // Find the largest power
                double max = 0;
                max = Math.max(Math.abs(frontLeftPower), Math.abs(backLeftPower));
                max = Math.max(Math.abs(frontRightPower), max);
                max = Math.max(Math.abs(backRightPower), max);

                // Divide everything by max (it's positive so we don't need to worry
                // about signs)
                frontLeftPower /= max;
                backLeftPower /= max;
                frontRightPower /= max;
                backRightPower /= max;
            }
            if (gamepad1.right_bumper) {
                lF.setPower(frontLeftPower * 0.25);
                lB.setPower(backLeftPower * 0.25);
                rF.setPower(frontRightPower * 0.25);
                rB.setPower(backRightPower * 0.25);
                telemetry.addLine("Speed one quarter");
                telemetry.update();
            } else if (gamepad1.left_bumper) {
                lF.setPower(frontLeftPower * 0.75);
                lB.setPower(backLeftPower * 0.75);
                rF.setPower(frontRightPower * 0.75);
                rB.setPower(backRightPower * 0.75);
                telemetry.addLine("Speed 3/4");
                telemetry.update();
            } else {
                lF.setPower(frontLeftPower);
                lB.setPower(backLeftPower);
                rF.setPower(frontRightPower);
                rB.setPower(backRightPower);
                telemetry.addLine("Speed full");
                telemetry.update();
            }


            if (gamepad2.a) {
                grabber.setPosition(0.7);
                telemetry.addLine("Open");
                telemetry.update();
            } else if (gamepad2.b) {
                grabber.setPosition(0.45);
                telemetry.addLine("Ball");
                telemetry.update();
            } else if (gamepad2.y) {
                grabber.setPosition(0.1);
                telemetry.addLine("Box");
                telemetry.update();
            }

            //set wrist position
           // wrist1.setPosition(gamepad2.right_trigger);

            //TODO
            //set wrist position option 2
            /*
            if(gamepad2.dpad_up){
                wrist1.setPosition(1);
            } else if(gamepad2.dpad_right){
                wrist1.setPosition(0.7);
            } else if(gamepad2.dpad_down){
                wrist1.setPosition(0.5);
            } else if(gamepad2.dpad_left){
                wrist1.setPosition(0.3);
            }
            */

            //TODO



            telemetry.addData("Arm Position", arm1.getCurrentPosition());


            ticks = ticks - (-(int) gamepad2.left_stick_y * 2);
            arm1.setTargetPosition(ticks);
            arm1.setMode(DcMotor.RunMode.RUN_TO_POSITION);
            telemetry.addData("ticks", ticks);

            if (gamepad1.x || gamepad2.x) {
              //  duckies.setPower(-0.8);
            } else {
               // duckies.setPower(0);
            }
        }
    }
}
