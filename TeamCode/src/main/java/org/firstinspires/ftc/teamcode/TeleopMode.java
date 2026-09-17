package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.ElapsedTime;

@TeleOp
public class TeleopMode extends LinearOpMode {
    private ElapsedTime runtime = new ElapsedTime();
    private DcMotor leftFrontDrive = null;
    private DcMotor leftBackDrive = null;
    private DcMotor rightFrontDrive = null;
    private DcMotor rightBackDrive = null;
    private DcMotor launcher = null;
    private CRServo leftFeeder = null;
    private CRServo rightFeeder = null;

    @Override
    public void runOpMode() {
        // Hardware setup
        leftFrontDrive  = hardwareMap.get(DcMotor.class, "left_Front_Drive");
        leftBackDrive   = hardwareMap.get(DcMotor.class, "left_Back_Drive");
        rightFrontDrive = hardwareMap.get(DcMotor.class, "right_Front_Drive");
        rightBackDrive  = hardwareMap.get(DcMotor.class, "right_Back_Drive");
        launcher        = hardwareMap.get(DcMotor.class, "launcher");
        leftFeeder      = hardwareMap.get(CRServo.class, "LeftFeeder");
        rightFeeder     = hardwareMap.get(CRServo.class, "RightFeeder");

        // Reverse left side motors for proper forward movement
        leftFrontDrive.setDirection(DcMotor.Direction.REVERSE);
        leftBackDrive.setDirection(DcMotor.Direction.REVERSE);
        rightFrontDrive.setDirection(DcMotor.Direction.FORWARD);
        rightBackDrive.setDirection(DcMotor.Direction.FORWARD);

        telemetry.addData("Status", "Initialized");
        telemetry.update();

        waitForStart();
        runtime.reset();

        while (opModeIsActive()) {
            double max;
            // Read joystick inputs
            double axial   = -gamepad1.left_stick_y;  // Forward/backward
            double lateral =  gamepad1.left_stick_x;  // Strafing
            double yaw     =  gamepad1.right_stick_x; // Turning

            // Calculate motor powers using mecanum equations
            double leftFrontPower  = axial + lateral + yaw;
            double rightFrontPower = axial - lateral - yaw;
            double leftBackPower   = axial - lateral + yaw;
            double rightBackPower  = axial + lateral - yaw;

            // Normalize powers to stay within [-1.0, 1.0]
            max = Math.max(Math.abs(leftFrontPower), Math.abs(rightFrontPower));
            max = Math.max(max, Math.abs(leftBackPower));
            max = Math.max(max, Math.abs(rightBackPower));

            if (max > 1.0) {
                leftFrontPower  /= max;
                rightFrontPower /= max;
                leftBackPower   /= max;
                rightBackPower  /= max;
            }

            // Set motor powers
            leftFrontDrive.setPower(leftFrontPower);
            rightFrontDrive.setPower(rightFrontPower);
            leftBackDrive.setPower(leftBackPower);
            rightBackDrive.setPower(rightBackPower);

            // Analog arm control (DC motor)
            double launchUpPower = gamepad2.right_trigger;
            double launchDownPower = gamepad2.left_trigger;
            double scaledLaunchPower = (launchUpPower - launchDownPower) * 0.6;
            launcher.setPower(scaledLaunchPower);


            // Arm2 servo control using Y button on game pad 2
            if (gamepad2.y) {
                leftFeeder.setPower(1.0);   // Sets left Feeder power to 1 (Clockwise)
                rightFeeder.setPower(-1.0); // Sets right Feeder power to -1 (Counter-clockwise)
            } else {
                leftFeeder.setPower(0.0);  // Sets left Feeder power to 0 when not being used
                rightFeeder.setPower(0.0); // Sets right Feeder Power to 0 when not being used
            }


            // Telemetry
            telemetry.addData("FL Wheel", leftFrontPower);
            telemetry.addData("FR Wheel", rightFrontPower);
            telemetry.addData("BL Wheel", leftBackPower);
            telemetry.addData("BR Wheel", rightBackPower);
            telemetry.addData("Launcher Power", scaledLaunchPower);
            telemetry.addData("LeftFeeder Power", leftFeeder.getPower());
            telemetry.addData("Right Feeder Power", rightFeeder.getPower());
            telemetry.update();
        }
    }
}
