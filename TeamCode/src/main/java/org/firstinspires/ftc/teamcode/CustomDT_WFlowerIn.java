package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.ElapsedTime;

@TeleOp(name = "CustomDT_CompRDY", group = "Linear OPMODE")
public class CustomDT_WFlowerIn extends LinearOpMode {
    private ElapsedTime runtime = new ElapsedTime();
    private DcMotorEx leftFrontDrive = null;
    private DcMotorEx leftBackDrive = null;
    private DcMotorEx rightFrontDrive = null;
    private DcMotorEx rightBackDrive = null;
    private DcMotorEx intakeMotor = null;
    private Servo leftFlower = null;
    private Servo rightFlower = null;
    private DcMotorEx shooter = null;

    private Servo shooterGate = null;


    @Override
    public void runOpMode() {
        // Hardware setup
        leftFrontDrive  = hardwareMap.get(DcMotorEx.class, "leftFrontDrive");
        leftBackDrive   = hardwareMap.get(DcMotorEx.class, "leftBackDrive");
        rightFrontDrive = hardwareMap.get(DcMotorEx.class, "rightFrontDrive");
        rightBackDrive  = hardwareMap.get(DcMotorEx.class, "rightBackDrive");
        intakeMotor     = hardwareMap.get(DcMotorEx.class, "intakeMotor");
        leftFlower      = hardwareMap.get(Servo.class, "leftFlower");
        rightFlower     = hardwareMap.get(Servo.class, "rightFlower");
        shooter         = hardwareMap.get(DcMotorEx.class, "shooter");
        shooterGate     = hardwareMap.get(Servo.class, "shooterGate");

        // Reverse left side motors for proper forward movement
        leftFrontDrive.setDirection(DcMotorEx.Direction.REVERSE);
        leftBackDrive.setDirection(DcMotorEx.Direction.REVERSE);
        rightFrontDrive.setDirection(DcMotorEx.Direction.FORWARD);
        rightBackDrive.setDirection(DcMotorEx.Direction.FORWARD);
        intakeMotor.setDirection(DcMotorSimple.Direction.REVERSE);
        leftFlower.setDirection(Servo.Direction.FORWARD);
        rightFlower.setDirection(Servo.Direction.REVERSE);
        shooter.setDirection(DcMotorEx.Direction.REVERSE);
        shooterGate.setDirection(Servo.Direction.FORWARD);

        telemetry.addData("Status", "Initialized");
        telemetry.update();
        leftFlower.setPosition(0.0);
        rightFlower.setPosition(0.0);
        shooterGate.setPosition(0.0);

        waitForStart();
        runtime.reset();

        while (opModeIsActive()) {
            double max;
            // Read joystick inputs
            double axial   = -gamepad1.left_stick_y;  // Forward/backward
            double lateral =  gamepad1.left_stick_x;  // Strafing
            double yaw     =  gamepad1.right_stick_x; // Turning

            // Mecanum equations
            double leftFrontPower  = axial + lateral + yaw;
            double rightFrontPower = axial - lateral - yaw;
            double leftBackPower   = axial - lateral + yaw;
            double rightBackPower  = axial + lateral - yaw;

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

            // Intake Motor
            if (gamepad2.a) {
                if (shooterGate.getPosition() == 0.1) {
                    intakeMotor.setPower(1); // Full Intake Mode
                }
                else {
                    intakeMotor.setPower(0.48); // Slow Intake Mode
                }
            }
            else if (gamepad2.b) {
                intakeMotor.setPower(-0.5); // Spit out stuck game elements
            }
            else if (gamepad2.x) {
                intakeMotor.setPower(0); // Intake Off
            }

            // Flower Intake
            if (gamepad2.dpad_up) {
                leftFlower.setPosition(0.301);
                rightFlower.setPosition(0.301);
            } else if (gamepad2.dpad_down) {
                leftFlower.setPosition(0.0);
                rightFlower.setPosition(0.0);
            }

            // Shooter
            double shootUpPower = gamepad2.right_trigger;
            double shootDownPower = gamepad2.left_trigger;
            double scaledShootingPower = (shootUpPower - shootDownPower) * 0.5;
            shooter.setPower(scaledShootingPower);

            // Shooter Gate

            if (gamepad2.dpad_right) {
                shooterGate.setPosition(0.1); // gate is open until button is released
            } else {
                shooterGate.setPosition(0.0); // gate is closed by default
            }


            // Telemetry
            telemetry.addData("FL Wheel", leftFrontPower);
            telemetry.addData("FR Wheel", rightFrontPower);
            telemetry.addData("BL Wheel", leftBackPower);
            telemetry.addData("BR Wheel", rightBackPower);
            telemetry.addData("intake Motor Power", intakeMotor.getPower());
            telemetry.addData("Left Flower Position", leftFlower.getPosition());
            telemetry.addData("Shooter Velocity", shooter.getVelocity());
            telemetry.addData("Right Flower Position", rightFlower.getPosition());
            telemetry.addData("Shooter Gate Position", shooterGate.getPosition());
            telemetry.update();
        }
    }
}