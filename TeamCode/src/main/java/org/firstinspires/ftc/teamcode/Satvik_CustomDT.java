package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.util.ElapsedTime;

@TeleOp
public class Satvik_CustomDT extends LinearOpMode {
    private ElapsedTime runtime = new ElapsedTime();
    private DcMotorEx leftFrontDrive = null;
    private DcMotorEx leftBackDrive = null;
    private DcMotorEx rightFrontDrive = null;
    private DcMotorEx rightBackDrive = null;
    private DcMotorEx intakeMotor = null;


    @Override
    public void runOpMode() {
        // Hardware setup
        leftFrontDrive  = hardwareMap.get(DcMotorEx.class, "leftFrontDrive");
        leftBackDrive   = hardwareMap.get(DcMotorEx.class, "leftBackDrive");
        rightFrontDrive = hardwareMap.get(DcMotorEx.class, "rightFrontDrive");
        rightBackDrive  = hardwareMap.get(DcMotorEx.class, "rightBackDrive");
        intakeMotor     = hardwareMap.get(DcMotorEx.class, "intakeMotor");

        // Reverse left side motors for proper forward movement
        leftFrontDrive.setDirection(DcMotorEx.Direction.REVERSE);
        leftBackDrive.setDirection(DcMotorEx.Direction.REVERSE);
        rightFrontDrive.setDirection(DcMotorEx.Direction.FORWARD);
        rightBackDrive.setDirection(DcMotorEx.Direction.FORWARD);
        intakeMotor.setDirection(DcMotorSimple.Direction.REVERSE);

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
                intakeMotor.setPower(0.5); // Intake Mode
            }
            else if (gamepad2.b) {
                intakeMotor.setPower(-0.5); //Spit out stuck game elements
            }
            else if (gamepad2.x) {
                intakeMotor.setPower(0); // Intake Off
            }

            // Telemetry
            telemetry.addData("FL Wheel", leftFrontPower);
            telemetry.addData("FR Wheel", rightFrontPower);
            telemetry.addData("BL Wheel", leftBackPower);
            telemetry.addData("BR Wheel", rightBackPower);
            telemetry.addData("intake Motor Power", intakeMotor.getPower());
            telemetry.update();
        }
    }
}