package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.util.ElapsedTime;

@Autonomous(name = "Leave", group = "Linear OPMODE")
public class Leave extends LinearOpMode {

    private ElapsedTime runtime = new ElapsedTime();

    // Hardware
    private DcMotorEx leftFrontDrive = null;
    private DcMotorEx leftBackDrive = null;
    private DcMotorEx rightFrontDrive = null;
    private DcMotorEx rightBackDrive = null;
    private DcMotorEx intakeMotor = null;
    private DcMotorEx flyWheel = null;

    // Constants
    private static final double COUNTS_PER_MOTOR_REV = 537;  // GoBILDA 5202 motor
    private static final double WHEEL_DIAMETER_INCHES = 4.0; // Wheel diameter
    private static final double COUNTS_PER_INCH = COUNTS_PER_MOTOR_REV / (Math.PI * WHEEL_DIAMETER_INCHES);

    @Override
    public void runOpMode() {

        // Hardware mapping
        leftFrontDrive  = hardwareMap.get(DcMotorEx.class, "left_Front_Drive");
        leftBackDrive   = hardwareMap.get(DcMotorEx.class, "left_Back_Drive");
        rightFrontDrive = hardwareMap.get(DcMotorEx.class, "right_Front_Drive");
        rightBackDrive  = hardwareMap.get(DcMotorEx.class, "right_Back_Drive");
        intakeMotor = hardwareMap.get(DcMotorEx.class, "intakeMotor");
        flyWheel = hardwareMap.get(DcMotorEx.class, "flyWheel");

        leftFrontDrive.setMode(DcMotorEx.RunMode.RUN_USING_ENCODER);
        leftBackDrive.setMode(DcMotorEx.RunMode.RUN_USING_ENCODER);
        rightFrontDrive.setMode(DcMotorEx.RunMode.RUN_USING_ENCODER);
        rightBackDrive.setMode(DcMotorEx.RunMode.RUN_USING_ENCODER);

        // Motor directions
        leftFrontDrive.setDirection(DcMotor.Direction.FORWARD);
        leftBackDrive.setDirection(DcMotor.Direction.FORWARD);
        rightFrontDrive.setDirection(DcMotor.Direction.REVERSE);
        rightBackDrive.setDirection(DcMotor.Direction.REVERSE);

        telemetry.addData("Status", "Initialized");
        telemetry.update();

        waitForStart();
        runtime.reset();

        if (opModeIsActive()) {
            // Move forward 5 inches
            moveAllMotorsDistance(-5, -0.4);
        }
    }

    // ---------------- MOVEMENT FUNCTIONS ----------------

    public void moveAllMotorsDistance(double inches, double power) {
        int moveCounts = (int)(inches * COUNTS_PER_INCH);

        leftFrontDrive.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        leftBackDrive.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        rightFrontDrive.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        rightBackDrive.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);

        leftFrontDrive.setTargetPosition(moveCounts);
        leftBackDrive.setTargetPosition(moveCounts);
        rightFrontDrive.setTargetPosition(moveCounts);
        rightBackDrive.setTargetPosition(moveCounts);

        leftFrontDrive.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        leftBackDrive.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        rightFrontDrive.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        rightBackDrive.setMode(DcMotor.RunMode.RUN_TO_POSITION);

        leftFrontDrive.setPower(power);
        leftBackDrive.setPower(power);
        rightFrontDrive.setPower(power);
        rightBackDrive.setPower(power);

        while (opModeIsActive() &&
                (leftFrontDrive.isBusy() || leftBackDrive.isBusy() ||
                        rightFrontDrive.isBusy() || rightBackDrive.isBusy())) {
            idle();
        }

        leftFrontDrive.setPower(0);
        leftBackDrive.setPower(0);
        rightFrontDrive.setPower(0);
        rightBackDrive.setPower(0);
    }
}