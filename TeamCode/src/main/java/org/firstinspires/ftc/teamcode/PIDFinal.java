package org.firstinspires.ftc.teamcode;


import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.hardware.gobilda.GoBildaPinpointDriver;
import org.firstinspires.ftc.robotcore.external.navigation.Pose2D;

import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;


@Autonomous(name = "Encoder IMU PID Auto", group = "PID")
public class PIDFinal extends LinearOpMode {

    //==========================
    // Hardware
    //==========================

    private DcMotor leftFrontDrive;
    private DcMotor rightFrontDrive;
    private DcMotor leftBackDrive;
    private DcMotor rightBackDrive;
    private GoBildaPinpointDriver pinpoint;




    //==========================
    // PID Controllers
    //==========================

    private PIDController drivePID;
    private PIDController headingPID;
    private PIDController strafePID;

    //==========================
    // Robot Constants
    //==========================

    // goBILDA 5202 312 RPM motors
    static final double TICKS_PER_REV = 537.7;
    static final double WHEEL_DIAMETER = 4.0;

    static final double TICKS_PER_INCH =
            TICKS_PER_REV / (Math.PI * WHEEL_DIAMETER);

    //==========================
    // PID Constants
    // Tune these later
    //==========================

    static final double DRIVE_KP = 0.0025;
    static final double DRIVE_KI = 0.0000;
    static final double DRIVE_KD = 0.0005;

    static final double TURN_KP = 0.055;
    static final double TURN_KI = 0.00;
    static final double TURN_KD = 0.00;

    static final double STRAFE_KP = 0.0035;
    static final double STRAFE_KI = 0.0000;
    static final double STRAFE_KD = 0.0002;

    @Override
    public void runOpMode() {

        //==========================
        // Hardware Mapping
        //==========================

        leftFrontDrive =
                hardwareMap.get(DcMotor.class, "leftFrontDrive");

        rightFrontDrive =
                hardwareMap.get(DcMotor.class, "rightFrontDrive");

        leftBackDrive =
                hardwareMap.get(DcMotor.class, "leftBackDrive");

        rightBackDrive =
                hardwareMap.get(DcMotor.class, "rightBackDrive");

        pinpoint = hardwareMap.get(
                GoBildaPinpointDriver.class,
                "pinpoint"
        );



        //==========================
        // Reverse motors if needed
        //==========================

        // Uncomment if your robot drives backwards.
        //
        leftFrontDrive.setDirection(DcMotor.Direction.REVERSE);
        leftBackDrive.setDirection(DcMotor.Direction.REVERSE);

        leftFrontDrive.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        rightFrontDrive.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        leftBackDrive.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        rightBackDrive.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

        //==========================
        // Encoder Setup
        //==========================

        leftFrontDrive.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        rightFrontDrive.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        leftBackDrive.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        rightBackDrive.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);

        leftFrontDrive.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        rightFrontDrive.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        leftBackDrive.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        rightBackDrive.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);

        //==========================
        // Create PID Controllers
        //==========================

        drivePID =
                new PIDController(DRIVE_KP, DRIVE_KI, DRIVE_KD);

        headingPID =
                new PIDController(TURN_KP, TURN_KI, TURN_KD);

        strafePID =
                new PIDController(STRAFE_KP, STRAFE_KI, STRAFE_KD);

        drivePID.setOutputRange(0.80);
        headingPID.setOutputRange(0.25);
        strafePID.setOutputRange(0.60);

        telemetry.addLine("Initialization Complete");
        telemetry.update();

        waitForStart();

        if (isStopRequested()) return;

        pinpoint.resetPosAndIMU();
        sleep(500);


        //==========================
        // Autonomous Routine
        //==========================

        driveStraight(115, 0);

        sleep(500);

        turnToAngle(88);

        sleep(500);

        driveStraight(20,88);
        sleep(500);

        strafeLeft(20, 88);

        sleep(500);

        strafeLeft(-20, 88);
        sleep(500);
        driveStraight(-20,88);
        sleep(500);
        turnToAngle(0);
        sleep(500);
        driveStraight(-115,0);

        stopMotors();
    }

//========================================================
// PART 2 STARTS BELOW
//========================================================

    //========================================================
    // DRIVE STRAIGHT USING ENCODERS + IMU
    //========================================================

    private void driveStraight(double inches, double headingDegrees) {
        double direction = Math.signum(inches);

        double targetTicks = Math.abs(inches) * TICKS_PER_INCH;

        drivePID.reset();
        headingPID.reset();

        drivePID.setSetPoint(targetTicks);
        headingPID.setSetPoint(Math.toRadians(headingDegrees));

        // Reset encoders before each move
        leftFrontDrive.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        rightFrontDrive.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        leftBackDrive.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        rightBackDrive.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);

        leftFrontDrive.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        rightFrontDrive.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        leftBackDrive.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        rightBackDrive.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);

        while (opModeIsActive()) {

            // Average encoder position
            double currentTicks =
                    (Math.abs(leftFrontDrive.getCurrentPosition())
                            + Math.abs(rightFrontDrive.getCurrentPosition())
                            + Math.abs(leftBackDrive.getCurrentPosition())
                            + Math.abs(rightBackDrive.getCurrentPosition()))
                            / 4.0;

            // Current robot heading
            double heading = getHeadingRadians();

            // PID outputs
            double drivePower =
                    drivePID.getComputedOutput(currentTicks);

            drivePower *= direction;

            double remaining = Math.abs(targetTicks - currentTicks);

            double slowdownFactor = remaining / 1200.0;

            slowdownFactor = Math.max(
                    0.35,
                    Math.min(1.0, slowdownFactor)
            );

            drivePower *= slowdownFactor;
            double turnCorrection =
                    headingPID.getComputedAngleOutput(heading);

            turnCorrection = Math.max(
                    -0.15,
                    Math.min(0.15, turnCorrection)
            );




            // Stop when close enough
            if (Math.abs(targetTicks - currentTicks) < 100) {
                break;
            }

            double leftPower = drivePower + turnCorrection;
            double rightPower = drivePower - turnCorrection;


            // Normalize motor powers
            double max =
                    Math.max(
                            1.0,
                            Math.max(
                                    Math.abs(leftPower),
                                    Math.abs(rightPower)
                            )
                    );

            leftPower /= max;
            rightPower /= max;

            leftFrontDrive.setPower(leftPower);
            leftBackDrive.setPower(leftPower);

            rightFrontDrive.setPower(rightPower);
            rightBackDrive.setPower(rightPower);

            telemetry.addLine("Drive Straight");

            telemetry.addData("Target Ticks", targetTicks);
            telemetry.addData("Current Ticks", currentTicks);

            telemetry.addData("Drive Error",
                    drivePID.getCurrentError());

            telemetry.addData("Heading",
                    Math.toDegrees(heading));

            telemetry.addData("Heading Error",
                    Math.toDegrees(headingPID.getCurrentError()));

            telemetry.addData("Drive Power", drivePower);
            telemetry.addData("Turn Correction", turnCorrection);

            telemetry.addData("Heading", Math.toDegrees(heading));
            telemetry.addData("Turn Correction", turnCorrection);

            telemetry.update();
        }


        stopMotors();

        sleep(200);
    }

//========================================================
// PART 3 STARTS BELOW
//========================================================

    //========================================================
    // TURN TO ANGLE USING IMU + ANGLE WRAP
    //========================================================

    private void turnToAngle(double targetAngleDegrees) {

        headingPID.setOutputRange(0.85);
        headingPID.reset();

        headingPID.setSetPoint(Math.toRadians(targetAngleDegrees));

        while (opModeIsActive()) {

            double heading = getHeadingRadians();

            double turnPower =
                    headingPID.getComputedAngleOutput(heading);

            // Minimum power to overcome friction
            if (turnPower > 0 && turnPower < 0.15) {
                turnPower = 0.15;
            }

            if (turnPower < 0 && turnPower > -0.15) {
                turnPower = -0.15;
            }

            double errorDeg =
                    Math.toDegrees(headingPID.getCurrentError());

            // Stop within 3 degrees
            if (Math.abs(errorDeg) < 1) {
                break;
            }

            leftFrontDrive.setPower(-turnPower);
            leftBackDrive.setPower(-turnPower);

            rightFrontDrive.setPower(turnPower);
            rightBackDrive.setPower(turnPower);

            telemetry.addLine("Turning");
            telemetry.addData("Target", targetAngleDegrees);
            telemetry.addData("Current", Math.toDegrees(heading));
            telemetry.addData("Error", errorDeg);
            telemetry.addData("Turn Power", turnPower);
            telemetry.update();
        }

        stopMotors();

        headingPID.setOutputRange(0.10);

        sleep(200);
    }

//========================================================
// PART 4 STARTS BELOW
//========================================================

    //========================================================
    // STRAFE LEFT USING ENCODERS + IMU
    //========================================================

    private void strafeLeft(double inches, double headingDegrees) {
        double direction = Math.signum(inches);

        double targetTicks = Math.abs(inches) * TICKS_PER_INCH;

        strafePID.reset();
        headingPID.reset();

        strafePID.setSetPoint(targetTicks);
        headingPID.setSetPoint(Math.toRadians(headingDegrees));

        // Reset encoders
        leftFrontDrive.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        rightFrontDrive.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        leftBackDrive.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        rightBackDrive.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);

        leftFrontDrive.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        rightFrontDrive.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        leftBackDrive.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        rightBackDrive.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);

        while (opModeIsActive()) {

            double currentTicks =
                    (Math.abs(leftFrontDrive.getCurrentPosition())
                            + Math.abs(rightFrontDrive.getCurrentPosition())
                            + Math.abs(leftBackDrive.getCurrentPosition())
                            + Math.abs(rightBackDrive.getCurrentPosition()))
                            / 4.0;

            double heading = getHeadingRadians();

            double strafePower =
                    strafePID.getComputedOutput(currentTicks);

            strafePower *= direction;

            double turnCorrection =
                    headingPID.getComputedAngleOutput(heading);

            turnCorrection = Math.max(
                    -0.12,
                    Math.min(0.12, turnCorrection)
            );

            if (Math.abs(targetTicks - currentTicks) < 20) {
                break;
            }

            double lf = -strafePower + turnCorrection;
            double rf =  strafePower - turnCorrection;
            double lb =  strafePower + turnCorrection;
            double rb = -strafePower - turnCorrection;

            double max = Math.max(
                    1.0,
                    Math.max(
                            Math.max(Math.abs(lf), Math.abs(rf)),
                            Math.max(Math.abs(lb), Math.abs(rb))
                    )
            );

            lf /= max;
            rf /= max;
            lb /= max;
            rb /= max;

            leftFrontDrive.setPower(lf);
            rightFrontDrive.setPower(rf);
            leftBackDrive.setPower(lb);
            rightBackDrive.setPower(rb);

            telemetry.addLine("Strafing Left");
            telemetry.addData("Target", targetTicks);
            telemetry.addData("Current", currentTicks);
            telemetry.addData("Heading", Math.toDegrees(heading));
            telemetry.update();
        }

        stopMotors();

        sleep(200);
    }

    //========================================================
    // STOP ALL MOTORS
    //========================================================

    private double getHeadingRadians() {

        pinpoint.update();

        Pose2D pose = pinpoint.getPosition();

        return Math.toRadians(
                pose.getHeading(AngleUnit.DEGREES)
        );
    }

    private void stopMotors() {

        leftFrontDrive.setPower(0);
        rightFrontDrive.setPower(0);
        leftBackDrive.setPower(0);
        rightBackDrive.setPower(0);

    }

}