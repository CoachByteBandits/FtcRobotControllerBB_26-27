package org.firstinspires.ftc.teamcode;

import com.qualcomm.hardware.gobilda.GoBildaPinpointDriver;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.util.ElapsedTime;
import org.firstinspires.ftc.robotcore.external.navigation.Pose2D;
import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;
import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;

@TeleOp(name = "Satvik's Odometry TeleopMode", group = "Linear OPMODE")
public class OdometryTeleopMode extends LinearOpMode {

    private ElapsedTime runtime = new ElapsedTime();

    private DcMotor leftFrontDrive = null;
    private DcMotor leftBackDrive = null;
    private DcMotor rightFrontDrive = null;
    private DcMotor rightBackDrive = null;
    private DcMotorEx intakeMotor = null;
    private GoBildaPinpointDriver pinpoint = null;


    @Override
    public void runOpMode() {

        // Hardware setup
        leftFrontDrive  = hardwareMap.get(DcMotor.class, "leftFrontDrive");
        leftBackDrive   = hardwareMap.get(DcMotor.class, "leftBackDrive");
        rightFrontDrive = hardwareMap.get(DcMotor.class, "rightFrontDrive");
        rightBackDrive  = hardwareMap.get(DcMotor.class, "rightBackDrive");
        intakeMotor     = hardwareMap.get(DcMotorEx.class, "intakeMotor");
        pinpoint        = hardwareMap.get(GoBildaPinpointDriver.class, "pinpoint");

        // Set the encoder resolution for the odometry pods
        pinpoint.setEncoderResolution(
                GoBildaPinpointDriver.GoBildaOdometryPods.goBILDA_4_BAR_POD
        );

        // Reverse left side motors for proper forward movement
        leftFrontDrive.setDirection(DcMotor.Direction.REVERSE);
        leftBackDrive.setDirection(DcMotor.Direction.REVERSE);

        rightFrontDrive.setDirection(DcMotor.Direction.FORWARD);
        rightBackDrive.setDirection(DcMotor.Direction.FORWARD);

        // Pinpoint encoder directions
        pinpoint.setEncoderDirections(
                GoBildaPinpointDriver.EncoderDirection.REVERSED,
                GoBildaPinpointDriver.EncoderDirection.REVERSED);


        // Reset odometry position
        pinpoint.resetPosAndIMU();


        telemetry.addData("Status", "Initialized");
        telemetry.update();


        waitForStart();
        runtime.reset();


        while (opModeIsActive()) {

            // Pinpoint update
            pinpoint.update();

            // Mecanum drive control
            double axial   = -gamepad1.left_stick_y;
            double lateral = gamepad1.left_stick_x;
            double yaw     = gamepad1.right_stick_x;


            double leftFrontPower  = axial + lateral + yaw;
            double rightFrontPower = axial - lateral - yaw;
            double leftBackPower   = axial - lateral + yaw;
            double rightBackPower  = axial + lateral - yaw;


            // Normalize powers
            double max = Math.max(
                    Math.max(Math.abs(leftFrontPower), Math.abs(rightFrontPower)),
                    Math.max(Math.abs(leftBackPower), Math.abs(rightBackPower))
            );


            if (max > 1.0) {
                leftFrontPower /= max;
                rightFrontPower /= max;
                leftBackPower /= max;
                rightBackPower /= max;
            }


            // Set movement motor powers
            leftFrontDrive.setPower(leftFrontPower);
            rightFrontDrive.setPower(rightFrontPower);
            leftBackDrive.setPower(leftBackPower);
            rightBackDrive.setPower(rightBackPower);

            // Intake Motor gamepad mapping
            if (gamepad2.a) {
                intakeMotor.setVelocity(3000);
            }else if (gamepad2.x) {
                intakeMotor.setVelocity(-1800);
            }
            else {
                intakeMotor.setVelocity(0);
            }
            // Telemetry
            telemetry.addData("FrontL Wheel", leftFrontPower);
            telemetry.addData("FrontR Wheel", rightFrontPower);
            telemetry.addData("BackL Wheel", leftBackPower);
            telemetry.addData("BackR Wheel", rightBackPower);
            // Pinpoint Position Telemetry
            telemetry.addData("X Position (mm)", pinpoint.getPosX(DistanceUnit.MM));
            telemetry.addData("Y Position (mm)", pinpoint.getPosY(DistanceUnit.MM));
            telemetry.addData("Heading (deg)", pinpoint.getHeading(AngleUnit.DEGREES));

            // Temporary telemetry (To be removed)
            telemetry.addData("Pinpoint Status", pinpoint.getDeviceStatus());
            telemetry.addData("Pinpoint Version", pinpoint.getDeviceVersion());

            telemetry.update();
        }
    }
}