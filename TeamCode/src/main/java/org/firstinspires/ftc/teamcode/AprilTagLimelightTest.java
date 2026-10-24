package org.firstinspires.ftc.teamcode;

import com.qualcomm.hardware.limelightvision.LLResult;
import com.qualcomm.hardware.limelightvision.LLResultTypes;
import com.qualcomm.hardware.limelightvision.Limelight3A;
import com.qualcomm.hardware.rev.RevHubOrientationOnRobot;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.hardware.IMU;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;

import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.Pose3D;
import org.firstinspires.ftc.robotcore.external.navigation.YawPitchRollAngles;

import java.util.List;

@Autonomous(name = "AprilTag Limelight Test", group = "Test")
public class AprilTagLimelightTest extends OpMode {
    private Limelight3A limelight;
    private IMU imu;


    private DcMotor frontLeft;
    private DcMotor frontRight;
    private DcMotor backLeft;
    private DcMotor backRight;

    private final double Kp = 0.015;
    private final double TOLERANCE_DEGREES = 1.5;
    private final double MIN_TURNING_POWER = 0.05;

    @Override
    public void init() {
        limelight = hardwareMap.get(Limelight3A.class, "limelight");
        limelight.pipelineSwitch(8);
        imu = hardwareMap.get(IMU.class, "imu");

        RevHubOrientationOnRobot revHubOrientationOnRobot = new RevHubOrientationOnRobot(
                RevHubOrientationOnRobot.LogoFacingDirection.BACKWARD,
                RevHubOrientationOnRobot.UsbFacingDirection.UP);

        imu.initialize(new IMU.Parameters(revHubOrientationOnRobot));


        frontLeft = hardwareMap.get(DcMotor.class, "frontLeft");
        frontRight = hardwareMap.get(DcMotor.class, "frontRight");
        backLeft = hardwareMap.get(DcMotor.class, "backLeft");
        backRight = hardwareMap.get(DcMotor.class, "backRight");


        frontLeft.setDirection(DcMotorSimple.Direction.REVERSE);
        backLeft.setDirection(DcMotorSimple.Direction.REVERSE);


        frontLeft.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        frontRight.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        backLeft.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        backRight.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
    }

    @Override
    public void start() {
        limelight.start();
    }

    @Override
    public void loop() {
        YawPitchRollAngles orientation = imu.getRobotYawPitchRollAngles();
        limelight.updateRobotOrientation(orientation.getYaw(AngleUnit.DEGREES));

        LLResult llResult = limelight.getLatestResult();
        double turnPower = 0.0;

        if (llResult != null && llResult.isValid()) {
            Pose3D botPose = llResult.getBotpose_MT2();
            double tx = llResult.getTx();

            telemetry.addData("Target x", tx);
            telemetry.addData("Target y", llResult.getTy());
            telemetry.addData("BotPose", botPose.toString());
            telemetry.addData("Yaw", botPose.getOrientation().getYaw(AngleUnit.DEGREES));

            if (Math.abs(tx) > TOLERANCE_DEGREES) {

                turnPower = tx * Kp;

                if (turnPower > 0) turnPower += MIN_TURNING_POWER;
                else turnPower -= MIN_TURNING_POWER;


                turnPower = Math.max(-0.4, Math.min(0.4, turnPower));
            } else {
                turnPower = 0.0;
            }

            List<LLResultTypes.FiducialResult> fiducials = llResult.getFiducialResults();
            for (LLResultTypes.FiducialResult fiducial : fiducials) {
                int tagID = fiducial.getFiducialId();
                telemetry.addData("Detected Tag ID", tagID);
                Pose3D targetPoseCamera = fiducial.getTargetPoseCameraSpace();

                if (targetPoseCamera != null) {
                    double x = targetPoseCamera.getPosition().x;
                    double y = targetPoseCamera.getPosition().y;
                    double z = targetPoseCamera.getPosition().z;

                    double directDistanceMeters = Math.sqrt(x*x + y*y + z*z);
                    double forwardInches = z * 39.3701;
                    double lateralInches = x * 39.3701;
                    double directInches = directDistanceMeters * 39.3701;

                    telemetry.addData("Tag " + tagID + " Forward (Inches)", "%.2f", forwardInches);
                    telemetry.addData("Tag " + tagID + " Lateral (Inches)", "%.2f", lateralInches);
                    telemetry.addData("Tag " + tagID + " Direct (Inches)", "%.2f", directInches);
                }
            }
        } else {
            telemetry.addData("Limelight", "No targets seen");
            turnPower = 0.0;
        }


        frontLeft.setPower(turnPower);
        backLeft.setPower(turnPower);
        frontRight.setPower(-turnPower);
        backRight.setPower(-turnPower);

        telemetry.addData("Calc. Turn", "%.3f", turnPower);
    }
}