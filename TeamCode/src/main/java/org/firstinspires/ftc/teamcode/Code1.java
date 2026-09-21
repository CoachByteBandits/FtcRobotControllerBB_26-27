package org.firstinspires.ftc.teamcode;

import com.acmerobotics.roadrunner.Pose2d;
import com.acmerobotics.roadrunner.TrajectoryActionBuilder;
import com.acmerobotics.roadrunner.Vector2d;
import com.acmerobotics.roadrunner.ftc.Actions;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

@Autonomous(name = "RoadRunner Figure 8 New")
public class Code1 extends LinearOpMode {

    // Figure-8 loop radius in inches
    public static double LOOP_RADIUS = 24.0;



    private MecanumDrive drive;

    @Override
    public void runOpMode() throws InterruptedException {


        Pose2d beginPose = new Pose2d(0, 0, 0);

        drive = new MecanumDrive(hardwareMap, beginPose);

        telemetry.addLine("Figure-8 initialized");
        telemetry.addData("Radius", LOOP_RADIUS);
        telemetry.addData("Diameter", LOOP_RADIUS * 2);
        telemetry.addLine("Using drive motor encoders + IMU");
        telemetry.addLine("Press START");
        telemetry.update();



        waitForStart();

        if (isStopRequested()) {
            return;
        }


        TrajectoryActionBuilder path = drive.actionBuilder(beginPose)
                .setTangent(Math.PI / 2.0)
                .splineTo(new Vector2d(LOOP_RADIUS, LOOP_RADIUS), -Math.PI / 4.0)
                .splineTo(new Vector2d(LOOP_RADIUS * 2, 0), -Math.PI / 2.0)
                .splineTo(new Vector2d(LOOP_RADIUS, -LOOP_RADIUS), -3.0 * Math.PI / 4.0)
                .splineToSplineHeading(new Pose2d(0, 0, 0), -Math.PI / 2.0)
                .splineTo(new Vector2d(-LOOP_RADIUS, LOOP_RADIUS), 3.0 * Math.PI / 4.0)
                .splineTo(new Vector2d(-LOOP_RADIUS * 2, 0), Math.PI / 2.0)
                .splineTo(new Vector2d(-LOOP_RADIUS, -LOOP_RADIUS), Math.PI / 4.0)
                .splineToSplineHeading(new Pose2d(0, 0, 0), -Math.PI / 2.0);

        Actions.runBlocking(path.build());


        double HALF_RADIUS = LOOP_RADIUS / 2.0;

        TrajectoryActionBuilder path = drive.actionBuilder(beginPose)


                .setTangent(Math.PI / 2.0)

                .splineTo(new Vector2d(HALF_RADIUS, HALF_RADIUS), Math.PI / 4.0)
                .splineTo(new Vector2d(LOOP_RADIUS, 0),0)
                .splineTo(new Vector2d(HALF_RADIUS, -HALF_RADIUS), -Math.PI / 4.0)
                .splineTo(new Vector2d(0, 0), -Math.PI / 2.0)

                .splineTo(new Vector2d(-HALF_RADIUS, -HALF_RADIUS), -3.0 * Math.PI / 4.0)
                .splineTo(new Vector2d(-LOOP_RADIUS, 0), Math.PI)
                .splineTo(new Vector2d(-HALF_RADIUS, HALF_RADIUS), 3.0 * Math.PI / 4.0)
                .splineTo(new Vector2d(0, 0), Math.PI / 2.0);

        Actions.runBlocking(path.build());

        Pose2d finalPose = drive.localizer.getPose();

        telemetry.addLine("FIGURE-8 COMPLETE");
        telemetry.addData("Final X", finalPose.position.x);
        telemetry.addData("Final Y", finalPose.position.y);
        telemetry.addData(
                "Final heading",
                Math.toDegrees(finalPose.heading.log())
        );
        telemetry.update();
    }


}
