package org.firstinspires.ftc.teamcode.Tests;

import com.acmerobotics.roadrunner.Pose2d;
import com.acmerobotics.roadrunner.SequentialAction;
import com.acmerobotics.roadrunner.TrajectoryActionBuilder;
import com.acmerobotics.roadrunner.Vector2d;
import com.acmerobotics.roadrunner.ftc.Actions;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

import org.firstinspires.ftc.robotcore.external.navigation.Pose2D;
import org.firstinspires.ftc.teamcode.MecanumDrive;

@Autonomous
public final class MyTestTry extends LinearOpMode {
    @Override
    public void runOpMode() throws InterruptedException {
       DriveConstants.resetConst();
        Pose2d beginPose = new Pose2d(DriveConstants.transPositionX, DriveConstants.transPositionY, DriveConstants.transHeading);
        MecanumDrive drive = new MecanumDrive(hardwareMap, beginPose);


            TrajectoryActionBuilder tab1 = drive.actionBuilder(beginPose)
                    .lineToX(25)
                    .turn(Math.toRadians(90))
                    .waitSeconds(1);

        TrajectoryActionBuilder tab2 = tab1.endTrajectory().fresh()
                .lineToY(25)
                .turn(Math.toRadians(90))
                .waitSeconds(1);

        TrajectoryActionBuilder tab3 = tab2.endTrajectory().fresh()
                .lineToX(0)
                .turn(Math.toRadians(90))
                .waitSeconds(1);

        TrajectoryActionBuilder tab4 = tab3.endTrajectory().fresh()
                .lineToY(0)
                .turn(Math.toRadians(90))
                .waitSeconds(1);

        waitForStart();

        Actions.runBlocking(
                new SequentialAction(
                        tab1.build(),
                        tab2.build(),
                        tab3.build(),
                        tab4.build()
                )
        );


    // Following code is to copy final angle of the auton to TeleOpp for field centric
        drive.updatePoseEstimate();
        Pose2d pose = drive.localizer.getPose();

        DriveConstants.transHeading = pose.heading.toDouble();
        DriveConstants.transPositionX = pose.position.x;
        DriveConstants.transPositionY = pose.position.y;

    }
}
