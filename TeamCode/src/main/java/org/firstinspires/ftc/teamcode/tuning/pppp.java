package org.firstinspires.ftc.teamcode.tuning;
import androidx.annotation.NonNull;
import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.dashboard.telemetry.TelemetryPacket;
import com.acmerobotics.roadrunner.Action;
//import com.acmerobotics.roadrunner.PositionPathSeqBuilder.lineToX();
import com.acmerobotics.roadrunner.Pose2d;
import com.acmerobotics.roadrunner.SequentialAction;
import com.acmerobotics.roadrunner.TrajectoryActionBuilder;
import com.acmerobotics.roadrunner.Vector2d;
import com.acmerobotics.roadrunner.ftc.Actions;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import org.firstinspires.ftc.teamcode.MecanumDrive;

import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;
import com.acmerobotics.roadrunner.Action;
import com.acmerobotics.roadrunner.SequentialAction;
import com.acmerobotics.roadrunner.SleepAction;

@Config
@Autonomous(name = "BLUE_TEST_AUTO_PIXEL", group = "Autonomous")
public class pppp extends LinearOpMode {
//    @Override
    private DcMotor leftLift, rightLift;

    public void runOpMode() {
        leftLift = hardwareMap.get(DcMotor.class, "left lift");
        rightLift = hardwareMap.get(DcMotor.class, "right lift");

        leftLift.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        rightLift.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        rightLift.setDirection(DcMotor.Direction.REVERSE);
        Pose2d initialPose = new Pose2d(0, 60, Math.toRadians(90));
        MecanumDrive drive = new MecanumDrive(hardwareMap, initialPose);
        // vision here that outputs position
        int visionOutputPosition = 1;

        TrajectoryActionBuilder tab1 = drive.actionBuilder(initialPose);
        TrajectoryActionBuilder tab2 = drive.actionBuilder(initialPose);
        TrajectoryActionBuilder tab3 = drive.actionBuilder(initialPose);
        Action trajectoryActionCloseOut = tab1.endTrajectory()
                .splineTo(new Vector2d(-46.20, -46.61), Math.toRadians(0))
                .build();

        int startPosition = visionOutputPosition;
        waitForStart();

        if (isStopRequested()) return;

        Action trajectoryActionChosen;
        if (startPosition == 1) {
            trajectoryActionChosen = tab1.build();
        } else if (startPosition == 2) {
            trajectoryActionChosen = tab2.build();
        } else {
            trajectoryActionChosen = tab3.build();
        }
//        Action liftUpAction = new SequentialAction(
//                () -> {
//                    leftLift.setPower(0.8);
//                    rightLift.setPower(0.8);
//                    return null;
//                },
//                new com.acmerobotics.roadrunner.SleepAction(0.7),
//                () -> {
//                    leftLift.setPower(0);
//                    rightLift.setPower(0);
//                    return null;
//                }
//        );
        Actions.runBlocking(
                new SequentialAction(
                        trajectoryActionChosen,
                        trajectoryActionCloseOut
                        )
        );
    }
}