package org.firstinspires.ftc.teamcode.tuning;

import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.roadrunner.Action;
import com.acmerobotics.roadrunner.Pose2d;
import com.acmerobotics.roadrunner.SequentialAction;
import com.acmerobotics.roadrunner.TrajectoryActionBuilder;
import com.acmerobotics.roadrunner.Vector2d;
import com.acmerobotics.roadrunner.ftc.Actions;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.CRServo;

import org.firstinspires.ftc.teamcode.MecanumDrive;

@Config
@Autonomous(name = "asll", group = "Autonomous")
public class sl extends LinearOpMode {
//    @Override
    private DcMotor leftLift, rightLift;
    private CRServo arm1, arm2;
    private CRServo open;

    public void runOpMode() {
        leftLift = hardwareMap.get(DcMotor.class, "left lift");
        rightLift = hardwareMap.get(DcMotor.class, "right lift");
        arm1 = hardwareMap.get(CRServo.class, "arm1");
        arm2 = hardwareMap.get(CRServo.class, "arm2");
        open = hardwareMap.get(CRServo.class, "open");
        leftLift.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        rightLift.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        rightLift.setDirection(DcMotor.Direction.REVERSE);
        Pose2d initialPose = new Pose2d(30.0, 60, Math.toRadians(90));
        MecanumDrive drive = new MecanumDrive(hardwareMap, initialPose);
        // vision here that outputs position
        int visionOutputPosition = 1;

        TrajectoryActionBuilder tab1 = drive.actionBuilder(initialPose);
        TrajectoryActionBuilder tab2 = drive.actionBuilder(initialPose);
        TrajectoryActionBuilder tab3 = drive.actionBuilder(initialPose);
        Action trajectoryActionCloseOut = tab1.endTrajectory()
                .strafeTo (new Vector2d (20, 40))
                .turnTo (90)
//                .afterTime(0, () -> {
//                    leftLift.setPower(-1);
//                    rightLift.setPower(-1);
//                })
//                .afterTime(0.8, () -> {
//                    leftLift.setPower(-0.1);
//                    rightLift.setPower(-0.1);
//                })
//                .afterTime (1.0, () -> {
//                    open.setPower(-1);
//                })
//                .afterTime(2.0, () -> {
//                    open.setPower(1);
//                })
//                .afterTime(2.3, () -> {
//                    open.setPower(0);
//                })
//                .waitSeconds(5)
                .afterTime(0, () -> {
                    leftLift.setPower(-1);
                    rightLift.setPower(-1);
                })
                .afterTime(0.8, () -> {
                    leftLift.setPower(-0.1);
                    rightLift.setPower(-0.1);
                })
                .afterTime(1, () -> {
                    arm1.setPower(-1.0);
                    arm2.setPower(1.0);
                })
                .afterTime(1.5, () -> {
                    arm1.setPower(0);
                    arm2.setPower(0);
                })
                .afterTime(1.8, () -> {
                    arm1.setPower(-1.0);
                    arm2.setPower   (1.0);
                })
                .afterTime(2.9, () -> {
                    arm1.setPower(0);
                    arm2.setPower(0);
                })
                .afterTime(2.9, () -> {
                    open.setPower(-1);
                })
                .afterTime(3.1, () -> {
                    open.setPower(0);
                })
                .afterTime(3.3, () -> {
                    open.setPower(1);
                })
                .afterTime(3.6, () -> {
                    open.setPower(0);
                })
                .afterTime(3.7, () -> {
                    arm1.setPower(1.0);
                    arm2.setPower(-1.0);
                })
                .afterTime(4.2, () -> {
                    arm1.setPower(0);
                    arm2.setPower(0);
                })
                .afterTime(4.5, () -> {
                    arm1.setPower(1.0);
                    arm2.setPower(-1.0);
                })
                .afterTime(4.9, () -> {
                    arm1.setPower(0);
                    arm2.setPower(0);
                })
                .turnTo(90)
                .afterTime(7.7, () -> {
                    leftLift.setPower(1);
                    rightLift.setPower(1);
                })
                .lineToY(43)
                .afterTime(8.4, () -> {
                    leftLift.setPower(0.0);
                    rightLift.setPower(0.0);
                })

//                .lineToY (43)
//                .waitSeconds(1)
//                .afterTime(2, () -> {
//                    open.setPower(-1);
//                })
//                .afterTime(2.4, () -> {
//                    open.setPower(0);
//                })
//                .afterTime(2.4, () -> {
//                    open.setPower(1);
//                })
//                .afterTime(2.6, () -> {
//                    open.setPower(0);
//                })
//                .afterTime(1, () -> {
//                    arm1.setPower(1.0);
//                    arm2.setPower(-1.0);
//                })
//                .afterTime(1.8, () -> {
//                    arm1.setPower(0.0);
//                    arm2.setPower(0.0);
//                })
//                .afterTime(0, () -> {
//                    leftLift.setPower(1);
//                    rightLift.setPower(1);
//                })
//                .afterTime(0.7, () -> {
//                    leftLift.setPower(0.0);
//                    rightLift.setPower(0.0);
//                })
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