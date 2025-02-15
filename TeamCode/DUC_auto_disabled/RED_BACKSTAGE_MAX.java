package org.firstinspires.ftc.teamcode.DUC_auto_disabled;

import static org.firstinspires.ftc.teamcode.lib.Hardware.closeClawAngle;
import static org.firstinspires.ftc.teamcode.lib.Hardware.openClawAngle;

import androidx.annotation.NonNull;

import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.dashboard.telemetry.TelemetryPacket;
import com.acmerobotics.roadrunner.Action;
import com.acmerobotics.roadrunner.ParallelAction;
import com.acmerobotics.roadrunner.Pose2d;
import com.acmerobotics.roadrunner.SequentialAction;
import com.acmerobotics.roadrunner.SleepAction;
import com.acmerobotics.roadrunner.TrajectoryActionBuilder;
import com.acmerobotics.roadrunner.Vector2d;
import com.acmerobotics.roadrunner.ftc.Actions;
import com.arcrobotics.ftclib.hardware.ServoEx;
import com.arcrobotics.ftclib.hardware.SimpleServo;
import com.arcrobotics.ftclib.hardware.motors.Motor;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.HardwareMap;

import org.firstinspires.ftc.teamcode.MecanumDrive;
import org.firstinspires.ftc.teamcode.PinpointDrive;

@Config
@Autonomous(name = "1RED_BACKSTAGE_MAX", group = "Autonomous")
@Disabled
public class RED_BACKSTAGE_MAX extends LinearOpMode {

    int armTickPosition = 0;
    int normalArmTickPosition = 100;

    @Override
    public void runOpMode() throws InterruptedException {
        Pose2d initialPose = new Pose2d(25, -64, Math.toRadians(270));
        MecanumDrive drive = new PinpointDrive(hardwareMap, initialPose);
        Claw claw = new Claw(hardwareMap);
        NormalClaw normalClaw = new NormalClaw(hardwareMap);
        Arm arm = new Arm(hardwareMap);
        NormalArm normalArm = new NormalArm(hardwareMap);


        Vector2d highRung = new Vector2d(10, -28);

        TrajectoryActionBuilder strafeToHighRung = drive.actionBuilder(new Pose2d(25, -64, Math.toRadians(270)))
                .strafeToLinearHeading(new Vector2d(2, -32), Math.toRadians(270));

        TrajectoryActionBuilder givePlayerFirstSample = drive.actionBuilder(new Pose2d(2, -32, Math.toRadians(270)))
                .strafeTo(new Vector2d(0, -40))
                .strafeToLinearHeading(new Vector2d(54, -40), Math.toRadians(90))
                .strafeTo(new Vector2d(54, -38));
                /*.strafeTo(new Vector2d(40, -15))
                .strafeTo(new Vector2d(50, -15))
                .strafeTo(new Vector2d(50, -54))
                .strafeTo(new Vector2d(62, -15))
                .strafeTo(new Vector2d(62, -54))
                .strafeTo(new Vector2d(60, -50))
                .strafeTo(new Vector2d(42, -50))
                .strafeTo(new Vector2d(42, -60));*/

        TrajectoryActionBuilder givePlayerFirstSampleContinued = drive.actionBuilder(new Pose2d(54, -36, Math.toRadians(90)))
                .strafeToLinearHeading(new Vector2d(54, -50), Math.toRadians(270));

        TrajectoryActionBuilder givePlayerSecondSample = drive.actionBuilder(new Pose2d(54, -50, Math.toRadians(270)))
                .strafeTo(new Vector2d(54, -45))
                .strafeToLinearHeading(new Vector2d(60, -45), Math.toRadians(90))
                .strafeTo(new Vector2d(61, -39));

        TrajectoryActionBuilder givePlayerSecondSampleContinued = drive.actionBuilder(new Pose2d(61, -39, Math.toRadians(90)))
                .strafeToLinearHeading(new Vector2d(60, -50), Math.toRadians(270));

        TrajectoryActionBuilder getFirstSpecimen = drive.actionBuilder(new Pose2d(60, -50, Math.toRadians(270)))
                .strafeTo(new Vector2d(60, -45))
                .strafeToLinearHeading(new Vector2d(52, -45), Math.toRadians(90))
                .strafeTo(new Vector2d(52, -55));



        Actions.runBlocking(claw.closeClaw());
        Actions.runBlocking(normalClaw.openClaw());

        waitForStart();
        if (isStopRequested()) return;

        Actions.runBlocking(
                new ParallelAction(
                        new SequentialAction(
                                new ParallelAction(
                                        strafeToHighRung.build(),
                                        arm.highRung()
                                ),
                                arm.highRung2(),
                                new SleepAction(0.25),
                                claw.openClaw(),
                                new SleepAction(0.25),
                                givePlayerFirstSample.build(),
                                normalClaw.closeClaw(),
                                new SleepAction(0.5),
                                givePlayerFirstSampleContinued.build(),
                                normalClaw.openClaw(),
                                givePlayerSecondSample.build(),
                                normalClaw.closeClaw(),
                                new SleepAction(0.5),
                                givePlayerSecondSampleContinued.build(),
                                normalClaw.openClaw(),
                                getFirstSpecimen.build()
                                /*new ParallelAction(
                                        arm.low(),
                                        getFirstSpecimen.build()
                                ),
                                claw.closeClaw(),
                                new ParallelAction(
                                        arm.highRung(),
                                        hangFirstSpecimen.build()
                                ),
                                arm.highRung2(),
                                new SleepAction(0.25),
                                claw.openClaw(),
                                new ParallelAction(
                                        arm.low(),
                                        goPark.build()
                                ),
                                claw.closeClaw()
                                /*new ParallelAction( I HATE OPTIMIZING FOR TIME!!!!!!!! GIVE ME DEAD ENCODERS NOW!!!!
                                        arm.highRung(),
                                        hangSecondSpecimen.build()
                                ),
                                arm.highRung2(),
                                new SleepAction(0.25),
                                claw.openClaw()*/
                        ),
                        arm.keepPosition(),
                        normalArm.keepPosition()
                )

        );

    }
    public class Arm {
        private Motor arm;

        public Arm(HardwareMap hardwareMap) {
            arm = new Motor(hardwareMap, "specimenArm");
            arm.setZeroPowerBehavior(Motor.ZeroPowerBehavior.BRAKE);
            arm.resetEncoder();
        }

        public class KeepPosition implements Action {
            @Override
            public boolean run(@NonNull TelemetryPacket packet) {
                arm.setRunMode(Motor.RunMode.PositionControl);
                arm.setPositionCoefficient(0.01);
                arm.setTargetPosition(armTickPosition);
                arm.set(1);
                arm.setPositionTolerance(10);
                return true;
            }
        }
        public Action keepPosition() {
            return new KeepPosition();
        }

        public class HighRung implements Action {
            @Override
            public boolean run(@NonNull TelemetryPacket packet) {
                armTickPosition = 2000;
                keepPosition();
                if (arm.atTargetPosition()) {
                    return false;
                } else {
                    return true;
                }
            }
        }
        public Action highRung() {
            return new HighRung();
        }
        public class HighRung2 implements Action {
            @Override
            public boolean run(@NonNull TelemetryPacket packet) {
                armTickPosition = 1300;
                if (arm.atTargetPosition()) {
                    return false;
                } else {
                    return true;
                }
            }
        }
        public Action highRung2() {
            return new HighRung2();
        }

        public class Low implements Action {
            @Override
            public boolean run(@NonNull TelemetryPacket packet) {
                armTickPosition = 0;
                if (arm.atTargetPosition()) {
                    return false;
                } else {
                    return true;
                }
            }
        }
        public Action low() {
            return new Low();
        }
    }
    public class Claw {
        private ServoEx claw;

        public Claw(HardwareMap hardwareMap) {
            claw = new SimpleServo(hardwareMap, "specimenClaw", 0, 180);
        }
        public class CloseClaw implements Action {
            @Override
            public boolean run(@NonNull TelemetryPacket packet) {
                claw.turnToAngle(closeClawAngle);
                return false;
            }
        }
        public Action closeClaw() {
            return new CloseClaw();
        }
        public class OpenClaw implements Action {
            @Override
            public boolean run(@NonNull TelemetryPacket packet) {
                claw.turnToAngle(openClawAngle);
                return false;
            }
        }
        public Action openClaw() {
            return new OpenClaw();
        }
    }

    public class NormalClaw {
        private ServoEx claw;

        public NormalClaw(HardwareMap hardwareMap) {
            claw = new SimpleServo(hardwareMap, "claw", 0, 180);
        }
        public class CloseClaw implements Action {
            @Override
            public boolean run(@NonNull TelemetryPacket packet) {
                claw.turnToAngle(closeClawAngle);
                if (claw.getAngle() == closeClawAngle) {
                    return false;
                } else {
                    return true;
                }
            }
        }
        public Action closeClaw() {
            return new CloseClaw();
        }
        public class OpenClaw implements Action {
            @Override
            public boolean run(@NonNull TelemetryPacket packet) {
                claw.turnToAngle(openClawAngle);
                if (claw.getAngle() == openClawAngle) {
                    return false;
                } else {
                    return true;
                }
            }
        }
        public Action openClaw() {
            return new NormalClaw.OpenClaw();
        }
    }


    public class NormalArm {
        private Motor arm;

        public NormalArm(HardwareMap hardwareMap) {
            arm = new Motor(hardwareMap, "arm");
            arm.setZeroPowerBehavior(Motor.ZeroPowerBehavior.BRAKE);
            arm.setInverted(true);
            arm.resetEncoder();
        }

        public class KeepPosition implements Action {
            @Override
            public boolean run(@NonNull TelemetryPacket packet) {
                arm.setRunMode(Motor.RunMode.PositionControl);
                arm.setPositionCoefficient(0.01);
                arm.setTargetPosition(normalArmTickPosition);
                arm.set(0.75);
                arm.setPositionTolerance(10);
                return true;
            }
        }
        public Action keepPosition() {
            return new NormalArm.KeepPosition();
        }

        public class HighBasket implements Action {
            @Override
            public boolean run(@NonNull TelemetryPacket packet) {
                armTickPosition = 2500;
                keepPosition();
                if (arm.atTargetPosition()) {
                    return false;
                } else {
                    return true;
                }
            }
        }
        public Action highBasket() {
            return new NormalArm.HighBasket();
        }
        public class Mid implements Action {
            @Override
            public boolean run(@NonNull TelemetryPacket packet) {
                armTickPosition = 1900;
                keepPosition();
                if (arm.atTargetPosition()) {
                    return false;
                } else {
                    return true;
                }
            }
        }
        public Action mid() {
            return new NormalArm.Mid();
        }

        public class Low implements Action {
            @Override
            public boolean run(@NonNull TelemetryPacket packet) {
                armTickPosition = 0;
                if (arm.atTargetPosition()) {
                    return false;
                } else {
                    return true;
                }
            }
        }
        public Action low() {
            return new NormalArm.Low();
        }
    }

}

