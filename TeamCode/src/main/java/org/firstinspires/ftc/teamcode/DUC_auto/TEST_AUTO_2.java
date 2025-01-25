package org.firstinspires.ftc.teamcode.DUC_auto;

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
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.teamcode.MecanumDrive;
import org.firstinspires.ftc.teamcode.PinpointDrive;

@Config
@Autonomous(name = "TEST2", group = "Autonomous")
public class TEST_AUTO_2 extends LinearOpMode {

    int armTickPosition = 0;
    int normalArmTickPosition = 0;
    int spoolTickPosition = 0;

    public static int armAddition = 2;
    public static double ypos1 = 0.5;
    public static double xpos1 = 0.5;
    public static double ypos2 = 2;
    public static double xpos2 = 1.5;


    @Override
    public void runOpMode() throws InterruptedException {
        Pose2d initialPose = new Pose2d(11, -64, Math.toRadians(270));
        MecanumDrive drive = new PinpointDrive(hardwareMap, initialPose);
        Spool spool = new Spool(hardwareMap);
        Claw claw = new Claw(hardwareMap);
        NormalClaw normalClaw = new NormalClaw(hardwareMap);
        Arm arm = new Arm(hardwareMap);
        NormalArm normalArm = new NormalArm(hardwareMap);
        TimerActions time = new TimerActions();

        TrajectoryActionBuilder trajectory0 = drive.actionBuilder(new Pose2d(11, -63.21, Math.toRadians(270.00)))
                .strafeTo(new Vector2d(2.29, -32));

        TrajectoryActionBuilder trajectory1 = drive.actionBuilder(new Pose2d(2.29, -32, Math.toRadians(270.00)))
                //.strafeTo(new Vector2d(5.29, -35.29))
                .strafeToLinearHeading(new Vector2d(33 + xpos1, -46 + ypos1), Math.toRadians(35.00));

        TrajectoryActionBuilder trajectory2 = drive.actionBuilder(new Pose2d(32 + xpos1, -46 + ypos1, Math.toRadians(35.00)))
                .turnTo(Math.toRadians(300.00));

        TrajectoryActionBuilder trajectory3 = drive.actionBuilder(new Pose2d(32 + xpos1, -46 + ypos1, Math.toRadians(310.00)))
                .strafeToLinearHeading(new Vector2d(40 + xpos2, -46 + ypos2), Math.toRadians(40.00));

        TrajectoryActionBuilder trajectory4 = drive.actionBuilder(new Pose2d(40 + xpos2, -46 + ypos2, Math.toRadians(40.00)))
                .turnTo(Math.toRadians(310.00));

        TrajectoryActionBuilder beforePicking = drive.actionBuilder(new Pose2d(41 + xpos2, -46 + ypos2, Math.toRadians(310.00)))
                .strafeToLinearHeading(new Vector2d(45, -56), Math.toRadians(90));

        TrajectoryActionBuilder picking = drive.actionBuilder(new Pose2d(45, -52.29, Math.toRadians(90.00)))
                .strafeToLinearHeading(new Vector2d(45, -64), Math.toRadians(90));

        TrajectoryActionBuilder rung = drive.actionBuilder(new Pose2d(45, -64, Math.toRadians(90.00)))
                .strafeToLinearHeading(new Vector2d(8.29, -31.29), Math.toRadians(270));

        TrajectoryActionBuilder beforePickingFromRung = drive.actionBuilder(new Pose2d(3.29, -31.29, Math.toRadians(270.00)))
                .strafeToLinearHeading(new Vector2d(45, -56), Math.toRadians(90));


        Actions.runBlocking(claw.closeClaw());
        Actions.runBlocking(normalClaw.openClaw());

        waitForStart();
        if (isStopRequested()) return;

        Actions.runBlocking(
                new ParallelAction(
                        new SequentialAction(
                                new ParallelAction(
                                        trajectory0.build(),
                                        arm.highRung()
                                ),
                                time.resetTimer(),
                                arm.highRung2(),
                                new SleepAction(0.25),
                                claw.openClaw(),
                                trajectory1.build(),
                                time.resetTimer(),
                                new ParallelAction(
                                        normalArm.mid(),
                                        arm.low()
                                ),
                                time.resetTimer(),
                                spool.highBasket(),
                                new SleepAction(0.75),
                                normalClaw.closeClaw(),
                                trajectory2.build(),
                                normalClaw.openClaw(),
                                trajectory3.build(),
                                normalClaw.closeClaw(),
                                trajectory4.build(),
                                normalClaw.openClaw(),
                                time.resetTimer(),
                                new ParallelAction(
                                        spool.low(),
                                        beforePicking.build()
                                ),
                                time.resetTimer(),
                                normalArm.low(),
                                claw.openClaw(),
                                picking.build(),
                                claw.closeClaw(),
                                new SleepAction(0.1),
                                new ParallelAction(
                                        arm.highRung(),
                                        rung.build()
                                ),
                                time.resetTimer(),
                                arm.highRung2(),
                                new SleepAction(0.25),
                                claw.openClaw(),
                                time.resetTimer(),
                                new ParallelAction(
                                        arm.low(),
                                        beforePickingFromRung.build()
                                ),
                                picking.build(),
                                claw.closeClaw(),
                                new SleepAction(0.1),
                                new ParallelAction(
                                        arm.highRung(),
                                        rung.build()
                                ),
                                time.resetTimer(),
                                arm.highRung2(),
                                new SleepAction(0.25),
                                claw.openClaw(),
                                time.resetTimer(),
                                new ParallelAction(
                                        arm.low(),
                                        beforePickingFromRung.build()
                                ),
                                picking.build(),
                                claw.closeClaw(),
                                new SleepAction(0.1),
                                new ParallelAction(
                                        arm.highRung(),
                                        rung.build()
                                ),
                                time.resetTimer(),
                                arm.highRung2(),
                                new SleepAction(0.25),
                                claw.openClaw(),
                                time.resetTimer(),
                                new ParallelAction(
                                        arm.low(),
                                        beforePickingFromRung.build()
                                )
                            ),
                        arm.keepPosition(),
                        normalArm.keepPosition(),
                        spool.keepPosition()
                )

        );

    }

    public static class TimerActions {
        public static ElapsedTime timer = new ElapsedTime();
        public TimerActions() {
            timer.reset();
        }
        public class ResetTimer implements Action {
            @Override
            public boolean run(@NonNull TelemetryPacket packet) {
                timer.reset();
                return false;
            }
        }
        public Action resetTimer() {
            return new ResetTimer();
        }

        public static double getTime() {
            return timer.milliseconds();
        }
    }


    public class Spool {
        private Motor spool;

        public Spool(HardwareMap hardwareMap) {
            spool = new Motor(hardwareMap, "spool");
            spool.setZeroPowerBehavior(Motor.ZeroPowerBehavior.BRAKE);
            spool.resetEncoder();
        }

        public class HighBasket implements Action {
            @Override
            public boolean run(@NonNull TelemetryPacket packet) {
                spoolTickPosition = 2600;
                double time = TimerActions.getTime();
                if (spool.atTargetPosition() || time >= 3000) {
                    return false;
                } else {
                    return true;
                }
            }
        }
        public Action highBasket() {
            return new Spool.HighBasket();
        }

        public class Low implements Action {
            @Override
            public boolean run(@NonNull TelemetryPacket packet) {
                spoolTickPosition = 0;
                double time = TimerActions.getTime();
                if (spool.atTargetPosition() || time >= 3000) {
                    return false;
                } else {
                    return true;
                }
            }
        }
        public Action low() {
            return new Spool.Low();
        }

        public class KeepPosition implements Action {
            @Override
            public boolean run(@NonNull TelemetryPacket packet) {
                spool.setRunMode(Motor.RunMode.PositionControl);
                spool.setPositionCoefficient(0.01);
                spool.setTargetPosition(spoolTickPosition);
                spool.set(1);
                spool.setPositionTolerance(20);
                return true;
            }
        }
        public Action keepPosition() {
            return new Spool.KeepPosition();
        }

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
                arm.set(0.75);
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
                packet.put("arm position", arm.getCurrentPosition());
                packet.put("arm target", armTickPosition);
                packet.put("arm at position", arm.atTargetPosition());
                return arm.atTargetPosition() || time >= 3000;
            }
        }
        public Action highRung2() {
            return new HighRung2();
        }

        public class Low implements Action {
            @Override
            public boolean run(@NonNull TelemetryPacket packet) {
                armTickPosition = 0;
                double time = TimerActions.getTime();
                packet.put("timer", time);
                packet.put("armPosition", arm.getCurrentPosition());
                packet.put("arm target position", armTickPosition);
                packet.put("arm at position", arm.atTargetPosition());
                return arm.atTargetPosition() || time >= 3000;
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
            return new OpenClaw();
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
                arm.setPositionTolerance(1);
                return true;
            }
        }
        public Action keepPosition() {
            return new KeepPosition();
        }

        public class HighBasket implements Action {
            @Override
            public boolean run(@NonNull TelemetryPacket packet) {
                normalArmTickPosition = 2500;
                keepPosition();
                if (arm.atTargetPosition()) {
                    return false;
                } else {
                    return true;
                }
            }
        }
        public Action highBasket() {
            return new HighBasket();
        }
        public class Mid implements Action {
            @Override
            public boolean run(@NonNull TelemetryPacket packet) {
                normalArmTickPosition = 555 + armAddition;
                keepPosition();
                if (arm.atTargetPosition()) {
                    keepPosition();
                    return false;
                } else {
                    return true;
                }
            }
        }
        public Action mid() {
            return new Mid();
        }

        public class Low implements Action {
            @Override
            public boolean run(@NonNull TelemetryPacket packet) {
                normalArmTickPosition = 0;
                time = TimerActions.getTime();
                if (arm.atTargetPosition() || time >= 500) {
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

}

