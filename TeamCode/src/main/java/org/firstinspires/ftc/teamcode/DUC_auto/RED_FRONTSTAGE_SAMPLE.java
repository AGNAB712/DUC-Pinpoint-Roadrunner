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
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.teamcode.MecanumDrive;
import org.firstinspires.ftc.teamcode.PinpointDrive;

@Config
@Autonomous(name = "1RED FRONTSTAGE SAMPLE", group = "Autonomous")
public class RED_FRONTSTAGE_SAMPLE extends LinearOpMode {

    int armTickPosition = 0;
    int normalArmTickPosition = 0;
    int spoolTickPosition = 0;

    public static int armAddition = 8;
    public static double ypos1 = -5;
    public static double xpos1 = -2;
    public static double ypos2 = 2;
    public static double xpos2 = 1.5;


    @Override
    public void runOpMode() throws InterruptedException {
        Pose2d initialPose = new Pose2d(-32, -64, Math.toRadians(1-180));
        MecanumDrive drive = new PinpointDrive(hardwareMap, initialPose);
        Spool spool = new Spool(hardwareMap);
        Claw claw = new Claw(hardwareMap);
        NormalClaw normalClaw = new NormalClaw(hardwareMap);
        Arm arm = new Arm(hardwareMap);
        NormalArm normalArm = new NormalArm(hardwareMap);
        TimerActions time = new TimerActions();
        drive.precision = 0;
        drive.precisionTime = 0;
        class PrecisionActions {
            class Precise implements Action {
                @Override
                public boolean run(@NonNull TelemetryPacket packet) {
                    drive.precision = 0.5;
                    drive.precisionTime = 0.5;
                    return false;
                }
            }
            class VeryPrecise implements Action {
                @Override
                public boolean run(@NonNull TelemetryPacket packet) {
                    drive.precision = 0.1;
                    drive.precisionTime = 3;
                    return false;
                }
            }
            class NotPrecise implements Action {
                @Override
                public boolean run(@NonNull TelemetryPacket packet) {
                    drive.precision = 0;
                    drive.precisionTime = 0;
                    return false;
                }
            }
            public Action precise() {
                return new Precise();
            }
            public Action veryPrecise() {
                return new VeryPrecise();
            }
            public Action notPrecise() {
                return new NotPrecise();
            }
        }
        PrecisionActions precisionController = new PrecisionActions();

        TrajectoryActionBuilder trajectory0 = drive.actionBuilder(initialPose) //from initial pose to high basket
                .strafeToLinearHeading(new Vector2d(-58, -58), Math.toRadians(45-180))
                .strafeTo(new Vector2d(-61, -61));

        TrajectoryActionBuilder trajectory1 = drive.actionBuilder(new Pose2d(-61, -61, Math.toRadians(45-180))) //basket to first sample
                .strafeToLinearHeading(new Vector2d(-46.5, -45), Math.toRadians(270-180));

        TrajectoryActionBuilder trajectory2 = drive.actionBuilder(new Pose2d(-47, -45, Math.toRadians(270-180))) //first sample to basket
                .strafeToLinearHeading(new Vector2d(-56, -54), Math.toRadians(224-180))
                .strafeToLinearHeading(new Vector2d(-58, -48), Math.toRadians(45-180))
                .strafeTo(new Vector2d(-59, -63.5));

        TrajectoryActionBuilder trajectory3setup = drive.actionBuilder(new Pose2d(-61, -61, Math.toRadians(45-180))) //basket to second sample
                .strafeToLinearHeading(new Vector2d(-56, -54), Math.toRadians(224-180));

        TrajectoryActionBuilder trajectory3 = drive.actionBuilder(new Pose2d(-56, -54, Math.toRadians(224-180))) //basket to second sample
                .strafeToLinearHeading(new Vector2d(-59.5, -45), Math.toRadians(270-180));

        TrajectoryActionBuilder trajectory2setup = drive.actionBuilder(new Pose2d(-59.5, -45, Math.toRadians(270-180))) //basket to second sample
                .strafeToLinearHeading(new Vector2d(-57.5, -54), Math.toRadians(224-180));

        TrajectoryActionBuilder trajectory2alt = drive.actionBuilder(new Pose2d(-57.5, -54, Math.toRadians(224-180))) //second sample to basket
                .strafeToLinearHeading(new Vector2d(-57+xpos1, -48+ypos1), Math.toRadians(45-180))
                .strafeTo(new Vector2d(-56+xpos1, -61.5+ypos1));

        TrajectoryActionBuilder trajectory2last = drive.actionBuilder(new Pose2d(-60.5+xpos1, -28+ypos1, Math.toRadians(1-180))) //third sample to basket
                .strafeTo(new Vector2d(-56, -28))
                .strafeToLinearHeading(new Vector2d(-60.5, -60.5), Math.toRadians(45-180));

        TrajectoryActionBuilder trajectory4 = drive.actionBuilder(new Pose2d(-59.5, -59, Math.toRadians(45-180))) //basket to third sample
                .strafeToLinearHeading(new Vector2d(-53, -28), Math.toRadians(1-180))
                .strafeTo(new Vector2d(-60.5, -28));

        TrajectoryActionBuilder trajectory5 = drive.actionBuilder(new Pose2d(-59.5, -61, Math.toRadians(45-180))) //park
                .strafeToLinearHeading(new Vector2d(-40, -17), Math.toRadians(1))
                .strafeTo(new Vector2d(-15, -17));







        Actions.runBlocking(claw.openClaw());
        Actions.runBlocking(normalClaw.closeClaw());

        waitForStart();
        if (isStopRequested()) return;

        Actions.runBlocking(
                new ParallelAction(
                        new SequentialAction(
                                new ParallelAction(
                                        trajectory0.build(),
                                        time.resetTimer(),
                                        normalArm.highBasket()
                                ),
                                time.resetTimer(),
                                spool.highBasket(),
                                new SleepAction(0.85),
                                normalClaw.openClaw(),
                                new SleepAction(0.1),
                                spool.mid(),
                                new SleepAction(0.35),
                                precisionController.precise(),
                                new ParallelAction(
                                        new SequentialAction(
                                                new SleepAction(0.25),
                                                time.resetTimer(),
                                                normalArm.mid()
                                        ),
                                        trajectory1.build()
                                ),
                                precisionController.notPrecise(),
                                new SleepAction(0.1),
                                normalClaw.closeClaw(),
                                new SleepAction(0.1),
                                new ParallelAction(
                                        time.resetTimer(),
                                        normalArm.highBasket(),
                                        trajectory2.build()
                                ),
                                time.resetTimer(),
                                spool.highBasket(),
                                new SleepAction(0.85),
                                normalClaw.openClaw(),
                                new SleepAction(0.1),
                                time.resetTimer(),
                                spool.mid(),
                                new SleepAction(0.35),
                                new ParallelAction(
                                        new SequentialAction(
                                                new SleepAction(0.5),
                                                time.resetTimer(),
                                                normalArm.mid()
                                        ),
                                        new SequentialAction(
                                                trajectory3setup.build(),
                                                precisionController.precise(),
                                                trajectory3.build()
                                        )
                                ),
                                precisionController.notPrecise(),
                                new SleepAction(0.1),
                                normalClaw.closeClaw(),
                                new SleepAction(0.1),
                                new ParallelAction(
                                        time.resetTimer(),
                                        normalArm.highBasket(),
                                        new SequentialAction(
                                                trajectory2setup.build(),
                                                trajectory2alt.build()
                                        )
                                ),
                                time.resetTimer(),
                                spool.highBasket(),
                                new SleepAction(0.85),
                                normalClaw.openClaw(),
                                new SleepAction(0.1),
                                time.resetTimer(),
                                spool.low(),
                                new SleepAction(0.25),
                                new ParallelAction(
                                        new SequentialAction(
                                                new SleepAction(0.25),
                                                normalArm.low()
                                        ),
                                        trajectory4.build()
                                ),
                                new SleepAction(0.1),
                                normalClaw.closeClaw(),
                                new SleepAction(0.25),
                                new ParallelAction(
                                        time.resetTimer(),
                                        new SequentialAction(new SleepAction(0.75), normalArm.highBasket()),
                                        trajectory2last.build()
                                ),
                                time.resetTimer(),
                                spool.highBasket(),
                                new SleepAction(0.85),
                                normalClaw.openClaw(),
                                new SleepAction(0.1),
                                time.resetTimer(),
                                new ParallelAction(
                                        spool.low(),
                                        trajectory5.build(),
                                        new SequentialAction(
                                                new SleepAction(2),
                                                normalArm.ascent()
                                        )
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
                spoolTickPosition = 2000;
                return false;
            }
        }
        public Action highBasket() {
            return new HighBasket();
        }

        public class Low implements Action {
            @Override
            public boolean run(@NonNull TelemetryPacket packet) {
                spoolTickPosition = 0;
                return false;
            }
        }
        public Action low() {
            return new Low();
        }

        public class Mid implements Action {
            @Override
            public boolean run(@NonNull TelemetryPacket packet) {
                spoolTickPosition = 500;
                return false;
            }
        }
        public Action mid() {
            return new Mid();
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
            return new KeepPosition();
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
                arm.setPositionTolerance(5);
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
                double time = TimerActions.getTime();
                if (arm.atTargetPosition() || time >= 2000) {
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
                normalArmTickPosition = 250 + armAddition;
                keepPosition();
                double time = TimerActions.getTime();
                if (arm.atTargetPosition() || time >= 2500) {
                    return false;
                } else {
                    return true;
                }
            }
        }
        public Action mid() {
            return new Mid();
        }

        public class Ascent implements Action {
            @Override
            public boolean run(@NonNull TelemetryPacket packet) {
                normalArmTickPosition = 1400;
                keepPosition();
                double time = TimerActions.getTime();
                if (arm.atTargetPosition() || time >= 2500) {
                    return false;
                } else {
                    return true;
                }
            }
        }
        public Action ascent() {
            return new Ascent();
        }

        public class Low implements Action {
            @Override
            public boolean run(@NonNull TelemetryPacket packet) {
                normalArmTickPosition = 20;
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

