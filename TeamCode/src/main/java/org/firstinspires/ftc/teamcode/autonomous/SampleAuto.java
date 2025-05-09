package org.firstinspires.ftc.teamcode.autonomous;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

import org.firstinspires.ftc.teamcode.drivetrain.ActionSequencer;
import org.firstinspires.ftc.teamcode.drivetrain.DrivetrainActions;
import org.firstinspires.ftc.teamcode.drivetrain.EncoderHandler;
import org.firstinspires.ftc.teamcode.drivetrain.Localizer;
import org.firstinspires.ftc.teamcode.drivetrain.GotoPointAction;
import org.firstinspires.ftc.teamcode.drivetrain.MoveSlidesPlaceholderAction;

@Autonomous(name = "Sample Auto [Reworked]", group = "Autonomous")
public class SampleAuto extends LinearOpMode {

    @Override
    public void runOpMode() {
        // Initialize hardware
        EncoderHandler encoderHandler = new EncoderHandler(hardwareMap);
        Localizer localizer = new Localizer();
        DrivetrainActions drivetrain = new DrivetrainActions(hardwareMap, localizer);
        ActionSequencer sequencer = new ActionSequencer();

        MoveSlidesPlaceholderAction moveSlidesAction1 = new MoveSlidesPlaceholderAction(1200);
        MoveSlidesPlaceholderAction moveSlidesAction2 = new MoveSlidesPlaceholderAction( 0);
        MoveSlidesPlaceholderAction moveSlidesAction3 = new MoveSlidesPlaceholderAction(600);

        // Add actions to the sequencer
        sequencer.addAction(new GotoPointAction(drivetrain, 12, 0, Math.toRadians(90), 1.0, 1.0, true));  // Move forward 12 inches
        sequencer.addAction(new GotoPointAction(drivetrain, 12, 12, Math.toRadians(90), 1.0, 1.0, true)); // Strafe right 12 inches
        sequencer.addAction(new GotoPointAction(drivetrain, 0, 12, Math.toRadians(180), 1.0, 1.0, true)); // Move back to left while turning

        // Add the moveSlidesAction with different trigger conditions
        sequencer.addAction(moveSlidesAction1);
        sequencer.addAction(moveSlidesAction2);
        sequencer.addAction(moveSlidesAction3);

        waitForStart();

        while (opModeIsActive() && !sequencer.isComplete()) {
            // Pass encoderHandler to localizer for position update
            localizer.updatePosition();  // Update position tracking using encoderHandler
            sequencer.run();             // Run current action

            // Example of setting trigger conditions dynamically:
            if (localizer.getX() > 5 && !moveSlidesAction1.isComplete()) {
                moveSlidesAction1.setTriggerCondition(true);  // Trigger first slide move action
            }

            if (localizer.getX() > 10 && !moveSlidesAction2.isComplete()) {
                moveSlidesAction2.setTriggerCondition(true);  // Trigger second slide move action
            }

            if (localizer.getX() > 15 && !moveSlidesAction3.isComplete()) {
                moveSlidesAction3.setTriggerCondition(true);  // Trigger third slide move action
            }
        }
    }
}
