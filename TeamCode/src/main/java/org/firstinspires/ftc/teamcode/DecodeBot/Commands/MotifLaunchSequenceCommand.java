package org.firstinspires.ftc.teamcode.DecodeBot.Commands;

import static org.firstinspires.ftc.teamcode.DecodeBot.Subsystems.BotPositions.GATE_WAIT;
import static org.firstinspires.ftc.teamcode.DecodeBot.Subsystems.BotPositions.INTAKE_WAIT;
import static org.firstinspires.ftc.teamcode.DecodeBot.Subsystems.BotPositions.KICKER_WAIT;
import static org.firstinspires.ftc.teamcode.DecodeBot.Subsystems.BotPositions.SWAP_WAIT;
import static org.firstinspires.ftc.teamcode.DecodeBot.Subsystems.GlobalVariables.currentArtifacts;
import static org.firstinspires.ftc.teamcode.DecodeBot.Subsystems.GlobalVariables.motif;

import com.arcrobotics.ftclib.command.Command;
import com.arcrobotics.ftclib.command.InstantCommand;
import com.arcrobotics.ftclib.command.ParallelCommandGroup;
import com.arcrobotics.ftclib.command.SequentialCommandGroup;
import com.arcrobotics.ftclib.command.WaitCommand;

import org.firstinspires.ftc.teamcode.DecodeBot.Subsystems.GlobalVariables;
import org.firstinspires.ftc.teamcode.DecodeBot.Subsystems.Intake;
import org.firstinspires.ftc.teamcode.DecodeBot.Subsystems.Storage;

import java.util.ArrayDeque;
import java.util.ArrayList;
import java.util.Deque;
import java.util.List;

public class MotifLaunchSequenceCommand extends SequentialCommandGroup {

    // External subsystems used by commands
    private final Intake intake;
    private final Storage storage;

    // wait constants (use your existing BotPositions constants)


    public MotifLaunchSequenceCommand(Intake intake, Storage storage) {
        this.intake = intake;
        this.storage = storage;

       intake.setCurrentArtifacts();


        // Parse currentArtifacts
        char storageSlot = currentArtifacts.charAt(0);
        // normalize spaces to '_' and upper-case the chars just in case:
        storageSlot = (storageSlot == '_' ? '_' : Character.toUpperCase(storageSlot));

        // Line is a queue representing slots 1..3 (front/closest to shooter is first)
        Deque<Character> line = new ArrayDeque<>();
        for (int i = 1; i <= 3; i++) {
            char c = currentArtifacts.charAt(i);
            if (c == '_') c = '_';
            else c = Character.toUpperCase(c);
            // keep spaces as spaces to preserve empty slots
            line.addLast(c);
        }

        // Build sequence of commands
        List<Command> seq = new ArrayList<>();

        // Make sure gate is open before any launch operations — we'll open on demand.
        // We'll track whether we've opened the gate already so we don't unnecessarily schedule multiple opens.
        boolean gateOpenScheduled = false;

        for (int i = 0; i < motif.length(); i++) {
            char target = Character.toUpperCase(motif.charAt(i)); // 'G' or 'P'

            // Ensure line front is the first element (may be space)
            char front = line.peekFirst();

            // CASE 1: target already at shooter slot
            if (front == target) {
                // ensure gate open
                if (!gateOpenScheduled) {
                    seq.add(openGate(storage));
                    gateOpenScheduled = true;
                }
                // launch front
                seq.add(launchOne(storage));
                // update model: front becomes empty
                line.removeFirst();
                line.addFirst('_');
                continue;
            }

            // CASE 2: target is in storage slot
            if (storageSlot == target) {
                // unstore: bring storage into front (this generally requires gate open depending on your mechanism;
                // we will ensure gate is open for launch operations)
                seq.add(unstore(storage));      // places stored ball into front
                storageSlot = '_';
                // ensure gate open for launch
                if (!gateOpenScheduled) {
                    seq.add(openGate(storage));
                    gateOpenScheduled = true;
                }
                seq.add(launchOne(storage));
                // update model
                line.removeFirst();
                line.addFirst('_');
                continue;
            }

            // CASE 3: target is deeper in line (in slot 2 or 3)
            // Step A: if front isn't empty, store it to open up room to shift
            boolean frontStored = false;
            if (front != '_' && storageSlot == '_') {
                // close gate before storing (automatic gate control)
                seq.add(closeGate(storage));
                seq.add(store(storage));       // store front into storage slot
                storageSlot = front;
                // update model: front becomes empty
                line.removeFirst();
                line.addFirst('_');
                frontStored = true;
            } else if (front != '_' && storageSlot != '_') {
                // Edge-case: storage already occupied. We must free storage first by unstore->launch or unstore->put front.
                // Simpler deterministic strategy: unstore to front (so storage becomes free), then store new front.
                // ensure gate open for unstore/launch combination if we need to launch a stored ball
                seq.add(unstore(storage)); // put stored ball to front
                // model: move stored into front
                char storedBall = storageSlot;
                storageSlot = '_';
                // push old front to next positions by treating unstore as placing in front (we simulate by inserting)
                line.removeFirst();
                line.addFirst(storedBall);

                // Now we can store the current front to free space
                seq.add(closeGate(storage));
                seq.add(store(storage));
                storageSlot = line.removeFirst(); // stored front
                line.addFirst('_');
                frontStored = true;
            }

            // Step B: shift until the target appears at front
            // We assume the target exists somewhere in the 3-slot line
            int safetyCounter = 0;
            while (line.peekFirst() != target && safetyCounter <= 3) {
                seq.add(moveOne(intake));
                // simulate moveOne shift:
                char a = line.removeFirst();
                char b = line.removeFirst();
                char c = line.removeFirst();
                // shift left: new front is b, middle becomes c, back becomes '_'
                line.addFirst(b);
                line.removeFirst(); // remove the old second we reinserted (we manipulated too many times)

                safetyCounter++;
            }

            // After target at front, ensure gate opened and launch
            if (!gateOpenScheduled) {
                seq.add(openGate(storage));
                gateOpenScheduled = true;
            }
            seq.add(launchOne(storage));
            // simulate removal from front
            line.removeFirst();
            line.addFirst('_');

            // Step C: if we had stored a front earlier, return it to the front (unstore)
            if (frontStored && storageSlot != '_') {
                seq.add(unstore(storage));
                // put stored back into front
                line.removeFirst();
                line.addFirst(storageSlot);
                storageSlot = '_';
            }
        }

        // Finally, ensure shot counter reset
        seq.add(new InstantCommand(() -> GlobalVariables.ballsShot = 0));

        // Add all commands as a single sequential group
        addCommands(new SequentialCommandGroup(seq.toArray(new Command[0])));
    }

    // ---------------------------
    // Helper commands (return Command objects so they can be composed)
    // ---------------------------

    public Command launchOne(Storage s) {
        return new ParallelCommandGroup(
                new SequentialCommandGroup(
                        new InstantCommand(s::raiseKicker),
                        new WaitCommand(KICKER_WAIT),
                        new InstantCommand(s::lowerKicker),
                        new WaitCommand(KICKER_WAIT)
                ),
                new InstantCommand(() -> GlobalVariables.ballsShot += 1)
        );
    }

    public Command moveOne(Intake i) {
        return new SequentialCommandGroup(
                new InstantCommand(i::in),
                new WaitCommand(INTAKE_WAIT),
                new InstantCommand(i::stop)
        );
    }

    public Command openGate(Storage s) {
        return new SequentialCommandGroup(
                new InstantCommand(s::openGate),
                new WaitCommand(GATE_WAIT)
        );
    }

    public Command closeGate(Storage s) {
        return new SequentialCommandGroup(
                new InstantCommand(s::closeGate),
                new WaitCommand(GATE_WAIT)
        );
    }

    public Command unstore(Storage s) {
        return new SequentialCommandGroup(
                new InstantCommand(s::returnSlot),
                new WaitCommand(SWAP_WAIT)
        );
    }

    public Command store(Storage s) {
        return new SequentialCommandGroup(
                new InstantCommand(s::storeSlot),
                new WaitCommand(SWAP_WAIT)
        );
    }
}
