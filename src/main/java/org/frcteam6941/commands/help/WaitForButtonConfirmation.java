package org.frcteam6941.commands.help;

import java.util.function.BooleanSupplier;

import edu.wpi.first.wpilibj2.command.CommandBase;

/**Simple wait command that finishes when a certain button is pressed. */
public class WaitForButtonConfirmation extends CommandBase{
    BooleanSupplier bSupplier;

    /**
     * Create a wait command that finishes when a certain button is pressed.
     * @param booleanSupplier Boolean supplier.
     */
    public WaitForButtonConfirmation(BooleanSupplier booleanSupplier){
        bSupplier = booleanSupplier;
    }

    @Override
    public void initialize(){
    }

    @Override
    public void execute(){
    }

    @Override
    public void end(boolean iterrupted){
    }

    @Override
    public boolean isFinished(){
        return bSupplier.getAsBoolean();
    }
}
