package dev.weaponboy.command_library.Subsystem;

import java.util.ArrayList;

import dev.weaponboy.command_library.Commands.Command;
import dev.weaponboy.command_library.OpmodeEX.OpModeEX;

@SuppressWarnings("unused")
public abstract class SubSystem implements SubSystemInt{

    OpModeEX opModeEX;

    Command defaultCommand;
    Command currentCommand;
    ArrayList<Command> commands = new ArrayList<>();

    public void registerSubsystem(OpModeEX opModeEX, Command defaultCommand){
        this.opModeEX = opModeEX;
        this.defaultCommand = defaultCommand;
        this.currentCommand = defaultCommand;
    }

    protected OpModeEX getOpModeEX() {
        return opModeEX;
    }

    protected void executeEX(){

        System.out.println(currentCommand.isFinished());

        if (currentCommand.isFinished() && !commands.isEmpty()){
            currentCommand = commands.get(0);
            commands.remove(currentCommand);
            currentCommand.init();
        }else if (currentCommand.isFinished() && commands.isEmpty()){
            currentCommand = defaultCommand;
            currentCommand.init();
        }

        currentCommand.execute();
    }

    @Override
    public void setDefaultCommand(Command defaultCommand) {
        this.defaultCommand = defaultCommand;
    }

    @Override
    public Command returnDefaultCommand() {
        return currentCommand;
    }

    @Override
    public Command getCurrentCommand() {
        return currentCommand;
    }

    @Override
    public void queueCommand(Command command){
        commands.add(command);
    }

    @Override
    public void clearQueue(){
        commands.clear();
    }

    @Override
    public void overrideCurrent(boolean override, Command command){
        if (override){
            commands.clear();
            currentCommand = command;
        }
    }

}
