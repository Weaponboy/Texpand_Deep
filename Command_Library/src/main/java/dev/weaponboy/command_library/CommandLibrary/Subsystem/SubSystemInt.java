package dev.weaponboy.command_library.CommandLibrary.Subsystem;

import dev.weaponboy.command_library.CommandLibrary.Commands.Command;

public interface SubSystemInt {

    void init();

    void execute();

    Command returnDefaultCommand();

    void setDefaultCommand(Command defaultCommand);

    Command getCurrentCommand();

    void queueCommand(Command command);

    void clearQueue();

    void overrideCurrent(boolean override, Command command);

}
