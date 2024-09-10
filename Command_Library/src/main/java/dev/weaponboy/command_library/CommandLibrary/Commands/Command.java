package dev.weaponboy.command_library.CommandLibrary.Commands;

public interface Command {

    void execute();

    void init();

    boolean isFinished();

}
