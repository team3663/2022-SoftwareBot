// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.log;

import java.io.File;
import java.io.PrintStream;
import java.util.Scanner;

/** Add your docs here. */
public class Log {

    public static void main(String[] args) throws Exception {
        File inputFile = new File(".\\src\\log\\input.txt");
        Scanner input = new Scanner(inputFile);

        File outputFile = new File(".\\src\\log\\output.txt");
        PrintStream output = new PrintStream(outputFile);

        while (input.hasNextLine()) {
            output.println(input.nextLine());
        }
        input.close();
    }
}
