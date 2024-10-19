package tutorial;

import java.util.concurrent.TimeUnit;

/** "Hello world" with some ANSI escape codes
 *
 *  See https://en.wikipedia.org/wiki/ANSI_escape_code
 */
public class HelloColors
{
    public static void main(String[] args) throws Exception
    {
        // Start of escape sequence   \033[
        // Clear                      2J (standalone, not closed with 'm')
        // Foreground color           30 .. 37
        // Background color           40 .. 47
        // Bold                       1
        // Blink                      5
        // Combine several elements   ;
        // Back to normal             0
        // End of escape sequence     m
        System.out.println("\033[2J");
        System.out.println("\033[32mHello, \033[0;41;1mcolorful\033[0m \033[34;1mWorld!\033[0m");

        // This prints the "bell".
        // Within the VS Code terminal, nothing happens,
        // unless you change the settings for "bell".
        // Try executing it in the Windows 'cmd' command prompt':
        // cd git\FRC2025
        // \Users\Public\wpilib\2025\jdk\bin\java src\main\java\tutorial\HelloColors.java
        System.out.println("\007");
        TimeUnit.SECONDS.sleep(2);
        System.out.println();
        System.out.println();
        System.out.println();
        System.out.println();
        System.out.println();
        System.out.println("Yesterday, my life was \033[31mboooring...");
        TimeUnit.SECONDS.sleep(2);
        System.out.println("\033[32mNow,");
        TimeUnit.SECONDS.sleep(1);
        System.out.println("      I'm using \033[31;42;1;5mJava!!\033[0m");
        for (int i=0;  i<10; ++i)
        {
            TimeUnit.MILLISECONDS.sleep(300);
            System.out.println();
        }
        System.out.println("\033[1mBye!\033[0m");
        System.out.println();
        System.out.println();
    }
}
