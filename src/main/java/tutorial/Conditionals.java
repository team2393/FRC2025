package tutorial;

public class Conditionals
{
    public static void main(String[] args)
    {
        boolean myCondition = true;

        if (myCondition)
        {
            System.out.println("True!");
        }

        if (!myCondition)
        {
            System.out.println("False!");
        }
        else
        {
            System.out.println("True this time!");
        }

        int myNumber = 5;

        if (myNumber > 5)
        {
            System.out.println("Greater than ten!");
        }
        else if (myNumber < 5)
        {
            System.out.println("Less than ten!");
        }
        else
        {
            System.out.println("Equals 5!");
        }

    }
}
