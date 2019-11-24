using System;

namespace StepperMotorDriverTest
{
    class Program
    {
        static void Main(string[] args)
        {
            Console.WriteLine("Hello Motor Controller!");
            var motorController = new StepperMotorController();

            Console.WriteLine("Type steps maxSpeed(steps/second) acceleration(steps/s²)");
            Console.WriteLine("Type \"end\" to quit");
            string input;
            do
            {
                input = Console.ReadLine();

                var inputArgs = input.Split(' ');
                int.TryParse(inputArgs[0], out int steps);
                int.TryParse(inputArgs[1], out int maxSpeed);
                int.TryParse(inputArgs[2], out int acceleration);

                Console.WriteLine($"Driving motor {steps} with max speed of {maxSpeed} steps/second and acceleration of {acceleration} steps/s²");
                motorController.DriveMotor(steps, maxSpeed, acceleration);
                Console.WriteLine($"Driving finished");
            }
            while (input != "end");

        }
    }
}
