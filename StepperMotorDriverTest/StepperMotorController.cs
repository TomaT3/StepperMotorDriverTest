using System;
using System.Collections.Generic;
using System.Device.Gpio;
using System.Diagnostics;
using System.Linq;
using System.Text;
using System.Threading;
using System.Threading.Tasks;

namespace StepperMotorDriverTest
{
    public class GpioPin
    {
        private readonly GpioController _controller;
        private readonly int _pinNumber;
        public GpioPin(GpioController controller, int pinNumber, PinMode pinMode)
        {
            _controller = controller;
            _pinNumber = pinNumber;

            controller.OpenPin(_pinNumber, pinMode);
        }

        public void Write(PinValue pinValue)
        {
            _controller.Write(_pinNumber, pinValue);
        }

        public PinValue Read()
        {
            return _controller.Read(_pinNumber);
        }
    }

    public class StepperMotorController
    {
        private static PinValue FORWARD = PinValue.High;
        private static PinValue BACKWARD = PinValue.Low;
        private static PinValue ENA_RELEASED = PinValue.High;
        private static PinValue ENA_LOCKED = PinValue.Low;

        GpioPin _enablePin;
        GpioPin _directionPin;        
        GpioPin _stepPin;

        public StepperMotorController()
        {
            GpioController controller = new GpioController();
            _enablePin = new GpioPin(controller, 17, PinMode.Output);
            _enablePin.Write(ENA_RELEASED);

            _directionPin = new GpioPin(controller, 27, PinMode.Output);
            _directionPin.Write(FORWARD);

            _stepPin = new GpioPin(controller, 22, PinMode.Output);
            _stepPin.Write(PinValue.Low);
        }

        public void DriveMotor(int steps, int maxSpeed, int acceleration)
        {
            var numberOfStepsForAcceleration = Convert.ToInt32(Math.Pow(maxSpeed, 2) / acceleration / 2);
            var numberOfStepsForDecceleration = 0;
            var numberOfStepsWithMaxSpeed = 0;


            if((2 * numberOfStepsForAcceleration) < steps)
            {
                numberOfStepsForDecceleration = numberOfStepsForAcceleration;
                numberOfStepsWithMaxSpeed = steps - numberOfStepsForAcceleration - numberOfStepsForDecceleration;
            }
            else
            {
                numberOfStepsForAcceleration = steps / 2;
                numberOfStepsForDecceleration = steps / 2;
            }

            var waitTimeBetweenSteps = CalculateWaitTimeBetweenSteps(
                numberOfStepsForAcceleration,
                numberOfStepsWithMaxSpeed,
                numberOfStepsForDecceleration,
                maxSpeed,
                acceleration);

            Console.WriteLine($"NumberOfSteps: {waitTimeBetweenSteps.Length} TimeForDriving: {waitTimeBetweenSteps.Sum()}");
            Console.WriteLine($"ShortestWaitTime: {waitTimeBetweenSteps.Min()} LongestWaitTime: {waitTimeBetweenSteps.Max()}");
            SetOutput(waitTimeBetweenSteps); //waitTimeBetweenSteps.Select(x => Convert.ToInt32(x)).ToArray()
        }

        private void SetOutput(double[] waitTimeBetweenSteps)
        {
            _enablePin.Write(ENA_LOCKED);
            NOP(500);

            for (int i = 0; i < waitTimeBetweenSteps.Length; i++)
            {
                _stepPin.Write(PinValue.High);
                NOP(waitTimeBetweenSteps[i]);

                _stepPin.Write(PinValue.Low);
                NOP(waitTimeBetweenSteps[i]);
            }

            _enablePin.Write(ENA_RELEASED);
        }

        private double[] CalculateWaitTimeBetweenSteps(
            int numberOfStepsForAcceleration,
            int numberOfStepsWithMaxSpeed,
            int numberOfStepsForDecceleration,
            int maxSpeed,
            int acceleration)
        {
            double[] waitTimeBetweenAccelerationSteps = new double[numberOfStepsForAcceleration];
            double[] waitTimeBetweenMaxSpeedSteps = new double[numberOfStepsWithMaxSpeed];
            double[] waitTimeBetweenDeccelerationSteps = new double[numberOfStepsForDecceleration];
            
            double[] tempAccelerationArray = GetAccelerationTimeArray(numberOfStepsForAcceleration, acceleration);

            for (int i = 0; i < tempAccelerationArray.Length; i++)
            {
                if (i != tempAccelerationArray.Length - 1)
                    waitTimeBetweenAccelerationSteps[i] = (tempAccelerationArray[i + 1] - tempAccelerationArray[i]) / 2.0;
                else
                    waitTimeBetweenAccelerationSteps[i] = 1.0 / Convert.ToDouble(maxSpeed)/ 2.0;
            }

            for (int i = 0; i < numberOfStepsWithMaxSpeed; i++)
            {
                waitTimeBetweenMaxSpeedSteps[i] = 1 / Convert.ToDouble(maxSpeed) / 2.0;
            }

            int j = 0;
            for (int i = waitTimeBetweenAccelerationSteps.Length - 1; i >= 0; i--)
            {
                waitTimeBetweenDeccelerationSteps[j] = waitTimeBetweenAccelerationSteps[i];
                j++;
            }

            ConvertArrayFromSecondsToMilliseconds(waitTimeBetweenAccelerationSteps);
            ConvertArrayFromSecondsToMilliseconds(waitTimeBetweenMaxSpeedSteps);
            ConvertArrayFromSecondsToMilliseconds(waitTimeBetweenDeccelerationSteps);

            List<double> waitTimeBetweenSteps = new List<double>(); //new double[waitTimeBetweenAccelerationSteps.Length + waitTimeBetweenMaxSpeedSteps.Length + waitTimeBetweenDeccelerationSteps.Length];

            waitTimeBetweenSteps = waitTimeBetweenAccelerationSteps.ToList<double>()
                .Concat<double>(waitTimeBetweenMaxSpeedSteps.ToList<double>())
                .Concat<double>(waitTimeBetweenDeccelerationSteps.ToList<double>())
                .ToList();
            //waitTimeBetweenAccelerationSteps.CopyTo(waitTimeBetweenSteps, 0);
            //waitTimeBetweenMaxSpeedSteps.CopyTo(waitTimeBetweenSteps, waitTimeBetweenAccelerationSteps.Length);
            //waitTimeBetweenDeccelerationSteps.CopyTo(waitTimeBetweenSteps, waitTimeBetweenAccelerationSteps.Length + waitTimeBetweenMaxSpeedSteps.Length);

            return waitTimeBetweenSteps.ToArray();
        }

        private void ConvertArrayFromSecondsToMilliseconds(double[] arrayToConvert)
        {
            for (int i = 0; i < arrayToConvert.Length; i++)
            {
                arrayToConvert[i] = arrayToConvert[i] * 1000.0;
            }
        }

        private static double[] GetAccelerationTimeArray(int numberOfStepsForAcceleration, double acceleration)
        {
            double[] tempAccelerationArray = new double[numberOfStepsForAcceleration];
            for (int i = 0; i < numberOfStepsForAcceleration; i++)
            {
                tempAccelerationArray[i] = Math.Sqrt(2.0 * i / acceleration);
            }

            return tempAccelerationArray;
        }

        private static void NOP(double durationMilliseconds)
        {
            //Thread.Sleep(durationMilliseconds);
            var sw = Stopwatch.StartNew();
            
            while (sw.Elapsed.Milliseconds < durationMilliseconds)
            {

            }
        }
    }
}

