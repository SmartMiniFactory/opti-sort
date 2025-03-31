using System;
using System.Timers;

namespace OptiSort.Classes
{

    internal class Watchdog
    {
        private Timer _watchdogTimer;
        private DateTime _lastSignalTime;
        private readonly TimeSpan _timeout = TimeSpan.FromSeconds(5); // 5-second timeout

        // Define the event
        public event EventHandler Elapsed;

        public Watchdog(double checkFrequency)
        {
            _watchdogTimer = new Timer(checkFrequency); // Check frequency in milliseconds
            _watchdogTimer.Elapsed += CheckWatchdog;
            _watchdogTimer.AutoReset = true;
        }

        public void Start() => _watchdogTimer.Start();

        public void Stop() => _watchdogTimer.Stop();

        public void Reset() => _lastSignalTime = DateTime.Now; // Call this method to reset the watchdog

        private void CheckWatchdog(object sender, ElapsedEventArgs e)
        {
            if (DateTime.Now - _lastSignalTime > _timeout)
            {
                Elapsed?.Invoke(this, EventArgs.Empty);
            }
        }
    }

}
