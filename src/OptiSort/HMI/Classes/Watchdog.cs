using System;
using System.Timers;

namespace OptiSort.Classes
{

    internal class Watchdog
    {
        private Timer _watchdogTimer;
        private DateTime _lastSignalTime;
        private readonly TimeSpan _timeout;

        // Define the event
        public event EventHandler Elapsed;

        public Watchdog(int timeout_ms)
        {
            _timeout = TimeSpan.FromMilliseconds(timeout_ms); ; // Set the timeout from the parameter
            _watchdogTimer = new Timer(500); // Check frequency in milliseconds
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
                _lastSignalTime = DateTime.Now; // Reset the last signal time to avoid multiple events
            }
        }
    }

}
