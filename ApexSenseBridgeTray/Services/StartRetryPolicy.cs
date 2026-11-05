using System;
using System.Collections.Generic;
using System.Linq;

namespace ApexSenseBridgeTray.Services
{
    internal sealed class StartRetryPolicy
    {
        internal const int IdentityCheckFailedExitCode = 3;
        internal const int IdenticalFailureLimit = 2;
        internal static readonly TimeSpan RetryDelay = TimeSpan.FromSeconds(10);
        internal static readonly TimeSpan IdentityRetryDelay = TimeSpan.FromSeconds(2);

        private sealed class FailureHistory
        {
            public int ExitCode;
            public string Reason;
            public int Repeats;
            public bool QuickRetryUsed;
            public bool Paused;
        }

        private readonly Dictionary<uint, FailureHistory> failures = new Dictionary<uint, FailureHistory>();
        private string controller;

        public bool HasPausedProcess
        {
            get { return failures.Values.Any(failure => failure.Paused); }
        }

        public TimeSpan? RecordFailure(uint processId, int exitCode, string reason)
        {
            FailureHistory failure;
            if (!failures.TryGetValue(processId, out failure))
            {
                failure = new FailureHistory();
                failures[processId] = failure;
            }
            bool identical = failure.Repeats > 0 && failure.ExitCode == exitCode &&
                             string.Equals(failure.Reason, reason, StringComparison.Ordinal);
            failure.Repeats = identical ? failure.Repeats + 1 : 1;
            failure.ExitCode = exitCode;
            failure.Reason = reason;
            if (exitCode == IdentityCheckFailedExitCode)
            {
                failure.Paused = false;
                if (failure.QuickRetryUsed) return RetryDelay;
                failure.QuickRetryUsed = true;
                return IdentityRetryDelay;
            }
            failure.Paused = failure.Repeats >= IdenticalFailureLimit;
            return failure.Paused ? (TimeSpan?)null : RetryDelay;
        }

        public bool IsPaused(uint processId)
        {
            FailureHistory failure;
            return failures.TryGetValue(processId, out failure) && failure.Paused;
        }

        public void Forget(uint processId)
        {
            failures.Remove(processId);
        }

        public void Reset()
        {
            failures.Clear();
        }

        public bool HasEntries
        {
            get { return failures.Count > 0; }
        }

        public void Retain(ICollection<uint> runningProcessIds)
        {
            var stopped = failures.Keys.Where(processId => !runningProcessIds.Contains(processId)).ToList();
            foreach (var processId in stopped) failures.Remove(processId);
        }

        public bool ObserveController(string status, bool sessionInProgress)
        {
            if (status == "unavailable")
            {
                if (!sessionInProgress && controller != null) controller = status;
                return false;
            }
            if (status != "apex4" && status != "apex5" && status != "apex6" && status != "beitong" && status != "disconnected")
                return false;
            var previous = controller;
            controller = status;
            return previous != null && previous != status;
        }
    }
}
