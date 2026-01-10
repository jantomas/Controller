using Hexapod.Core.Enums;
using Hexapod.Core.Events;
using Hexapod.Core.Models;
using Hexapod.Core.Services;
using Microsoft.Extensions.Logging;

namespace Hexapod.Core.StateMachine;

/// <summary>
/// Manages the operation mode state machine for the hexapod.
/// </summary>
public interface IOperationStateMachine
{
    /// <summary>
    /// Gets the current operation mode.
    /// </summary>
    OperationMode CurrentMode { get; }

    /// <summary>
    /// Attempts to transition to a new operation mode.
    /// </summary>
    Task<bool> TransitionToAsync(OperationMode targetMode, string reason, CancellationToken cancellationToken = default);

    /// <summary>
    /// Checks if a transition to the target mode is valid.
    /// </summary>
    bool CanTransitionTo(OperationMode targetMode);

    /// <summary>
    /// Forces a transition to safe mode (emergency use only).
    /// </summary>
    Task ForceSafeModeAsync(string reason, CancellationToken cancellationToken = default);

    /// <summary>
    /// Forces an emergency stop (highest priority).
    /// </summary>
    Task ForceEmergencyStopAsync(string reason, CancellationToken cancellationToken = default);
}

/// <summary>
/// Implementation of the operation state machine.
/// </summary>
public sealed class OperationStateMachine : IOperationStateMachine
{
    private readonly IEventBus _eventBus;
    private readonly ILogger<OperationStateMachine> _logger;
    private readonly SemaphoreSlim _transitionLock = new(1, 1);
    private OperationMode _currentMode = OperationMode.Initializing;

    // Valid state transitions
    private static readonly Dictionary<OperationMode, HashSet<OperationMode>> ValidTransitions = new()
    {
        [OperationMode.Initializing] = new HashSet<OperationMode>
        {
            OperationMode.SemiAutonomous,
            OperationMode.Standby,
            OperationMode.SafeMode
        },
        [OperationMode.Autonomous] = new HashSet<OperationMode>
        {
            OperationMode.SemiAutonomous,
            OperationMode.RemoteControl,
            OperationMode.SafeMode,
            OperationMode.Standby,
            OperationMode.EmergencyStop
        },
        [OperationMode.SemiAutonomous] = new HashSet<OperationMode>
        {
            OperationMode.Autonomous,
            OperationMode.RemoteControl,
            OperationMode.SafeMode,
            OperationMode.Standby,
            OperationMode.EmergencyStop
        },
        [OperationMode.RemoteControl] = new HashSet<OperationMode>
        {
            OperationMode.Autonomous,
            OperationMode.SemiAutonomous,
            OperationMode.SafeMode,
            OperationMode.Standby,
            OperationMode.EmergencyStop
        },
        [OperationMode.SafeMode] = new HashSet<OperationMode>
        {
            OperationMode.SemiAutonomous,
            OperationMode.RemoteControl,
            OperationMode.Standby,
            OperationMode.EmergencyStop
        },
        [OperationMode.Standby] = new HashSet<OperationMode>
        {
            OperationMode.SemiAutonomous,
            OperationMode.RemoteControl,
            OperationMode.SafeMode,
            OperationMode.EmergencyStop
        },
        [OperationMode.EmergencyStop] = new HashSet<OperationMode>
        {
            OperationMode.SafeMode,
            OperationMode.Standby
        }
    };

    public OperationStateMachine(IEventBus eventBus, ILogger<OperationStateMachine> logger)
    {
        _eventBus = eventBus;
        _logger = logger;
    }

    public OperationMode CurrentMode => _currentMode;

    public bool CanTransitionTo(OperationMode targetMode)
    {
        if (_currentMode == targetMode)
            return false;

        return ValidTransitions.TryGetValue(_currentMode, out var validTargets) 
               && validTargets.Contains(targetMode);
    }

    public async Task<bool> TransitionToAsync(
        OperationMode targetMode, 
        string reason, 
        CancellationToken cancellationToken = default)
    {
        await _transitionLock.WaitAsync(cancellationToken);
        try
        {
            if (!CanTransitionTo(targetMode))
            {
                _logger.LogWarning(
                    "Invalid state transition attempted: {Current} -> {Target}. Reason: {Reason}",
                    _currentMode, targetMode, reason);
                return false;
            }

            var previousMode = _currentMode;
            _currentMode = targetMode;

            _logger.LogInformation(
                "Operation mode changed: {Previous} -> {New}. Reason: {Reason}",
                previousMode, _currentMode, reason);

            var @event = new OperationModeChangedEvent
            {
                EventId = Guid.NewGuid().ToString(),
                Timestamp = DateTimeOffset.UtcNow,
                Source = nameof(OperationStateMachine),
                PreviousMode = previousMode,
                NewMode = _currentMode,
                Reason = reason
            };

            _eventBus.Publish(@event);
            return true;
        }
        finally
        {
            _transitionLock.Release();
        }
    }

    public async Task ForceSafeModeAsync(string reason, CancellationToken cancellationToken = default)
    {
        await _transitionLock.WaitAsync(cancellationToken);
        try
        {
            var previousMode = _currentMode;
            _currentMode = OperationMode.SafeMode;

            _logger.LogWarning(
                "Forced transition to SafeMode from {Previous}. Reason: {Reason}",
                previousMode, reason);

            var @event = new OperationModeChangedEvent
            {
                EventId = Guid.NewGuid().ToString(),
                Timestamp = DateTimeOffset.UtcNow,
                Source = nameof(OperationStateMachine),
                PreviousMode = previousMode,
                NewMode = OperationMode.SafeMode,
                Reason = $"FORCED: {reason}"
            };

            _eventBus.Publish(@event);
        }
        finally
        {
            _transitionLock.Release();
        }
    }

    public async Task ForceEmergencyStopAsync(string reason, CancellationToken cancellationToken = default)
    {
        await _transitionLock.WaitAsync(cancellationToken);
        try
        {
            var previousMode = _currentMode;
            _currentMode = OperationMode.EmergencyStop;

            _logger.LogCritical(
                "EMERGENCY STOP from {Previous}. Reason: {Reason}",
                previousMode, reason);

            var emergencyEvent = new EmergencyEvent
            {
                EventId = Guid.NewGuid().ToString(),
                Timestamp = DateTimeOffset.UtcNow,
                Source = nameof(OperationStateMachine),
                Type = EmergencyType.OperatorAlert,
                Description = reason,
                RequiresImmediateAction = true
            };

            _eventBus.Publish(emergencyEvent);

            var modeEvent = new OperationModeChangedEvent
            {
                EventId = Guid.NewGuid().ToString(),
                Timestamp = DateTimeOffset.UtcNow,
                Source = nameof(OperationStateMachine),
                PreviousMode = previousMode,
                NewMode = OperationMode.EmergencyStop,
                Reason = $"EMERGENCY: {reason}"
            };

            _eventBus.Publish(modeEvent);
        }
        finally
        {
            _transitionLock.Release();
        }
    }
}
