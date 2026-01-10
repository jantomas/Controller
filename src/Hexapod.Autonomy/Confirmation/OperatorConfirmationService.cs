using System.Collections.Concurrent;
using Hexapod.Autonomy.Decision;
using Hexapod.Core.Configuration;
using Hexapod.Core.Enums;
using Hexapod.Core.Events;
using Hexapod.Core.Models;
using Hexapod.Core.Services;
using Microsoft.Extensions.Logging;
using Microsoft.Extensions.Options;

using DecisionModel = Hexapod.Core.Models.Decision;

namespace Hexapod.Autonomy.Confirmation;

/// <summary>
/// Interface for the operator confirmation service.
/// </summary>
public interface IOperatorConfirmationService
{
    /// <summary>
    /// Requests operator confirmation for a decision.
    /// </summary>
    Task<ConfirmationResult> RequestConfirmationAsync(
        DecisionModel decision, 
        CancellationToken cancellationToken = default);

    /// <summary>
    /// Submits an operator response to a pending confirmation.
    /// </summary>
    Task<bool> SubmitResponseAsync(
        string decisionId, 
        string selectedOptionId, 
        string? notes = null,
        CancellationToken cancellationToken = default);

    /// <summary>
    /// Gets pending confirmations.
    /// </summary>
    IReadOnlyList<DecisionModel> GetPendingConfirmations();

    /// <summary>
    /// Cancels a pending confirmation.
    /// </summary>
    Task CancelConfirmationAsync(string decisionId, CancellationToken cancellationToken = default);
}

/// <summary>
/// Result of a confirmation request.
/// </summary>
public record ConfirmationResult
{
    public required string DecisionId { get; init; }
    public required ConfirmationStatus Status { get; init; }
    public string? SelectedOptionId { get; init; }
    public string? OperatorNotes { get; init; }
    public required DateTimeOffset Timestamp { get; init; }
    public TimeSpan? ResponseTime { get; init; }
}

/// <summary>
/// Status of a confirmation request.
/// </summary>
public enum ConfirmationStatus
{
    Pending,
    Confirmed,
    Rejected,
    TimedOut,
    Cancelled
}

/// <summary>
/// Implementation of the operator confirmation service.
/// Handles semi-autonomous operation with operator oversight.
/// </summary>
public sealed class OperatorConfirmationService : IOperatorConfirmationService
{
    private readonly IEventBus _eventBus;
    private readonly ILogger<OperatorConfirmationService> _logger;
    private readonly AutonomyConfiguration _config;
    private readonly ConcurrentDictionary<string, PendingConfirmation> _pendingConfirmations = new();

    public OperatorConfirmationService(
        IEventBus eventBus,
        IOptions<HexapodConfiguration> config,
        ILogger<OperatorConfirmationService> logger)
    {
        _eventBus = eventBus;
        _logger = logger;
        _config = config.Value.Autonomy;

        // Subscribe to operator responses via event bus
        _eventBus.Subscribe<OperatorResponseEvent>()
            .Subscribe(OnOperatorResponse);
    }

    public async Task<ConfirmationResult> RequestConfirmationAsync(
        DecisionModel decision, 
        CancellationToken cancellationToken = default)
    {
        var timeout = TimeSpan.FromSeconds(_config.ConfirmationTimeoutSeconds);
        var deadline = DateTimeOffset.UtcNow.Add(timeout);
        
        var pending = new PendingConfirmation
        {
            Decision = decision,
            RequestedAt = DateTimeOffset.UtcNow,
            Deadline = deadline,
            CompletionSource = new TaskCompletionSource<ConfirmationResult>()
        };

        _pendingConfirmations[decision.DecisionId] = pending;

        _logger.LogInformation(
            "Requesting operator confirmation for decision {Id}, timeout: {Timeout}s",
            decision.DecisionId, timeout.TotalSeconds);

        // Publish event for remote notification
        _eventBus.Publish(new ConfirmationRequiredEvent
        {
            EventId = Guid.NewGuid().ToString(),
            Timestamp = DateTimeOffset.UtcNow,
            Source = nameof(OperatorConfirmationService),
            Decision = decision,
            Timeout = timeout
        });

        // Start timeout task
        var timeoutTask = Task.Delay(timeout, cancellationToken)
            .ContinueWith(_ => HandleTimeout(decision.DecisionId), TaskContinuationOptions.OnlyOnRanToCompletion);

        try
        {
            // Wait for response or timeout
            using var cts = CancellationTokenSource.CreateLinkedTokenSource(cancellationToken);
            cts.CancelAfter(timeout);

            return await pending.CompletionSource.Task.WaitAsync(cts.Token);
        }
        catch (OperationCanceledException) when (!cancellationToken.IsCancellationRequested)
        {
            // Timeout occurred
            return await HandleTimeout(decision.DecisionId);
        }
        finally
        {
            _pendingConfirmations.TryRemove(decision.DecisionId, out _);
        }
    }

    public Task<bool> SubmitResponseAsync(
        string decisionId, 
        string selectedOptionId, 
        string? notes = null,
        CancellationToken cancellationToken = default)
    {
        if (!_pendingConfirmations.TryGetValue(decisionId, out var pending))
        {
            _logger.LogWarning("No pending confirmation found for decision {Id}", decisionId);
            return Task.FromResult(false);
        }

        var responseTime = DateTimeOffset.UtcNow - pending.RequestedAt;

        // Validate selected option
        var option = pending.Decision.Options.FirstOrDefault(o => o.OptionId == selectedOptionId);
        if (option == null)
        {
            _logger.LogWarning("Invalid option {Option} for decision {Id}", selectedOptionId, decisionId);
            return Task.FromResult(false);
        }

        var result = new ConfirmationResult
        {
            DecisionId = decisionId,
            Status = ConfirmationStatus.Confirmed,
            SelectedOptionId = selectedOptionId,
            OperatorNotes = notes,
            Timestamp = DateTimeOffset.UtcNow,
            ResponseTime = responseTime
        };

        pending.CompletionSource.TrySetResult(result);

        _logger.LogInformation(
            "Operator confirmed decision {Id} with option {Option} in {Time}s",
            decisionId, selectedOptionId, responseTime.TotalSeconds);

        return Task.FromResult(true);
    }

    public IReadOnlyList<DecisionModel> GetPendingConfirmations()
    {
        return _pendingConfirmations.Values
            .Where(p => p.Deadline > DateTimeOffset.UtcNow)
            .Select(p => p.Decision)
            .ToList()
            .AsReadOnly();
    }

    public Task CancelConfirmationAsync(string decisionId, CancellationToken cancellationToken = default)
    {
        if (_pendingConfirmations.TryRemove(decisionId, out var pending))
        {
            var result = new ConfirmationResult
            {
                DecisionId = decisionId,
                Status = ConfirmationStatus.Cancelled,
                Timestamp = DateTimeOffset.UtcNow
            };

            pending.CompletionSource.TrySetResult(result);
            _logger.LogInformation("Confirmation cancelled for decision {Id}", decisionId);
        }

        return Task.CompletedTask;
    }

    private void OnOperatorResponse(OperatorResponseEvent response)
    {
        _ = SubmitResponseAsync(response.DecisionId, response.SelectedOptionId, response.Notes);
    }

    private Task<ConfirmationResult> HandleTimeout(string decisionId)
    {
        if (!_pendingConfirmations.TryRemove(decisionId, out var pending))
        {
            return Task.FromResult(new ConfirmationResult
            {
                DecisionId = decisionId,
                Status = ConfirmationStatus.TimedOut,
                Timestamp = DateTimeOffset.UtcNow
            });
        }

        _logger.LogWarning("Confirmation timeout for decision {Id}", decisionId);

        // Determine timeout action based on configuration
        string? selectedOption = null;
        var status = ConfirmationStatus.TimedOut;

        if (_config.TimeoutAction == "SafeAction")
        {
            // Select the safest option (lowest risk)
            var safeOption = pending.Decision.Options
                .OrderBy(o => o.Risk)
                .ThenByDescending(o => o.Confidence)
                .FirstOrDefault();

            if (safeOption != null)
            {
                selectedOption = safeOption.OptionId;
                _logger.LogInformation(
                    "Auto-selecting safe option {Option} for timed out decision {Id}",
                    selectedOption, decisionId);
            }
        }

        var result = new ConfirmationResult
        {
            DecisionId = decisionId,
            Status = status,
            SelectedOptionId = selectedOption,
            Timestamp = DateTimeOffset.UtcNow,
            ResponseTime = pending.Deadline - pending.RequestedAt
        };

        pending.CompletionSource.TrySetResult(result);
        return Task.FromResult(result);
    }

    private class PendingConfirmation
    {
        public required DecisionModel Decision { get; init; }
        public required DateTimeOffset RequestedAt { get; init; }
        public required DateTimeOffset Deadline { get; init; }
        public required TaskCompletionSource<ConfirmationResult> CompletionSource { get; init; }
    }
}

/// <summary>
/// Autonomy manager that coordinates decision-making, confirmation, and execution.
/// </summary>
public sealed class AutonomyManager
{
    private readonly IDecisionEngine _decisionEngine;
    private readonly IOperatorConfirmationService _confirmationService;
    private readonly IEventBus _eventBus;
    private readonly ILogger<AutonomyManager> _logger;
    private readonly AutonomyConfiguration _config;

    public AutonomyManager(
        IDecisionEngine decisionEngine,
        IOperatorConfirmationService confirmationService,
        IEventBus eventBus,
        IOptions<HexapodConfiguration> config,
        ILogger<AutonomyManager> logger)
    {
        _decisionEngine = decisionEngine;
        _confirmationService = confirmationService;
        _eventBus = eventBus;
        _logger = logger;
        _config = config.Value.Autonomy;
    }

    /// <summary>
    /// Processes a situation and determines the appropriate action.
    /// </summary>
    public async Task<ActionResult> ProcessSituationAsync(
        DecisionContext context,
        CancellationToken cancellationToken = default)
    {
        // Generate decision
        var decision = await _decisionEngine.EvaluateAsync(context, cancellationToken);

        // Check if confirmation is needed
        if (_decisionEngine.RequiresConfirmation(decision, context.OperationMode))
        {
            _logger.LogInformation("Decision {Id} requires operator confirmation", decision.DecisionId);
            
            var confirmationResult = await _confirmationService.RequestConfirmationAsync(
                decision, cancellationToken);

            return ProcessConfirmationResult(decision, confirmationResult);
        }

        // Auto-select best option
        var bestOption = SelectBestOption(decision);
        
        return new ActionResult
        {
            DecisionId = decision.DecisionId,
            SelectedOption = bestOption,
            Status = ActionStatus.Approved,
            Source = ActionSource.Autonomous,
            Timestamp = DateTimeOffset.UtcNow
        };
    }

    private ActionResult ProcessConfirmationResult(DecisionModel decision, ConfirmationResult confirmation)
    {
        var selectedOption = confirmation.SelectedOptionId != null
            ? decision.Options.FirstOrDefault(o => o.OptionId == confirmation.SelectedOptionId)
            : null;

        var status = confirmation.Status switch
        {
            ConfirmationStatus.Confirmed => ActionStatus.Approved,
            ConfirmationStatus.Rejected => ActionStatus.Rejected,
            ConfirmationStatus.TimedOut => selectedOption != null ? ActionStatus.Approved : ActionStatus.Deferred,
            ConfirmationStatus.Cancelled => ActionStatus.Cancelled,
            _ => ActionStatus.Deferred
        };

        if (status == ActionStatus.Deferred)
        {
            selectedOption = SelectSafeOption(decision);
        }

        return new ActionResult
        {
            DecisionId = decision.DecisionId,
            SelectedOption = selectedOption,
            Status = status,
            Source = confirmation.Status == ConfirmationStatus.Confirmed 
                ? ActionSource.Operator 
                : ActionSource.Autonomous,
            OperatorNotes = confirmation.OperatorNotes,
            Timestamp = DateTimeOffset.UtcNow
        };
    }

    private DecisionOption? SelectBestOption(DecisionModel decision)
    {
        // Select option with best confidence/risk ratio
        return decision.Options
            .OrderByDescending(o => o.Confidence * (1.0 - (int)o.Risk / 4.0))
            .FirstOrDefault();
    }

    private DecisionOption? SelectSafeOption(DecisionModel decision)
    {
        // Select safest option
        return decision.Options
            .OrderBy(o => o.Risk)
            .ThenByDescending(o => o.Confidence)
            .FirstOrDefault();
    }
}

/// <summary>
/// Result of processing a situation.
/// </summary>
public record ActionResult
{
    public required string DecisionId { get; init; }
    public DecisionOption? SelectedOption { get; init; }
    public required ActionStatus Status { get; init; }
    public required ActionSource Source { get; init; }
    public string? OperatorNotes { get; init; }
    public required DateTimeOffset Timestamp { get; init; }
}

/// <summary>
/// Status of a processed action.
/// </summary>
public enum ActionStatus
{
    Approved,
    Rejected,
    Deferred,
    Cancelled
}

/// <summary>
/// Source of the action decision.
/// </summary>
public enum ActionSource
{
    Autonomous,
    Operator
}
