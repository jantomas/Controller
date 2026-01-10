using Hexapod.Core.Configuration;
using Hexapod.Core.Enums;
using Hexapod.Core.Events;
using Hexapod.Core.Models;
using Hexapod.Core.Services;
using Microsoft.Extensions.Logging;
using Microsoft.Extensions.Options;

using DecisionModel = Hexapod.Core.Models.Decision;

namespace Hexapod.Autonomy.Decision;

/// <summary>
/// Interface for the decision-making engine.
/// </summary>
public interface IDecisionEngine
{
    /// <summary>
    /// Evaluates a situation and produces a decision.
    /// </summary>
    Task<DecisionModel> EvaluateAsync(DecisionContext context, CancellationToken cancellationToken = default);

    /// <summary>
    /// Assesses the risk level of a proposed action.
    /// </summary>
    RiskLevel AssessRisk(ProposedAction action, SensorState sensorState);

    /// <summary>
    /// Gets whether a decision requires operator confirmation.
    /// </summary>
    bool RequiresConfirmation(DecisionModel decision, OperationMode operationMode);

    /// <summary>
    /// Records a decision outcome for learning.
    /// </summary>
    Task RecordOutcomeAsync(string decisionId, DecisionOutcome outcome, CancellationToken cancellationToken = default);
}

/// <summary>
/// Context for making a decision.
/// </summary>
public record DecisionContext
{
    public required DecisionType Type { get; init; }
    public required SensorState SensorState { get; init; }
    public required SceneAnalysis? SceneAnalysis { get; init; }
    public required MissionStatus? CurrentMission { get; init; }
    public required OperationMode OperationMode { get; init; }
    public ProposedAction? ProposedAction { get; init; }
    public IReadOnlyList<DetectedObject>? DetectedObjects { get; init; }
}

/// <summary>
/// Represents a proposed action to evaluate.
/// </summary>
public record ProposedAction
{
    public required string ActionType { get; init; }
    public required string Description { get; init; }
    public GeoPosition? TargetPosition { get; init; }
    public double? Distance { get; init; }
    public TerrainType? TargetTerrain { get; init; }
    public bool CrossesBoundary { get; init; }
    public bool IsIrreversible { get; init; }
}

/// <summary>
/// Outcome of a decision for learning purposes.
/// </summary>
public record DecisionOutcome
{
    public required string DecisionId { get; init; }
    public required string SelectedOptionId { get; init; }
    public required bool WasSuccessful { get; init; }
    public required TimeSpan ExecutionTime { get; init; }
    public double? EnergyCost { get; init; }
    public string? Notes { get; init; }
    public required DateTimeOffset Timestamp { get; init; }
}

/// <summary>
/// Decision-making engine implementation.
/// </summary>
public sealed class DecisionEngine : IDecisionEngine
{
    private readonly IEventBus _eventBus;
    private readonly ILogger<DecisionEngine> _logger;
    private readonly AutonomyConfiguration _config;
    private readonly RiskLevel _confirmationThreshold;

    public DecisionEngine(
        IEventBus eventBus,
        IOptions<HexapodConfiguration> config,
        ILogger<DecisionEngine> logger)
    {
        _eventBus = eventBus;
        _logger = logger;
        _config = config.Value.Autonomy;
        _confirmationThreshold = Enum.Parse<RiskLevel>(_config.ConfirmationRiskThreshold);
    }

    public async Task<DecisionModel> EvaluateAsync(DecisionContext context, CancellationToken cancellationToken = default)
    {
        _logger.LogDebug("Evaluating decision for type: {Type}", context.Type);

        var options = await GenerateOptionsAsync(context, cancellationToken);
        var selectedRisk = options.Count > 0 ? options.Max(o => o.Risk) : RiskLevel.None;

        var decision = new DecisionModel
        {
            DecisionId = Guid.NewGuid().ToString(),
            Description = GenerateDescription(context),
            RiskLevel = selectedRisk,
            Type = context.Type,
            Options = options,
            CreatedAt = DateTimeOffset.UtcNow,
            ExpiresAt = DateTimeOffset.UtcNow.AddSeconds(_config.ConfirmationTimeoutSeconds),
            RequiresConfirmation = RequiresConfirmation(selectedRisk, context.OperationMode)
        };

        _logger.LogInformation("Decision generated: {Id}, Risk: {Risk}, RequiresConfirmation: {Confirm}",
            decision.DecisionId, decision.RiskLevel, decision.RequiresConfirmation);

        return decision;
    }

    public RiskLevel AssessRisk(ProposedAction action, SensorState sensorState)
    {
        var riskFactors = new List<(string Factor, RiskLevel Level)>();

        // Battery level risk
        if (sensorState.PowerStatus != null)
        {
            if (sensorState.PowerStatus.BatteryPercentage < 10)
                riskFactors.Add(("CriticalBattery", RiskLevel.Critical));
            else if (sensorState.PowerStatus.BatteryPercentage < 20)
                riskFactors.Add(("LowBattery", RiskLevel.High));
            else if (sensorState.PowerStatus.BatteryPercentage < 30)
                riskFactors.Add(("ModerateBattery", RiskLevel.Medium));
        }

        // Terrain risk
        if (action.TargetTerrain.HasValue)
        {
            var terrainRisk = action.TargetTerrain.Value switch
            {
                TerrainType.Water => RiskLevel.Critical,
                TerrainType.Stairs => RiskLevel.High,
                TerrainType.Slope => RiskLevel.Medium,
                TerrainType.Rough => RiskLevel.Low,
                _ => RiskLevel.None
            };
            if (terrainRisk > RiskLevel.None)
                riskFactors.Add(("Terrain", terrainRisk));
        }

        // Boundary crossing risk
        if (action.CrossesBoundary)
            riskFactors.Add(("BoundaryCrossing", RiskLevel.High));

        // Irreversibility risk
        if (action.IsIrreversible)
            riskFactors.Add(("IrreversibleAction", RiskLevel.High));

        // Orientation risk (tipping)
        if (sensorState.Orientation != null)
        {
            var maxTilt = Math.Max(
                Math.Abs(sensorState.Orientation.Roll),
                Math.Abs(sensorState.Orientation.Pitch));
            
            if (maxTilt > 30)
                riskFactors.Add(("HighTilt", RiskLevel.High));
            else if (maxTilt > 20)
                riskFactors.Add(("ModerateTilt", RiskLevel.Medium));
        }

        // Distance-based obstacle risk
        if (action.Distance.HasValue && action.Distance.Value < _config.MinObstacleClearance)
            riskFactors.Add(("ObstacleProximity", RiskLevel.High));

        // Return highest risk level
        return riskFactors.Count > 0 
            ? riskFactors.Max(f => f.Level) 
            : RiskLevel.None;
    }

    public bool RequiresConfirmation(DecisionModel decision, OperationMode operationMode)
    {
        return RequiresConfirmation(decision.RiskLevel, operationMode);
    }

    private bool RequiresConfirmation(RiskLevel riskLevel, OperationMode operationMode)
    {
        // Autonomous mode - no confirmation unless critical
        if (operationMode == OperationMode.Autonomous)
            return riskLevel >= RiskLevel.Critical;

        // Semi-autonomous mode - confirmation based on threshold
        if (operationMode == OperationMode.SemiAutonomous)
            return riskLevel >= _confirmationThreshold;

        // Remote control mode - operator is in control
        return false;
    }

    public Task RecordOutcomeAsync(string decisionId, DecisionOutcome outcome, CancellationToken cancellationToken = default)
    {
        _logger.LogInformation("Recording decision outcome: {Id}, Success: {Success}",
            decisionId, outcome.WasSuccessful);

        // In a full implementation, this would store to a learning database
        return Task.CompletedTask;
    }

    private async Task<IReadOnlyList<DecisionOption>> GenerateOptionsAsync(
        DecisionContext context, 
        CancellationToken cancellationToken)
    {
        var options = new List<DecisionOption>();

        switch (context.Type)
        {
            case DecisionType.PathSelection:
                options = GeneratePathOptions(context);
                break;

            case DecisionType.ObstacleAvoidance:
                options = GenerateObstacleAvoidanceOptions(context);
                break;

            case DecisionType.TerrainTraversal:
                options = GenerateTerrainOptions(context);
                break;

            case DecisionType.BoundaryViolation:
                options = GenerateBoundaryOptions(context);
                break;

            case DecisionType.EmergencyAction:
                options = GenerateEmergencyOptions(context);
                break;

            default:
                options.Add(CreateDefaultOption());
                break;
        }

        return options.AsReadOnly();
    }

    private List<DecisionOption> GeneratePathOptions(DecisionContext context)
    {
        var options = new List<DecisionOption>
        {
            new DecisionOption
            {
                OptionId = "continue",
                Description = "Continue on current path",
                Confidence = 0.8,
                Risk = RiskLevel.Low,
                TimeCost = TimeSpan.Zero
            },
            new DecisionOption
            {
                OptionId = "alternate_left",
                Description = "Take alternate path to the left",
                Confidence = 0.6,
                Risk = RiskLevel.Medium,
                TimeCost = TimeSpan.FromMinutes(2)
            },
            new DecisionOption
            {
                OptionId = "alternate_right",
                Description = "Take alternate path to the right",
                Confidence = 0.6,
                Risk = RiskLevel.Medium,
                TimeCost = TimeSpan.FromMinutes(2)
            },
            new DecisionOption
            {
                OptionId = "wait",
                Description = "Wait and reassess",
                Confidence = 0.9,
                Risk = RiskLevel.None,
                TimeCost = TimeSpan.FromMinutes(1)
            }
        };

        return options;
    }

    private List<DecisionOption> GenerateObstacleAvoidanceOptions(DecisionContext context)
    {
        var options = new List<DecisionOption>
        {
            new DecisionOption
            {
                OptionId = "avoid_left",
                Description = "Avoid obstacle by going left",
                Confidence = 0.7,
                Risk = RiskLevel.Low,
                EnergyCost = 5
            },
            new DecisionOption
            {
                OptionId = "avoid_right",
                Description = "Avoid obstacle by going right",
                Confidence = 0.7,
                Risk = RiskLevel.Low,
                EnergyCost = 5
            },
            new DecisionOption
            {
                OptionId = "stop",
                Description = "Stop and wait for obstacle to clear",
                Confidence = 0.95,
                Risk = RiskLevel.None,
                EnergyCost = 0
            },
            new DecisionOption
            {
                OptionId = "reverse",
                Description = "Reverse and find alternative route",
                Confidence = 0.6,
                Risk = RiskLevel.Medium,
                EnergyCost = 10
            }
        };

        return options;
    }

    private List<DecisionOption> GenerateTerrainOptions(DecisionContext context)
    {
        var terrain = context.SceneAnalysis?.PrimaryTerrain ?? TerrainType.Unknown;
        var options = new List<DecisionOption>();

        if (terrain == TerrainType.Water)
        {
            options.Add(new DecisionOption
            {
                OptionId = "avoid_water",
                Description = "Avoid water hazard - find alternate route",
                Confidence = 0.95,
                Risk = RiskLevel.Low
            });
        }
        else if (terrain == TerrainType.Stairs)
        {
            options.Add(new DecisionOption
            {
                OptionId = "climb_stairs",
                Description = "Attempt to climb stairs (high risk)",
                Confidence = 0.4,
                Risk = RiskLevel.High,
                EnergyCost = 50
            });
            options.Add(new DecisionOption
            {
                OptionId = "find_ramp",
                Description = "Find ramp or alternative route",
                Confidence = 0.7,
                Risk = RiskLevel.Medium,
                TimeCost = TimeSpan.FromMinutes(5)
            });
        }
        else
        {
            options.Add(new DecisionOption
            {
                OptionId = "proceed_cautiously",
                Description = "Proceed with appropriate gait",
                Confidence = 0.8,
                Risk = RiskLevel.Low
            });
        }

        return options;
    }

    private List<DecisionOption> GenerateBoundaryOptions(DecisionContext context)
    {
        return new List<DecisionOption>
        {
            new DecisionOption
            {
                OptionId = "stop_at_boundary",
                Description = "Stop at boundary and await instructions",
                Confidence = 0.95,
                Risk = RiskLevel.None
            },
            new DecisionOption
            {
                OptionId = "cross_boundary",
                Description = "Cross boundary (requires confirmation)",
                Confidence = 0.5,
                Risk = RiskLevel.High
            },
            new DecisionOption
            {
                OptionId = "return_to_zone",
                Description = "Return to safe zone",
                Confidence = 0.9,
                Risk = RiskLevel.Low
            }
        };
    }

    private List<DecisionOption> GenerateEmergencyOptions(DecisionContext context)
    {
        return new List<DecisionOption>
        {
            new DecisionOption
            {
                OptionId = "emergency_stop",
                Description = "Immediate emergency stop",
                Confidence = 1.0,
                Risk = RiskLevel.None
            },
            new DecisionOption
            {
                OptionId = "safe_position",
                Description = "Move to safe position then stop",
                Confidence = 0.8,
                Risk = RiskLevel.Medium
            },
            new DecisionOption
            {
                OptionId = "return_home",
                Description = "Return to home position",
                Confidence = 0.6,
                Risk = RiskLevel.High
            }
        };
    }

    private DecisionOption CreateDefaultOption()
    {
        return new DecisionOption
        {
            OptionId = "default",
            Description = "Take default safe action",
            Confidence = 0.8,
            Risk = RiskLevel.Low
        };
    }

    private string GenerateDescription(DecisionContext context)
    {
        return context.Type switch
        {
            DecisionType.PathSelection => "Select path for navigation",
            DecisionType.ObstacleAvoidance => "Obstacle detected - select avoidance strategy",
            DecisionType.TerrainTraversal => $"Terrain traversal: {context.SceneAnalysis?.PrimaryTerrain}",
            DecisionType.BoundaryViolation => "Mission boundary encountered",
            DecisionType.EmergencyAction => "Emergency situation detected",
            DecisionType.MissionModification => "Mission modification required",
            DecisionType.ResourceManagement => "Resource management decision needed",
            _ => "Decision required"
        };
    }
}
