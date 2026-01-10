using Hexapod.Core.Models;

namespace Hexapod.Vision.Abstractions;

/// <summary>
/// Interface for camera capture.
/// </summary>
public interface ICameraCapture : IDisposable
{
    /// <summary>
    /// Gets the camera resolution width.
    /// </summary>
    int Width { get; }

    /// <summary>
    /// Gets the camera resolution height.
    /// </summary>
    int Height { get; }

    /// <summary>
    /// Gets whether the camera is currently capturing.
    /// </summary>
    bool IsCapturing { get; }

    /// <summary>
    /// Initializes the camera.
    /// </summary>
    Task InitializeAsync(CancellationToken cancellationToken = default);

    /// <summary>
    /// Starts capturing frames.
    /// </summary>
    Task StartCaptureAsync(CancellationToken cancellationToken = default);

    /// <summary>
    /// Stops capturing frames.
    /// </summary>
    Task StopCaptureAsync(CancellationToken cancellationToken = default);

    /// <summary>
    /// Captures a single frame.
    /// </summary>
    Task<CameraFrame> CaptureFrameAsync(CancellationToken cancellationToken = default);

    /// <summary>
    /// Gets a stream of frames.
    /// </summary>
    IAsyncEnumerable<CameraFrame> GetFrameStreamAsync(CancellationToken cancellationToken = default);
}

/// <summary>
/// Represents a captured camera frame.
/// </summary>
public record CameraFrame
{
    public required byte[] Data { get; init; }
    public required int Width { get; init; }
    public required int Height { get; init; }
    public required int Channels { get; init; }
    public required DateTimeOffset Timestamp { get; init; }
    public required long FrameNumber { get; init; }
}

/// <summary>
/// Interface for AI inference engine.
/// </summary>
public interface IInferenceEngine : IDisposable
{
    /// <summary>
    /// Gets the model name.
    /// </summary>
    string ModelName { get; }

    /// <summary>
    /// Gets whether the engine is initialized.
    /// </summary>
    bool IsInitialized { get; }

    /// <summary>
    /// Gets the expected input dimensions.
    /// </summary>
    (int Width, int Height, int Channels) InputDimensions { get; }

    /// <summary>
    /// Initializes the inference engine with the specified model.
    /// </summary>
    Task InitializeAsync(string modelPath, CancellationToken cancellationToken = default);

    /// <summary>
    /// Runs inference on the input data.
    /// </summary>
    Task<InferenceResult> InferAsync(byte[] inputData, CancellationToken cancellationToken = default);
}

/// <summary>
/// Represents the result of an inference operation.
/// </summary>
public record InferenceResult
{
    public required byte[] RawOutput { get; init; }
    public required float[] Scores { get; init; }
    public required TimeSpan InferenceTime { get; init; }
    public required DateTimeOffset Timestamp { get; init; }
}

/// <summary>
/// Interface for object detection.
/// </summary>
public interface IObjectDetector
{
    /// <summary>
    /// Detects objects in the given frame.
    /// </summary>
    Task<IReadOnlyList<DetectedObject>> DetectAsync(CameraFrame frame, CancellationToken cancellationToken = default);
}

/// <summary>
/// Interface for terrain classification.
/// </summary>
public interface ITerrainClassifier
{
    /// <summary>
    /// Classifies the terrain in the given frame.
    /// </summary>
    Task<TerrainClassification> ClassifyAsync(CameraFrame frame, CancellationToken cancellationToken = default);
}

/// <summary>
/// Represents terrain classification result.
/// </summary>
public record TerrainClassification
{
    public required Core.Enums.TerrainType PrimaryType { get; init; }
    public required double Confidence { get; init; }
    public required IReadOnlyDictionary<Core.Enums.TerrainType, double> AllScores { get; init; }
    public required double TraversabilityScore { get; init; }
    public required DateTimeOffset Timestamp { get; init; }
}

/// <summary>
/// Interface for scene analysis combining detection and classification.
/// </summary>
public interface ISceneAnalyzer
{
    /// <summary>
    /// Analyzes the scene from the given frame.
    /// </summary>
    Task<SceneAnalysis> AnalyzeAsync(CameraFrame frame, CancellationToken cancellationToken = default);
}
