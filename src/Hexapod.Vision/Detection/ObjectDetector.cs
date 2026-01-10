using Hexapod.Core.Configuration;
using Hexapod.Core.Enums;
using Hexapod.Core.Models;
using Hexapod.Vision.Abstractions;
using Microsoft.Extensions.Logging;
using Microsoft.Extensions.Options;

namespace Hexapod.Vision.Detection;

/// <summary>
/// YOLOv8 object detector using Hailo-8L accelerator.
/// Optimized for real-time obstacle and object detection.
/// </summary>
public sealed class YoloObjectDetector : IObjectDetector
{
    private readonly IInferenceEngine _inferenceEngine;
    private readonly ILogger<YoloObjectDetector> _logger;
    private readonly ObjectDetectionModelConfig _config;
    private readonly string[] _classNames;

    // COCO dataset class names (80 classes)
    private static readonly string[] CocoClassNames = {
        "person", "bicycle", "car", "motorcycle", "airplane", "bus", "train", "truck",
        "boat", "traffic light", "fire hydrant", "stop sign", "parking meter", "bench",
        "bird", "cat", "dog", "horse", "sheep", "cow", "elephant", "bear", "zebra",
        "giraffe", "backpack", "umbrella", "handbag", "tie", "suitcase", "frisbee",
        "skis", "snowboard", "sports ball", "kite", "baseball bat", "baseball glove",
        "skateboard", "surfboard", "tennis racket", "bottle", "wine glass", "cup",
        "fork", "knife", "spoon", "bowl", "banana", "apple", "sandwich", "orange",
        "broccoli", "carrot", "hot dog", "pizza", "donut", "cake", "chair", "couch",
        "potted plant", "bed", "dining table", "toilet", "tv", "laptop", "mouse",
        "remote", "keyboard", "cell phone", "microwave", "oven", "toaster", "sink",
        "refrigerator", "book", "clock", "vase", "scissors", "teddy bear", "hair drier",
        "toothbrush"
    };

    // Classes considered as obstacles
    private static readonly HashSet<string> ObstacleClasses = new(StringComparer.OrdinalIgnoreCase)
    {
        "person", "bicycle", "car", "motorcycle", "bus", "train", "truck",
        "dog", "cat", "horse", "cow", "sheep", "elephant", "bear",
        "chair", "couch", "potted plant", "bed", "dining table",
        "rock", "tree", "fence", "wall"
    };

    // Classes considered as hazards
    private static readonly HashSet<string> HazardClasses = new(StringComparer.OrdinalIgnoreCase)
    {
        "fire hydrant", "person", "car", "motorcycle", "truck",
        "dog", "cat"
    };

    public YoloObjectDetector(
        IInferenceEngine inferenceEngine,
        IOptions<HexapodConfiguration> config,
        ILogger<YoloObjectDetector> logger)
    {
        _inferenceEngine = inferenceEngine;
        _logger = logger;
        _config = config.Value.Vision.Models.ObjectDetection;
        _classNames = CocoClassNames;
    }

    public async Task<IReadOnlyList<DetectedObject>> DetectAsync(
        CameraFrame frame, 
        CancellationToken cancellationToken = default)
    {
        if (!_inferenceEngine.IsInitialized)
        {
            _logger.LogWarning("Inference engine not initialized");
            return Array.Empty<DetectedObject>();
        }

        try
        {
            // Preprocess image for YOLO input
            var inputData = PreprocessImage(frame);

            // Run inference
            var result = await _inferenceEngine.InferAsync(inputData, cancellationToken);

            // Post-process YOLO output
            var detections = PostProcessYoloOutput(result, frame.Width, frame.Height);

            _logger.LogDebug("Detected {Count} objects in {Time}ms", 
                detections.Count, result.InferenceTime.TotalMilliseconds);

            return detections;
        }
        catch (Exception ex)
        {
            _logger.LogError(ex, "Object detection failed");
            return Array.Empty<DetectedObject>();
        }
    }

    private byte[] PreprocessImage(CameraFrame frame)
    {
        var (targetWidth, targetHeight, _) = _inferenceEngine.InputDimensions;
        
        // Resize and normalize image for YOLO input
        // Using letterbox resizing to maintain aspect ratio
        var scale = Math.Min(
            (double)targetWidth / frame.Width,
            (double)targetHeight / frame.Height);
        
        var newWidth = (int)(frame.Width * scale);
        var newHeight = (int)(frame.Height * scale);
        var padX = (targetWidth - newWidth) / 2;
        var padY = (targetHeight - newHeight) / 2;

        // Allocate output buffer (RGB, float32)
        var outputSize = targetWidth * targetHeight * 3;
        var output = new byte[outputSize * sizeof(float)];
        
        // Fill with padding color (gray = 114)
        var floatArray = new float[outputSize];
        Array.Fill(floatArray, 114f / 255f);

        // Resize and copy image data
        // In production, use SixLabors.ImageSharp for proper image processing
        ResizeAndCopy(frame.Data, frame.Width, frame.Height, frame.Channels,
            floatArray, targetWidth, targetHeight, newWidth, newHeight, padX, padY);

        Buffer.BlockCopy(floatArray, 0, output, 0, output.Length);
        return output;
    }

    private void ResizeAndCopy(
        byte[] source, int srcWidth, int srcHeight, int srcChannels,
        float[] dest, int dstWidth, int dstHeight,
        int newWidth, int newHeight, int padX, int padY)
    {
        // Simple nearest-neighbor resize for demonstration
        // Production code should use proper bilinear/bicubic interpolation
        for (int y = 0; y < newHeight; y++)
        {
            for (int x = 0; x < newWidth; x++)
            {
                var srcX = (int)(x * srcWidth / (double)newWidth);
                var srcY = (int)(y * srcHeight / (double)newHeight);
                
                var srcIdx = (srcY * srcWidth + srcX) * srcChannels;
                var dstIdx = ((y + padY) * dstWidth + (x + padX)) * 3;

                if (srcIdx + 2 < source.Length && dstIdx + 2 < dest.Length)
                {
                    // Normalize to 0-1 range
                    dest[dstIdx] = source[srcIdx] / 255f;     // R
                    dest[dstIdx + 1] = source[srcIdx + 1] / 255f; // G
                    dest[dstIdx + 2] = source[srcIdx + 2] / 255f; // B
                }
            }
        }
    }

    private List<DetectedObject> PostProcessYoloOutput(
        InferenceResult result, 
        int imageWidth, 
        int imageHeight)
    {
        var detections = new List<DetectedObject>();
        var scores = result.Scores;

        // YOLOv8 output format: (batch, 84, 8400)
        // 84 = 4 (bbox) + 80 (class scores)
        // 8400 = number of predictions (20x20 + 40x40 + 80x80) for 640x640 input

        const int numClasses = 80;
        const int numPredictions = 8400;
        const int predictionSize = 84;

        for (int i = 0; i < numPredictions && i * predictionSize + predictionSize <= scores.Length; i++)
        {
            var baseIdx = i * predictionSize;
            
            // Get class with highest score
            var maxScore = 0f;
            var maxClassId = -1;
            
            for (int c = 0; c < numClasses; c++)
            {
                var score = scores[baseIdx + 4 + c];
                if (score > maxScore)
                {
                    maxScore = score;
                    maxClassId = c;
                }
            }

            if (maxScore < _config.ConfidenceThreshold || maxClassId < 0)
                continue;

            // Extract bounding box (center_x, center_y, width, height)
            var cx = scores[baseIdx];
            var cy = scores[baseIdx + 1];
            var w = scores[baseIdx + 2];
            var h = scores[baseIdx + 3];

            // Convert to top-left corner format
            var x = (cx - w / 2) * imageWidth / _config.InputWidth;
            var y = (cy - h / 2) * imageHeight / _config.InputHeight;
            var width = w * imageWidth / _config.InputWidth;
            var height = h * imageHeight / _config.InputHeight;

            var className = maxClassId < _classNames.Length 
                ? _classNames[maxClassId] 
                : $"class_{maxClassId}";

            detections.Add(new DetectedObject
            {
                Label = className,
                Confidence = maxScore,
                BoundingBox = new BoundingBox
                {
                    X = Math.Max(0, x),
                    Y = Math.Max(0, y),
                    Width = Math.Min(width, imageWidth - x),
                    Height = Math.Min(height, imageHeight - y)
                },
                EstimatedDistance = EstimateDistance(height, className),
                Category = GetCategory(className),
                IsObstacle = ObstacleClasses.Contains(className),
                IsHazard = HazardClasses.Contains(className),
                Timestamp = result.Timestamp
            });
        }

        // Apply Non-Maximum Suppression
        return ApplyNms(detections, _config.NmsThreshold);
    }

    private List<DetectedObject> ApplyNms(List<DetectedObject> detections, double nmsThreshold)
    {
        if (detections.Count <= 1)
            return detections;

        // Sort by confidence
        var sorted = detections.OrderByDescending(d => d.Confidence).ToList();
        var result = new List<DetectedObject>();

        while (sorted.Count > 0)
        {
            var best = sorted[0];
            result.Add(best);
            sorted.RemoveAt(0);

            // Remove overlapping detections
            sorted.RemoveAll(d => 
                d.Label == best.Label && 
                CalculateIoU(best.BoundingBox, d.BoundingBox) > nmsThreshold);
        }

        return result;
    }

    private static double CalculateIoU(BoundingBox a, BoundingBox b)
    {
        var x1 = Math.Max(a.X, b.X);
        var y1 = Math.Max(a.Y, b.Y);
        var x2 = Math.Min(a.X + a.Width, b.X + b.Width);
        var y2 = Math.Min(a.Y + a.Height, b.Y + b.Height);

        var intersectionArea = Math.Max(0, x2 - x1) * Math.Max(0, y2 - y1);
        var unionArea = a.Width * a.Height + b.Width * b.Height - intersectionArea;

        return unionArea > 0 ? intersectionArea / unionArea : 0;
    }

    private static double? EstimateDistance(double boundingBoxHeight, string className)
    {
        // Rough distance estimation based on known object sizes and bounding box height
        // This is a simplified model - real implementation would use camera calibration
        
        var knownHeights = new Dictionary<string, double>(StringComparer.OrdinalIgnoreCase)
        {
            ["person"] = 1.7,
            ["car"] = 1.5,
            ["dog"] = 0.5,
            ["cat"] = 0.25,
            ["chair"] = 0.9,
            ["bicycle"] = 1.0
        };

        if (!knownHeights.TryGetValue(className, out var realHeight))
            return null;

        // Using pinhole camera model: distance = (realHeight * focalLength) / pixelHeight
        const double focalLength = 500; // Approximate focal length in pixels
        
        return boundingBoxHeight > 0 ? (realHeight * focalLength) / boundingBoxHeight : null;
    }

    private static string GetCategory(string className)
    {
        return className.ToLowerInvariant() switch
        {
            "person" => "Human",
            "bicycle" or "car" or "motorcycle" or "bus" or "train" or "truck" => "Vehicle",
            "dog" or "cat" or "horse" or "cow" or "sheep" or "bird" => "Animal",
            "chair" or "couch" or "bed" or "dining table" => "Furniture",
            _ => "Object"
        };
    }
}

/// <summary>
/// Terrain classifier using MobileNetV3 on Hailo-8L.
/// </summary>
public sealed class TerrainClassifier : ITerrainClassifier
{
    private readonly IInferenceEngine _inferenceEngine;
    private readonly ILogger<TerrainClassifier> _logger;
    private readonly ClassificationModelConfig _config;

    // Custom terrain classes (would be trained on terrain dataset)
    private static readonly string[] TerrainClassNames = {
        "flat_concrete", "flat_asphalt", "flat_grass",
        "rough_gravel", "rough_dirt", "rough_rocks",
        "slope_up", "slope_down",
        "stairs", "water", "sand", "vegetation"
    };

    public TerrainClassifier(
        IInferenceEngine inferenceEngine,
        IOptions<HexapodConfiguration> config,
        ILogger<TerrainClassifier> logger)
    {
        _inferenceEngine = inferenceEngine;
        _logger = logger;
        _config = config.Value.Vision.Models.TerrainClassification;
    }

    public async Task<TerrainClassification> ClassifyAsync(
        CameraFrame frame, 
        CancellationToken cancellationToken = default)
    {
        if (!_inferenceEngine.IsInitialized)
        {
            return CreateDefaultClassification();
        }

        try
        {
            // Preprocess image for MobileNet input
            var inputData = PreprocessImage(frame);

            // Run inference
            var result = await _inferenceEngine.InferAsync(inputData, cancellationToken);

            // Post-process classification output
            return PostProcessClassification(result);
        }
        catch (Exception ex)
        {
            _logger.LogError(ex, "Terrain classification failed");
            return CreateDefaultClassification();
        }
    }

    private byte[] PreprocessImage(CameraFrame frame)
    {
        var (targetWidth, targetHeight, _) = _inferenceEngine.InputDimensions;
        
        // Standard ImageNet preprocessing for MobileNet
        // Resize to 224x224 and normalize with ImageNet mean/std
        var outputSize = targetWidth * targetHeight * 3;
        var floatArray = new float[outputSize];

        // Simplified preprocessing
        for (int i = 0; i < Math.Min(outputSize, frame.Data.Length / frame.Channels * 3); i++)
        {
            var srcIdx = i / 3 * frame.Channels + i % 3;
            if (srcIdx < frame.Data.Length)
            {
                // Normalize with ImageNet statistics
                float[] mean = { 0.485f, 0.456f, 0.406f };
                float[] std = { 0.229f, 0.224f, 0.225f };
                var channel = i % 3;
                floatArray[i] = (frame.Data[srcIdx] / 255f - mean[channel]) / std[channel];
            }
        }

        var output = new byte[outputSize * sizeof(float)];
        Buffer.BlockCopy(floatArray, 0, output, 0, output.Length);
        return output;
    }

    private TerrainClassification PostProcessClassification(InferenceResult result)
    {
        var scores = result.Scores;
        var allScores = new Dictionary<TerrainType, double>();
        
        // Map scores to terrain types
        var maxScore = 0.0;
        var maxType = TerrainType.Unknown;

        for (int i = 0; i < Math.Min(scores.Length, TerrainClassNames.Length); i++)
        {
            var terrainType = ClassNameToTerrainType(TerrainClassNames[i]);
            var score = scores[i];
            
            if (allScores.ContainsKey(terrainType))
            {
                allScores[terrainType] = Math.Max(allScores[terrainType], score);
            }
            else
            {
                allScores[terrainType] = score;
            }

            if (score > maxScore)
            {
                maxScore = score;
                maxType = terrainType;
            }
        }

        return new TerrainClassification
        {
            PrimaryType = maxType,
            Confidence = maxScore,
            AllScores = allScores,
            TraversabilityScore = CalculateTraversability(maxType, maxScore),
            Timestamp = result.Timestamp
        };
    }

    private static TerrainType ClassNameToTerrainType(string className)
    {
        return className.ToLowerInvariant() switch
        {
            var s when s.Contains("flat") => TerrainType.Flat,
            var s when s.Contains("rough") || s.Contains("gravel") || s.Contains("rocks") => TerrainType.Rough,
            var s when s.Contains("slope") => TerrainType.Slope,
            var s when s.Contains("stairs") => TerrainType.Stairs,
            var s when s.Contains("water") => TerrainType.Water,
            var s when s.Contains("sand") => TerrainType.Sand,
            var s when s.Contains("vegetation") || s.Contains("grass") => TerrainType.Vegetation,
            _ => TerrainType.Unknown
        };
    }

    private static double CalculateTraversability(TerrainType terrain, double confidence)
    {
        var baseTraversability = terrain switch
        {
            TerrainType.Flat => 1.0,
            TerrainType.Vegetation => 0.8,
            TerrainType.Rough => 0.7,
            TerrainType.Sand => 0.6,
            TerrainType.Slope => 0.5,
            TerrainType.Stairs => 0.4,
            TerrainType.Water => 0.1,
            TerrainType.Obstacle => 0.0,
            _ => 0.5
        };

        // Weight by confidence
        return baseTraversability * confidence + 0.5 * (1 - confidence);
    }

    private static TerrainClassification CreateDefaultClassification()
    {
        return new TerrainClassification
        {
            PrimaryType = TerrainType.Unknown,
            Confidence = 0.0,
            AllScores = new Dictionary<TerrainType, double>(),
            TraversabilityScore = 0.5,
            Timestamp = DateTimeOffset.UtcNow
        };
    }
}
