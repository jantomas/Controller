using System.Runtime.InteropServices;
using Hexapod.Core.Configuration;
using Hexapod.Vision.Abstractions;
using Microsoft.Extensions.Logging;
using Microsoft.Extensions.Options;

namespace Hexapod.Vision.Hailo;

/// <summary>
/// Hailo Runtime API interop for Hailo-8L accelerator.
/// Provides P/Invoke bindings to the HailoRT library.
/// </summary>
internal static class HailoRT
{
    private const string LibraryName = "hailort";

    public enum hailo_status
    {
        HAILO_SUCCESS = 0,
        HAILO_UNINITIALIZED = 1,
        HAILO_INVALID_ARGUMENT = 2,
        HAILO_OUT_OF_HOST_MEMORY = 3,
        HAILO_TIMEOUT = 4,
        // ... additional status codes
    }

    [StructLayout(LayoutKind.Sequential)]
    public struct hailo_device_id_t
    {
        [MarshalAs(UnmanagedType.ByValTStr, SizeConst = 128)]
        public string id;
    }

    [StructLayout(LayoutKind.Sequential)]
    public struct hailo_stream_info_t
    {
        public int direction;
        public int format_type;
        public int hw_shape_width;
        public int hw_shape_height;
        public int hw_shape_features;
    }

    // Device management
    [DllImport(LibraryName, CallingConvention = CallingConvention.Cdecl)]
    public static extern hailo_status hailo_scan_devices(
        IntPtr params_ptr, 
        [Out] hailo_device_id_t[] device_ids, 
        ref int device_count);

    [DllImport(LibraryName, CallingConvention = CallingConvention.Cdecl)]
    public static extern hailo_status hailo_create_vdevice(
        IntPtr params_ptr, 
        out IntPtr vdevice);

    [DllImport(LibraryName, CallingConvention = CallingConvention.Cdecl)]
    public static extern hailo_status hailo_release_vdevice(IntPtr vdevice);

    // HEF (Hailo Executable Format) management
    [DllImport(LibraryName, CallingConvention = CallingConvention.Cdecl)]
    public static extern hailo_status hailo_create_hef_file(
        [MarshalAs(UnmanagedType.LPStr)] string hef_path, 
        out IntPtr hef);

    [DllImport(LibraryName, CallingConvention = CallingConvention.Cdecl)]
    public static extern hailo_status hailo_release_hef(IntPtr hef);

    // Network configuration
    [DllImport(LibraryName, CallingConvention = CallingConvention.Cdecl)]
    public static extern hailo_status hailo_configure_vdevice(
        IntPtr vdevice, 
        IntPtr hef, 
        IntPtr params_ptr, 
        out IntPtr configured_network_group);

    // Inference
    [DllImport(LibraryName, CallingConvention = CallingConvention.Cdecl)]
    public static extern hailo_status hailo_infer(
        IntPtr network_group,
        IntPtr input_buffers,
        int input_count,
        IntPtr output_buffers,
        int output_count);
}

/// <summary>
/// Hailo-8L inference engine implementation.
/// Optimized for running YOLOv8 and MobileNet models on Hailo-8L (13 TOPS).
/// </summary>
public sealed class HailoInferenceEngine : IInferenceEngine
{
    private readonly ILogger<HailoInferenceEngine> _logger;
    private readonly HailoConfig _config;
    private IntPtr _vdevice = IntPtr.Zero;
    private IntPtr _hef = IntPtr.Zero;
    private IntPtr _networkGroup = IntPtr.Zero;
    private bool _isInitialized;
    private string _modelName = string.Empty;
    private (int Width, int Height, int Channels) _inputDimensions;

    public string ModelName => _modelName;
    public bool IsInitialized => _isInitialized;
    public (int Width, int Height, int Channels) InputDimensions => _inputDimensions;

    public HailoInferenceEngine(
        IOptions<HexapodConfiguration> config,
        ILogger<HailoInferenceEngine> logger)
    {
        _logger = logger;
        _config = config.Value.Vision.Hailo;
    }

    public async Task InitializeAsync(string modelPath, CancellationToken cancellationToken = default)
    {
        if (!_config.Enabled)
        {
            _logger.LogInformation("Hailo acceleration is disabled in configuration");
            return;
        }

        _logger.LogInformation("Initializing Hailo-8L inference engine with model: {ModelPath}", modelPath);

        try
        {
            // Check if running on Linux ARM64 (Raspberry Pi)
            if (!RuntimeInformation.IsOSPlatform(OSPlatform.Linux) || 
                RuntimeInformation.ProcessArchitecture != Architecture.Arm64)
            {
                _logger.LogWarning("Hailo-8L is only supported on Linux ARM64. Using simulation mode.");
                await InitializeSimulationModeAsync(modelPath, cancellationToken);
                return;
            }

            // Scan for Hailo devices
            var deviceIds = new HailoRT.hailo_device_id_t[4];
            var deviceCount = deviceIds.Length;
            
            var status = HailoRT.hailo_scan_devices(IntPtr.Zero, deviceIds, ref deviceCount);
            if (status != HailoRT.hailo_status.HAILO_SUCCESS || deviceCount == 0)
            {
                throw new InvalidOperationException($"No Hailo devices found. Status: {status}");
            }

            _logger.LogInformation("Found {Count} Hailo device(s): {Id}", 
                deviceCount, deviceIds[0].id);

            // Create virtual device
            status = HailoRT.hailo_create_vdevice(IntPtr.Zero, out _vdevice);
            if (status != HailoRT.hailo_status.HAILO_SUCCESS)
            {
                throw new InvalidOperationException($"Failed to create Hailo device. Status: {status}");
            }

            // Load HEF model file
            status = HailoRT.hailo_create_hef_file(modelPath, out _hef);
            if (status != HailoRT.hailo_status.HAILO_SUCCESS)
            {
                throw new InvalidOperationException($"Failed to load HEF file. Status: {status}");
            }

            // Configure network
            status = HailoRT.hailo_configure_vdevice(_vdevice, _hef, IntPtr.Zero, out _networkGroup);
            if (status != HailoRT.hailo_status.HAILO_SUCCESS)
            {
                throw new InvalidOperationException($"Failed to configure network. Status: {status}");
            }

            _modelName = Path.GetFileNameWithoutExtension(modelPath);
            _inputDimensions = DetermineInputDimensions(modelPath);
            _isInitialized = true;

            _logger.LogInformation("Hailo inference engine initialized successfully. " +
                "Input dimensions: {Width}x{Height}x{Channels}", 
                _inputDimensions.Width, _inputDimensions.Height, _inputDimensions.Channels);
        }
        catch (DllNotFoundException ex)
        {
            _logger.LogWarning(ex, "Hailo runtime library not found. Using simulation mode.");
            await InitializeSimulationModeAsync(modelPath, cancellationToken);
        }
        catch (Exception ex)
        {
            _logger.LogError(ex, "Failed to initialize Hailo inference engine");
            throw;
        }
    }

    public async Task<InferenceResult> InferAsync(byte[] inputData, CancellationToken cancellationToken = default)
    {
        if (!_isInitialized)
        {
            throw new InvalidOperationException("Inference engine not initialized");
        }

        var startTime = DateTime.UtcNow;

        // In simulation mode, return dummy results
        if (_vdevice == IntPtr.Zero)
        {
            await Task.Delay(10, cancellationToken); // Simulate inference time
            return CreateSimulatedResult(startTime);
        }

        try
        {
            // Prepare input buffer
            var inputHandle = GCHandle.Alloc(inputData, GCHandleType.Pinned);
            try
            {
                // Allocate output buffer
                var outputSize = CalculateOutputSize();
                var outputData = new byte[outputSize];
                var outputHandle = GCHandle.Alloc(outputData, GCHandleType.Pinned);

                try
                {
                    // Run inference
                    var status = HailoRT.hailo_infer(
                        _networkGroup,
                        inputHandle.AddrOfPinnedObject(),
                        1,
                        outputHandle.AddrOfPinnedObject(),
                        1);

                    if (status != HailoRT.hailo_status.HAILO_SUCCESS)
                    {
                        throw new InvalidOperationException($"Inference failed. Status: {status}");
                    }

                    var inferenceTime = DateTime.UtcNow - startTime;
                    var scores = ParseOutputScores(outputData);

                    return new InferenceResult
                    {
                        RawOutput = outputData,
                        Scores = scores,
                        InferenceTime = inferenceTime,
                        Timestamp = DateTimeOffset.UtcNow
                    };
                }
                finally
                {
                    outputHandle.Free();
                }
            }
            finally
            {
                inputHandle.Free();
            }
        }
        catch (Exception ex)
        {
            _logger.LogError(ex, "Inference failed");
            throw;
        }
    }

    private async Task InitializeSimulationModeAsync(string modelPath, CancellationToken cancellationToken)
    {
        await Task.Delay(100, cancellationToken); // Simulate initialization time
        
        _modelName = Path.GetFileNameWithoutExtension(modelPath);
        _inputDimensions = DetermineInputDimensions(modelPath);
        _isInitialized = true;
        
        _logger.LogInformation("Hailo inference engine initialized in SIMULATION mode");
    }

    private static (int Width, int Height, int Channels) DetermineInputDimensions(string modelPath)
    {
        var modelName = Path.GetFileNameWithoutExtension(modelPath).ToLowerInvariant();
        
        // Determine input dimensions based on model type
        if (modelName.Contains("yolo"))
        {
            return (640, 640, 3); // YOLOv8 default
        }
        else if (modelName.Contains("mobilenet") || modelName.Contains("efficientnet"))
        {
            return (224, 224, 3); // Classification models
        }
        else if (modelName.Contains("resnet"))
        {
            return (224, 224, 3); // ResNet default
        }
        
        return (640, 640, 3); // Default
    }

    private int CalculateOutputSize()
    {
        // Output size depends on model type
        if (_modelName.Contains("yolo", StringComparison.OrdinalIgnoreCase))
        {
            // YOLOv8n outputs: (1, 84, 8400) for 80 classes + 4 bbox coords
            return 84 * 8400 * sizeof(float);
        }
        
        // Classification models output class probabilities
        return 1000 * sizeof(float); // ImageNet classes
    }

    private InferenceResult CreateSimulatedResult(DateTime startTime)
    {
        var outputSize = CalculateOutputSize();
        var random = new Random();
        var scores = new float[_modelName.Contains("yolo") ? 80 : 10];
        
        // Generate random scores for simulation
        for (int i = 0; i < scores.Length; i++)
        {
            scores[i] = (float)random.NextDouble();
        }

        return new InferenceResult
        {
            RawOutput = new byte[outputSize],
            Scores = scores,
            InferenceTime = DateTime.UtcNow - startTime,
            Timestamp = DateTimeOffset.UtcNow
        };
    }

    private static float[] ParseOutputScores(byte[] outputData)
    {
        var floatCount = outputData.Length / sizeof(float);
        var scores = new float[floatCount];
        Buffer.BlockCopy(outputData, 0, scores, 0, outputData.Length);
        return scores;
    }

    public void Dispose()
    {
        if (_networkGroup != IntPtr.Zero)
        {
            // Release network group
            _networkGroup = IntPtr.Zero;
        }

        if (_hef != IntPtr.Zero)
        {
            HailoRT.hailo_release_hef(_hef);
            _hef = IntPtr.Zero;
        }

        if (_vdevice != IntPtr.Zero)
        {
            HailoRT.hailo_release_vdevice(_vdevice);
            _vdevice = IntPtr.Zero;
        }

        _isInitialized = false;
        _logger.LogInformation("Hailo inference engine disposed");
    }
}
