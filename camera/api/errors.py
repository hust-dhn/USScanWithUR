from .InuStreamsPyth import InuError, EErrorCode
from enum import IntEnum


class ErrorCode(IntEnum):
    STATUS_OK = EErrorCode.OK
    GENERAL_EXCEPTION = EErrorCode.General
    SERIALIZATION_EXCEPTION = EErrorCode.Serialization
    EVENT_EXCEPTION = EErrorCode.Event
    SYNC_OBJECT_EXCEPTION = EErrorCode.SyncObject
    THREAD_EXCEPTION = EErrorCode.Thread
    SHARED_MEMORY_EXCEPTION = EErrorCode.SharedMemory
    BUFFER_ALLOCATION_EXCEPTION = EErrorCode.BufferAllocation
    SERVICE_NOT_FOUND = EErrorCode.ServiceNotFound
    IPC_EXCEPTION = EErrorCode.IPC
    FILE_OPEN_ERROR = EErrorCode.FileOpen
    SERVICE_FAILURE = EErrorCode.ServiceFailure
    SERVICE_NOT_INITALIAZED = EErrorCode.ServiceNotInitaliazed
    USB_DISCONNECT = EErrorCode.USBDisconnect
    USB_CONNECT = EErrorCode.USBConnect
    STREAM_DISCONNECTED = EErrorCode.StreamDisconnected
    CHANNEL_ID_INVALID = EErrorCode.ChannelIDInvalid
    NEGATIVE_DISPARITY_OFFSET = EErrorCode.NegativeDisparityOffset
    NO_MATCHING_FORMAT = EErrorCode.NoMatchingFormat
    UNEXPECTED_INIT_PARAMETER = EErrorCode.UnexpectedInitParameter
    WRONG_BOOT_TYPE = EErrorCode.WrongBootType
    MESSAGE_UNHANDLED = EErrorCode.MessageUnhandled
    MODULE_READY = EErrorCode.ModuleReady
    MODULE_NOT_READY = EErrorCode.ModuleNotReady
    MISMATCH_BUFFER_SIZE = EErrorCode.MismatchBufferSize
    FW_GENERAL_ERROR = EErrorCode.FWGeneralError
    PASSWORD_ERROR = EErrorCode.PasswordError
    MAP_NOT_EMPTY_ERROR = EErrorCode.MapNotEmptyError
    MODULE_NOT_STARTED = EErrorCode.ModuleNotStarted
    NOT_SUPPORTED_BY_CHIP_REVISION = EErrorCode.NotSupportedByChipRevision
    FAILED_TO_UNREGISTER = EErrorCode.FailedToUnregister
    FAILED_TO_CLOSE_STREAM = EErrorCode.FailedToCloseStream
    INVALID_INTERLEAVED_TYPE = EErrorCode.InvalidInterleavedType
    CROP_WIDTH_SHOULD_BE_16_MULTIPLIER = EErrorCode.CropWidthShouldBe16multiplier
    INVALID_FRAME_FROM_SERVICE = EErrorCode.InvalidFrameFromService
    INVALID_CHANNEL_FORMAT = EErrorCode.InvalidChannelFormat
    SET_CHANNEL_FORMAT_NOT_SUPPORTED = EErrorCode.SetChannelFormatNotSupported
    UNSUPPORTED_PLATFORM = EErrorCode.UnsupportedPlatform
    CONFIDENCE_UNSUPPORTED = EErrorCode.ConfidenceUnsupported
    MISSSING_STORAGE_EXCEPTION = EErrorCode.MisssingStorage
    MISSSING_TABLE_EXCEPTION = EErrorCode.MisssingTable
    MISSSING_KEY_EXCEPTION = EErrorCode.MisssingKey
    SAVE_EXCEPTION = EErrorCode.Save

    INIT_EXCEPTION = EErrorCode.Init
    FINALIZE_EXCEPTION = EErrorCode.Finalize
    START_EXCEPTION = EErrorCode.Start
    STOP_EXCEPTION = EErrorCode.Stop
    ACQUISITION_EXCEPTION = EErrorCode.Acquisition
    SERVICE_STATE_EXCEPTION = EErrorCode.ServiceState
    WRONG_WORK_MESSAGE_TYPE = EErrorCode.WrongWorkMessageType
    FAILED_TO_LOAD_REGISTER = EErrorCode.FailedToLoadRegister
    CANT_GET_INFO_FROM_BOTH_SENSORS = EErrorCode.CantGetInfoFromBothSensors
    FAILED_TO_WRITE_TO_SENSOR_UVC = EErrorCode.FailedToWriteToSensorUVC
    FAILED_TO_LOAD_LUT = EErrorCode.FailedToLoadLUT
    FAILED_TO_RETRIEVE_SENSOR_ID = EErrorCode.FaileToRetrieveSensorID
    CEVA_READ_ERROR = EErrorCode.CevaRead
    CEVA_LOAD_ERROR = EErrorCode.CevaLoad
    SET_FW_PARAMS = EErrorCode.SetFWParams
    GET_FW_PARAMS = EErrorCode.GetFWParams
    START_FW_ALGORITHM = EErrorCode.StartFWAlgorithm
    READ_REGISTER = EErrorCode.ReadRegister
    FPS_MISSMATCH_ERROR = EErrorCode.FPSMissMatchError
    FAILED_2_CONNECT_WITH_SENSOR = EErrorCode.Failed2ConnectWithSensor
    FAILED_BIND_NODS = EErrorCode.FailedBindNods

    CREATE_SERVICE_ERROR = EErrorCode.CreateService
    CREATE_REGONIZER_ERROR = EErrorCode.CreateRegonizer
    SETTING_REGONIZER_ERROR = EErrorCode.SettingRegonizer
    INIT_RECOGNIZER_ERROR = EErrorCode.InitRecognizer
    CREATE_SEARCH_ERROR = EErrorCode.CreateSearch
    SETTING_SEARCH_ERROR = EErrorCode.SettingSearch
    RECOGNITION_ERROR = EErrorCode.Recognition
    RECOGNITION_CLIPPED_ERROR = EErrorCode.RecognitionClipped
    WAVE_FILE_ERR = EErrorCode.WaveFile
    RECOG_SAMPLE_CONVERT_ERROR = EErrorCode.RecogSampleConvert
    RECOG_PIP_EERROR = EErrorCode.RecogPipe

    BAD_INJECTION_STREAM = EErrorCode.BadInjectionStream
    INJECTION_ERROR = EErrorCode.Injection
    FAILED_WRITE_INJECTION_ERROR = EErrorCode.FailedWriteInjection

    EVENT_WAIT_TIMEOUT_ERROR = EErrorCode.EventWaitTimeout
    STREAM_WRONG_STATE = EErrorCode.StreamWrongState
    REQUEST_CANCELED = EErrorCode.RequestCanceled

    IN_CALIBRATION_MODE_ERROR = EErrorCode.InCalibrationMode

    IN_CONFIGURATION_ERROR = EErrorCode.InConfiguration
    HANDS_PROCESSING = EErrorCode.HandsProcessing
    HANDS_INIT_SCENE = EErrorCode.HandsInitScene

    ILLEGAL_AGC_VALUE = EErrorCode.IllegalAGCValue
    IN_CALIBRATION_PROCESS = EErrorCode.InCalibrationProcess
    CALIBARATION_ERROR = EErrorCode.Calibaration
    CALIBRATION_TOO_LONG = EErrorCode.CalibrationTooLong
    NO_AGC_FILTER = EErrorCode.NoAGCFilter
    AUTO_CONTROL_ERROR = EErrorCode.AutoControl
    AGC_CONFIG_FILE_ERROR = EErrorCode.AGCConfigFile

    MISSING_CONFIG_FILES = EErrorCode.MissingConfigFiles
    FAILED_TO_INIT_LIBRARY = EErrorCode.FailedToInitLibrary
    CONVERSION_ERROR = EErrorCode.Conversion
    FRAME_ANALYSIS_ERROR = EErrorCode.FrameAnalysis
    LIBRARY_ISNT_INITIALIZED = EErrorCode.LibraryIsntInitialized

    HEAD_ALGORITHM_FAILED = EErrorCode.HeadAlgorithmFailed

    FR_INIT = EErrorCode.FRInit
    FR_PROCESS = EErrorCode.FRProcess
    ENROLL_NOT_STARTED = EErrorCode.EnrollNotStarted
    ENROLL_PROCESS = EErrorCode.EnrollProcess
    ENROLL_PROCESS_ERROR = EErrorCode.EnrollProcess
    ENROLL_DIR_DOESNT_EXIST = EErrorCode.EnrollDirDoesntExist

    INTERLEAVE_MISSMATCH = EErrorCode.InterleaveMissMatch

    RECORDING_ALREADY_STARTED = EErrorCode.RecordingAlreadyStarted
    NO_FILES_TO_PLAY = EErrorCode.NoFilesToPlay
    RECORDING_ERROR = EErrorCode.RcordingError

    COMPRESSION_ERROR = EErrorCode.Compression
    DECOMPRESSION_ERROR = EErrorCode.Decompression
    READ_FROM_FLASH = EErrorCode.ReadFromFlash
    WRITE_TO_FLASH = EErrorCode.WriteToFlash
    GET_FLASH_SIZE = EErrorCode.GetFlashSize
    FLASH_NOT_ENOUGH_SPACE = EErrorCode.FlashNotEnoughSpace
    CALIBRATION_DATA_LOAD = EErrorCode.CalibrationDataLoad
    WRONG_MODEL_NAME = EErrorCode.WrongModelName
    CHMOD_ERROR = EErrorCode.ChmodError
    FLASH_DYNAMIC_CALIB_PRE_COND = EErrorCode.FlashDynamicCalibPreCond
    ERASE_DYNAMIC_CALIB = EErrorCode.EraseDynamicCalib
    FLASH_DYNAMIC_CALIB = EErrorCode.FlashDynamicCalib

    DI_MODE_IS_NOT_SUPPORTED = EErrorCode.DIModeIsNotSupported
    DI_MODE_AND_VIDEO_MISMATCH = EErrorCode.DIModeAndVideoMismatch
    DI_MODE_ALGORITHM_ERROR = EErrorCode.DIModeAlgorithmError
    DEPTH_FROM_CEVA_CONFIG_FILE_ERROR = EErrorCode.DepthFromCevaConfigFile
    BAD_POINT_CLOUD_FPS = EErrorCode.BadPointCloudFPS
    DROPPED_POINT_CLOUD_FRAME = EErrorCode.DroppedPointCloudFrame
    BAD_POINT_CLOUD_DECIMATION_FACTOR = EErrorCode.BadPointCloudDecimationFactor
    INFORMATION_MAP_WITH_BLOB = EErrorCode.InformatioMapWithBlob
    BLOB_NOT_SUPPORTED = EErrorCode.BlobNotSupported
    BAD_POINT_CLOUD_FROM_FW = EErrorCode.BadPointCloudFromFW
    STATIC_TEMPORAL_FILTER_ALREADY_STARTED = EErrorCode.StaticTemporalFilterAlreadyStarted
    TEMPORAL_FILTER_ALREADY_STARTED = EErrorCode.TemporalFilterAlreadyStarted
    TEMPORAL_FILTER_FAILED_TO_START_IN_FW = EErrorCode.TemporalFilterFailedToStartInFW

    NO_SENSOR_CONFIGURATION = EErrorCode.NoSensorConfiguration
    NO_WEB_CAM_OPTICAL_DATA = EErrorCode.NoWebCamOpticalData
    FAILED_LOADING_INU_MODELS_DB = EErrorCode.FailedLoadingInuModelsDB
    MODEL_NAME_NOT_FOUND = EErrorCode.ModelNameNotFound
    GET_SENSOR_PARAMS_ERROR = EErrorCode.GetSensorParams
    UNEXPECTED_NEGATIVE_BASELINE = EErrorCode.UnexpectedNegativeBaseline
    IN_CALIBRATION_MODE_CALIBRATION_DATA_NOT_LOADED = EErrorCode.InCalibrationModeCalibrationDataNotLoaded
    INTERLEAVE_MODE_NOT_SUPPORTED_BY_CHANNEL = EErrorCode.InterleaveModeNotSupportedByChannel

    FAILED_TO_GET_IMU_HEADER = EErrorCode.FailedToGetIMUHeader
    ZERO_IMU_CHANNELS = EErrorCode.ZeroIMUChannels
    FAILED_TO_SET_IMU_FPS = EErrorCode.FailedToSetIMUFPS
    IN_FIELD_IMU_ALGO_IS_NOT_SUPPORTED = EErrorCode.InfieldIMUAlgoIsNotSupported
    ROTATION_NOT_DETECTED = EErrorCode.RotationNotDetected
    REST_WINDOW_TOOS_MALL = EErrorCode.RestWindowTooSmall
    NOT_ENOUGH_DATA_IN_CSV = EErrorCode.NotEnoughDataInCSV

    FT_IS_NOT_SUPPORTED = EErrorCode.FTIsNotSupported

    PROCESSING_PAUSED = EErrorCode.ProcessingPaused
    CNN_FILE_OPEN_FAILED = EErrorCode.CnnFileOpenFailed
    CNN_ACK_READ_FAILED = EErrorCode.CnnAckReadFailed
    INIT_AI_LIBRARY_FAILED = EErrorCode.InitAILibraryFailed
    FACE_RECOGNITION_AI = EErrorCode.FaceRecognitionAI
    OBJECTS_DETECTION_AI = EErrorCode.ObjectsDetectionAI
    CNN_ATTRIBUTES_SIZE = EErrorCode.CnnAttributesSize
    CNN_START_FAILED = EErrorCode.CnnStartFailed
    CNN_STOP_FAILE = EErrorCode.CnnStartFailed
    CNN_RELEASE_NETWORKS_FAILED = EErrorCode.CnnReleaseNetworksFailed

    ALTERNATE_PROJECTOR_NOT_SUPPORTED = EErrorCode.AlternateProjectorNotSupported

    SLAM_BY_ALGO_REQUIRE_INTERLEAVE = EErrorCode.SlamByAlgoRequiersInterleave


class Error:
    """!  Slam frame.

    Role: Encapsulates internal error presentation and provides comprehensive error codes.

    Responsibilities:
          1. Knows how to generate a meaning error description (string or EErrorCode)
    """

    # @brief    InuStreamsPyth.InuError
    #
    error: InuError = None

    def __init__(self, error: InuError):
        """! The CnnFields class initializer.
            @param error as InuStreamsPyth.InuError .
            @return  An instance of the Error object.
        """
        self.error = error

    @property
    def code(self) -> ErrorCode:
        """!  Returns InuDev internal error code.
        """
        return self.error.code

    @property
    def description(self) -> int:
        """! Conversion to comprehensive error description..
        """
        return self.error.description
