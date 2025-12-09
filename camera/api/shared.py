from .InuStreamsPyth import Sensor, EImageFormat, CnnLoadP, EChannelType, ExposureP, SensorControlP, Point2D, \
    AutoExposureP, SensorControlROIP, CropP, AlternateProjectorM, ChannelD, ROIParams, EImageFormatFw, \
    EInterliveModeFw, Point3DFloat
from enum import IntEnum
from typing import Union


class ImageFormatFw(IntEnum):
    """!  ImageFormat FW enum class.
        Comment: Defines Image Fw Formats
    """
    GRAY16 = EImageFormatFw.Gray16
    BAYER16 = EImageFormatFw.Bayer16
    RGB888 = EImageFormatFw.RGB888
    RGB666 = EImageFormatFw.RGB666
    RGB565 = EImageFormatFw.RGB565
    RGB555 = EImageFormatFw.RGB555
    RGB444 = EImageFormatFw.RGB444
    YUV420_8BIT = EImageFormatFw.YUV420_8bit
    YUV420_8BITLEGACY = EImageFormatFw.YUV420_8bitLegacy
    YUV420_10BIT = EImageFormatFw.YUV420_10bit
    YUV222_8BIT = EImageFormatFw.YUV222_8bit
    YUV222_10BIT = EImageFormatFw.YUV222_10bit
    RAW6 = EImageFormatFw.Raw6
    RAW7 = EImageFormatFw.Raw7
    RAW8 = EImageFormatFw.Raw8
    RAW10 = EImageFormatFw.Raw10
    RAW12 = EImageFormatFw.Raw12
    RAW14 = EImageFormatFw.Raw14
    GEN8 = EImageFormatFw.Gen8
    GEN12 = EImageFormatFw.Gen12
    GEN16 = EImageFormatFw.Gen16
    GEN24 = EImageFormatFw.Gen24
    GEN32 = EImageFormatFw.Gen32
    GEN64 = EImageFormatFw.Gen64
    GEN96 = EImageFormatFw.Gen96
    GEN672 = EImageFormatFw.Gen672
    DEPTH = EImageFormatFw.Depth
    DISPARITY = EImageFormatFw.Disparity
    DISPARITY_DEBUG = EImageFormatFw.DisparityDebug
    HISTOGRAM = EImageFormatFw.Histogram
    NA = EImageFormatFw.NA
    NOF_FORMATS = EImageFormatFw.NofFormats


class InterliveModeFw(IntEnum):
    """!  InterliveMode FW enum class.
        Comment: Defines FW interlive Mode
    """
    LINE_BY_LINE = EInterliveModeFw.LineByLine
    PIXEL_BY_PIX = EInterliveModeFw.PixelByPixel
    FRAME_BY_FRAME = EInterliveModeFw.FrameByFrame
    UNKNOWN = EInterliveModeFw.Unknown
    NOF_INTERLIVE_MODE = EInterliveModeFw.NofInterliveMode


class ProjectorLevel(IntEnum):
    """! ProjectorLevel enum class.
        Role:  Projector intensity levels.
    """
    OFF = Sensor.EProjectorLevel.Off
    LOW = Sensor.EProjectorLevel.Low
    HIGH = Sensor.EProjectorLevel.High


class ProjectorType(IntEnum):
    """! ProjectorType enum class.
        Role: All projectors that can be assembled in InuSensor.
    """
    PATTERNS = Sensor.EProjectors.Patterns
    FLOOD = Sensor.EProjectors.Flood


class StreamType(IntEnum):
    """! StreamType enum class.
        Role: All streams which are supported in InuSensor.
    """
    DEPTH = 1
    STEREO = 2
    GENERAL_CAMERA = 4
    IMU = 8
    FEATURES_TRACKING = 16
    HISTOGRAM = 32
    SLAM = 64
    TRACKING = 128
    USER_DEFINE = 256
    CNN = 512
    POINT_CLOUD = 1024
    INJECTION = 2048
    CNN_APP = 4096
    TEMPERATURE = 8192
    GENERAL_CAMERA_REGISTERED = 16384


class ChannelType(IntEnum):
    """! ChannelType enum class.
        Role:HW Channels Types supported by Inuitive Sensor.
    """
    UNKNOWN = EChannelType.Unknown
    # Output frames from the sensors that are not tracking nor IR.
    GENERAL_CAMERA = EChannelType.GeneralCamera
    # Output frames from the sensors that are used by the chip to identify features.
    TRACKING = EChannelType.Tracking
    # Output frames from the IR sensors
    STEREO = EChannelType.Stereo
    # Output depth from the chip
    DEPTH = EChannelType.Depth
    # Output features from the chip
    FEATURES_TRACKING = EChannelType.FeaturesTracking


class ImageFormat(IntEnum):
    """! ImageFormat enum class.
        Role: All Image formats supported formats in InuSensor.
    """
    #  Used when channel format is not of  image type (e.g. Features)
    UNDEFINED_FORMAT = EImageFormat.UndefinedFormat
    # Used to request the default channel format from the FW when starting the Sensor
    DEFAULT_FORMAT = EImageFormat.DefaultFormat
    # Empty buffer
    EMPTY = EImageFormat.Empty
    # Bytes per pixel : Z - Buffer(16 bits per pixel)
    DEPTH = EImageFormat.Depth
    # Bytes per pixel : B, G and R
    BGR = EImageFormat.BGR
    # 4 Bytes per pixel : B, G, R and Alpha(which is always 0xff - 100 %)
    BGRA = EImageFormat.BGRA
    # 2 Bytes per pixel: 4 MSB of confidence and 12 LSB of Disparities
    DISPARITY = EImageFormat.Disparity
    # 4 Bytes per pixel: R, G, B and Alpha (which is always 0xff - 100%)
    RGBA = EImageFormat.RGBA
    # bytes per pixel: Compressed Y, U, V
    YUV422 = EImageFormat.RGBA
    # 3 Bytes per pixel: R, G and B
    RGB = EImageFormat.RGB
    # 2 bytes per pixel: Compressed Y, V, U
    YVU422 = EImageFormat.YVU422
    # 2 Bytes per pixel: 10 bits Y (the 11th bit indicates overflow)
    Y = EImageFormat.YVU422
    # 2 Bytes per pixel: Bayer GRBG
    BAYER_GRBG = EImageFormat.BayerGRBG
    # 2 Bytes per pixel: Bayer RGGB
    BAYER_RGGB = EImageFormat.BayerRGGB
    # Bytes per pixel: Bayer BGGR
    BAYER_BGGR = EImageFormat.BayerBGGR
    # 2 Bytes per pixel: Bayer GBRG
    BAYER_GBRG = EImageFormat.BayerGBRG
    # 1 Bytes per pixel: Class/Label per pixel
    SEGMENTATION_IMAGE = EImageFormat.SegmentationImage
    # 2 Bytes per pixel: 10 bits Y located in msb
    Y10MSB = EImageFormat.Y10MSB
    # 3 Bytes per pixel: yyyyyyyy uuuu00yy 00uuuuuu
    YUV_10B = EImageFormat.YUV_10B
    # 1 Byte  per pixel: Y only
    RAW8 = EImageFormat.YUV_10B


# class SensorResolution(IntEnum):
#     """! SensorResolution enum class.
#         Role: All resolutions supported by Inuitive Sensor.
#     """
#     # Sensor default resolutions.
#     DEFAULT = ESensorResolution.DefaultResolution
#     # Sensor's binning mode (reduced resolution provided by sensor).
#     BINNING = ESensorResolution.Binning
#     # Vertical binning resolution.
#     VERTICAL_BINNING = ESensorResolution.VerticalBinning
#     #  Full sensor resolution
#     FULL = ESensorResolution.Full


class ExposureParams:
    """! ExposureParams class.
        Role: Controls ExposureParams .
    """

    # @brief    InuStreamsPyth.ExposureP
    #
    params = None

    def __init__(self, params: ExposureP = None):
        # @brief     The ExposureParams class initializer.
        #
        # @return  An instance of the ExposureParams object.
        if params is None:
            self.params = ExposureP()
        else:
            self.params = params

    @property
    def exposure_time(self) -> int:
        # @brief    Exposure Time of the sensor .
        #
        # @return exposure_time as int.
        return self.params.ExposureTime

    @exposure_time.setter
    def exposure_time(self, value: int) -> None:
        # @brief    exposure_time setter.
        self.params.ExposureTime = value

    @property
    def digital_gain(self) -> float:
        # @brief    Digital Gain of the sensor.
        #
        # @return digital_gain as float.
        return self.params.DigitalGain

    @digital_gain.setter
    def digital_gain(self, value: int) -> None:
        # @brief    digital_gain setter
        self.params.DigitalGain = value

    @property
    def analog_gain(self) -> float:
        # @brief    Digital Gain of the sensor.
        #
        # @return analog_gain as float.
        return self.params.AnalogGain

    @analog_gain.setter
    def analog_gain(self, value: int) -> None:
        # @brief    analog_gain setter
        self.params.AnalogGain = value


class Point2Dim:
    """! Point2D class.
        Role: Controls Point2D  .
    """

    # @brief    InuStreamsPyth.Point2D
    #
    point: Point2D = None

    def __init__(self, first: Union[int, Point2D] = 0, second: int = 0):
        """! The Point2D class initializer.
            @return  An instance of the Point2Dim object.
        """
        if type(first) is Point2D:
            self.point = first
        else:
            self.point = Point2D(first, second)

    @property
    def x(self) -> int:
        # @ x
        return self.point.X

    @x.setter
    def x(self, value: int) -> None:
        # @brief    x setter.
        self.point.x = value

    @property
    def y(self) -> int:
        # @ y
        return self.point.y

    @y.setter
    def y(self, value: int) -> None:
        # @brief    y setter.
        self.point.y = value


class Point3DimFloat:
    """! Point3DimFloat class.
        Role: Controls Point3DimFloat  .
    """

    # @brief    InuStreamsPyth.Point3DFloat
    #
    point: Point3DFloat = None

    def __init__(self, first: Union[float, Point3DFloat] = None, second: float = 0.0, third: float = 0.0):
        """! The Point2D class initializer.
            @return  An instance of the Point2Dim object.
        """
        if type(first) is Point3DFloat:
            self.point = first
        else:
            self.point = Point3DFloat(first, second, third)

    @property
    def x(self) -> float:
        # @ x
        return self.point.X

    @x.setter
    def x(self, value: float) -> None:
        # @brief    x setter.
        self.point.X = value

    @property
    def y(self) -> float:
        # @ y
        return self.point.Y

    @y.setter
    def y(self, value: float) -> None:
        # @brief    y setter.
        self.point.Y = value

    @property
    def z(self) -> float:
        # @ z
        return self.point.Z

    @z.setter
    def z(self, value: float) -> None:
        # @brief    z setter.
        self.point.Z = value


class ROIParameters:
    """! ROIParameters class.
        Role : ROI (region of interest) for automatic sensor control  algorithm.
    """

    # @brief    InuStreamsPyth.ROIParams
    #
    params: ROIParams = None

    def __init__(self, top: Union[ROIParams, int] = 0, left: int = 0, right: int = 0, bottom: int = 0):
        # @brief     The ROIParameters class initializer.
        #
        # @return  An instance of the ROIParameters object.
        if top is ROIParams:
            self.params = top
        else:
            self.params = ROIParams(top, left, right, bottom)

    @property
    def roi_top_left(self) -> Point2Dim:
        # @brief Top left corner of ROI.
        #
        # @return roi_top_left.
        return Point2Dim(self.params.ROITopLeft)

    @roi_top_left.setter
    def roi_top_left(self, value: Point2Dim) -> None:
        # @brief    roi_top_left setter.
        self.params.ROITopLeft = value.point

    @property
    def roi_bottom_right(self) -> Point2Dim:
        # @brief Bottom right corner of ROI.
        #
        # @return roi_bottom_right.
        return Point2Dim(self.params.ROIBottomRight)

    @roi_bottom_right.setter
    def roi_bottom_right(self, value: Point2Dim) -> None:
        # @brief    roi_bottom_right setter.
        self.params.ROIBottomRight = value.point


class SensorControlROIParams(ROIParameters):
    """! SensorControlROIParams class.
        Role : ROI (region of interest) for automatic sensor control  algorithm.
    """

    # @brief    InuStreamsPyth.SensorControlROIP
    #
    params: SensorControlROIP = None

    def __init__(self, params: SensorControlROIP = None):
        # @brief     The SensorControlROIParams class initializer.
        #
        # @return  An instance of the SensorControlROIParams object.
        if params is None:
            self.params = SensorControlROIP()
            ROIParameters.__init__(params.ROITopLeft.X, params.ROITopLeft.Y, params.ROIBottomRight.X,
                                   params.ROIBottomRight.Y)
        else:
            self.params = params

    @property
    def use_roi(self) -> bool:
        # @brief If true then ROI is applied in automatic sensor control.
        #
        # @return use_roi on/off.
        return self.params.UseROI

    @use_roi.setter
    def use_roi(self, value: bool) -> None:
        # @brief    use_roi setter.
        self.params.UseROI = value


class SensorControlParams(ExposureParams):
    """! SensorControlParams class.
        Role : Sensor control parameters.
    """

    def __init__(self, params: SensorControlP = None):
        # @brief     The SensorControlParams class initializer.
        #
        # @return  An instance of the SensorControlParams object.
        if params is None:
            self.params = SensorControlP()
        else:
            self.params = params
            ExposureParams.__init__(self, params)

    # @brief    InuStreamsPyth.SensorControlP
    #
    params = None

    @property
    def auto_control(self) -> bool:
        # @brief If this flag is true then automatic algorithm for gain and exposure time adjustment (AGC) is activated.
        #
        # @return auto_control on/off.
        return self.params.AutoControl

    @auto_control.setter
    def auto_control(self, value: bool) -> None:
        # @brief    auto_control setter.
        self.params.AutoControl = value

    @property
    def sensor_control_roi_params(self) -> SensorControlROIParams:
        # @brief ROI of each sensor.
        #
        # @return auto_control on/off.
        return SensorControlROIParams(self.params.GetRoiParams())

    @auto_control.setter
    def sensor_control_roi_params(self, value: SensorControlROIParams) -> None:
        # @brief    sensor_control_roi_params setter.
        self.params.self.params.SetRoiParams(value.params)


class AutoExposureParams:
    """!
        Role : AutoExposureParams class.
    """

    # @brief    InuStreamsPyth.AutoExposureP
    #
    params: AutoExposureP = None

    def __init__(self, params: AutoExposureP = None):
        # @brief     The AutoExposureParams class initializer.
        #
        # @return  An instance of the AutoExposureParams object.
        if params is None:
            self.params = AutoExposureP()
        else:
            self.params = params

    @property
    def n_sat_max(self) -> int:
        # @brief If number (in %) of saturated pixel is above this value, then the frame is saturated.
        #
        # @return n_sat_max.
        return self.params.NSatMax

    @n_sat_max.setter
    def n_sat_max(self, value: int) -> None:
        # @brief    n_sat_max setter.
        self.params.NSatMax = value

    @property
    def gl_sat(self) -> int:
        # @brief Pixels (only inside ROI) above/equal this value are defined as saturated.
        #
        # @return gl_sat.
        return self.params.GlSat

    @gl_sat.setter
    def gl_sat(self, value: int) -> None:
        # @brief    gl_sat setter.
        self.params.GlSat = value

    @property
    def md_max(self) -> int:
        # @brief Max allowed value for median.
        #
        # @return md_max.
        return self.params.MdMax

    @md_max.setter
    def md_max(self, value: int) -> None:
        # @brief    md_max setter.
        self.params.MdMax = value

    @property
    def md_min(self) -> int:
        # @brief Min allowed value for median.
        #
        # @return md_min.
        return self.params.MdMin

    @md_min.setter
    def md_min(self, value: int) -> None:
        # @brief    md_min setter.
        self.params.MdMin = value

    @property
    def exposure_step_resolution(self) -> int:
        # @brief Exposure step resolution.
        #
        # @return exposure_step_resolution.
        return self.params.ExposureStepResolution

    @exposure_step_resolution.setter
    def exposure_step_resolution(self, value: int) -> None:
        # @brief    exposure_step_resolution setter.
        self.params.ExposureStepResolution = value

    @property
    def delta_sat_max(self) -> int:
        # @brief In case of saturation: threshold (in %) to choose aggressive or normal step. If (number (in %)
        #   of saturated pixel- nSatMax)> deltaSatMax Can choose aggressive step.
        #
        # @return delta_sat_max.
        return self.params.DeltaSatMax

    @delta_sat_max.setter
    def delta_sat_max(self, value: int) -> None:
        # @brief    delta_sat_max setter.
        self.params.DeltaSatMax = value

    @property
    def aggressive_step(self) -> int:
        # @brief  Aggressive ET step.
        #
        # @return aggressive_step.
        return self.params.AggressiveStep

    @aggressive_step.setter
    def aggressive_step(self, value: int) -> None:
        # @brief    aggressive_step setter.
        self.params.AggressiveStep = value

    @property
    def no_activation_period(self) -> int:
        # @brief  Waiting frames between each activation.
        #
        # @return no_activation_period.
        return self.params.NoActivationPeriod

    @no_activation_period.setter
    def no_activation_period(self, value: int) -> None:
        # @brief    no_activation_period setter.
        self.params.NoActivationPeriod = value

    @property
    def exposure_max(self) -> int:
        # @brief  Exposure max value.
        #
        # @return exposure_max.
        return self.params.ExposureMax

    @exposure_max.setter
    def exposure_max(self, value: int) -> None:
        # @brief    exposure_max setter.
        self.params.ExposureMax = value

    @property
    def exposure_min(self) -> int:
        # @brief  Exposure min value.
        #
        # @return exposure_min.
        return self.params.ExposureMin

    @exposure_min.setter
    def exposure_min(self, value: int) -> None:
        # @brief    exposure_min setter.
        self.params.ExposureMin = value

    @property
    def debug(self) -> int:
        # @brief   Activate debug prints.
        #
        # @return debug.
        return self.params.Debug

    @debug.setter
    def debug(self, value: int) -> None:
        # @brief    debug setter.
        self.params.Debug = value

    @property
    def snr_target(self) -> float:
        # @brief   TSNR target value.
        #
        # @return snr_target.
        return self.params.SnrTarget

    @snr_target.setter
    def snr_target(self, value: float) -> None:
        # @brief    snr_target setter.
        self.params.SnrTarget = value

    @property
    def slope_weight(self) -> float:
        # @brief  Memory weight factor for current slope.
        #
        # @return slope_weight.
        return self.params.SlopeWeight

    @slope_weight.setter
    def slope_weight(self, value: float) -> None:
        # @brief    slope_weight setter.
        self.params.SlopeWeight = value

    @property
    def alg_version(self) -> int:
        # @brief  Auto exposure algorithm version.
        #
        # @return alg_version.
        return self.params.AlgVersion

    @alg_version.setter
    def alg_version(self, value: int) -> None:
        # @brief    alg_version setter.
        self.params.AlgVersion = value


class CropParams:
    """!
        Role : Defines the cropping rectangle (Width, Height) and Upper left corner position (StartX, StartY).
   """

    # @brief    InuStreamsPyth.CropP
    #
    params: CropP = None

    def __init__(self, params: CropP = None):
        # @brief     The CropParams class initializer.
        #
        # @return  An instance of the CropParams object.
        if params is None:
            self.params = CropP()
        else:
            self.params = params

    @property
    def width(self) -> int:
        # @brief width.
        #
        # @return width.
        return self.params.Width

    @width.setter
    def width(self, value: int) -> None:
        # @brief    width setter.
        self.params.Width = value

    @property
    def height(self) -> int:
        # @brief height.
        #
        # @return height.
        return self.params.Height

    @height.setter
    def height(self, value: int) -> None:
        # @brief    height setter.
        self.params.Height = value

    @property
    def start_x(self) -> int:
        # @brief start_x.
        #
        # @return start_x.
        return self.params.start_x

    @start_x.setter
    def start_x(self, value: int) -> None:
        # @brief    start_x setter.
        self.params.StartX = value

    @property
    def start_y(self) -> int:
        # @brief start_y.
        #
        # @return start_y.
        return self.params.StartY

    @start_x.setter
    def start_y(self, value: int) -> None:
        # @brief    start_y setter.
        self.params.StartY = value


class ChannelDimensions(CropParams):
    """!
        Role : Defines the dimension of the resized image
   """

    # @brief    InuStreamsPyth.ChannelD
    #
    params: ChannelD = None

    def __init__(self, params: ChannelD = None):
        # @brief     The ChannelDimensions class initializer.
        #
        # @return  An instance of the ChannelDimensions object.
        if params is None:
            self.params = ChannelD()
        else:
            self.params = params
            CropParams.params = params

    @property
    def buffer_width(self) -> int:
        # @brief  The width of the output image from the channel after scaling in the chip.
        #
        # @return buffer_width.
        return self.params.BufferWidth

    @buffer_width.setter
    def buffer_width(self, value: int) -> None:
        # @brief    buffer_width setter.
        self.params.BufferWidth = value

    @property
    def buffer_height(self) -> int:
        # @brief  The height of the output image from the channel after scaling in the chip.
        #
        # @return buffer_height.
        return self.params.BufferHeight

    @buffer_height.setter
    def buffer_height(self, value: int) -> None:
        # @brief    buffer_height setter.
        self.params.BufferHeight = value


class AlternateProjectorMode:
    """!
        Role : AlternateProjectorMode
   """

    # @brief    InuStreamsPyth.AlternateProjectorM
    #
    mode: AlternateProjectorM = None

    def __init__(self, mode: AlternateProjectorM = None):
        # @brief     The AlternateProjectorMode class initializer.
        #
        # @return  An instance of the AlternateProjectorMode object.
        if mode is None:
            self.mode = AlternateProjectorM()
        else:
            self.mode = mode

    @property
    def enable(self) -> bool:
        # @brief  Alternate Mode Enable.
        #
        # @return enable.
        return self.params.AlternateModeEnable

    @enable.setter
    def enable(self, value: int) -> None:
        # @brief    enable setter.
        self.params.AlternateModeEnable = value

    @property
    def num_frames_pattern(self) -> int:
        # @brief  Num Frames Pattern.
        #
        # @return num_frames_pattern.
        return self.params.NumFramesPattern

    @num_frames_pattern.setter
    def num_frames_pattern(self, value: int) -> None:
        # @brief    num_frames_pattern setter.
        self.params.NumFramesPattern = value

    @property
    def num_frames_flood(self) -> int:
        # @brief  Num Frames Flood.
        #
        # @return num_frames_flood.
        return self.params.NumFramesFlood

    @num_frames_flood.setter
    def num_frames_flood(self, value: int) -> None:
        # @brief    num_frames_flood setter.
        self.params.NumFramesFlood = value
