from .shared import ChannelType
from .InuStreamsPyth import HwInformation, MapUintHwChannel, MapUintSensorParams, MapStringInjectorParams, \
    InjectorP, VectorUint, EChannelType, EInterleaveMode, ESensorModel, ESensorRole, VectorUint, SensorP
from enum import IntEnum


class InjectorParams:
    """!  Injectors's info
    """

    params: InjectorP = None
    """!  InuStreamsPyth.InjectorP.
    """

    def __init__(self, params: InjectorP):
        """!
             The InjectorParams class initializer.
             return  An instance of the InjectorParams object.
        """
        self.params = params

    @property
    def streamer_name(self) -> str:
        """! Getter for streamer_name.
        """
        return self.params.StreamerName

    @property
    def connected_channels(self) -> VectorUint:
        """! Getter for connected_channels.
        """
        return self.params.ConnectedChannels


class SensorModel(IntEnum):
    """!  Sensor Models class.
    """
    NONE = ESensorModel.NONE
    AR_130_E = ESensorModel.AR_130_E
    AR_134_E = ESensorModel.AR_134_E
    AR_135_E = ESensorModel.AR_135_E
    AR_135X_ = ESensorModel.AR_135X_E
    AR_430_E = ESensorModel.AR_430_E
    AR_0234_E = ESensorModel.AR_0234_E
    APTINA_1040_E = ESensorModel.APTINA_1040_E
    OV_7251_E = ESensorModel.OV_7251_E
    OV_2685_E = ESensorModel.OV_2685_E
    GC_2145_E = ESensorModel.GC_2145_E
    XC_9160_E = ESensorModel.XC_9160_E
    OV_9282_E = ESensorModel.OV_9282_E
    OV_5675_E = ESensorModel.OV_5675_E
    OV_8856_E = ESensorModel.OV_8856_E
    OV_4689_E = ESensorModel.OV_4689_E
    CGS_132_E = ESensorModel.CGS_132_E
    OV_9782_E = ESensorModel.OV_9782_E
    CGS_031_E = ESensorModel.CGS_031_E
    GENERIC_E = ESensorModel.GENERIC_E
    AUTO_DETECT_E = ESensorModel.AUTO_DETECT_E


class SensorRole(IntEnum):
    """!  Sensor Role class.
    """
    Left = ESensorRole.Left
    Right = ESensorRole.Right
    Mono = ESensorRole.Mono


class SensorParams:
    """!  Sensor's info
    """

    params: SensorP = None
    """!  InuStreamsPyth.InjectorP.
    """

    def __init__(self, params: SensorP = None):
        """!
             The SensorParams class initializer.
             return  An instance of the SensorParams object.
        """
        if params is None:
            self.params = SensorP()
        else:
            self.params = params

    @property
    def sensor_id(self) -> int:
        # @brief Sensor Id.
        #
        # @return Sensor Id.
        return self.params.Id

    @sensor_id.setter
    def sensor_id(self, value: int) -> None:
        # @brief    sensor_id setter.
        self.params.Id = value

    @property
    def lens_type(self) -> int:
        # @brief lens_type.
        #
        # @return lens_type.
        return self.params.LensType

    @lens_type.setter
    def lens_type(self, value: int) -> None:
        # @brief    lens_type setter.
        self.params.LensType = value

    @property
    def model(self) -> SensorModel:
        # @brief sensor model.
        #
        # @return model.
        return self.params.Model

    @model.setter
    def model(self, value: SensorModel) -> None:
        # @brief    model setter.
        self.params.Model = value

    @property
    def role(self) -> SensorRole:
        # @brief sensor Role.
        #
        # @return Role.
        return self.params.Role

    @role.setter
    def role(self, value: SensorRole) -> None:
        # @brief    Role setter.
        self.params.Role = value

    @property
    def support_histogram(self) -> int:
        # @brief support_histogram.
        #
        # @return support_histogram.
        return self.params.SupportHistogram

    @support_histogram.setter
    def support_histogram(self, value: int) -> None:
        # @brief    support_histogram setter.
        self.params.SupportHistogram = value

    @property
    def connected_channels(self) -> VectorUint:
        # @brief Connected Channels.
        #
        # @return Connected Channels.
        return self.params.ConnectedChannels

    @connected_channels.setter
    def connected_channels(self, value: VectorUint) -> None:
        # @brief    Connected Channels setter.
        self.params.ConnectedChannels = value


class InterleaveMode(IntEnum):
    """! InterleaveMode enum class.
    """
    # This value will be returned in InuSensor::Init CHwInfo for each channel which doesn't support interleave.
    NOT_SUPPORTED = EInterleaveMode.NotSupported
    # Get data only from right sensor.
    RIGHT_SENSOR = EInterleaveMode.RightSensor
    # Get data only from right sensor.
    LEFT_SENSOR = EInterleaveMode.LeftSensor
    # Get data from both sensors
    INTERLEAVE = EInterleaveMode.Interleave
    # when this value is provided the configuration will be taken from InuModelDB.csv.
    DEFAULT = EInterleaveMode.Default


class HwInfo:
    info: HwInformation = None
    """!  InuStreamsPyth.HwInformation.
    """

    def __init__(self, info: HwInformation = None):
        """!
             The HwInfo class initializer.
             return  An instance of the HwInfo object.
        """
        if info is None:
            self.info = HwInformation()
        else:
            self.info = info

    @property
    def channels(self) -> MapUintHwChannel:
        """! Get for channels.
            Return   map of active channels.
        """
        return self.info.Channels

    def get_sensors(self, channel_id: int = -1) -> MapUintSensorParams:
        """! Get sensors.
            Return   map of active sensors.
        """
        if channel_id == -1:
            return self.info.Sensors
        else:
            return self.info.GetSensorsPerChannel(channel_id)

    def get_injectors(self, channel_id: int = -1) -> MapStringInjectorParams:
        """! Get injector.
            Return   InjectorParams per channel id.
        """
        if channel_id == -1:
            return self.info.Injectors
        else:
            return self.info.GetInjectorsPerChannel(channel_id)

    def set_channel_interleave(self, channel_id: int, mode: InterleaveMode):
        """! Setter injector.
            Sets which sensors will send frames when sensors are attached to channel.
        """
        self.info.SetChannelInterleave(channel_id, EInterleaveMode.EInterleaveMode(mode))

    def set_channel_type_interleave(self, channel_type: ChannelType, mode: InterleaveMode):
        """! Setter injector.
            Sets which sensors will send frames when sensors are attached to channel.
        """
        self.info.SetChannelTypeInterleave(EChannelType(channel_type), EInterleaveMode.EInterleaveMode(mode))
