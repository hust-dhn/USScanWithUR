from .base_stream import BaseStream
from .depth_stream import DepthStream, DepthOutputFormat, DepthPostProcessing
from .image_stream import ImageStream, ImageOutputFormat, ImagePostProcessing
from .stereo import StereoStream, StereoOutputFormat
from .features_tracking import FeaturesTrackingStream, FeaturesTrackingFrame, FeaturesTrackingOutputFormat
from .imu import ImuStream
from .slam import SlamStream, SlamTransformationsParams
from .injection import InjectionStream
from .histogram import HistogramStream
from .cnn_app import CnnAppStream, CnnAppOutputFormat
from .cnn import CnnStream
from .point_cloud import PointCloudStream, VoxelFilterParams, PointCloudRegistrationType
from .temperature import TemperatureStream, SensorTemperatureType
from .cnn_defs import CnnParams
from .errors import Error
from .hw_info import HwInfo
from .calibration_data import CalibrationData
from .shared import StreamType, ProjectorType, ProjectorLevel, SensorControlParams, AutoExposureParams, CropParams, \
    ChannelDimensions, AlternateProjectorMode, Point2Dim
from .InuStreamsPyth import MapUintPoint, Sensor, VectorROIParams, MapEntitiesIDVersion, ESensorResolution, \
    EInjectionType, DeviceParamsExt, InuError
    
from typing import Union
from enum import IntEnum


class ConnectionState(IntEnum):
    """!  ConnectionState enum class.
        Represents all possible states of connection with Sensor
    """
    # @brief Initial  state, connection  has  not been established
    UNKNOWN_CONNECTION = Sensor.EConnectionState.UnknownConnection
    # @brief Sensor is connected
    CONNECTED = Sensor.EConnectionState.Connected
    # @brief No sensor is connected
    DISCONNECTED = Sensor.EConnectionState.Disconnected
    # @briefCan't communicate with InuService
    SERVICE_DISCONNECTED = Sensor.EConnectionState.ServiceDisconnected


class SensorState(IntEnum):
    """!  SensorState enum class.
        Represents all possible states of Sensor.
    """
    # @brief Uninitialized  state
    UNINITIALIZED = Sensor.Uninitialized
    # @brief Initialized  state
    INITIALIZED = Sensor.Initialized
    # @brief Started  state
    STARTED = Sensor.Started
    # @brief SensorDisconnected  state
    SENSOR_DISCONNECTED = Sensor.SensorDisconnected
    # @brief ServiceNotConnected  state
    SERVICE_NOT_CONNECTED = Sensor.ServiceNotConnected


class SensorResolution(IntEnum):
    """!  SensorResolution enum class.
        All resolutions supported by Inuitive Sensor.
    """
    DEFAULT = ESensorResolution.Default  # Sensor   default    resolutions
    BINNING = ESensorResolution.Binning  # Sensor's binning mode (reduced resolution provided by sensor)
    VERTICAL_BINNING = ESensorResolution.VerticalBinning  # Vertical  binning   resolution
    FULL = ESensorResolution.Full  # Full sensor resolution


class InjectionType(IntEnum):
    """!
        The Injection Types
    """
    NONE = EInjectionType.InjectionNone
    USER_DEFINE = EInjectionType.InjectionUserDefine
    IDVE = EInjectionType.Injection2IDVE


class TemperatureType(IntEnum):
    """!
        The Sensor Temperature Types
    """
    SENSOR_1 = Sensor.TemperatureSensor1,
    SENSOR_2 = Sensor.TemperatureSensor2


class DeviceParams:
    """! Sensor parameters
    """
    params = None
    """! InuStreamsPyth.DeviceParamsExt
    """

    def __init__(self, params: DeviceParamsExt = None):
        """! The DeviceParams class initializer.
            param params : DeviceParamsExt
            return  An instance of the DeviceParams.
        """
        if params is None:
            self.params = DeviceParamsExt()
        else:
            self.params = params

    @property
    def sw_graph(self) -> str:
        """! The path to FW's "software graph.
        """
        return self.params.GraphXmlPath

    @sw_graph.setter
    def sw_graph(self, value: str) -> None:
        """! sw_graph setter
        """
        self.params.sw_graph = value

    @property
    def hw_graph(self) -> str:
        """! hw_graph
        """
        return self.params.mHwGraphXmlPath

    @hw_graph.setter
    def hw_graph(self, value: str) -> None:
        """! hw_graph setter
        """
        self.params.mHwGraphXmlPath = value

    @property
    def resolution(self) -> SensorResolution:
        """! Sensor Resolution.
        """
        return self.params.SensorRes

    @resolution.setter
    def resolution(self, value: SensorResolution) -> None:
        """! resolution setter
        """
        self.params.SensorRes = ESensorResolution.ESensorResolution(value)

    @property
    def fps(self) -> int:
        """! Frame rate (number of frames per second).
        """
        return self.params.FPS

    @fps.setter
    def fps(self, value: int) -> None:
        """! fps setter
        """
        self.params.FPS = value

    @property
    def uart_port_name(self) -> str:
        """!
            Uart port name convention(if not provided, default value taken from InuServiceParams.xml)
            If provided, UART sync mode used will be PllLockUart.
        """
        return self.params.UartPortName

    @uart_port_name.setter
    def uart_port_name(self, value: str) -> None:
        """! uart_port_name setter
        """
        self.params.UartPortName = value

    @property
    def simulation_mode(self) -> bool:
        """! simulation_mode
        """
        return self.params.SimulationMode

    @simulation_mode.setter
    def simulation_mode(self, value: bool) -> None:
        """! simulation_mode setter
        """
        self.params.SimulationMode = value

    @property
    def calibration_mode(self) -> bool:
        """! calibration_mode
        """
        return self.params.IsCalibrationMode

    @calibration_mode.setter
    def calibration_mode(self, value: bool) -> None:
        """! calibration_mode setter
        """
        self.params.IsCalibrationMode = value

    @property
    def simulation_input_folder(self) -> bool:
        """! simulation_mode
        """
        return self.params.SimulationInputFolder

    @simulation_input_folder.setter
    def simulation_input_folder(self, value: bool) -> None:
        """! simulation_input_folder setter
        """
        self.params.SimulationInputFolder = value

    @property
    def calibration_dir_name(self) -> str:
        """! calibration_dir_name
            if it is not empty then calibration data is loaded from this directory
        """
        return self.params.CalibrationDirName

    @calibration_dir_name.setter
    def calibration_dir_name(self, value: str) -> None:
        """! calibration_dir_name setter
        """
        self.params.CalibrationDirName = value

    @property
    def injection_type(self) -> InjectionType:
        """! injection_type
            if it is not empty then calibration data is loaded from this directory
        """
        return InjectionType(self.params.InjectionType)

    @injection_type.setter
    def injection_type(self, value: InjectionType) -> None:
        """! injection_type setter
        """
        self.params.InjectionType = EInjectionType.EInjectionType(value)

    @property
    def boot_path(self) -> str:
        """! boot_path
            Boot path for bypass mode(Calibration).
        """
        return self.params.BootPath

    @boot_path.setter
    def boot_path(self, value: str) -> None:
        """! boot_path setter
        """
        self.params.BootPath = value


class InuSensor:
    # @brief    InuStreamsPyth.Sensor.
    #
    _sensor = None

    # @brief    HwInfo.
    #
    _hw_information = HwInfo()

    # @brief   IP address string.
    #
    _ip_address = None

    def __init__(self, service_id: str = '', ip_address: str = ''):
        self._sensor = Sensor(service_id)
        self._ip_address = ip_address
        """! The Sensor class initializer.
            @param serviceId  The Service Id string.
            @param ip_address  The IP Address string.
            @return  An instance of the Sensor initialized with the specified serviceId and ip_address.
        """

    def create_stream(self, stream_type: StreamType, create_arg: Union[int, str] = BaseStream.DEFAULT_CHANNEL_ID) -> \
            Union[DepthStream, StereoStream, ImageStream, ImuStream, FeaturesTrackingStream, HistogramStream,
            SlamStream, CnnAppStream, CnnStream, PointCloudStream, InjectionStream, TemperatureStream]:
        # @brief    SetInjectResolution
        #
        # @Generate all kinds of InuDev streams
        # @param streamType     EStreamType.
        # @param id             The streamer id - channel id or streamer name.
        # @return               An instance of different types of streams with the specified serviceId and ip_address
        if stream_type == StreamType.DEPTH:
            if create_arg == BaseStream.DEFAULT_CHANNEL_ID:
                return DepthStream(self._sensor.CreateDepthStream())
            return DepthStream(self._sensor.CreateDepthStream(create_arg))
        elif stream_type == StreamType.STEREO:
            if create_arg == BaseStream.DEFAULT_CHANNEL_ID:
                return StereoStream(self._sensor.CreateStereoStream())
            return StereoStream(self._sensor.CreateStereoStream(create_arg))
        elif stream_type == StreamType.GENERAL_CAMERA:
            if type(id) == str:
                return ImageStream(self._sensor.CreateImageStream(create_arg))
            elif create_arg == BaseStream.DEFAULT_CHANNEL_ID:
                return ImageStream(self._sensor.CreateImageStream())
            return ImageStream(self._sensor.CreateImageStream(create_arg))
        elif stream_type == StreamType.IMU:
            if create_arg == BaseStream.DEFAULT_CHANNEL_ID:
                return ImuStream(self._sensor.CreateImuStream())
            return ImuStream(self._sensor.CreateImuStream(create_arg))
        elif stream_type == StreamType.FEATURES_TRACKING:
            if type(create_arg) == str:
                return FeaturesTrackingStream(self._sensor.CreateFeaturesTrackingStream(create_arg))
            return FeaturesTrackingStream(self._sensor.CreateFeaturesTrackingStream())
        elif stream_type == StreamType.HISTOGRAM:
            return HistogramStream(self._sensor.CreateHistogramStream(create_arg))
        elif stream_type == StreamType.SLAM:
            if create_arg == BaseStream.DEFAULT_CHANNEL_ID:
                return SlamStream(self._sensor.CreateSlamStream())
            return SlamStream(self._sensor.CreateSlamStream(create_arg))
        elif stream_type == StreamType.TRACKING:
            return ImageStream(self._sensor.CreateImageStream(2))
        elif stream_type is StreamType.USER_DEFINE:
            raise Exception("StreamType.USER_DEFINE doesn't supported yet.")
        elif stream_type == StreamType.CNN_APP:
            if create_arg == BaseStream.DEFAULT_CHANNEL_ID:
                return CnnAppStream(self._sensor.CreateCnnAppStream())
            return CnnAppStream(self._sensor.CreateCnnAppStream(create_arg))
        elif stream_type == StreamType.POINT_CLOUD:
            if create_arg == BaseStream.DEFAULT_CHANNEL_ID:
                return PointCloudStream(self._sensor.CreatePointCloudStream())
            return PointCloudStream(self._sensor.CreatePointCloudStream(create_arg))
        elif stream_type == StreamType.INJECTION:
            return InjectionStream(self._sensor.CreateInjectionStream(create_arg), self._sensor)
        elif stream_type == StreamType.CNN:
            if create_arg is None:
                return CnnStream(self._sensor.CreateCnnStream("Sout_cnn_0"))
            return CnnStream(self._sensor.CreateCnnStream(create_arg))
        elif stream_type == StreamType.TEMPERATURE:
            if create_arg == BaseStream.DEFAULT_CHANNEL_ID:
                return TemperatureStream(self._sensor.CreateTemperaturesStream(SensorTemperatureType.ALL))
            return TemperatureStream(self._sensor.CreateTemperaturesStream(create_arg))
        return None

    def create_started_stream(self, stream_type: StreamType, create_arg: Union[int, str] =
    BaseStream.DEFAULT_CHANNEL_ID, callback_function=None,
                              arg_1: Union[str, DepthOutputFormat, StereoOutputFormat,
                              ImageOutputFormat, FeaturesTrackingOutputFormat, SlamTransformationsParams,
                              CnnAppOutputFormat, PointCloudRegistrationType, VoxelFilterParams] = None,
                              arg_2: Union[str, DepthPostProcessing, ImagePostProcessing, int] = None) -> Union[
        DepthStream, StereoStream,
        ImageStream, ImuStream, FeaturesTrackingStream, HistogramStream, SlamStream, CnnAppStream, CnnStream,
        PointCloudStream, InjectionStream]:
        # @brief    SetInjectResolution
        #
        # @Generate all kinds of InuDev streams
        # @param streamType     EStreamType.
        # @param id             The streamer id - channel id or streamer name.
        # @return               An instance of different types of streams with the specified serviceId and ip_address
        stream = None
        if stream_type == StreamType.DEPTH:
            if create_arg == BaseStream.DEFAULT_CHANNEL_ID:
                stream = DepthStream(self._sensor.CreateDepthStream())
            else:
                stream = DepthStream(self._sensor.CreateDepthStream(create_arg))
            if arg_1 is None and arg_2 is None:
                stream.init()
            elif arg_2 is None:
                stream.init(arg_1)
            else:
                stream.init(arg_1, arg_2)
            stream.start()
        elif stream_type == StreamType.STEREO:
            if create_arg == BaseStream.DEFAULT_CHANNEL_ID:
                stream = StereoStream(self._sensor.CreateStereoStream())
            else:
                stream = StereoStream(self._sensor.CreateStereoStream(create_arg))
            if arg_1 is None:
                stream.init()
            else:
                stream.init(arg_1)
            stream.start()
        elif stream_type == StreamType.GENERAL_CAMERA:
            if type(id) == str:
                stream = ImageStream(self._sensor.CreateImageStream(create_arg))
            elif create_arg == BaseStream.DEFAULT_CHANNEL_ID:
                stream = ImageStream(self._sensor.CreateImageStream())
            else:
                stream = ImageStream(self._sensor.CreateImageStream(create_arg))
            if arg_1 is None and arg_2 is None:
                stream.init()
            elif arg_2 is None:
                stream.init(arg_1)
            else:
                stream.init(arg_1, arg_2)
            stream.start()
        elif stream_type == StreamType.IMU:
            if create_arg == BaseStream.DEFAULT_CHANNEL_ID:
                stream = ImuStream(self._sensor.CreateImuStream())
            else:
                stream = ImuStream(self._sensor.CreateImuStream(create_arg))
            stream.init()
            stream.start()
        elif stream_type == StreamType.FEATURES_TRACKING:
            if type(create_arg) == str:
                stream = FeaturesTrackingStream(self._sensor.CreateFeaturesTrackingStream(create_arg))
            else:
                stream = FeaturesTrackingStream(self._sensor.CreateFeaturesTrackingStream())
            if arg_1 is None:
                stream.init()
            else:
                stream.init(arg_1)
            stream.start()
        elif stream_type == StreamType.HISTOGRAM:
            stream = HistogramStream(self._sensor.CreateHistogramStream(create_arg))
            stream.init()
            stream.start()
        elif stream_type == StreamType.SLAM:
            if create_arg == BaseStream.DEFAULT_CHANNEL_ID:
                stream = SlamStream(self._sensor.CreateSlamStream())
            else:
                stream = SlamStream(self._sensor.CreateSlamStream(create_arg))
            if arg_1 is None:
                stream.init()
            else:
                stream.init(arg_1)
            stream.start()
        elif stream_type == StreamType.TRACKING:
            if create_arg == BaseStream.DEFAULT_CHANNEL_ID:
                stream = ImageStream(self._sensor.CreateImageStream(2))
            else:
                stream = ImageStream(self._sensor.CreateImageStream(2))
            if arg_1 is None and arg_2 is None:
                stream.init()
            elif arg_2 is None:
                stream.init(arg_1)
            else:
                stream.init(arg_1, arg_2)
            stream.start()
        elif stream_type == StreamType.USER_DEFINE:
            raise Exception("StreamType.USER_DEFINE doesn't supported yet.")
        elif stream_type == StreamType.CNN_APP:
            if create_arg == BaseStream.DEFAULT_CHANNEL_ID:
                stream = CnnAppStream(self._sensor.CreateCnnAppStream())
            else:
                stream = CnnAppStream(self._sensor.CreateCnnAppStream(create_arg))
            if arg_1 is None:
                stream.init()
            else:
                stream.init(arg_1)
            stream.start()
        elif stream_type == StreamType.POINT_CLOUD:
            if create_arg == BaseStream.DEFAULT_CHANNEL_ID:
                stream = PointCloudStream(self._sensor.CreatePointCloudStream())
            else:
                stream = PointCloudStream(self._sensor.CreatePointCloudStream(create_arg))
            if arg_1 is None and arg_2 is None:
                stream.init()
            elif arg_2 is None:
                stream.init(arg_1)
            else:
                stream.init(arg_1, arg_2)
            stream.start()
        elif stream_type == StreamType.INJECTION:
            stream = InjectionStream(self._sensor.CreateInjectionStream(create_arg), self._sensor)
            stream.init()
            stream.start()
        elif stream_type == StreamType.CNN:
            if create_arg is None:
                stream = CnnStream(self._sensor.CreateCnnStream("Sout_cnn_0"))
            else:
                stream = CnnStream(self._sensor.CreateCnnStream(create_arg))
            stream.init()
            stream.start()
            if arg_1 is not None and arg_2 is not None:
                stream.start_network(arg_1, arg_2)
        elif stream_type == StreamType.TEMPERATURE:
            if create_arg == BaseStream.DEFAULT_CHANNEL_ID:
                stream = TemperatureStream(self._sensor.CreateTemperaturesStream(SensorTemperatureType.ALL))
            else:
                stream = TemperatureStream(self._sensor.CreateTemperaturesStream(create_arg))
            stream.init()
            stream.start()
        if stream is not None:
            if callback_function is not None:
                stream.register = callback_function
        return stream

    def init(self, device_params: DeviceParams = None, cnn_load_params: CnnParams = None) -> HwInfo:
        # @brief    Service initialization.
        #
        # Invoked once before initialization of any other InuDev stream. After invoking Init method the Device is
        #   still in low power consumption. This function will provide a map with all available HW channels on which a
        #   client application can receive streams of frames.
        # @param    deviceParams    Initialized the Device with these input parameters. It will be set to all assembled
        #   cameras.
        # @param    cnn_load_params   CNN network. The service will load the specified CNN network in parallel while
        #   InuSensor is initialized & started to reduce CNN loading time.
        # @return HwInfo
        # class and will receive the Device HW configuration.
        if device_params is None and cnn_load_params is None:
            self._sensor.Init(self._hw_information.info)
        elif device_params is not None and cnn_load_params is None:
            self._sensor.Init(self._hw_information.info, device_params.params)
        elif device_params is None and cnn_load_params is not None:
            self._sensor.Init(self._hw_information.info, cnn_load_params.params)
        else:
            self._sensor.Init(self._hw_information.info, device_params.params, cnn_load_params.params)
        return self._hw_information

    def start(self, hw_information: HwInfo = None):# -> tuple[MapUintPoint, HwInfo]:
        # @brief    Start acquisition of frames.
        #
        # Shall be invoked only after the service is successfully initialized and before any request
        # @param  hw_information	initialized the Device with these ChannelsParams per channel.
        # @return MapUintPoint      Image size that is provided by the Device [Width,Height] according to channelID.
        # @return HwInfo		Return real params with which the server works.
        channels_size = MapUintPoint()
        if hw_information is None:
            hw_information = self._hw_information
        self._sensor.Start(channels_size, self._hw_information.info)
        return channels_size, self._hw_information

    def terminate(self) -> None:
        # @brief    Service Termination.
        #
        # Shall be invoked when the service is no longer in use and after frames acquisition has stopped.
        self.register = None
        self.stop()
        self._sensor.Terminate()

    def stop(self) -> None:
        # @brief    Stop acquisition of frames.
        #
        # Shall be invoked after requests for frames are no longer sent and before service termination
        # (only if start() was invoked).
        self._sensor.Stop()

    def connect(self) -> None:
        # @brief    Try to connect to Inuitive Sensor.
        #
        # Communicate with InuService and try to connect to Inuitive _sensor.
        self._sensor.Connect()

    def disconnect(self, disconnect_usb: bool) -> None:
        # @brief    Try to disconnect from Inuitive Sensor.
        #
        # Stop Communicate to InuService.
        self._sensor.Disconnect(disconnect_usb)

    @property
    def connection_state(self) -> ConnectionState:
        # @brief    Get the connection state of the sensor.
        #
        # @return ConnectionState
        return ConnectionState(self._sensor.ConnectionState)

    @property
    def sensor_state(self) -> SensorState:
        # @brief  Get the Sensor state.
        #
        # @return SensorState
        return SensorState(self._sensor.State)

    def get_sensor_temperature(self, temp_type: TemperatureType) -> float:
        # @brief    Get the Sensor Temperature.
        #
        # @param    type  Temperature sensor type.
        # @return   Temperature    returns the temperature in Celsius .
        return self._sensor.GetSensorTemperature(Sensor.ETemperatureType(temp_type))

    # @register.setter
    def register(self, callback) -> None:
        # @brief    The Callback to one of Sensor.
        #
        _callback = None

        # @brief    Registration for receiving InuSensor state notifications (push).
        #
        # The provided callback function is called only when the Device state is changed.
        # @param callback function which is invoked whenever the sensor state is changed.
        def _callback_cast(sensor: Sensor, connection_state: ConnectionState, error: InuError) -> None:
            # @brief    Prototype of callback function which is used by the Register method.
            #
            # This function is invoked any time a frame is ready, or if an error occurs. The parameters of this function
            # are: Caller stream object, received depth frame and result code.
            _callback(InuSensor(sensor), ConnectionState(connection_state), Error(error))

        self._sensor.Register(_callback_cast)
        _callback = callback

    register = property(None, register)

    def reset(self) -> None:
        # @brief	SW reset of InuSensor, it resets both SW and HW.
        #
        self._sensor.Reset()

    @property
    def version(self) -> MapEntitiesIDVersion:
        # @brief     Get information about the SW and HW components.
        #
        # @return    Version description of each component.
        version = self._sensor.Version
        if len(version) == 0:
            return None
        else:
            return version

    @property
    def control_params(self, sensor_id: int = -1):  # -> MapSensorControlParams:
        # @brief    Get All Sensors Control (AGC) data.
        #
        # It should be called only if after any related stream (e.g. StereoImage, Webcam, etc.) was initiated.
        # An empty map should be provided, in case the map is not empty the service will clear the map.
        # @return MapSensorControlParams   Will return with the sensor's control params per sensor.
        if sensor_id == -1:
            return self._sensor.SensorsControlParams
        else:
            return self._sensor.GetSensorControlParams(sensor_id)

    def set_control_params(self, sensor_id: int, params: SensorControlParams) -> None:
        # @brief    Set Sensor Control (AGC) data.
        #
        # It should be called only if after any related stream (e.g. StereoImage, Webcam, etc.) was initiated.
        # Sets Device control params for specific sensor.
        # @param    sensorId        The sensor id.
        # @param    params          New Sensor Control parameters.
        self._sensor.SetSensorControlParams(sensor_id, params)

    def get_auto_exposure_params(self, sensor_id: int,
                                 projector_type: ProjectorType = ProjectorType.PATTERNS) -> AutoExposureParams:
        # @brief    Get Sensor Control Auto Exposure data.
        #
        # It should be called only if after any related stream (e.g. StereoImage, Webcam, etc.) was initiated.
        # Gets Auto Exposure params for specific sensor.
        # @param sensorId           The sensor id.
        # @param projectorType      Specify the requested projector type by default - ePatterns.
        # @return AutoExposureParams    Auto Exposure parameters.
        params = AutoExposureParams()
        self._sensor.GetAutoExposureParams(params.params, sensor_id, Sensor.EProjectors(projector_type))
        return params

    def set_auto_exposure_params(self, auto_exposure_params: AutoExposureParams, sensor_id: int,
                                 projector_type: ProjectorType = ProjectorType.PATTERNS) -> None:
        # @brief    Set Sensor Control Auto Exposure configuration.
        #
        # It should be called only if after any related stream (e.g. StereoImage, Webcam, etc.) was initiated.
        # Sets Auto Exposure params for specific sensor.
        # @param autoExposureParams         New Auto Exposure parameters.
        # @param sensorId                   The sensor id.
        # @param ProjectorType              Specify the requested projector type by default - ePatterns.
        self._sensor.SetAutoExposureParams(auto_exposure_params.params, sensor_id,
                                           Sensor.EProjectors(projector_type))

    def load_registers_configuration_file(self, file_name: str) -> None:
        # @brief    Load Registers from input file.
        #
        # It should be called only if after the Device was initiated.
        # The file format should be provided by Inuitive technical staff.
        # @param    fileName    Input file name provided by Inuitive.
        return self._sensor.LoadRegistersConfigurationFile(file_name)

    def get_calibration_data(self, channel_id: int, temperature: int = -1, ) -> CalibrationData:
        # @brief		Get Calibration data information.
        #
        # @param    temperature         Optical data of which temperature, in case of default the optical data of
        #   active calibration is returned.
        # @param    channelID           Channel ID. @return   InuError  If
        # InuSensor isn't initialized then StateError is returned. @return   CalibrationData     Output optical data
        # information.
        data = CalibrationData()
        if temperature == -1:
            self._sensor.GetCalibrationData(data.data, channel_id)
        else:
            self._sensor.GetCalibrationData(data.data, channel_id, temperature)
        return data

    def set_projector_level(self, level: ProjectorLevel, projector_id: ProjectorType) -> None:
        # @brief		Set one of the assembled projectors' state
        #
        # @param  level         High - high power, Low low power, Off - projector off.
        # @param  projectorID   Projector name, eNumOfProjectors is illegal value.
        # @return   InuError    Error code, OK if operation successfully completed.
        self._sensor.SetProjectorLevel(Sensor.EProjectorLevel(level), Sensor.EProjectors(projector_id))

    def get_projector_level(self, projector_id: ProjectorType) -> ProjectorLevel:
        # @brief		Get one of the assembled projectors' state.
        #
        # @param   EProjectors          Projector name, eNumOfProjectors is illegal value.
        # @return  EProjectorLevel      High - high power, Low low power, Off - projector off.
        return self._sensor.GetProjectorLevel(Sensor.EProjectors(projector_id))

    def record(self, destination_directory: str, templateName: str = '') -> None:
        # @brief Record Device streams
        #
        # @param  destinationDirectory  Destination directory for recording output. Send empty string to stop recording.
        # @param  templateName          String which will be concatenate to output file name.
        self._sensor.Record(destination_directory, templateName)

    def snapshot(self, destination_directory: str, templateName: str = '') -> None:
        # @brief Record Device streams
        #
        # @param[in]    destinationDirectory  Destination directory for recording output. Send empty string to stop
        #       recording.
        # @param[in]    templateName          String which will be concatenated to output file name.
        self._sensor.Snapshot(destination_directory, templateName)

    def load_cnn_networks(self, load_params: CnnParams) -> None:
        # @brief    Load network from input file name.
        #
        # @param    loadParams    Loaded network parameters.
        self._sensor.LoadCnnNetworks(load_params.params)

    def release_cnn_networks(self) -> None:
        # @brief   Release all previously loaded CNN networks.
        #
        return self._sensor.ReleaseCnnNetworks()

    @property
    def device_time(self) -> int:
        # @brief    GetDeviceTime
        #
        # Should be called only after the Device had initiated. @return  The value generally represents the number of
        #   seconds since 00:00 hours, Jan 1, 1970 UTC (i.e., the current unix timestamp).
        return self._sensor.DeviceTime

    def set_channel_cropping(self, channel_id: int, crop_params: CropParams) -> None:
        # @brief    SetChannelCropping
        #
        # Set channel as "croppable" & define the cropping rectangle size and position.
        # The position of the rectangle can be moved in runtime by calling the stream's API SetCroppingROI
        # @param channelId
        # @param CropParams     Defines the size of the cropped rectangle and its position
        self._sensor.SetChannelCropping(channel_id, crop_params.params)

    def set_channel_dimensions(self, channel_id: int, channel_dimensions: ChannelDimensions) -> None:
        # @brief    SetChannelDimensions
        #
        # Enables to change channel dimensions (not all channels supports this operation).
        # Should be called only after the sensor had started.
        # @param    channelID           The id of the output channel on which the scaling operation will affect.
        # @param    channelDimensions   The dimension of the actual data inside the output image after scaling.
        self._sensor.SetChannelDimensions(channel_id, channel_dimensions.params)

    def set_sensor_histograms_roi(self, histograms_roi: VectorROIParams, sensor_id: int) -> None:
        # @brief    Set Sensor Histograms ROI (AGC) data.
        #
        # It should be called only after the Device was started.
        # Sets the sensor's histogram's ROIs.
        # Each sensor has 3 histograms the first histogram in the vector will be used for Automatic Sensor Control
        # At least one ROI should be provided
        # @param histogramsROI      Vector of histograms.
        # @param sensor_id           The sensor id.
        self._sensor.SetSensorHistogramsROI(histograms_roi, sensor_id)

    @property
    def alternate_projector_mode(self) -> AlternateProjectorMode:
        # @brief    AlternateProjectorMode getter
        #
        # Get specific API for projector toggle
        # @return AlternateProjectorMode
        return AlternateProjectorMode(self._sensor.AlternateProjectorMode)

    @alternate_projector_mode.setter
    def alternate_projector_mode(self, value: AlternateProjectorMode) -> None:
        # @brief    AlternateProjectorMode setter
        #
        # Specific API for projector toggle Should be called after the sensor had started, and before the IR/DEPTH
        # channel is started after the sensor had started. value	AlternateProjectorMode
        self._sensor.AlternateProjectorMode = value

    def calibrate_imu(self, csv_path: str = '', yml_path: str = '') -> None:
        # @brief    CalibrateImu
        #
        # Runs the process of IMU calibration based on recorded IMU data and updates configuration accordingly @param
        # csvPath	    Path for the file with the recorded IMU data. If not provided the file will be searched and
        # taken from the last recording directory. @param ymlPath	    Path for the file with the IMU calibration. if
        # not provided the file will be taken from the connected sensor The calculated bias values will be saved to
        # the provided YML if provided, if not it will be saved to the calibration directory
        self._sensor.CalibrateImu(csv_path, yml_path)

    def start_node(self, node_name: str) -> None:
        # @brief    StartNode
        #
        # Starts a HW Node
        # @param                node_name
        self._sensor.StartNode(node_name)

    def stop_node(self, node_name: str) -> None:
        # @brief    StopNode
        #
        # Stops a HW Node
        # @param node_name           Node name
        # @return CInuError    Error code, OK if operation successfully completed.
        self._sensor.StopNode(node_name)

    def set_inject_resolution(self, streamer_name: str, channel_size: Point2Dim) -> None:
        # @brief    SetInjectResolution
        #
        # @Should be called only after the sensor had initialized and before it was started.
        # @param streamerName The Streamer Name on which the write operation will affect.
        # @param channelSize The size of the actual data inside the input image after writing.
        # @return InuError    Error code, EErrorCode.OK if operation successfully completed.
        self._sensor.SetInjectResolution(streamer_name, channel_size.point)


def create_sensor(service_id: str = '', ip_address: str = '') -> InuSensor:
    return InuSensor(service_id)


def create_started_sensor(callback=None, device_params: DeviceParams = None, cnn_load_params: CnnParams = None,
                          service_id: str = '', ip_address: str = ''): # -> tuple[InuSensor, MapUintPoint, HwInfo]:
    sensor = InuSensor(service_id)
    sensor.init(device_params, cnn_load_params)
    channels_size, hw_information = sensor.start()
    sensor.register = callback
    return sensor, channels_size, hw_information
