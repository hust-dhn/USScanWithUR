from .library_loader import *
from .base_stream import BaseStream
from .base_frame import BaseFrame
from .shared import Point3DimFloat
from .errors import Error
from .InuStreamsPyth import ImuF, ImuS, MapImuTypePoint3D, InuError
from enum import IntEnum


class ImuType(IntEnum):
    """!  Imu Type enum class.
        Represents the type of the sensor which ImuData refer to.
    """
    ACCELEROMETER = ImuF.EImuType.Accelerometer
    GYROSCOPE = ImuF.EImuType.Gyroscope
    MAGNETOMETER = ImuF.EImuType.Magnetometer
    NUM_OF_IMU_TYPES = ImuF.EImuType.NumOfImuTypes


class ImuFrame(BaseFrame):
    """!  Imu frame.

    Role: Represents collection of IMU data that is provided by IMU device.

    """

    # @brief    InuStreamsPyth.ImuF.
    #
    imu_frame: ImuF = None

    def __init__(self, frame: ImuF):
        self.imu_frame = frame
        BaseFrame.__init__(self, frame)
        """! The Imu Frame class initializer.
            @param frame  The ImuFrame  from InuStreamsPyth.
            @return  An instance of the ImuFr initialized with the specified InuStreamsPyth.ImuF  object.
        """

    @property
    def sensors_data(self) -> MapImuTypePoint3D:
        # @brief     Represents collection of IMU data that is provided by IMU device.
        #
        # @return collection of IMU data.
        return self.imu_frame.SensorsData

    def get_sensor_data(self, imu_type: ImuType) -> Point3DimFloat:
        # @brief     Represents collection of IMU data that is provided by IMU device.
        #
        # @param imu_type  type of the sensor which CImuData refer to
        # @return 3D float point
        return Point3DimFloat(self.imu_frame.GetSensorData(imu_type))

    @property
    def sub_index_frame(self) -> int:
        # @brief     Index of the same IMU type, i.e. continuous index of all IMU frames of the same type.
        #
        # @return Index of the same IMU type.
        return self.imu_frame.SubIndexFrame

    @property
    def temperature(self) -> float:
        # @brief     Current temperature of IMU in Celsius deg. std::numeric_limits<float>::max() if temperature is
        #   not available
        #
        # @return Current temperature of IMU in Celsius deg
        return self.imu_frame.Temperature


class ImuStream(BaseStream):
    """! Interface for Imu service.

    Role: Controls IMU streaming service and provides general or IMU frames.
          IMU frames are provided only if the connected device supports IMU HW components.
          The caller application should be familiar with provided frames and should know how to interpret them.

    Responsibilities:
          1. Derives BaseStream class
          2. Knows how to acquire one imu frame (pull)
          3. Knows how to provide a continuous stream of imu frames (push)
    """

    # @brief    InuStreamsPyth.ImuS.
    #
    _stream: ImuS = None

    def __init__(self, stream: ImuS):
        """! The Imu stream class initializer.
            @param stream  The InuStreamsPyth.ImuS.
            @return  An instance of the Imu stream initialized with the specified InuStreamsPyth.ImuS object.
        """
        BaseStream.__init__(self, stream)
        self._stream = stream

    def register(self, callback) -> None:
        """!
            Registration/De registration for receiving stream frames (push)

            The provided callback function is called when a new frame is ready (non-blocking).
            It shall be called only after a start() was invoked but before any invocation of a stop() is invoked.
            @param  callback  The Callback function which is invoked when a new frame is ready. Send nullptr to
                unregister for receiving frames.
        """
        def _callback_cast(stream: ImuS, frame: ImuF, error: InuError) -> None:
            """!
                Prototype of callback function which is used by the Register method.

                This function is invoked any time a frame is ready, or if an error occurs. The parameters of this
                function are:
                @param stream Caller Imu stream object.
                @param frame  Caller Imu frame object.
                @param error  Result code.
            """
            BaseStream.callback(ImuStream(stream), ImuFrame(frame), Error(error))
        BaseStream.callback = callback
        if callback is None:
            self._stream.Register(None)
        else:
            self._stream.Register(_callback_cast)
    register = property(None, register)

    @property
    def frame(self) -> ImuFrame:
        # @brief   Retrieves new frame (pull)
        #
        # This method returns when a new frame is ready (blocking) or if an input timeout has elapsed.
        # It shall be called only after a start() was invoked but before any invocation of a stop() is invoked.
        # @return  The returned stereo frame.
        return ImuFrame(self._stream.GetFrame())

    @property
    def temperature(self) -> float:
        # @brief      Get the IMU sensor temperature
        #
        # @return returns the temperature in Celsius .
        return self._stream.Temperature

    @property
    def imu_params(self) -> MapImuTypePoint3D:
        # @brief  Get IMU params
        #
        # @Detailed description:        IMU params that are currently used
        return self._stream.ImuParams

    @imu_params.setter
    def imu_params(self, value: MapImuTypePoint3D) -> None:
        # @brief    temporal_filter_params setter
        #
        # @Detailed description:    Set new IMU params.
        #
        # @param[in]   value	    new IMU params
        self._stream.ImuParams = value
