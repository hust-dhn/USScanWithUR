from .base_stream import BaseStream
from .base_frame import BaseFrame
from .errors import Error
from .InuStreamsPyth import TemperatureF, TemperatureS, InuError
from enum import IntEnum


class SensorTemperatureType(IntEnum):
    """!
        Represents the various temperature sensors.
        The values will be used with bitwise operands in CTemperaturesStream::Init function
    """
    SENSOR_1 = TemperatureF.ESensorTemperatureType.Sensor1
    SENSOR_2 = TemperatureF.ESensorTemperatureType.Sensor2
    PVT = TemperatureF.ESensorTemperatureType.PVT
    ALL = TemperatureF.ESensorTemperatureType.All


class TemperatureFrame(BaseFrame):
    """!  Temperature frame.
        Role:  Represents the type of the sensor which CTemperature data refer to.
    """

    # @brief    InuStreamsPyth.TemperatureF.
    #
    temperature_frame = None

    def __init__(self, frame: TemperatureF):
        self.temperature_frame = frame
        BaseFrame.__init__(self, frame)
        """! The Imu Frame class initializer.
            @param frame  The ImuFrame  from InuStreamsPyth.
            @return  An instance of the ImuFr initialized with the specified InuStreamsPyth.ImuF  object.
        """

    @property
    def sensor_1(self) -> float:
        # @brief     Retrieve temperature value from specified temperature sensor.
        #
        # @return temperature value.
        return self.temperature_frame.Temperature(TemperatureF.ESensorTemperatureType(SensorTemperatureType.SENSOR_1))

    @property
    def sensor_2(self) -> float:
        # @brief     Retrieve temperature value from specified temperature sensor.
        #
        # @return temperature value.
        return self.temperature_frame.Temperature(TemperatureF.ESensorTemperatureType(SensorTemperatureType.SENSOR_2))

    @property
    def pvt(self) -> float:
        # @brief     Retrieve temperature value from specified temperature sensor.
        #
        # @return temperature value.
        return self.temperature_frame.Temperature(TemperatureF.ESensorTemperatureType(SensorTemperatureType.PVT))


class TemperatureStream(BaseStream):
    """! Interface for Temperatures Stream.

    Role: Enables to receive temperature from various temperature sensors embeded in the device.
          The caller application should be familiar with provided frames and should know how to interpret them.

    Responsibilities:
          1. Derives BaseStream class
          2. Knows how to acquire one temperature frame (pull)
          3. Knows how to provide a continuous stream of temperatures frames (push)
    """

    # @brief    InuStreamsPyth.TemperaturesF.
    #
    _stream: TemperatureF = None

    def __init__(self, stream: TemperatureS):
        """! The Imu stream class initializer.
            @param stream  The InuStreamsPyth.TemperatureS.
            @return  An instance of the Temperature stream initialized with the specified InuStreamsPyth.TemperatureS
            object.
        """
        BaseStream.__init__(self, stream)
        self._stream = stream

    def terminate(self) -> None:
        """!
            Stop frames acquisition, stop ant termination service.
        """
        self.register = None
        self.stop()
        self._stream.Terminate()

    def init(self) -> None:
        # @brief    Service initialization.
        #
        # Hall be invoked once before starting frames acquisition.
        self._stream.Init()

    def register(self, callback) -> None:
        """!
            Registration/De registration for receiving stream frames (push)

            The provided callback function is called when a new frame is ready (non-blocking).
            It shall be called only after a start() was invoked but before any invocation of a stop() is invoked.
            @param  callback  The Callback function which is invoked when a new frame is ready. Send nullptr to
                unregister for receiving frames.
        """
        def _callback_cast(stream: TemperatureS, frame: TemperatureF, error: InuError) -> None:
            """!
                Prototype of callback function which is used by the Register method.

                This function is invoked any time a frame is ready, or if an error occurs. The parameters of this
                function are:
                @param stream Caller Temperature stream object.
                @param frame  Caller Temperature frame object.
                @param error  Result code.
            """
            print("Temperature _callback_cast")
            BaseStream.callback(TemperatureStream(stream), TemperatureFrame(frame), Error(error))
        BaseStream.callback = callback
        self._stream.Register(_callback_cast)
    register = property(None, register)

    @property
    def frame(self) -> TemperatureFrame:
        # @brief   Retrieves new frame (pull)
        #
        # This method returns when a new frame is ready (blocking) or if an input timeout has elapsed.
        # It shall be called only after a start() was invoked but before any invocation of a stop() is invoked.
        # @return  The returned stereo frame.
        return TemperatureFrame(self._stream.GetFrame())
