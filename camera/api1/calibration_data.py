from .InuStreamsPyth import CalibrationD, CalibrImuData, MapIntCalibrationOneSensor


class CalibrationData:
    """!  Calibration Data.
        All parameters that describes all cameras and peripherals of this device (Inuitive Sensor) and
        the geometrics relations between them. This information is calculated by a calibration process

    """

    # @brief    InuStreamsPyth.CalibrationData.
    #
    data: CalibrationD = None

    def __init__(self, data: CalibrationD = None):
        if data is None:
            self.data = CalibrationD()
        else:
            self.data = data

        """! The CalibrationData class initializer.
            @param frame  The CalibrationD  from InuStreamsPyth.
            @return An instance of the CalibrationData initialized with the specified InuStreamsPyth.CalibrationD object.
        """

    @property
    def version(self) -> str:
        # @brief  Get calibration Version
        #
        # @Detailed description:    Version that is currently used.
        return self.data.Version

    @property
    def sensors(self) -> MapIntCalibrationOneSensor:
        # @brief  Get sensors  calibration
        #
        # @Detailed description:    Calibration data of all sensors, ordered by the unique sensor ID.
        return self.data.Sensors

    @property
    def base_lines(self):  # -> std::map< std::pair<int, int>, double>
        # @brief  Get baselines per sensor.
        #
        # @Detailed description:    Get The  Euclidean Distance between 2 sensors.
        return self.data.Baselines

    @property
    def imu(self)-> CalibrImuData:
        # @brief  Get  IMU calibration parameters.
        #
        # @Detailed description:     IMU calibration parameters.
        return self.data.Imu




















# class IntrinsicData:
#     """!  IntrinsicData class.
#         The Intrinsic parameters of the camera.
#     """
#
#     class CameraModel(IntEnum):
#         """!  ECameraModel enum class.
#             Different camera model types.
#         """
#         # f * tan(theta)
#         F_TAN_THETHA = CalibrIntrinsicData.ECameraModel.FTanThetha
#         # f * theta
#         F_THETHA = CalibrIntrinsicData.ECameraModel.FThetha
#         # Spline
#         SPLINE = CalibrIntrinsicData.ECameraModel.Spline
#         F_THETHA_EXT = CalibrIntrinsicData.ECameraModel.FThethaExt
#         EQUI_DISTANCE = CalibrIntrinsicData.ECameraModel.FThethaExt
#
#     @property
#     def valid(self) -> bool:
#         # @brief     True if the intrinsic parameters are valid.
#         #
#         # @return True if the intrinsic parameters are valida.
#         return self.data.Valid

#     @property
#     def sensors_data(self) -> MapImuTypePoint3D:
#         # @brief     Represents collection of IMU data that is provided by IMU device.
#         #
#         # @return collection of IMU data.
#         return self.imu_frame.SensorsData
#
#     def get_sensor_data(self, imu_type: Type) -> Point3DFloat:
#         # @brief     Represents collection of IMU data that is provided by IMU device.
#         #
#         # @param imu_type  type of the sensor which CImuData refer to
#         # @return 3D float point
#         return self.imu_frame.GetSensorData(imu_type)
#
#     @property
#     def sub_index_frame(self) -> int:
#         # @brief     Index of the same IMU type, i.e. continuous index of all IMU frames of the same type.
#         #
#         # @return Index of the same IMU type.
#         return self.imu_frame.SubIndexFrame
#
#     @property
#     def temperature(self) -> float:
#         # @brief     Current temperature of IMU in Celsius deg. std::numeric_limits<float>::max() if temperature is
#         #   not available
#         #
#         # @return Current temperature of IMU in Celsius deg
#         return self.imu_frame.Temperature
#
#
# class ImuStream(BaseStream):
#     """! Interface for Imu service.
#
#     Role: Controls IMU streaming service and provides general or IMU frames.
#           IMU frames are provided only if the connected device supports IMU HW components.
#           The caller application should be familiar with provided frames and should know how to interpret them.
#
#     Responsibilities:
#           1. Derives BaseStream class
#           2. Knows how to acquire one imu image frame (pull)
#           3. Knows how to provide a continuous stream of imu frames (push)
#     """
#
#     # @brief    InuStreamsPyth.ImuS.
#     #
#     _stream = None
#
#     def __init__(self, stream: ImuS):
#         """! The Imu stream class initializer.
#             @param stream  The InuStreamsPyth.ImuS.
#             @return  An instance of the Imu stream initialized with the specified InuStreamsPyth.ImuS object.
#         """
#         BaseStream.__init__(self, stream)
#         self._stream = stream
#
#     def init(self) -> None:
#         # @brief    Service initialization.
#         #
#         # Hall be invoked once before starting frames acquisition.
#         self._stream.Init()
#
#     def register(self, callback) -> None:
#         """!
#             Registration/De registration for receiving stream frames (push)
#
#             The provided callback function is called when a new frame is ready (non-blocking).
#             It shall be called only after a start() was invoked but before any invocation of a stop() is invoked.
#             @param  callback  The Callback function which is invoked when a new frame is ready. Send nullptr to
#                 unregister for receiving frames.
#         """
#         def _callback_cast(stream: ImuS, frame: ImuF, error: InuError) -> None:
#             """!
#                 Prototype of callback function which is used by the Register method.
#
#                 This function is invoked any time a frame is ready, or if an error occurs. The parameters of this
#                 function are:
#                 @param stream Caller image stream object.
#                 @param frame  Caller image frame object.
#                 @param error  Result code.
#             """
#             BaseStream._callback(ImuStream(stream), ImuFrame(frame), error)
#         BaseStream._callback = callback
#         self._stream.Register(_callback_cast)
#     register = property(None, register)
#
#     @property
#     def frame(self) -> ImuFrame:
#         # @brief   Retrieves new frame (pull)
#         #
#         # This method returns when a new frame is ready (blocking) or if an input timeout has elapsed.
#         # It shall be called only after a start() was invoked but before any invocation of a stop() is invoked.
#         # @return  The returned stereo frame.
#         return ImuFrame(self.stream.GetFrame())
#
#     @property
#     def temperature(self) -> float:
#         # @brief      Get the IMU sensor temperature
#         #
#         # @return returns the temperature in Celsius .
#         return self.stream.Temperature
#
#     @property
#     def imu_params(self) -> MapImuTypePoint3D:
#         # @brief  Get IMU params
#         #
#         # @Detailed description:        IMU params that are currently used
#         return self.stream.ImuParams
#
#     @imu_params.setter
#     def imu_params(self, value: MapImuTypePoint3D) -> None:
#         # @brief    temporal_filter_params setter
#         #
#         # @Detailed description:    Set new IMU params.
#         #
#         # @param[in]   value	    new IMU params
#         self.ImuParams.imu_params = value
