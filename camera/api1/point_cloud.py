from .base_stream import BaseStream
from .base_frame import BaseFrame
from .errors import Error
from .InuStreamsPyth import PointCloudF, PointCloudS, InuError, VoxelFilterP
import numpy as np
from typing import Union
from enum import IntEnum


class PointCloudOutputFormat(IntEnum):
    """!  OutputFormat enum class.
        Represents all possible PointCloud output types.
    """
    DEFAULT = PointCloudF.EFormat.ShortPoints
    FLOAT_POINTS = PointCloudF.EFormat.FloatPoints
    SHORT_POINTS = PointCloudF.EFormat.ShortPoints
    FLOAT_POINTS_RGB = PointCloudF.EFormat.FloatPointsRGB


class PointCloudFrame(BaseFrame):
    """!  PointCloud frame.
    Role: Represents  an PointCloud Frame that is  provided  by InuDev PointCloud stream

    Responsibilities:
          1. Frame attributes: Format, Confidence,  NumOfPoints  and number of bytes per pixel.
          2. Knows how to manage the  buffer.
    """

    # @brief    InuStreamsPyth.PointCloudF.
    #
    point_cloud_frame = None

    def __init__(self, frame: PointCloudF):
        self.point_cloud_frame = frame
        BaseFrame.__init__(self, frame)
        """! The PointCloudFrame Frame class initializer.
            @param frame  The PointCloudF  from InuStreamsPyth.
            @return  An instance of the ImuFr initialized with the specified InuStreamsPyth.PointCloudF  object.
        """

    @property
    def num_of_points(self) -> int:
        # @brief    num_of_points getter.
        return self.point_cloud_frame.NumOfPoints

    @property
    def format(self) -> PointCloudOutputFormat:
        # @brief    point-cloud frame format getter.
        return PointCloudOutputFormat(self.point_cloud_frame.Format)

    @property
    def image_size(self) -> int:
        # @brief    number of pixels getter.
        return self.point_cloud_frame.BufferSize

    @property
    def bytes_per_point(self) -> int:
        # @brief    Number of bytes that are used to represent each point.
        return self.point_cloud_frame.BytesPerPoint

    @property
    def bytes_per_format(self) -> int:
        # @brief    Number of bytes that are used to represent each point.
        return self.point_cloud_frame.BytesPerFormat

    @property
    def exposure_params(self):  # std::map<uint16_t, InuDev::CExposureParams>
        # @brief    Exposure parameters while this image was grabbed.
        return self.point_cloud_frame.ExposureParams

    @property
    def data(self) -> np.array:
        if self.point_cloud_frame.Format == PointCloudF.EFormat.FloatPoints:
            return self.point_cloud_frame.FloatData
        elif self.point_cloud_frame.Format == PointCloudF.EFormat.ShortPoints:
            return self.point_cloud_frame.ShortData
        elif self.point_cloud_frame.Format == PointCloudF.EFormat.FloatPointsRGB:
            return self.point_cloud_frame.FloatRGBData
        else:
            return None


class PointCloudRegistrationType(IntEnum):
    """!  PointCloudRegistrationType enum class.
        Represents all possible PointCloud registration types.
    """
    # No registration
    NONE_REGISTRATION = PointCloudS.EPointCloudRegistration.NoneRegistration
    # Register PointCloud to specific channel
    REGISTRATION = PointCloudS.EPointCloudRegistration.Registration


class VoxelFilterParams:
    """! VoxelFilterParams class.
        Role: Controls Voxel filter params .
    """
    # @brief    InuStreamsPyth.VoxelFilterP
    #
    params = None

    def __init__(self, params: VoxelFilterP = None):
        """! The VoxelFilterParams class initializer.
            @return  An instance of the VoxelFilterP object.
        """
        if params is None:
            self.params = VoxelFilterP()
        else:
            self.params = params

    @property
    def max_depth(self) -> int:
        # @brief    max_depth getter.
        return self.params.MaxDepth

    @max_depth.setter
    def max_depth(self, value: int) -> None:
        # @brief    max_depth setter.
        self.params.MaxDepth = value

    @property
    def leaf_size_x(self) -> int:
        # @brief    leaf_size_x getter.
        return self.params.VoxelLeafSizeX

    @leaf_size_x.setter
    def leaf_size_x(self, value: int) -> None:
        # @brief    leaf_size_x setter.
        self.params.VoxelLeafSizeX = value

    @property
    def leaf_size_y(self) -> int:
        # @brief    leaf_size_x getter.
        return self.params.VoxelLeafSizeY

    @leaf_size_y.setter
    def leaf_size_y(self, value: int) -> None:
        # @brief    leaf_size_y setter.
        self.params.VoxelLeafSizeY = value

    @property
    def leaf_size_z(self) -> int:
        # @brief    leaf_size_z getter.
        return self.params.VoxelLeafSizeZ

    @leaf_size_z.setter
    def leaf_size_z(self, value: int) -> None:
        # @brief    leaf_size_z setter.
        self.params.VoxelLeafSizeZ = value


class PointCloudStream(BaseStream):
    """! Interface for PointCloud service.

    Role: Controls PointCloud streaming service and provides general or PointCloud frames.
          IMU frames are provided only if the connected device supports PointCloud HW components.
          The caller application should be familiar with provided frames and should know how to interpret them.

    Responsibilities:
          1. Derives BaseStream class
          2. Knows how to acquire one PointCloud image frame (pull)
          3. Knows how to provide a continuous stream of PointCloud frames (push)
    """

    # @brief    InuStreamsPyth.PointCloudS.
    #
    _stream = None

    def __init__(self, stream: PointCloudS):
        """! The Imu stream class initializer.
            @param stream  The InuStreamsPyth.ImuS.
            @return  An instance of the PointCloud stream initialized with the specified
            InuStreamsPyth.PointCloudS object.
        """
        BaseStream.__init__(self, stream)
        self._stream = stream

    def init(self,
             arg1: Union[PointCloudRegistrationType, VoxelFilterParams] = None,
             reg_channel_id: int = BaseStream.DEFAULT_CHANNEL_ID) -> None:
        # @brief    Service initialization.
        #
        # @param  arg1              Registration type With/without registration or VoxelFilterParams.
        # @param  regChannelID      The Register Channel ID - in the case of  RegisteredImage only
        # Hall be invoked once before starting frames acquisition.
        if type(arg1) == PointCloudRegistrationType:
            self._stream.Init(arg1, reg_channel_id)
        elif type(arg1) == VoxelFilterParams:
            self._stream.Init(arg1.params)
        else:
            self._stream.Init(PointCloudS.EPointCloudRegistration.NoneRegistration,
                              BaseStream.DEFAULT_CHANNEL_ID)

    def terminate(self) -> None:
        """!
            Stop frames acquisition, stop ant termination service.
        """
        self.register = None
        self.stop()
        self._stream.Terminate()

    def register(self, callback) -> None:
        """!
            Registration/De registration for receiving stream frames (push)

            The provided callback function is called when a new frame is ready (non-blocking).
            It shall be called only after a start() was invoked but before any invocation of a stop() is invoked.
            @param  callback  The Callback function which is invoked when a new frame is ready. Send nullptr to
                unregister for receiving frames.
        """

        def _callback_cast(stream: PointCloudS, frame: PointCloudF, error: InuError) -> None:
            """!
                Prototype of callback function which is used by the Register method.

                This function is invoked any time a frame is ready, or if an error occurs. The parameters of this
                function are:
                @param stream Caller image stream object.
                @param frame  Caller image frame object.
                @param error  Result code.
            """
            BaseStream.callback(PointCloudStream(stream), PointCloudFrame(frame), Error(error))

        BaseStream.callback = callback
        self._stream.Register(_callback_cast)

    register = property(None, register)

    @property
    def frame(self) -> PointCloudFrame:
        # @brief   Retrieves new frame (pull)
        #
        # This method returns when a new frame is ready (blocking) or if an input timeout has elapsed.
        # It shall be called only after a start() was invoked but before any invocation of a stop() is invoked.
        # @return  The returned stereo frame.
        return PointCloudFrame(self._stream.GetFrame())
