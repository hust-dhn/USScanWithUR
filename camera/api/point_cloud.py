from .base_stream import BaseStream
from .base_frame import BaseFrame
from .errors import Error
from .InuStreamsPyth import PointCloudF, PointCloudS, InuError, VoxelFilterP, SegmentationP, DepthS
from .depth_properties import DepthProperties
from .depth_stream import DepthPostProcessing
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


class SegmentationParams(VoxelFilterParams):
    """! SegmentationParams class.
        Role: Controls Segmentation filter params .
    """
    # @brief    InuStreamsPyth.VoxelFilterP
    #
    params = None

    def __init__(self, params: SegmentationP = None):
        """! The Segmentation params class initializer.
            @return  An instance of the SegmentationParams object.
        """
        VoxelFilterParams .__init__(self, params)
        if params is None:
            self.params = SegmentationP()
        else:
            self.params = params

    @property
    def calibration_frame_interval(self) -> int:
        # @brief    calibration_frame_interval getter.
        return self.params.CalibrationFrameInterval

    @calibration_frame_interval.setter
    def calibration_frame_interval(self, value: int) -> None:
        # @brief    calibration_frame_interval setter.
        self.params.CalibrationFrameInterval = value

    @property
    def ground_threshold(self) -> int:
        # @brief    ground_threshold getter.
        return self.params.GroundThreshold

    @ground_threshold.setter
    def ground_threshold(self, value: int) -> None:
        # @brief    ground_threshold setter.
        self.params.GroundThreshold = value

    @property
    def add_floor_flag(self) -> bool:
        # @brief    add_floor_flag getter.
        return self.params.AddFloorFlag

    @add_floor_flag.setter
    def add_floor_flag(self, value: bool) -> None:
        # @brief    add_floor_flag setter.
        self.params.AddFloorFlag = value

    @property
    def clustering_min_neighbors(self) -> int:
        # @brief    clustering_min_neighbors getter.
        return self.params.ClusteringMinNeighbors

    @clustering_min_neighbors.setter
    def clustering_min_neighbors(self, value: int) -> None:
        # @brief    clustering_min_neighbors setter.
        self.params.ClusteringMinNeighbors = value

    @property
    def clustering_epsilon(self) -> float:
        # @brief    clustering_epsilon getter.
        return self.params.ClusteringEpsilon

    @clustering_epsilon.setter
    def clustering_epsilon(self, value: int) -> None:
        # @brief    clustering_epsilon setter.
        self.params.ClusteringEpsilon = value

    @property
    def cluster_min_points(self) -> float:
        # @brief    cluster_min_points getter.
        return self.params.ClusterMinPoints

    @cluster_min_points.setter
    def cluster_min_points(self, value: int) -> None:
        # @brief    cluster_min_points setter.
        self.params.ClusterMinPoints = value


class PointCloudStream(BaseStream, DepthProperties):
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
        DepthProperties.__init__(self, stream)
        self._stream = stream

    def init(self, arg1: Union[int, VoxelFilterParams, SegmentationParams, DepthPostProcessing] = None,
             arg2: DepthPostProcessing = None) -> None:
        # @brief    Service initialization.
        #
        # @param  arg1              Registration type With/without registration or VoxelFilterParams.
        # @param  regChannelID      The Register Channel ID - in the case of  RegisteredImage only
        # Hall be invoked once before starting frames acquisition.
        if arg1 is None:
            self._stream.Init()
        elif arg2 is None:
            if type(arg1) is SegmentationParams or type(arg1) is VoxelFilterParams:
                if arg2 is None:
                    self._stream.Init(arg1.params, DepthS.EPostProcessing(DepthPostProcessing.DEFAULT))
                else:
                    self._stream.Init(arg1.params, DepthS.EPostProcessing(arg2))
            elif type(arg1) is DepthPostProcessing:
                self._stream.Init(DepthS.EPostProcessing(arg1))
            else:
                self._stream.Init(arg1)
        else:
            self._stream.Init(arg1, DepthS.EPostProcessing(arg2))

    def register(self, callback) -> None:
        """!
             Registration/De registration for receiving stream frames (push)

             All streams should use the same callback function when calling for “register”.
             You can find an example for a callback function here: “multithread_callback_example.py” where
              “_stream_callback_func“ can receive frames from different types of streams and acts differently based
              on the stream type.

             The provided callback function is called when a new frame is ready (non-blocking).
             It shall be called only after a start() was invoked but before any invocation of a stop() is invoked.
             If you need more than 1 stream you have to give in number of streams only 1 Callback function and after
             checking Stream type inside perform needed process.
             The parameters of this function are:
             @param  callback  The Callback function which is invoked when a new frame is ready.
              Send None to unregister for receiving frames.
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
        if callback is None:
            self._stream.Register(None)
        else:
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

    @property
    def color_channel_id(self) -> int:
        # @brief    cluster_min_points getter.
        return self.params.ColorChannelID
