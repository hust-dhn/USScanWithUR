from .shared import Point2Dim
from .InuStreamsPyth import DisparityP, TemporalFilterP, OutlierRemoveP, HoleFillFilterP, StaticTemporalFilterP, \
    BlobFilterP


class DisparityParams:
    """! DepthProperties class.
        Role: Controls DepthProperties .
    """
    # @brief    InuStreamsPyth.DisparityP
    #
    params = None

    def __init__(self, params: DisparityP = None):
        # @brief     The DepthProperties class initializer.
        #
        # @return  An instance of the DisparityParams object.
        if params is None:
            self.params = DisparityP()
        else:
            self.params = params

    @property
    def confidence_threshold_region1(self) -> int:
        # @brief    All pixels in the Region 1 area with lower confidence value should be set to invalid, should be
        #   used in Host disparity2depth algorithm.
        #
        # @return confidence_threshold_region1 as int
        return self.params.confidence_threshold_region1

    @confidence_threshold_region1.setter
    def confidence_threshold_region1(self, value: int) -> None:
        # @brief    confidence_threshold_region1 setter
        self.params.confidence_threshold_region1 = value

    @property
    def confidence_threshold_region2(self) -> int:
        # @brief    All pixels in the Region 2 area with lower conf value should be set to invalid, should be used
        #     in Host disparity2depth algorithm.  If  ConfidenceStartRegion2 == ConfidenceEndRegion2 == 0 than this
        #     values will not be used in Host disparity2depth algorithm.
        #
        # @return confidence_threshold_region2 as int
        return self.params.confidence_threshold_region2

    @confidence_threshold_region2.setter
    def confidence_threshold_region2(self, value: int) -> None:
        # @brief    confidence_threshold_region2 setter
        self.params.confidence_threshold_region2 = value

    @property
    def max_distance(self) -> int:
        # @brief    Maximum depth in mm, depth above this value will be invalidated.
        #
        # @return max_distance as int
        return self.params.max_distance

    @max_distance.setter
    def max_distance(self, value: int) -> None:
        # @brief    max_distance setter
        self.params.max_distance = value

    @property
    def depth_scale(self) -> int:
        # @brief    Depth scaling factor, 1.0f is to get depth in mm (default).
        #
        # @return depth_scale as int
        return self.params.depth_scale

    @depth_scale.setter
    def depth_scale(self, value: int) -> None:
        # @brief    depth_scale setter
        self.params.depth_scale = value

    @property
    def point_cloud_scale(self) -> int:
        # @brief    Point cloud scaling factor, 0.001f is to get point cloud in meters (default).
        #
        # @return point_cloud_scale as int
        return self.params.point_cloud_scale

    @point_cloud_scale.setter
    def point_cloud_scale(self, value: int) -> None:
        # @brief    point_cloud_scale setter
        self.params.point_cloud_scale = value


class TemporalFilterParams:
    """! TemporalFilterParams class.
        Role: Controls TemporalFilterParams .
    """

    # @brief    InuStreamsPyth.TemporalFilterP
    #
    _params = None

    def __init__(self, params: TemporalFilterP = None):
        # @brief   The TemporalFilterParams class initializer.
        #
        # @return  An instance of the TemporalFilterParams object.
        if params is None:
            self._params = TemporalFilterP()
        else:
            self._params = params

    @property
    def stable_threshold(self) -> int:
        # @brief   Temporal Filter stable threshold .
        #
        # @return stable_threshold as int
        return self._params.stable_threshold

    @stable_threshold.setter
    def stable_threshold(self, value: int) -> None:
        # @brief    stable_threshold setter
        self._params.stable_threshold = value

    @property
    def rapid_threshold(self) -> int:
        # @brief   Temporal Filter rapid threshold .
        #
        # @return rapid_threshold as int
        return self._params.rapid_threshold

    @rapid_threshold.setter
    def rapid_threshold(self, value: int) -> None:
        # @brief    rapid_threshold setter
        self._params.rapid_threshold = value


class OutlierRemoveParams:
    """! OutlierRemoveParams class.
        Role: Controls OutlierRemoveParams .
    """

    # @brief    InuStreamsPyth.OutlierRemoveP
    #
    _params = None

    def __init__(self, params: OutlierRemoveP = None):
        """! The OutlierRemoveParams class initializer.
            @return  An instance of the OutlierRemoveParams object.
        """
        if params is None:
            self._params = OutlierRemoveP()
        else:
            self._params = params

    @property
    def max_percent(self) -> int:
        # @brief   OutlierRemove Filter max percent.
        #
        # @return max_percent as int
        return self._params.max_percent

    @max_percent.setter
    def max_percent(self, value: int) -> None:
        # @brief    max_percent setter
        self._params.max_percent = value

    @property
    def min_dist(self) -> int:
        # @brief   Filter min dist.
        #
        # @return min_dist as int
        return self._params.min_dist

    @min_dist.setter
    def min_dist(self, value: int) -> None:
        # @brief    min_dist setter
        self._params.min_dist = value


class HoleFillFilterParams:
    """! HoleFillFilterParams class.
        Role: Controls HoleFillFilterParams .
    """

    # @brief    InuStreamsPyth.OutlierRemoveP
    #
    _params = None

    def __init__(self, params: OutlierRemoveP = None):
        """! The HoleFillFilterParams class initializer.
            @return  An instance of the HoleFillFilterParams object.
        """
        if params is None:
            self._params = OutlierRemoveP()
        else:
            self._params = params

    @property
    def max_radius(self) -> int:
        # @brief   Hole Fill Max Radius.
        #
        # @return max_radius as int
        return self._params.max_radius

    @max_radius.setter
    def max_radius(self, value: int) -> None:
        # @brief    max_radius setter
        self._params.max_radius = value


class StaticTemporalFilterParams:
    """! StaticTemporalFilterParams class.
        Role: Controls StaticTemporalFilterParams .
    """

    # @brief    InuStreamsPyth.StaticTemporalFilterP
    #
    _params = None

    def __init__(self, params: StaticTemporalFilterP = None):
        """! The StaticTemporalFilterParams class initializer.
            @return  An instance of the StaticTemporalFilterParams object.
        """
        if params is None:
            self._params = StaticTemporalFilterP()
        else:
            self._params = params

    @property
    def filter_length(self) -> int:
        # @brief   Static Temporal Filter BAll.
        #
        # @return filter_length as int.
        return self._params.filter_length

    @filter_length.setter
    def filter_length(self, value: int) -> None:
        # @brief    filter_length setter.
        self._params.filter_length = value

    @property
    def ball(self) -> int:
        # @brief   Static Temporal Filter Length.
        #
        # @return ball as int.
        return self._params.ball

    @ball.setter
    def filter_length(self, value: int) -> None:
        # @brief    ball setter.
        self._params.ball = value

    @property
    def thread_num(self) -> int:
        # @brief  Static Temporal Filter Num Thread.
        #
        # @return thread_num as int.
        return self.thread_num

    @thread_num.setter
    def thread_num(self, value: int) -> None:
        # @brief    thread_num setter.
        self.thread_num = value


class BlobFilterParams:
    """! BlobFilterParams class.
        Role: Controls BlobFilterParams .
    """

    # @brief    InuStreamsPyth.BlobFilterP
    #
    _params = None

    def __init__(self, params: BlobFilterP = None):
        """! The BlobFilterParams class initializer.
            @return  An instance of the BlobFilterParams object.
        """
        if params is None:
            self._params = BlobFilterP()
        else:
            self._params = params

    @property
    def blob_mode(self) -> int:
        # @ Blob algorithm mode : 2 - Blob1, 3 - Blob2, 4 - Blob 1 + 2
        # @ Blob mode 2 –remove small patches
        # @ Blob mode 3 – fill small patches
        # @ Blob mode 4 – both mode 2 and mode 3
        # @return blob_mode as int.
        return self._params.mode

    @blob_mode.setter
    def blob_mode(self, value: int) -> None:
        # @brief    blob_mode setter.
        self._params.mode = value

    @property
    def blob_max_size(self) -> int:
        # @ Maximum number of pixels which be considered as Blob.
        # @ Less than this number will be a candidate to filtering, larger than this value will be not a candidate
        #   to be blob for filtering.
        return self._params.max_size

    @blob_max_size.setter
    def blob_max_size(self, value: int) -> None:
        # @brief    blob_max_size setter.
        self._params.max_size = value

    @property
    def blob_max_height(self) -> int:
        # @ The maximum number of rows which be considered as Blob.
        # @ Less than this number will be a candidate to filtering, larger than this value will be not a candidate
        #   to be blob for filtering.
        return self._params.max_height

    @blob_max_height.setter
    def blob_max_height(self, value: int) -> None:
        # @brief    blob_max_height setter.
        self._params.max_height = value

    @property
    def blob_disparity_threshold(self) -> int:
        # @ Blob algorithm : disparity threshold, will be executed as pre - process
        return self._params.disparity_threshold

    @blob_disparity_threshold.setter
    def blob_disparity_threshold(self, value: int) -> None:
        # @brief    blob_disparity_threshold setter.
        self._params.disparity_threshold = value

    @property
    def disp_diff_same_blob(self) -> float:
        # @This parameter represents the max difference for pixels disparity to be considered as the same Blob.
        # @As this parameter is bigger more pixels will labeled as the same blob.
        return self._params.disp_diff_same_blob

    @disp_diff_same_blob.setter
    def disp_diff_same_blob(self, value: float) -> None:
        # @brief    blob_max_height setter.
        self._params.disp_diff_same_blob = value


class DepthProperties:
    """! Interface for DepthProperties.

    Role: Controls DepthProperties .

    Responsibilities:
    """

    # @brief    InuStreamsPyth.DepthStream or InuStreamsPyth.ImageStream.
    #
    _depth_properties = None

    def __init__(self, stream):
        self._depth_properties = stream
        """! The DepthProperties class initializer.
            @param stream  The InuStreamsPyth.DepthStream or InuStreamsPyth.ImageStream.
            @return  An instance of the DepthProperties initialized with the specified stream object.
        """

    @property
    def default_post_processing_depth_mode(self) -> int:
        # @brief default_post_processing_depth_mode getter
        #
        # @Detailed description:        Get current DefaultPostProcessingMode
        # @return                       The DefaultPostProcessingMode value
        return self._depth_properties.DefaultPostProcessingMode

    @property
    def post_processing_depth_mode(self) -> int:
        # @brief disparity_params getter
        #
        # @Detailed description:        Get current PostProcessingDepthMode
        # @return                       The PostProcessingDepthMode value
        return self._depth_properties.PostProcessingDepthMode

    @property
    def disparity_params(self) -> DisparityParams:
        # @brief disparity_params getter
        #
        # @Detailed description:        Get current DisparityParams
        # @return                       The Disparity Params
        return DisparityParams(self._depth_properties.DisparityParams)

    @disparity_params.setter
    def disparity_params(self, value: DisparityParams) -> None:
        # @brief    disparity_params setter
        #
        # @Detailed description:    Send the DisparityParams to temporal filter.
        #                           By default, is set from InuServiceParams.xml
        # @param[in] value	        New DisparityParams
        self._depth_properties.DisparityParams = value.params

    @property
    def temporal_filter_params(self) -> TemporalFilterParams:
        # @brief temporal_filter_params getter
        #
        # @Detailed description:        Get current TemporalFilterParams.
        # @return                       The current Temporal Filter Params
        return TemporalFilterParams(self._depth_properties.TemporalFilterParams)

    @temporal_filter_params.setter
    def temporal_filter_params(self, value: TemporalFilterParams) -> None:
        # @brief    temporal_filter_params setter
        #
        # @Detailed description:    Send the TemporalFilterParams to temporal filter.
        #                           By default, it is set from InuServiceParams.xml
        # @param[in]   value	    New DisparityParams
        self._depth_properties.TemporalFilterParams.params = value

    @property
    def outlier_remove_params(self) -> OutlierRemoveParams:
        # @brief outlier_remove_params getter
        #
        # @Detailed description:        Get current OutlierRemoveParams.
        # @return                       The current OutlierRemoveParams
        return OutlierRemoveParams(self._depth_properties.OutlierRemoveParams)

    @outlier_remove_params.setter
    def outlier_remove_params(self, value: OutlierRemoveParams) -> None:
        # @brief    outlier_remove_params setter
        #
        # @Detailed description:    Send the OutlierRemoveParams to temporal filter.
        #                           By default, it is set from InuServiceParams.xml
        # @param[in]   value	    New OutlierRemoveParams
        self._depth_properties.OutlierRemoveParams.params = value

    @property
    def hole_fill_params(self) -> HoleFillFilterParams:
        # @brief hole_fill_params getter
        #
        # @Detailed description:        Get current HoleFillParams.
        # @return                       The current HoleFillParams
        return HoleFillFilterParams(self._depth_properties.HoleFillParams)

    @hole_fill_params.setter
    def hole_fill_params(self, value: HoleFillFilterParams) -> None:
        # @brief    hole_fill_params setter
        #
        # @Detailed description:    Send the HoleFillParams to temporal filter.
        #                           By default, it is set from InuServiceParams.xml
        # @param[in]   value	    New HoleFillParams
        self._depth_properties.HoleFillParams.params = value

    @property
    def static_temporal_filter_params(self) -> StaticTemporalFilterParams:
        # @brief static_temporal_filter_params getter
        #
        # @Detailed description:        Get current StaticTemporalFilterParams.
        # @return                       The current StaticTemporalFilterParams
        return StaticTemporalFilterParams(self._depth_properties.StaticTemporalFilterParams)

    @static_temporal_filter_params.setter
    def static_temporal_filter_params(self, value: StaticTemporalFilterParams) -> None:
        # @brief    static_temporal_filter_params setter
        #
        # @Detailed description:    Send the StaticTemporalFilterParams to temporal filter.
        #                           By default, it is set from InuServiceParams.xml
        # @param[in]   value	    New StaticTemporalFilterParams
        self._depth_properties.StaticTemporalFilterParams.params = value

    @property
    def blob_filter_params(self) -> BlobFilterParams:
        # @brief blob_filter_params getter
        #
        # @Detailed description:        Get current BlobFilterParams.
        # @return                       The current BlobFilterParams
        return BlobFilterParams(self._depth_properties.BlobFilterParams)

    @blob_filter_params.setter
    def blob_filter_params(self, value: BlobFilterParams) -> None:
        # @brief    blob_filter_params setter
        #
        # @Detailed description:    Send the BlobFilterParams to temporal filter.
        #                           By default, it is set from InuServiceParams.xml
        # @param[in]   value	    New BlobFilterParams
        self._depth_properties.BlobFilterParams.params = value


class CroppingROI:
    """! Interface for CroppingROI.

    Role: Controls CroppingROI .

    Responsibilities:
    """

    # @brief    InuStreamsPyth.DepthStream or InuStreamsPyth.ImageStream.
    #
    _cropping_roi = None

    def __init__(self, stream):
        self.cropping = stream
        """! The DepthProperties class initializer.
            @param stream  The InuStreamsPyth.DepthStream or InuStreamsPyth.ImageStream.
            @return  An instance of the DepthProperties initialized with the specified stream object.
        """

    def cropping_roi(self, start: Point2Dim) -> None:
        # @brief    Moves the cropping region of interest in runtime
        #
        # Enables to move the region of interest rectangle that was set using the InuSensor::SetChannelCropping
        # before starting the device. In case InuSensor.SetChannelCropping wasn't called the operation will fail.
        # Moving the rectangle outside the bookbinderies of the viewable area will cause the image to freeze. @param
        # start:     The position in Point2Dim of the upper left corner of the rectangle
        self.cropping.SetCroppingROI(start.x, start.y)

    _cropping_roi = property(None, cropping_roi)
