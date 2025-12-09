from .InuStreamsPyth import CnnLoadP
from enum import IntEnum


class CnnProcessingFlags(IntEnum):
    """!
        Optional special pre/post processing for the images that are processed by the loaded CNN should be set to
        ProcessingFlags
    """
    NONE = CnnLoadP.CnnNone
    # Performs the yoloV3 post-process on the arc processor within the module. The output is a list of bounding Boxes.
    # The first 4 bytes in the output blob contains number of boxes (int format), and the boxes of type BBOX follows.
    YOLO_V3_POST_PROCESS = CnnLoadP.YoloV3PostProcess
    # Convert 12 bits to RGB 24 bits input  format to the CNN
    CONVERT_12_BITS_TO_888 = CnnLoadP.Convert12BitTo888
    # Convert 12 bits to 8 bits grayscale input  format to the CNN
    CONVERT_12_BITS_TO_8 = CnnLoadP.Convert12BitTo8
    #  Convert RGB 24 bits to 8 bits grayscale input  format to the CNN
    CONVERT_888_BITS_TO_8 = CnnLoadP.Convert888BitsTo8
    FACE_SSD_POST_PROCESS = CnnLoadP.FaceSsdPPosrprocess
    FREE_ON_SWITCH = CnnLoadP.FreeOnOnSwitch
    YOLACT_POST_PROCESS = CnnLoadP.YolachPostprocess
    SWAP_YUV = CnnLoadP.SwapYuv
    YOLO_V7_POST_PROCESS = CnnLoadP.YoloV7Postprocess


class NetworkEngineType(IntEnum):
    """!  CnnNetworkEngineType enum class.
        Defines the types of Cnn network engine.
    """
    CEVA = CnnLoadP.ENetworkEngineType.Ceva
    SYNOPSYS = CnnLoadP.ENetworkEngineType.Synopsys


class ImageRawScale(IntEnum):
    """!  CnnInputImageRawScale enum class.
        Defines Cnn input fraction bits.
    """
    SCALE_1 = CnnLoadP.EInputImageRawScale.RowScale1
    SCALE_256 = CnnLoadP.EInputImageRawScale.RowScale256


class ChannelSwap(IntEnum):
    """!  ChannelSwap enum class.
        Decides whether to swap the channel order of the input image.
    """
    NO_SWAP = CnnLoadP.EChannelSwap.NoSwap
    SWAP = CnnLoadP.EChannelSwap.Swap


class ResultsFormat(IntEnum):
    """!  ResultsFormat enum class.
        Choose network output format between fixed-point 16 bit element to floating-point 32 bit.
    """
    FIXED_POINT_QUERY = CnnLoadP.EResultsFormat.FixedPointQuery
    FLOATING_POINT_QUERY = CnnLoadP.EResultsFormat.FoatingPointQuery


class NetType(IntEnum):
    """! NetType enum class.
        The type of CNN network.
    """
    OTHER_CNN = CnnLoadP.ENetType.OtherCnn
    YOLO_CNN = CnnLoadP.ENetType.YoloCnn


class BitAccuracy(IntEnum):
    """! BitAccuracy enum class.
        Network output format between fixed-point 16 bit element to floating-point 32 bit.
    """
    ACCURACY_8BIT = CnnLoadP.EBitAccuracy.Accuracy8Bit
    ACCURACY_16BIT = CnnLoadP.EBitAccuracy.Accuracy16Bit
    INVALID = CnnLoadP.EBitAccuracy.Invalid


class CnnVersion(IntEnum):
    """! CnnVersion enum class.
    """
    V_2017_0 = CnnLoadP.ECnnVersion.Version2017_0
    V_2017_ED1 = CnnLoadP.ECnnVersion.Version2017_ED1
    V_2017_ED2 = CnnLoadP.ECnnVersion.Version2017_ED2
    V_2018_0 = CnnLoadP.ECnnVersion.Version2018_0
    V_2018_3 = CnnLoadP.ECnnVersion.Version2018_3
    V_CURRENT = CnnLoadP.ECnnVersion.VersionCurrent


class CnnParams:
    params = None

    def __init__(self, params: CnnLoadP = None):
        """!
             The CnnLoadParams class initializer.
             return  An instance of the CnnLoadParams object.
        """
        if params is None:
            self.params = CnnLoadP()
        else:
            self.params = params

    @property
    def network_id(self) -> str:
        """! Getter for	Network unique ID.
        """
        return self.params.NetworkID

    @network_id.setter
    def network_id(self, value: str) -> None:
        """! network_id setter
        """
        self.params.NetworkID = value

    @property
    def network_file_name(self) -> str:
        """! Getter for	cnn network attributes.
        """
        return self.params.NetworkFilename

    @network_file_name.setter
    def network_file_name(self, value: str) -> None:
        """! network_file_name setter
        """
        self.params.NetworkFilename = value

    @property
    def engine_type(self) -> NetworkEngineType:
        """! Getter for Engine Type: ceva or synopsis.
        """
        return NetworkEngineType(self.params.CnnNetworkEngineType)

    @engine_type.setter
    def engine_type(self, value: NetworkEngineType) -> None:
        """! engine_type setter
        """
        self.params.CnnNetworkEngineType = CnnLoadP.ENetworkEngineType(value)

    @property
    def input_image_raw_scale(self) -> ImageRawScale:
        """! Getter for input_image_raw_scale,
        """
        return ImageRawScale(self.params.CnnInputImageRawScale)

    @input_image_raw_scale.setter
    def input_image_raw_scale(self, value: ImageRawScale) -> None:
        """! Setter for input_image_raw_scale,
        """
        self.params.CnnInputImageRawScale = CnnLoadP.EInputImageRawScale(value)

    @property
    def engine_type(self) -> NetworkEngineType:
        """! input_image_raw_scale getter
        """
        return NetworkEngineType(self.params.CnnNetworkEngineType)

    @engine_type.setter
    def engine_type(self, value: NetworkEngineType) -> None:
        """! input_image_raw_scale setter
        """
        self.params.CnnNetworkEngineType = CnnLoadP.ENetworkEngineType(value)

    @property
    def channel_swap(self) -> ChannelSwap:
        """! Getter for channel_swap.
        """
        return ChannelSwap(self.params.CnnChannelSwap)

    @channel_swap.setter
    def channel_swap(self, value: ChannelSwap) -> None:
        """! channel_swap setter
        """
        self.params.CnnChannelSwap = CnnLoadP.EChannelSwap(value)

    @property
    def results_format(self) -> ResultsFormat:
        """! Getter for results_format.
        """
        return ResultsFormat(self.params.CnnResultsFormat)

    @results_format.setter
    def results_format(self, value: ResultsFormat) -> None:
        """! results_format setter
        """
        self.params.CnnResultsFormat = CnnLoadP.EResultsFormat(value)

    @property
    def net_type(self) -> NetType:
        """! Getter for net_type.
        """
        return NetType(self.params.CnnNetType)

    @results_format.setter
    def net_type(self, value: NetType) -> None:
        """! net_type setter
        """
        self.params.CnnNetType = CnnLoadP.ENetType(value)

    @property
    def bit_accuracy(self) -> BitAccuracy:
        """! Getter for bit_accuracy.
        """
        return BitAccuracy(self.params.BitAccuracy)

    @bit_accuracy.setter
    def bit_accuracy(self, value: BitAccuracy) -> None:
        """! bit_accuracy setter
        """
        self.params.BitAccuracy = CnnLoadP.EBitAccuracy(value)

    @property
    def ddr_workspace_size(self) -> int:
        """! Getter for ddr_workspace_size.
        """
        return self.params.DdrWorkspaceSize

    @ddr_workspace_size.setter
    def ddr_workspace_size(self, value: int) -> None:
        """! ddr_workspace_size setter
        """
        self.params.DdrWorkspaceSize = value

    @property
    def internal_workspace_size(self) -> int:
        """! Getter for internal_workspace_size.
        """
        return self.params.InternalWorkspaceSize

    @internal_workspace_size.setter
    def ddr_workspace_size(self, value: int) -> None:
        """! internal_workspace_size setter
        """
        self.params.InternalWorkspaceSize = value

    @property
    def version(self) -> CnnVersion:
        """! Getter for version.
        """
        return self.params.Version

    @version.setter
    def version(self, value: CnnVersion) -> None:
        """! version setter
        """
        self.params.Version = value

    @property
    def pipe_depth(self) -> int:
        """! Getter for pipe_depth.
        """
        return self.params.PipeDepth

    @pipe_depth.setter
    def pipe_depth(self, value: int) -> None:
        """! pipe_depth setter
        """
        self.params.PipeDepth = value

    @property
    def processing_flags(self) -> int:
        """! Getter for processing_flags.
        """
        return self.params.ProcessingFlags

    @pipe_depth.setter
    def processing_flags(self, value: int) -> None:
        """! processing_flags setter
        """
        self.params.ProcessingFlags = value
