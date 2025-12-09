from .depth_stream import DepthStream
from .image_stream import ImageStream
from .image_frame import ImageFrame
from .stereo import StereoStream, StereoFrame
from .InuStreamsPyth import SuperF, InuError, FramesSyncStrategy
from enum import IntEnum
from typing import Union


class SuperFramesSyncStrategy(IntEnum):
    """!
        Represents the various SuperFrame strategies.
    """
    AUTO = FramesSyncStrategy.Auto
    FAME_INDEX = FramesSyncStrategy.FrameIndex
    TIMESTAMP = FramesSyncStrategy.Timestamp


class SuperFramesStrategy:

    def __init__(self):
        self.strategy = FramesSyncStrategy()
        """! The SuperFramesStrategy class initializer.
        """
    strategy: FramesSyncStrategy = None

    @property
    def strategy_type(self) -> SuperFramesSyncStrategy:
        """! The Strategy Type
        """
        return SuperFramesSyncStrategy(self.strategy.StrategyType)

    @strategy_type.setter
    def strategy_type(self, value: SuperFramesSyncStrategy) -> None:
        """! strategy_type setter
        """
        self.strategy.StrategyType = FramesSyncStrategy.EFramesSyncStrategy(value)

    @property
    def max_threshold(self) -> int:
        """! The MaxThreshold
        """
        return self.strategy.MaxThreshold

    @max_threshold.setter
    def max_threshold(self, value: int) -> None:
        """! strategy_type setter
        """
        self.strategy.MaxThreshold = value


class SuperFrame:
    """!
        Represents the Super Frame.
    """

    frame: SuperF = None

    def __init__(self, frame: SuperF):
        self.frame = frame
        """! 
            The Supper Frame class initializer.
            @param frame  The SuperF  from InuStreamsPyth.
            @return  An instance of the Image initialized with the specified InuStreamsPyth.SuperF  object.
        """

    def get_frame(self, stream: Union[StereoStream, DepthStream, ImageStream]) -> Union[ImageFrame, StereoFrame]:
        if isinstance(stream, StereoStream):
            return StereoFrame(self.frame.GetStereoFrame(stream.stream))
        elif isinstance(stream, DepthStream):
            return ImageFrame(self.frame.GetDepthFrame(stream.stream))
        elif isinstance(stream, ImageStream):
            return ImageFrame(self.frame.GetImageFrame(stream.stream))
        else:
            return None


