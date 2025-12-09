from .depth_properties import DepthProperties
from .depth_stream import DepthPostProcessing
from .image_frame import ImageFrame
from .image_stream import ImageStream, ImageOutputFormat, ImagePostProcessing
from .errors import Error
from .InuStreamsPyth import ImageRegisteredS, ImageF, InuError, ImageS, DepthS

from enum import IntEnum


class ImageStreamRegistered(ImageStream, DepthProperties):
    """! Interface for Image service.

    Role: Controls depth images streaming service and provides image frames.

    Responsibilities:
          1. Derives BaseStream class
          2. Knows how to acquire one depth image frame (pull)
          3. Knows how to provide a continuous stream of depth image frames (push)
    """

    # @brief    InuStreamsPyth.ImageS.
    #
    _stream: ImageRegisteredS = None

    def __init__(self, stream: ImageRegisteredS):
        ImageStream.__init__(self, stream)
        DepthProperties.__init__(self, stream)
        self._stream = stream
        """! The Depth stream class initializer.
            @param stream  The InuStreamsPyth.ImageRegisteredS.
            @return  An instance of the Image stream initialized with the specified InuStreamsPyth.ImageS object.
        """

    def init(self, form: ImageOutputFormat, processing: ImagePostProcessing, depth_pp: DepthPostProcessing) -> None:
        # @brief    Service initialization.
        #
        # Hall be invoked once before starting frames acquisition.
        # @param  format            The Output format that should be invoked.
        # @param  processing        The PostProcessing algorithms that should be invoked.
        # @param  depth_pp          The Depth PostProcessing algorithms that should be invoked.
        self._stream.Init(ImageS.EOutputFormat(form), ImageS.EPostProcessing(processing), DepthS.EPostProcessing(int(depth_pp)))

