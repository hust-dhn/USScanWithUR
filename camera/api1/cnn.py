from .base_stream import BaseStream
from .base_frame import BaseFrame
from .cnn_defs import NetworkEngineType
from .shared import InterliveModeFw, ImageFormatFw
from .errors import Error
from .InuStreamsPyth import CnnF, CnnS, CnnTailHdr, CevaF, SynopsysF, CnnFl, CnnFrameH, CnnImageDescr, InuError
import numpy as np


class CevaFields:
    """!   CevaFields.
         Fields relevant for ceva
    """

    # @brief InuStreamsPyth.CevaF.
    #
    ceva_fields = None
    """!   InuStreamsPyth.CevaF..
    """

    def __init__(self, ceva_fields: CevaF = None):
        """! The CevaFields class initializer.
            @param ceva_fields  The CevaF from InuStreamsPyth.
            @return  An instance of the CevaFields object.
        """
        if ceva_fields is None:
            self.ceva_fields = CevaF()
        else:
            self.ceva_fields = ceva_fields

    @property
    def frac_bits(self) -> int:
        """! In case of using fixed point: the scale factor in which the fixed point data is
            represented. Output x(float) is represented as x(float) = scale_factor * x(fixed)
        """
        return self.ceva_fields.FracBits

    @frac_bits.setter
    def frac_bits(self, value: int) -> None:
        """!   frac_bits setter
        """
        # @brief    frac_bits setter
        self.ceva_fields.FracBits = value


class SynopsysFields:
    """!   SynopsysFields.
         Fields relevant for Synopsys
    """

    synopsys_fields = None
    """!    InuStreamsPyth.SynopsysF.
    """

    def __init__(self, synopsys_fields: SynopsysF = None):
        """! The SynopsysFields class initializer.
            @param synopsys_fields as InuStreamsPyth.SynopsysF .
            @return  An instance of the SynopsysFields object.
        """
        if synopsys_fields is None:
            self.synopsys_fields = SynopsysF()
        else:
            self.synopsys_fields = synopsys_fields

    @property
    def scale(self) -> float:
        """! Fixed point values should be divided by this scale.
        """
        return self.synopsys_fields.Scale

    @scale.setter
    def scale(self, value: float) -> None:
        """! scale  setter.
        """
        self.synopsys_fields.scale = value

    @property
    def alloc_count(self) -> int:
        """! Relevant for Bboxes output - Number of allocated bboxes.
        """
        return self.synopsys_fields.AllocCount

    @alloc_count.setter
    def alloc_count(self, value: int) -> None:
        """!  alloc_count setter
        """
        self.synopsys_fields.AllocCount = value

    @property
    def valid_count(self) -> int:
        """! Relevant for Bboxes output - Number of valid bboxes.
        """
        return self.synopsys_fields.ValidCount

    @valid_count.setter
    def valid_count(self, value: int) -> None:
        """!  valid_count setter
        """
        self.synopsys_fields.ValidCount = value

    @property
    def bbox_scale(self) -> int:
        """! Relevant for Bboxes output - Use short with scale instead of float.
        """
        return self.synopsys_fields.BboxScale

    @bbox_scale.setter
    def bbox_scale(self, value: int) -> None:
        """!  bbox_scale setter
        """
        self.synopsys_fields.BboxScale = value

    @property
    def confidence_scale(self) -> int:
        """! Relevant for Bboxes output - Use short with scale instead of float.
        """
        return self.synopsys_fields.ConfidenceScale

    @confidence_scale.setter
    def confidence_scale(self, value: int) -> None:
        """!  confidence_scale setter
        """
        self.synopsys_fields.ConfidenceScale = value


class CnnFields:
    """!   CnnFields.
         Cnn Fields  structure
    """

    cnn_fields = None
    """!    InuStreamsPyth.CnnFl.
    """

    def __init__(self, cnn_fields: CnnFl = None):
        """! The CnnFields class initializer.
            @param cnn_fields as InuStreamsPyth.CnnFl .
            @return  An instance of the CnnFields object.
        """
        if cnn_fields is None:
            self.cnn_fields = CnnF()
        else:
            self.cnn_fields = cnn_fields

    @property
    def ceva_fields(self) -> CevaFields:
        """! Fields relevant for ceva.
        """
        return self.cnn_fields.CevaFields

    @ceva_fields.setter
    def ceva_fields(self, value: CevaFields) -> None:
        """!  ceva_fields setter
        """
        self.cnn_fields.CevaFields = value

    @property
    def synopsys_fields(self) -> SynopsysFields:
        """! Fields relevant for Synopsys.
        """
        return self.cnn_fields.SynopsysFields

    @synopsys_fields.setter
    def synopsys_fields(self, value: SynopsysFields) -> None:
        """!  synopsys_fields setter
        """
        self.cnn_fields.SynopsysFields = value


class CnnTailHeader:
    """!   CnnTailHeader.
     This structure describes the output buffer of a network
    """

    tail_hdr = None
    """!    InuStreamsPyth.CnnTailHdr.
    """

    def __init__(self, tail_hdr: CnnTailHdr = None):
        """! The TailHdr class initializer.
            @param tail_hdr as InuStreamsPyth.CnnTailHdr .
            @return  An instance of the TailHdr object.
        """
        if tail_hdr is None:
            self.tail_hdr = CnnTailHdr()
        else:
            self.tail_hdr = tail_hdr

    @property
    def element_count(self) -> int:
        """! Number of elements in a tail
        """
        return self.tail_hdr.ElementCount

    @element_count.setter
    def element_count(self, value: int) -> None:
        """! element_count setter
        """
        self.tail_hdr.ElementCount = value

    @property
    def element_size(self) -> int:
        """! Element size in bytes.
        """
        return self.tail_hdr.ElementSize

    @element_size.setter
    def element_size(self, value: int) -> None:
        """! element_size setter
        """
        self.tail_hdr.ElementSize = value

    @property
    def width(self) -> int:
        """! Buffer width in element.
        """
        return self.tail_hdr.Width

    @width.setter
    def width(self, value: int) -> None:
        """! width setter
        """
        self.tail_hdr.Width = value

    @property
    def height(self) -> int:
        """! Buffer height in element.
        """
        return self.tail_hdr.Height

    @width.setter
    def height(self, value: int) -> None:
        """! height setter
        """
        self.tail_hdr.Height = value

    @property
    def inputs(self) -> int:
        """! Number of input maps.
        """
        return self.tail_hdr.Inputs

    @inputs.setter
    def inputs(self, value: int) -> None:
        """! inputs setter
        """
        self.tail_hdr.Inputs = value

    @property
    def cnn_fields(self) -> CnnFields:
        """! Fields relevant for ceva and Synopsys
        """
        return self.tail_hdr.CnnFields

    @element_size.setter
    def cnn_fields(self, value: int) -> None:
        """! cnn_fields setter
        """
        self.tail_hdr.CnnFields = value


class CnnHeader:
    """!  Cnn Frame header.
        Comment: Is the description of the result buffer from dsp to.
    """

    cnn_frame_hdr = None
    """!    InuStreamsPyth.CnnFrameH.
    """

    def __init__(self, cnn_frame_hdr: CnnFrameH):
        self.cnn_frame_hdr = cnn_frame_hdr
        BaseFrame.__init__(self, cnn_frame_hdr)
        """! The CnnFrameHdr class initializer.
            @param frame  The CnnFrameH  from InuStreamsPyth.
            @return  An instance of the ImuFr initialized with the specified InuStreamsPyth.CnnFrameH  object.
        """

    @property
    def total_result_size(self) -> int:
        """! The total results size.
        """
        return self.cnn_frame_hdr.TotalResultSize

    @property
    def num_of_tails(self) -> int:
        """! Number of concatenated CCnnTailHdr, at the results blob.
        """
        return self.cnn_frame_hdr.NumOfTails

    @property
    def network_id(self) -> int:
        """! The network id that had returned the result.
        """
        return self.cnn_frame_hdr.NetworkId

    @property
    def frame_id(self) -> int:
        """! Webcam frame number.
        """
        return self.cnn_frame_hdr.FrameId

    @property
    def tail_header(self):
        """! The tail header for each tail.
        """
        return self.cnn_frame_hdr.TailHeader

    @property
    def offset_to_blob(self):
        """! The offset to tail , assuming tail 0 offset is 0.
        """
        return self.cnn_frame_hdr.OffsetToBlob


class CnnFrame(BaseFrame):
    """!  Cnn frame.
    Role:Represents a CNN frame

    Responsibilities:
          1. Buffer attributes: data and size.
          2. Memory management control.

    Comment: The CNN frame conssist of the data which is part of the CGeneralFrame & the
             CCnnFrameHdr which is the metadata which describes the data.
    """

    CNN_FRAME_MAX_TAILS = 30

    cnn_frame = None
    """!    InuStreamsPyth.CnnF.
    """

    def __init__(self, frame):
        self.cnn_frame = frame
        BaseFrame.__init__(self, frame)
        """! The CnnFrame Frame class initializer.
            @param frame  The CnnF  from InuStreamsPyth.
            @return  An instance of the ImuFr initialized with the specified InuStreamsPyth.CnnF  object.
        """

    @property
    def frame_header(self) -> CnnHeader:
        """! Getter for Frame header.
        """
        return CnnHeader(self.cnn_frame.FrameHeader)

    @property
    def engine_type(self) -> NetworkEngineType:
        """! Getter for Engine Type: ceva or synopsis.
        """
        return NetworkEngineType(self.cnn_frame.EngineType)

    @property
    def buffer(self) -> np.array:
        # @brief   Pixels data buffer.
        return np.asanyarray(self.cnn_frame)

class CnnImageDescriptor:
    """! .
        Role: Describes the input buffer for a CNN network.
    """

    image_descr: CnnImageDescr = None
    """!    InuStreamsPyth.CnnImageDescr.
    """

    def __init__(self, image_descr: CnnImageDescr = None):
        """! The ImageDescriptor class initializer.
            @param image_descr  The CnnImageDescr  from InuStreamsPyth.
            @return  An instance of the ImageDescriptor object.
        """
        if image_descr is None:
            self.image_descr = CnnImageDescr()
        else:
            self.image_descr = image_descr

    @property
    def width(self) -> int:
        """! Image width.
        """
        return self.image_descr.Width

    @width.setter
    def width(self, value: int) -> None:
        """! width setter
        """
        self.image_descr.Width = value

    @property
    def height(self) -> int:
        """! Image height.
        """
        return self.image_descr.Height

    @height.setter
    def height(self, value: int) -> None:
        """! height setter
        """
        self.image_descr.Height = value

    @property
    def format(self) -> ImageFormatFw:
        """! Image Format RGB YUV RAW.
        """
        return ImageFormatFw(self.image_descr.Format)

    @format.setter
    def format(self, value: ImageFormatFw) -> None:
        """! format setter
        """
        self.image_descr.Format = value

    @property
    def x(self) -> int:
        """! ROI x starting point.
        """
        return self.image_descr.X

    @x.setter
    def x(self, value: int) -> None:
        """! x setter
        """
        self.image_descr.X = value

    @property
    def y(self) -> int:
        """! ROI y starting point.
        """
        return self.image_descr.Y

    @y.setter
    def y(self, value: int) -> None:
        """! y setter
        """
        self.image_descr.Y = value

    @property
    def stride(self) -> int:
        """! Width of image buffer in memory (the step needed to jump to next line).
        """
        return self.image_descr.Stride

    @stride.setter
    def stride(self, value: int) -> None:
        """! stride setter
        """
        self.image_descr.Stride = value

    @property
    def buffer_height(self) -> int:
        """! Image buffer height.
        """
        return self.image_descr.BufferHeight

    @buffer_height.setter
    def buffer_height(self, value: int) -> None:
        """! buffer_height setter
        """
        self.image_descr.BufferHeight = value

    @property
    def bits_per_pixel(self) -> int:
        """! Number of bits per pixel.
        """
        return self.image_descr.BitsPerPixel

    @bits_per_pixel.setter
    def bits_per_pixel(self, value: int) -> None:
        """! bits_per_pixel setter
        """
        self.image_descr.BitsPerPixel = value

    @property
    def real_bits_mask(self) -> int:
        """! repacked related.
        """
        return self.image_descr.RealBitsMask

    @real_bits_mask.setter
    def real_bits_mask(self, value: int) -> None:
        """! real_bits_mask setter
        """
        self.image_descr.RealBitsMask = value

    @property
    def num_interleave_images(self) -> int:
        """! Number of images in buffer.
        """
        return self.image_descr.NumInterleaveImages

    @num_interleave_images.setter
    def num_interleave_images(self, value: int) -> None:
        """! num_interleave_images setter
        """
        self.image_descr.NumInterleaveImages = value

    @property
    def interleave_mode(self) -> InterliveModeFw:
        """! Interleave mode: line by line, pixel by pixel, frame by frame.
        """
        return InterliveModeFw(self.image_descr.InterleaveMode)

    @interleave_mode.setter
    def interleave_mode(self, value: InterliveModeFw) -> None:
        """! interleave_mode setter
        """
        self.image_descr.InterleaveMode = InterliveModeFw(value)


class CnnStream(BaseStream):
    """! Interface for Cnn App service.

    Role: Controls Cnn App streaming service and provides general or Cnn App frames.
          IMU frames are provided only if the connected device supports Cnn App HW components.
          The caller application should be familiar with provided frames and should know how to interpret them.

    Responsibilities:
          1. Derives BaseStream class
          2. Knows how to acquire one depth image frame (pull)
          3. Knows how to provide a continuous stream of depth image frames (push)
    """

    stream = None
    """!    InuStreamsPyth.CnnS.
    """

    def __init__(self, stream: CnnS = None):
        """! The Cnn stream class initializer.
            @param stream  The InuStreamsPyth.CnnS.
            @return  An instance of the Cnn stream initialized with the specified InuStreamsPyth.CnnS object.
        """
        BaseStream.__init__(self, stream)
        self.stream = stream

    def init(self) -> None:
        """!
            Service initialization.
            Have be invoked once before starting frames acquisition.
        """
        self.stream.Init()

    def terminate(self) -> None:
        """!
            Stop frames acquisition, stop ant termination service.
        """
        self.stop_network()
        self.register = None
        self.stop()
        self.stream.Terminate()

    def register(self, callback) -> None:
        """!
            Registration/De registration for receiving stream frames (push)

            The provided callback function is called when a new frame is ready (non-blocking).
            It shall be called only after a start() was invoked but before any invocation of a stop() is invoked.
            @param  callback  The Callback function which is invoked when a new frame is ready. Send nullptr to
                unregister for receiving frames.
        """
        def _callback_cast(stream: CnnS, frame: CnnF, error: InuError) -> None:
            """!
                Prototype of callback function which is used by the Register method.

                This fun ction is invoked any time a frame is ready, or if an error occurs. The parameters of this
                function are:
                @param stream Caller Cnn stream object.
                @param frame  Caller Cnn frame object.
                @param error  Result code.
            """
            BaseStream.callback(CnnStream(stream), CnnFrame(frame), Error(error))
        BaseStream.callback = callback
        self.stream.Register(_callback_cast)
    register = property(None, register)

    @property
    def frame(self) -> CnnFrame:
        """!
            Retrieves new frame (pull).

            This method returns when a new frame is ready (blocking) or if an input timeout has elapsed.
            It shall be called only after a start() was invoked but before any invocation of a stop() is invoked.
        """
        return CnnFrame(self.stream.GetFrame())

    def start_network(self, network_id: str, network_node_name: str) -> None:
        """!
            Start CNN identified by its unique ID & The node in the "sw graph" where the network should execute.
            @param network_id              The unique identifier of the network.
            @param network_node_name       The node in the "sw graph" where the network should execute.
        """
        self.stream.StartNetwork(network_id, network_node_name)

    def stop_network(self) -> None:
        """! Stop a CNN network.
        """
        self.stream.StopNetwork()

    def write_data(self, streamer: str, image_descriptor, data: np.array, data_size: int, frame_id: int = 0) -> None:
        """!
            Write buffer to CNN service.
            @ param     streamer          The name of the input streamer in the "sw graph", the buffer will be written to.
            @ param     image_descriptor   Describes the buffer.
            @ param     data               Data buffer.
            @ param     data_size          Buffer length.
        """
        self.stream.WriteData(streamer, image_descriptor._image_descr, data, data_size)
