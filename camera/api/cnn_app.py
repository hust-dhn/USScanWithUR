from .base_stream import BaseStream
from .base_frame import BaseFrame
from .image_frame import ImageFrame
from .shared import Point2Dim
from .errors import Error
from .InuStreamsPyth import CnnAppF, CnnAppS, MapUintString, VectorString, VectorRecognizedFaces, \
    VectorDetectedObject, ClassificationD, RecognizedF, DetectedO, InuError
from typing import Union
from enum import IntEnum


class DetectedObject:
    """!  Detected Object struct.

    Role: Represents DetectedObject.

    """
    object: DetectedO = None

    def __init__(self, object: DetectedO):
        self.object = object

    @property
    def class_id(self) -> str:
        # @brief	Classification identifier of this object
        return self.object.ClassID

    @property
    def confidence(self) -> float:
        # @brief	Confidence score of this object
        return self.object.Confidence

    @property
    def closed_rect_top_left(self) -> Point2Dim:
        # @brief	Top Left coordinates of detected object.
        return Point2Dim(self.object.ClosedRectTopLeft)

    @property
    def closed_rect_size(self)-> Point2Dim:
        # @brief	Size of recognized face.
        return Point2Dim(self.object.ClosedRectSize)

    @property
    def object_color(self) -> float:
        # @brief	Color of the object(if applicable - Yolact).
        return self.object.ObjectColor


class FacePose(IntEnum):
    """!  EFacePose enum class.
        Define Cnn App's FacePose formats.
    """
    # Unrecognized position .
    NONE_POSE = CnnAppF.EFacePose.NonePose
    # Looking to the left (Horizontal).
    LEFT = CnnAppF.EFacePose.Left
    # Looking to the right (Horizontal).
    RIGHT = CnnAppF.EFacePose.Right
    # Looking to the top (Vertical).
    TOP = CnnAppF.EFacePose.Top
    # Object Looking to the bottom (Vertical)
    BOTTOM = CnnAppF.EFacePose.Bottom
    # Looking to the center (Horizontal/Vertical).
    CENTER = CnnAppF.EFacePose.Center


class RecognizedFace:
    """!  Recognized Face struct.

    Role: Represents RecognizedFace Object.

    """
    face: RecognizedF = None

    LANDMARKS_POINTS = 5

    def __init__(self, face: RecognizedF):
        self.face = face

    @property
    def face_id(self) -> str:
        # @brief	Classification identifier of this object
        return self.face.FaceId

    @property
    def confidence(self) -> float:
        # @brief	Confidence score of this object
        return self.face.Confidence;

    @property
    def closed_rect_top_left(self) -> Point2Dim:
        # @brief	Top Left coordinates of detected object.
        return Point2Dim(self.face.ClosedRectTopLeft)

    @property
    def closed_rect_size(self) -> Point2Dim:
        # @brief	Size of recognized face.
        return Point2Dim(self.face.ClosedRectSize)

    @property
    def horizontal_position(self) -> FacePose:
        # @brief	Horizontal face position (Left/Right/Center).
        return FacePose(self.face.poseH)

    @property
    def vertical_position(self) -> FacePose:
        # @brief	Vertical face position (Top/Bottom/Center).
        return FacePose(self.face.poseH)

    @property
    def landmarks(self) -> VectorRecognizedFaces:
        # @brief	Vertical face position (Top/Bottom/Center).
        return self.face.Landmarks


class ClassificationData:
    """!  Classification Data struct.
    Role: Represents ClassificationData.
    """
    data: ClassificationD = None

    def __init__(self, data: ClassificationD):
        self.data = data

    @property
    def class_id(self) -> str:
        # @brief	Classification identifier of this object
        return self.data.ClassID

    @property
    def confidence(self) -> float:
        # @brief	Confidence score of this object
        return self.data.Confidence


class CnnAppOutputFormat(IntEnum):
    """!  OutputType enum class.
        Define Cnn App's Output formats
    """
    UNKNOWN = CnnAppF.EOutputType.Unknown
    DEFAULT = CnnAppF.EOutputType.ObjectDetection
    # Object detection SSD fast but less accurate
    OBJECT_DETECTION = CnnAppF.EOutputType.ObjectDetection
    SEGMENTATION = CnnAppF.EOutputType.Segmentation
    CLASSIFICATION = CnnAppF.EOutputType.Classification
    FACE_RECOGNITION = CnnAppF.EOutputType.FaceRecognition
    # Similar behavior as eObjectDetection but a different network(YoloV3) slower & accurate.
    OBJECT_DETECTION_YOLO_V3 = CnnAppF.EOutputType.ObjectDetectionYoloV3
    ENROLL_FACE = CnnAppF.EOutputType.EnrollFace
    YOLACT = CnnAppF.EOutputType.Yolact
    OBJECT_DETECTION_YOLO_V7 = CnnAppF.EOutputType.ObjectDetectionYoloV7


class CnnAppFrame(BaseFrame):
    """!  CnnApp frame.

    Role: Represents generic buffer that is provided by some of InuDev streams

    Responsibilities:
          1. Buffer attributes: data and size.
          2. Memory management control.

    Comment: The interpretation of provided buffers should be done by the caller application.
             The caller application should be familiar with the internal format of
             each provided frame.
    """

    def __init__(self, frame: CnnAppF):
        self.cnn_app_frame = frame
        BaseFrame.__init__(self, frame)
        """! 
            The Imu Frame class initializer.
            @param frame  The CnnAppFrame  from InuStreamsPyth.
            @return  An instance of the ImuFr initialized with the specified InuStreamsPyth.CnnAppFrame  object.
        """

    # @brief    InuStreamsPyth.CnnAppF.
    #
    cnn_app_frame = None

    @property
    def output_type(self) -> CnnAppOutputFormat:
        # @brief Getter for Cnn OutputType
        return CnnAppOutputFormat(self.cnn_app_frame.OutputType)

    @property
    def data(self) -> Union[VectorRecognizedFaces, VectorDetectedObject, ImageFrame, ClassificationData]:
        # @brief	Getter for cnn app frame data
        cnn_output_type = self.output_type
        if cnn_output_type == CnnAppOutputFormat.OBJECT_DETECTION or \
                cnn_output_type == CnnAppOutputFormat.OBJECT_DETECTION_YOLO_V3 or \
                cnn_output_type == CnnAppOutputFormat.OBJECT_DETECTION_YOLO_V7 or \
                cnn_output_type == CnnAppOutputFormat.YOLACT:
            return self.cnn_app_frame.ObjectData
        elif cnn_output_type == CnnAppOutputFormat.FACE_RECOGNITION:
            return self.cnn_app_frame.FaceData
        elif cnn_output_type == CnnAppOutputFormat.SEGMENTATION:
            return ImageFrame(self.cnn_app_frame.Segmentation)
        elif cnn_output_type == CnnAppOutputFormat.CLASSIFICATION:
            return ClassificationData(self.cnn_app_frame.Classification)
        else:  # Unknown or EnrollFace
            return None

    @property
    def width(self) -> int:
        # @ Getter for Width
        return self.cnn_app_frame.Width

    @property
    def height(self) -> int:
        # @ Getter for Height
        return self.cnn_app_frame.Height


class CnnAppStream(BaseStream):
    """! Interface for Cnn App service.

    Role: Controls Cnn App streaming service and provides general or Cnn App frames.
          IMU frames are provided only if the connected device supports Cnn App HW components.
          The caller application should be familiar with provided frames and should know how to interpret them.

    Responsibilities:
          1. Derives BaseStream class
          2. Knows how to acquire one depth image frame (pull)
          3. Knows how to provide a continuous stream of depth image frames (push)
    """

    # @brief    InuStreamsPyth.CnnAppS.
    #
    _stream = None

    def __init__(self, stream: CnnAppS):
        """! The Cnn App stream class initializer.
            @param stream  The InuStreamsPyth.CnnAppS.
            @return  An instance of the Cnn App stream initialized with the specified InuStreamsPyth.CnnAppS object.
        """
        BaseStream.__init__(self, stream)
        self._stream = stream

    def init(self, form: CnnAppOutputFormat = CnnAppOutputFormat.DEFAULT) -> None:
        # @brief    Service initialization.
        #
        # Hall be invoked once before starting frames acquisition.
        self._stream.Init(CnnAppF.EOutputType(form))

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
        def _callback_cast(stream: CnnAppS, frame: CnnAppF, error: InuError) -> None:
            """!
                Prototype of callback function which is used by the Register method.

                This function is invoked any time a frame is ready, or if an error occurs. The parameters of this
                function are:
                @param stream Caller CnnApp stream object.
                @param frame  Caller CnnApp frame object.
                @param error  Result code.
            """
            BaseStream.callback(CnnAppStream(stream), CnnAppFrame(frame), Error(error))
        BaseStream.callback = callback
        self._stream.Register(_callback_cast)
    register = property(None, register)

    @property
    def frame(self) -> CnnAppFrame:
        # @brief   Retrieves new frame (pull)
        #
        # This method returns when a new frame is ready (blocking) or if an input timeout has elapsed.
        # It shall be called only after a start() was invoked but before any invocation of a stop() is invoked.
        # @return  The returned Cnn App frame.
        return CnnAppFrame(self._stream.GetFrame())

    @property
    def segmentation_labels(self) -> MapUintString:
        # @brief    Getter segmentation labels for segmentation image
        #
        # @return SegmentationLabels Labels returned from AI lib.
        return self._stream.SegmentationLabels

    def enroll_person(self, person_name: str, path: str = None) -> None:
        # @brief    Enroll new person(face) for Face recognition AI, can be used only after Enroll stream is
        # ` initialized and started.
        #
        # @param person_name - Name of the person to enroll @param path - Path to folder with images to enroll,
        #   default images will be taken from "C:\Program Files\Inuitive\InuDev\config\AI\FaceDetection\faces"
        return self._stream.EnrollPerson(person_name, path)

    def delete_person(self, person_name: str) -> None:
        # @brief    Delete person(face) from Face recognition AI Database, can be used only after Enroll stream is
        #   initialized and started.
        #
        # @param person_name - Name of the person to enroll
        return self._stream.DeletePerson(person_name)

    @property
    def list_of_faces(self) -> VectorString:
        # @brief    Retrieve vector of names from face recognition AI Database, can be used only after Enroll stream
        #   is initialized and started.
        #
        # @return Names of enrolls
        return self._stream.ListOfFaces

    @property
    def image_channel(self) -> int:
        # @brief    Retrieve Image channel ID connected to the CNNApp Stream. Can only be called after CnnAppStream
        #   is started.
        #
        # @return Image channel ID as set by default or by user.
        return self._stream.ImageChannel
