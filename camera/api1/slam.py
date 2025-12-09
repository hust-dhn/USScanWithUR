from .base_stream import BaseStream
from .base_frame import BaseFrame
from .errors import Error
from .InuStreamsPyth import SlamF, SlamS, InuError, SlamTransformationsP
import numpy as np
from enum import IntEnum


# @brief    Define Slam State options
#
class SlamState(IntEnum):
    """! State enum class.
        Role: All Slam state options
    """
    NOT_SUPPORTED_STATE = SlamF.ESlamState.NotSupporttedState
    TRACK_OK = SlamF.ESlamState.TrackOK
    TRACK_LOST_NO_RELOC = SlamF.ESlamState.TrackLostNoReloc
    TRACK_LOST_NUM_FEATURES = SlamF.ESlamState.TrackLostNumFeatures
    TRACK_LOST_NUM_STEREO_PAIRS = SlamF.ESlamState.TrackLostNumStereoPairs
    TRACK_LOST_NUM_MATCHER = SlamF.ESlamState.TrackLostNumMatcher
    TRACK_LOST_NUM_INLINERS1 = SlamF.ESlamState.TrackLostNumInliners1
    TRACK_LOST_NUM_INLINERS2 = SlamF.ESlamState.TrackLostNumInliners2
    TRACK_LOST = SlamF.ESlamState.TrackLost
    RELOCALIZATION_SUCCEEDED = SlamF.ESlamState.RelocalizationSucceeded
    LOOP_CLOSED = SlamF.ESlamState.LoopClosed
    IMU_NO_VISUAL = SlamF.ESlamState.ImuNoVisual
    SLAM_MAP_POINT_NOT_SYNC = SlamF.ESlamState.SlamMapPointNotSync
    FRAME_UNEXPECTED_ERROR = SlamF.ESlamState.FrameUnexpectedError


class SlamInternalState(IntEnum):
    """! InternalState enum class.
        Role: All Slam Internal state options
    """
    NOT_SUPPORTED_INTERNAL_STATE = SlamF.ESlamInternalState.NotSupporttedInternalState
    KEY_FRAME = SlamF.ESlamInternalState.Keyframe
    IN_TRACK = SlamF.ESlamInternalState.InTrack
    IN_RELOCALIZATION = SlamF.ESlamInternalState.InRelocalization
    IN_LOOP_CLOSING = SlamF.ESlamInternalState.InLoopClosing
    IN_LBA = SlamF.ESlamInternalState.InLBA
    UNEXPECTED_INTERNAL_ERROR = SlamF.ESlamInternalState.UnexpectedInternalError


class SlamFrame(BaseFrame):
    """!  Slam frame.

    Role: Represents frames that are provided by CSlamStream

    """

    def __init__(self, frame: SlamF):
        self.slam_frame = frame
        BaseFrame.__init__(self, frame)
        """! The Imu Frame class initializer.
            @param frame  The SlamFrame  from InuStreamsPyth.
            @return  An instance of the SlamFrame initialized with the specified InuStreamsPyth.SlamF  object.
        """

    # @brief    InuStreamsPyth.SlamF.
    #
    slam_frame = None

    @property
    def state(self) -> int:
        # @brief  Current Slam state reported to the application
        #
        # @return  TheSlam state.
        return self.slam_frame.State

    @property
    def internal_state(self) -> int:
        # @brief  Current Slam state reported to the application
        #
        # @return  The Slam internal state.
        return self.slam_frame.InternalState

    @property
    def pose4x4body2world(self) -> np.array:
        # @brief  The transformation matrix describing how the body moves in the World coordinate system
        #   (used to display the trajectory in 3D in the World coordinate system).
        #
        # @return  array float[16].
        return self.slam_frame.Pose4x4BodyToWorld

    @property
    def pose4x4world2body(self) -> np.array:
        # @brief   The transformation matrix describing how objects in the World coordinate system
        #   move relatively to a fixed coordinate system at the body of the robot (used
        #   for graphical display to overlay object over the camera image).
        #
        # @return  array float[16].
        return self.slam_frame.Pose4x4WorldToBody


class SlamCoordinateSystem(IntEnum):
    """! CoordinateSystem enum class.
        Role: Use coordinates transformations according to enumerator: calibrated or virtual coordinate system.
    """
    # @ brief    Calibrated cameras option. use CameraToModule, ModuleToBody, BodyToWorld transformations to
    # user's coordinate system, ignoring virtual camera. - CALIBRATED_CAMERA_USER_TRANSFORMATION,
    CALIBRATED_CAMERAUSER_TRANSFORMATION = SlamS.ECoordinateSystem.CalibratedCameraUserTransformation
    # @ brief   default: Virtual cameras option. use the left camera system, ignoring CameraToModule,
    # ModuleToBody, BodyToWorld - VIRTUAL_CAMERA_LEFT_CAMERA
    VIRTUAL_CAMERA_LEFT_CAMERA = SlamS.ECoordinateSystem.VirtualCameraLeftCamera
    # @ brief   Virtual cameras option. use the right camera system, ignoring CameraToModule, ModuleToBody,
    #       BodyToWorld - VIRTUAL_CAMERA_RIGHT_CAMERA
    VIRTUAL_CAMERA_RIGHT_CAMERA = SlamS.ECoordinateSystem.VirtualCameraRightCamera
    # @ brief   Virtual cameras option. use the baseline center camera system, ignoring CameraToModule,
    #       ModuleToBody, BodyToWorld - VIRTUAL_CAMERA_BASELINE_CENTER
    VIRTUAL_CAMERA_BASELINE_CENTER = SlamS.ECoordinateSystem.VirtualCameraBaselineCenter


class SetMode(IntEnum):
    """! SetMode enum class.
        Role: SetMode options.
    """
    # @ brief     This option resets the database and maintains tracking with the current pose,
    # It is suitable for a user who may keep a small database (and usually doesn't need loop-closure).
    # Keeping the database small helps achieve very fast re-localization and beneficial culling. The 'reset'
    # operation should be performed by the tracking processor and should succeed unless the subsequent frame
    # after the 'reset' fails to track. In such a case, we cannot reset the database due to localization
    # use of the database The user can verify the success of the 'reset' operation by testing the
    # 'internalState' status in the output buffer.
    SET_RESET_DATABASE_MAINTAIN_POSE = SlamS.ESlamSetMode.SetResetDatabaseMaintainPose
    # @ brief   This option resets the database and establishes a new pose from the user, It is suitable for
    # a user who has additional pose indication from an external resource The 'reset' operation should be
    # executed by the tracking processor and should succeed unless the subsequent frame following the 'reset'
    # fails to track. In such a case, we cannot reset the database due to the localization use of the
    # database. The user can verify the success of the 'reset' & 'set' operations by testing the output pose
    # or by testing the 'internalState' status in the output buffer. In a different scenario, if the 'reset'
    # & 'set' commands are sent during "Tracking lost," they should succeed unless the subsequent frame
    # following the 'reset' doesn't contain enough pairs in the stereo matcher, falling below the minimum
    # criteria for a keyframe.
    SET_RESET_DATABASE_SET_POSE = SlamS.ESlamSetMode.SetResetDatabaseSetPose


class SlamTransformationsParams:
    """!  Transformations Params.

    Role: SLAM output transformation section

    """

    _params: SlamTransformationsP = None

    def __init__(self, params: SlamTransformationsP):
        """! The Slam stream class initializer. @param params  The InuStreamsPyth.SlamTransformationsParams.
        @return  An instance of the Slam stream initialized with the specified
                InuStreamsPyth.SlamTransformationsParams object.
        """
        self._params = params

    @property
    def camera_system(self) -> SlamCoordinateSystem:
        # @brief   Retrieves Slam CoordinateSystem.
        #
        # @return  The returned slam CoordinateSystem.
        return SlamCoordinateSystem(self._params.CameraSystem)

    @camera_system.setter
    def camera_system(self, value: SlamCoordinateSystem) -> None:
        # @brief    camera_system setter.
        self._params.CameraSystem = SlamS.ECoordinateSystem(value)

    @property
    def camera2module(self) -> np.array:
        # @brief   Transformation from the module system to physical module system.
        #   The transformation is from the left camera, in slam coordinate system, to the module.
        #   The transformation is a 4x4 matrix ([[R,t],[0,0,0,1]], R is 3x3 rotation), row-wise,
        #   as one row vector. (converted to 4x4).
        # @return  The CameraToModule array.
        return self._params.CameraToModule

    @camera_system.setter
    def camera2module(self, value: np.array) -> None:
        # @brief    camera2module setter.
        self._params.CameraToModule = value

    @property
    def module2body(self) -> np.array:
        # @brief   Transformation from the module system to physical module system.
        #
        # Transformation from the module system to the robot's body system (user specified).
        #   The transformation is a 4x4 matrix ([[R,t],[0,0,0,1]], R is 3x3 rotation), row-wise,
        #   as one row vector. (converted to 4x4).
        # @return  The ModuleToBody array.
        return self._params.ModuleToBody

    @camera_system.setter
    def module2body(self, value: np.array) -> None:
        # @brief    module2body setter.
        self._params.ModuleToBody = value

    @property
    def body02world(self) -> np.array:
        # @brief   Transformation from the module system to physical module system.
        #
        # Transformation from the body system at frame 0 to world (user) system (stationary, fixed system).
        #   The transformation is a 4x4 matrix ([[R,t],[0,0,0,1]], R is 3x3 rotation), row-wise,
        #   as one row vector. (converted to 4x4).
        # @return  The ModuleToBody array.
        return self._params.ModuleToBody

    @body02world.setter
    def body02world(self, value: np.array) -> None:
        # @brief    body02world setter.
        self._params.Body0ToWorld = value


class SlamStream(BaseStream):
    """! Interface for SLAM service.

    Role: Controls Slam streaming service and provides Slam frames.

    Responsibilities:
          1. Derives BaseStream class
          2. Knows how to acquire one Slam frame (pull)
          3. Knows how to provide a continuous stream of slam frames (push)
    """

    _stream = None

    def __init__(self, stream: SlamTransformationsParams):
        """! The Slam stream class initializer.
            @param stream  The InuStreamsPyth.SlamStream.
            @return  An instance of the Slam stream initialized with the specified InuStreamsPyth.SlamStream object.
        """
        BaseStream.__init__(self, stream)
        self._stream = stream

    def init(self, transformations_params: SlamTransformationsParams = None) -> None:
        # @brief    Service initialization.
        #
        # Hall be invoked once before starting frames acquisition.
        # @param  format            The Output format that should be invoked.
        if transformations_params is None:
            self._stream.Init()
        else:
            self._stream.Init(transformations_params.params)

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
        def _callback_cast(stream: SlamS, frame: SlamF, error: InuError) -> None:
            """!
                Prototype of callback function which is used by the Register method.

                This function is invoked any time a frame is ready, or if an error occurs. The parameters of this
                function are:
                @param stream Caller image stream object.
                @param frame  Caller image frame object.
                @param error  Result code.
            """
            BaseStream.callback(SlamStream(stream), SlamFrame(frame), Error(error))
        BaseStream.callback = callback
        self._stream.Register(_callback_cast)
    register = property(None, register)

    @property
    def frame(self) -> SlamFrame:
        # @brief   Retrieves new frame (pull)
        #
        # This method returns when a new frame is ready (blocking) or if an input timeout has elapsed.
        # It shall be called only after a start() was invoked but before any invocation of a stop() is invoked.
        # @return  The returned stereo frame.
        return SlamFrame(self._stream.GetFrame())

    @property
    def transformations_params(self):
        # @brief   Composing Transformations Params.
        #
        # @return  The Transformations Params.
        return self._stream.TransformationsParams
