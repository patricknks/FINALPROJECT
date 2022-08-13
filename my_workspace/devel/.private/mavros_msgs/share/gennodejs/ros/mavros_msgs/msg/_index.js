
"use strict";

let CameraImageCaptured = require('./CameraImageCaptured.js');
let VehicleInfo = require('./VehicleInfo.js');
let ActuatorControl = require('./ActuatorControl.js');
let Tunnel = require('./Tunnel.js');
let GPSRTK = require('./GPSRTK.js');
let RTCM = require('./RTCM.js');
let GPSINPUT = require('./GPSINPUT.js');
let ExtendedState = require('./ExtendedState.js');
let ADSBVehicle = require('./ADSBVehicle.js');
let CamIMUStamp = require('./CamIMUStamp.js');
let LandingTarget = require('./LandingTarget.js');
let VFR_HUD = require('./VFR_HUD.js');
let AttitudeTarget = require('./AttitudeTarget.js');
let OpticalFlowRad = require('./OpticalFlowRad.js');
let Waypoint = require('./Waypoint.js');
let GPSRAW = require('./GPSRAW.js');
let OnboardComputerStatus = require('./OnboardComputerStatus.js');
let TerrainReport = require('./TerrainReport.js');
let CompanionProcessStatus = require('./CompanionProcessStatus.js');
let EstimatorStatus = require('./EstimatorStatus.js');
let ESCInfo = require('./ESCInfo.js');
let FileEntry = require('./FileEntry.js');
let ManualControl = require('./ManualControl.js');
let NavControllerOutput = require('./NavControllerOutput.js');
let CommandCode = require('./CommandCode.js');
let RCOut = require('./RCOut.js');
let BatteryStatus = require('./BatteryStatus.js');
let HilStateQuaternion = require('./HilStateQuaternion.js');
let LogData = require('./LogData.js');
let HomePosition = require('./HomePosition.js');
let ESCStatusItem = require('./ESCStatusItem.js');
let Thrust = require('./Thrust.js');
let ESCStatus = require('./ESCStatus.js');
let Mavlink = require('./Mavlink.js');
let OverrideRCIn = require('./OverrideRCIn.js');
let WaypointList = require('./WaypointList.js');
let TimesyncStatus = require('./TimesyncStatus.js');
let Param = require('./Param.js');
let Vibration = require('./Vibration.js');
let WheelOdomStamped = require('./WheelOdomStamped.js');
let HilSensor = require('./HilSensor.js');
let State = require('./State.js');
let LogEntry = require('./LogEntry.js');
let HilGPS = require('./HilGPS.js');
let PositionTarget = require('./PositionTarget.js');
let ParamValue = require('./ParamValue.js');
let ESCTelemetry = require('./ESCTelemetry.js');
let RTKBaseline = require('./RTKBaseline.js');
let MountControl = require('./MountControl.js');
let WaypointReached = require('./WaypointReached.js');
let HilActuatorControls = require('./HilActuatorControls.js');
let MagnetometerReporter = require('./MagnetometerReporter.js');
let RadioStatus = require('./RadioStatus.js');
let DebugValue = require('./DebugValue.js');
let ESCTelemetryItem = require('./ESCTelemetryItem.js');
let HilControls = require('./HilControls.js');
let Trajectory = require('./Trajectory.js');
let StatusText = require('./StatusText.js');
let ESCInfoItem = require('./ESCInfoItem.js');
let PlayTuneV2 = require('./PlayTuneV2.js');
let Altitude = require('./Altitude.js');
let RCIn = require('./RCIn.js');
let GlobalPositionTarget = require('./GlobalPositionTarget.js');

module.exports = {
  CameraImageCaptured: CameraImageCaptured,
  VehicleInfo: VehicleInfo,
  ActuatorControl: ActuatorControl,
  Tunnel: Tunnel,
  GPSRTK: GPSRTK,
  RTCM: RTCM,
  GPSINPUT: GPSINPUT,
  ExtendedState: ExtendedState,
  ADSBVehicle: ADSBVehicle,
  CamIMUStamp: CamIMUStamp,
  LandingTarget: LandingTarget,
  VFR_HUD: VFR_HUD,
  AttitudeTarget: AttitudeTarget,
  OpticalFlowRad: OpticalFlowRad,
  Waypoint: Waypoint,
  GPSRAW: GPSRAW,
  OnboardComputerStatus: OnboardComputerStatus,
  TerrainReport: TerrainReport,
  CompanionProcessStatus: CompanionProcessStatus,
  EstimatorStatus: EstimatorStatus,
  ESCInfo: ESCInfo,
  FileEntry: FileEntry,
  ManualControl: ManualControl,
  NavControllerOutput: NavControllerOutput,
  CommandCode: CommandCode,
  RCOut: RCOut,
  BatteryStatus: BatteryStatus,
  HilStateQuaternion: HilStateQuaternion,
  LogData: LogData,
  HomePosition: HomePosition,
  ESCStatusItem: ESCStatusItem,
  Thrust: Thrust,
  ESCStatus: ESCStatus,
  Mavlink: Mavlink,
  OverrideRCIn: OverrideRCIn,
  WaypointList: WaypointList,
  TimesyncStatus: TimesyncStatus,
  Param: Param,
  Vibration: Vibration,
  WheelOdomStamped: WheelOdomStamped,
  HilSensor: HilSensor,
  State: State,
  LogEntry: LogEntry,
  HilGPS: HilGPS,
  PositionTarget: PositionTarget,
  ParamValue: ParamValue,
  ESCTelemetry: ESCTelemetry,
  RTKBaseline: RTKBaseline,
  MountControl: MountControl,
  WaypointReached: WaypointReached,
  HilActuatorControls: HilActuatorControls,
  MagnetometerReporter: MagnetometerReporter,
  RadioStatus: RadioStatus,
  DebugValue: DebugValue,
  ESCTelemetryItem: ESCTelemetryItem,
  HilControls: HilControls,
  Trajectory: Trajectory,
  StatusText: StatusText,
  ESCInfoItem: ESCInfoItem,
  PlayTuneV2: PlayTuneV2,
  Altitude: Altitude,
  RCIn: RCIn,
  GlobalPositionTarget: GlobalPositionTarget,
};
