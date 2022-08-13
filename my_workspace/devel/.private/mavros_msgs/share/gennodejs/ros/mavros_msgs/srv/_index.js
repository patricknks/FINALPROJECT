
"use strict";

let WaypointPull = require('./WaypointPull.js')
let CommandVtolTransition = require('./CommandVtolTransition.js')
let FileWrite = require('./FileWrite.js')
let ParamSet = require('./ParamSet.js')
let ParamGet = require('./ParamGet.js')
let CommandLong = require('./CommandLong.js')
let WaypointClear = require('./WaypointClear.js')
let CommandTriggerInterval = require('./CommandTriggerInterval.js')
let FileList = require('./FileList.js')
let StreamRate = require('./StreamRate.js')
let FileMakeDir = require('./FileMakeDir.js')
let FileClose = require('./FileClose.js')
let CommandHome = require('./CommandHome.js')
let CommandInt = require('./CommandInt.js')
let LogRequestList = require('./LogRequestList.js')
let CommandBool = require('./CommandBool.js')
let WaypointPush = require('./WaypointPush.js')
let MessageInterval = require('./MessageInterval.js')
let CommandTOL = require('./CommandTOL.js')
let CommandAck = require('./CommandAck.js')
let VehicleInfoGet = require('./VehicleInfoGet.js')
let FileRemoveDir = require('./FileRemoveDir.js')
let SetMavFrame = require('./SetMavFrame.js')
let SetMode = require('./SetMode.js')
let FileChecksum = require('./FileChecksum.js')
let ParamPull = require('./ParamPull.js')
let FileRemove = require('./FileRemove.js')
let CommandTriggerControl = require('./CommandTriggerControl.js')
let FileTruncate = require('./FileTruncate.js')
let LogRequestData = require('./LogRequestData.js')
let ParamPush = require('./ParamPush.js')
let FileOpen = require('./FileOpen.js')
let MountConfigure = require('./MountConfigure.js')
let LogRequestEnd = require('./LogRequestEnd.js')
let FileRead = require('./FileRead.js')
let WaypointSetCurrent = require('./WaypointSetCurrent.js')
let FileRename = require('./FileRename.js')

module.exports = {
  WaypointPull: WaypointPull,
  CommandVtolTransition: CommandVtolTransition,
  FileWrite: FileWrite,
  ParamSet: ParamSet,
  ParamGet: ParamGet,
  CommandLong: CommandLong,
  WaypointClear: WaypointClear,
  CommandTriggerInterval: CommandTriggerInterval,
  FileList: FileList,
  StreamRate: StreamRate,
  FileMakeDir: FileMakeDir,
  FileClose: FileClose,
  CommandHome: CommandHome,
  CommandInt: CommandInt,
  LogRequestList: LogRequestList,
  CommandBool: CommandBool,
  WaypointPush: WaypointPush,
  MessageInterval: MessageInterval,
  CommandTOL: CommandTOL,
  CommandAck: CommandAck,
  VehicleInfoGet: VehicleInfoGet,
  FileRemoveDir: FileRemoveDir,
  SetMavFrame: SetMavFrame,
  SetMode: SetMode,
  FileChecksum: FileChecksum,
  ParamPull: ParamPull,
  FileRemove: FileRemove,
  CommandTriggerControl: CommandTriggerControl,
  FileTruncate: FileTruncate,
  LogRequestData: LogRequestData,
  ParamPush: ParamPush,
  FileOpen: FileOpen,
  MountConfigure: MountConfigure,
  LogRequestEnd: LogRequestEnd,
  FileRead: FileRead,
  WaypointSetCurrent: WaypointSetCurrent,
  FileRename: FileRename,
};
