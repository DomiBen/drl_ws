// Auto-generated. Do not edit!

// (in-package trajectory_planner.srv)


"use strict";

const _serializer = _ros_msg_utils.Serialize;
const _arraySerializer = _serializer.Array;
const _deserializer = _ros_msg_utils.Deserialize;
const _arrayDeserializer = _deserializer.Array;
const _finder = _ros_msg_utils.Find;
const _getByteLength = _ros_msg_utils.getByteLength;

//-----------------------------------------------------------


//-----------------------------------------------------------

class SetGcodeCmdRequest {
  constructor(initObj={}) {
    if (initObj === null) {
      // initObj === null is a special case for deserialization where we don't initialize fields
      this.gcode = null;
    }
    else {
      if (initObj.hasOwnProperty('gcode')) {
        this.gcode = initObj.gcode
      }
      else {
        this.gcode = '';
      }
    }
  }

  static serialize(obj, buffer, bufferOffset) {
    // Serializes a message object of type SetGcodeCmdRequest
    // Serialize message field [gcode]
    bufferOffset = _serializer.string(obj.gcode, buffer, bufferOffset);
    return bufferOffset;
  }

  static deserialize(buffer, bufferOffset=[0]) {
    //deserializes a message object of type SetGcodeCmdRequest
    let len;
    let data = new SetGcodeCmdRequest(null);
    // Deserialize message field [gcode]
    data.gcode = _deserializer.string(buffer, bufferOffset);
    return data;
  }

  static getMessageSize(object) {
    let length = 0;
    length += _getByteLength(object.gcode);
    return length + 4;
  }

  static datatype() {
    // Returns string type for a service object
    return 'trajectory_planner/SetGcodeCmdRequest';
  }

  static md5sum() {
    //Returns md5sum for a message object
    return '0a46afe04400af260ebb86395c467524';
  }

  static messageDefinition() {
    // Returns full string definition for message
    return `
    string gcode
    
    `;
  }

  static Resolve(msg) {
    // deep-construct a valid message object instance of whatever was passed in
    if (typeof msg !== 'object' || msg === null) {
      msg = {};
    }
    const resolved = new SetGcodeCmdRequest(null);
    if (msg.gcode !== undefined) {
      resolved.gcode = msg.gcode;
    }
    else {
      resolved.gcode = ''
    }

    return resolved;
    }
};

class SetGcodeCmdResponse {
  constructor(initObj={}) {
    if (initObj === null) {
      // initObj === null is a special case for deserialization where we don't initialize fields
      this.result = null;
    }
    else {
      if (initObj.hasOwnProperty('result')) {
        this.result = initObj.result
      }
      else {
        this.result = 0;
      }
    }
  }

  static serialize(obj, buffer, bufferOffset) {
    // Serializes a message object of type SetGcodeCmdResponse
    // Serialize message field [result]
    bufferOffset = _serializer.int32(obj.result, buffer, bufferOffset);
    return bufferOffset;
  }

  static deserialize(buffer, bufferOffset=[0]) {
    //deserializes a message object of type SetGcodeCmdResponse
    let len;
    let data = new SetGcodeCmdResponse(null);
    // Deserialize message field [result]
    data.result = _deserializer.int32(buffer, bufferOffset);
    return data;
  }

  static getMessageSize(object) {
    return 4;
  }

  static datatype() {
    // Returns string type for a service object
    return 'trajectory_planner/SetGcodeCmdResponse';
  }

  static md5sum() {
    //Returns md5sum for a message object
    return '034a8e20d6a306665e3a5b340fab3f09';
  }

  static messageDefinition() {
    // Returns full string definition for message
    return `
    int32 result
    
    `;
  }

  static Resolve(msg) {
    // deep-construct a valid message object instance of whatever was passed in
    if (typeof msg !== 'object' || msg === null) {
      msg = {};
    }
    const resolved = new SetGcodeCmdResponse(null);
    if (msg.result !== undefined) {
      resolved.result = msg.result;
    }
    else {
      resolved.result = 0
    }

    return resolved;
    }
};

module.exports = {
  Request: SetGcodeCmdRequest,
  Response: SetGcodeCmdResponse,
  md5sum() { return '063984922b4c75aaeda3d4d697e63bac'; },
  datatype() { return 'trajectory_planner/SetGcodeCmd'; }
};
