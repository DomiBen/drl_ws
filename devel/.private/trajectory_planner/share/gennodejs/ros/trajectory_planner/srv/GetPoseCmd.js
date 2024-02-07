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

class GetPoseCmdRequest {
  constructor(initObj={}) {
    if (initObj === null) {
      // initObj === null is a special case for deserialization where we don't initialize fields
    }
    else {
    }
  }

  static serialize(obj, buffer, bufferOffset) {
    // Serializes a message object of type GetPoseCmdRequest
    return bufferOffset;
  }

  static deserialize(buffer, bufferOffset=[0]) {
    //deserializes a message object of type GetPoseCmdRequest
    let len;
    let data = new GetPoseCmdRequest(null);
    return data;
  }

  static getMessageSize(object) {
    return 0;
  }

  static datatype() {
    // Returns string type for a service object
    return 'trajectory_planner/GetPoseCmdRequest';
  }

  static md5sum() {
    //Returns md5sum for a message object
    return 'd41d8cd98f00b204e9800998ecf8427e';
  }

  static messageDefinition() {
    // Returns full string definition for message
    return `
    
    `;
  }

  static Resolve(msg) {
    // deep-construct a valid message object instance of whatever was passed in
    if (typeof msg !== 'object' || msg === null) {
      msg = {};
    }
    const resolved = new GetPoseCmdRequest(null);
    return resolved;
    }
};

class GetPoseCmdResponse {
  constructor(initObj={}) {
    if (initObj === null) {
      // initObj === null is a special case for deserialization where we don't initialize fields
      this.result = null;
      this.state = null;
      this.x = null;
      this.y = null;
      this.z = null;
      this.a = null;
      this.b = null;
      this.c = null;
      this.jointAngle_1 = null;
      this.jointAngle_2 = null;
      this.jointAngle_3 = null;
      this.jointAngle_4 = null;
      this.jointAngle_5 = null;
      this.jointAngle_6 = null;
      this.jointAngle_7 = null;
    }
    else {
      if (initObj.hasOwnProperty('result')) {
        this.result = initObj.result
      }
      else {
        this.result = 0;
      }
      if (initObj.hasOwnProperty('state')) {
        this.state = initObj.state
      }
      else {
        this.state = 0;
      }
      if (initObj.hasOwnProperty('x')) {
        this.x = initObj.x
      }
      else {
        this.x = 0.0;
      }
      if (initObj.hasOwnProperty('y')) {
        this.y = initObj.y
      }
      else {
        this.y = 0.0;
      }
      if (initObj.hasOwnProperty('z')) {
        this.z = initObj.z
      }
      else {
        this.z = 0.0;
      }
      if (initObj.hasOwnProperty('a')) {
        this.a = initObj.a
      }
      else {
        this.a = 0.0;
      }
      if (initObj.hasOwnProperty('b')) {
        this.b = initObj.b
      }
      else {
        this.b = 0.0;
      }
      if (initObj.hasOwnProperty('c')) {
        this.c = initObj.c
      }
      else {
        this.c = 0.0;
      }
      if (initObj.hasOwnProperty('jointAngle_1')) {
        this.jointAngle_1 = initObj.jointAngle_1
      }
      else {
        this.jointAngle_1 = 0.0;
      }
      if (initObj.hasOwnProperty('jointAngle_2')) {
        this.jointAngle_2 = initObj.jointAngle_2
      }
      else {
        this.jointAngle_2 = 0.0;
      }
      if (initObj.hasOwnProperty('jointAngle_3')) {
        this.jointAngle_3 = initObj.jointAngle_3
      }
      else {
        this.jointAngle_3 = 0.0;
      }
      if (initObj.hasOwnProperty('jointAngle_4')) {
        this.jointAngle_4 = initObj.jointAngle_4
      }
      else {
        this.jointAngle_4 = 0.0;
      }
      if (initObj.hasOwnProperty('jointAngle_5')) {
        this.jointAngle_5 = initObj.jointAngle_5
      }
      else {
        this.jointAngle_5 = 0.0;
      }
      if (initObj.hasOwnProperty('jointAngle_6')) {
        this.jointAngle_6 = initObj.jointAngle_6
      }
      else {
        this.jointAngle_6 = 0.0;
      }
      if (initObj.hasOwnProperty('jointAngle_7')) {
        this.jointAngle_7 = initObj.jointAngle_7
      }
      else {
        this.jointAngle_7 = 0.0;
      }
    }
  }

  static serialize(obj, buffer, bufferOffset) {
    // Serializes a message object of type GetPoseCmdResponse
    // Serialize message field [result]
    bufferOffset = _serializer.int32(obj.result, buffer, bufferOffset);
    // Serialize message field [state]
    bufferOffset = _serializer.int32(obj.state, buffer, bufferOffset);
    // Serialize message field [x]
    bufferOffset = _serializer.float32(obj.x, buffer, bufferOffset);
    // Serialize message field [y]
    bufferOffset = _serializer.float32(obj.y, buffer, bufferOffset);
    // Serialize message field [z]
    bufferOffset = _serializer.float32(obj.z, buffer, bufferOffset);
    // Serialize message field [a]
    bufferOffset = _serializer.float32(obj.a, buffer, bufferOffset);
    // Serialize message field [b]
    bufferOffset = _serializer.float32(obj.b, buffer, bufferOffset);
    // Serialize message field [c]
    bufferOffset = _serializer.float32(obj.c, buffer, bufferOffset);
    // Serialize message field [jointAngle_1]
    bufferOffset = _serializer.float32(obj.jointAngle_1, buffer, bufferOffset);
    // Serialize message field [jointAngle_2]
    bufferOffset = _serializer.float32(obj.jointAngle_2, buffer, bufferOffset);
    // Serialize message field [jointAngle_3]
    bufferOffset = _serializer.float32(obj.jointAngle_3, buffer, bufferOffset);
    // Serialize message field [jointAngle_4]
    bufferOffset = _serializer.float32(obj.jointAngle_4, buffer, bufferOffset);
    // Serialize message field [jointAngle_5]
    bufferOffset = _serializer.float32(obj.jointAngle_5, buffer, bufferOffset);
    // Serialize message field [jointAngle_6]
    bufferOffset = _serializer.float32(obj.jointAngle_6, buffer, bufferOffset);
    // Serialize message field [jointAngle_7]
    bufferOffset = _serializer.float32(obj.jointAngle_7, buffer, bufferOffset);
    return bufferOffset;
  }

  static deserialize(buffer, bufferOffset=[0]) {
    //deserializes a message object of type GetPoseCmdResponse
    let len;
    let data = new GetPoseCmdResponse(null);
    // Deserialize message field [result]
    data.result = _deserializer.int32(buffer, bufferOffset);
    // Deserialize message field [state]
    data.state = _deserializer.int32(buffer, bufferOffset);
    // Deserialize message field [x]
    data.x = _deserializer.float32(buffer, bufferOffset);
    // Deserialize message field [y]
    data.y = _deserializer.float32(buffer, bufferOffset);
    // Deserialize message field [z]
    data.z = _deserializer.float32(buffer, bufferOffset);
    // Deserialize message field [a]
    data.a = _deserializer.float32(buffer, bufferOffset);
    // Deserialize message field [b]
    data.b = _deserializer.float32(buffer, bufferOffset);
    // Deserialize message field [c]
    data.c = _deserializer.float32(buffer, bufferOffset);
    // Deserialize message field [jointAngle_1]
    data.jointAngle_1 = _deserializer.float32(buffer, bufferOffset);
    // Deserialize message field [jointAngle_2]
    data.jointAngle_2 = _deserializer.float32(buffer, bufferOffset);
    // Deserialize message field [jointAngle_3]
    data.jointAngle_3 = _deserializer.float32(buffer, bufferOffset);
    // Deserialize message field [jointAngle_4]
    data.jointAngle_4 = _deserializer.float32(buffer, bufferOffset);
    // Deserialize message field [jointAngle_5]
    data.jointAngle_5 = _deserializer.float32(buffer, bufferOffset);
    // Deserialize message field [jointAngle_6]
    data.jointAngle_6 = _deserializer.float32(buffer, bufferOffset);
    // Deserialize message field [jointAngle_7]
    data.jointAngle_7 = _deserializer.float32(buffer, bufferOffset);
    return data;
  }

  static getMessageSize(object) {
    return 60;
  }

  static datatype() {
    // Returns string type for a service object
    return 'trajectory_planner/GetPoseCmdResponse';
  }

  static md5sum() {
    //Returns md5sum for a message object
    return '5aef3792e74d728815d4dfbf8b1f490f';
  }

  static messageDefinition() {
    // Returns full string definition for message
    return `
    int32 result
    int32 state
    float32 x
    float32 y
    float32 z
    float32 a
    float32 b
    float32 c
    float32 jointAngle_1
    float32 jointAngle_2
    float32 jointAngle_3
    float32 jointAngle_4
    float32 jointAngle_5
    float32 jointAngle_6
    float32 jointAngle_7
    
    
    `;
  }

  static Resolve(msg) {
    // deep-construct a valid message object instance of whatever was passed in
    if (typeof msg !== 'object' || msg === null) {
      msg = {};
    }
    const resolved = new GetPoseCmdResponse(null);
    if (msg.result !== undefined) {
      resolved.result = msg.result;
    }
    else {
      resolved.result = 0
    }

    if (msg.state !== undefined) {
      resolved.state = msg.state;
    }
    else {
      resolved.state = 0
    }

    if (msg.x !== undefined) {
      resolved.x = msg.x;
    }
    else {
      resolved.x = 0.0
    }

    if (msg.y !== undefined) {
      resolved.y = msg.y;
    }
    else {
      resolved.y = 0.0
    }

    if (msg.z !== undefined) {
      resolved.z = msg.z;
    }
    else {
      resolved.z = 0.0
    }

    if (msg.a !== undefined) {
      resolved.a = msg.a;
    }
    else {
      resolved.a = 0.0
    }

    if (msg.b !== undefined) {
      resolved.b = msg.b;
    }
    else {
      resolved.b = 0.0
    }

    if (msg.c !== undefined) {
      resolved.c = msg.c;
    }
    else {
      resolved.c = 0.0
    }

    if (msg.jointAngle_1 !== undefined) {
      resolved.jointAngle_1 = msg.jointAngle_1;
    }
    else {
      resolved.jointAngle_1 = 0.0
    }

    if (msg.jointAngle_2 !== undefined) {
      resolved.jointAngle_2 = msg.jointAngle_2;
    }
    else {
      resolved.jointAngle_2 = 0.0
    }

    if (msg.jointAngle_3 !== undefined) {
      resolved.jointAngle_3 = msg.jointAngle_3;
    }
    else {
      resolved.jointAngle_3 = 0.0
    }

    if (msg.jointAngle_4 !== undefined) {
      resolved.jointAngle_4 = msg.jointAngle_4;
    }
    else {
      resolved.jointAngle_4 = 0.0
    }

    if (msg.jointAngle_5 !== undefined) {
      resolved.jointAngle_5 = msg.jointAngle_5;
    }
    else {
      resolved.jointAngle_5 = 0.0
    }

    if (msg.jointAngle_6 !== undefined) {
      resolved.jointAngle_6 = msg.jointAngle_6;
    }
    else {
      resolved.jointAngle_6 = 0.0
    }

    if (msg.jointAngle_7 !== undefined) {
      resolved.jointAngle_7 = msg.jointAngle_7;
    }
    else {
      resolved.jointAngle_7 = 0.0
    }

    return resolved;
    }
};

module.exports = {
  Request: GetPoseCmdRequest,
  Response: GetPoseCmdResponse,
  md5sum() { return '5aef3792e74d728815d4dfbf8b1f490f'; },
  datatype() { return 'trajectory_planner/GetPoseCmd'; }
};
