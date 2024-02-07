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

class SetJointCmdRequest {
  constructor(initObj={}) {
    if (initObj === null) {
      // initObj === null is a special case for deserialization where we don't initialize fields
      this.jointAngle_1 = null;
      this.jointAngle_2 = null;
      this.jointAngle_3 = null;
      this.jointAngle_4 = null;
      this.jointAngle_5 = null;
      this.jointAngle_6 = null;
      this.speed = null;
    }
    else {
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
      if (initObj.hasOwnProperty('speed')) {
        this.speed = initObj.speed
      }
      else {
        this.speed = 0;
      }
    }
  }

  static serialize(obj, buffer, bufferOffset) {
    // Serializes a message object of type SetJointCmdRequest
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
    // Serialize message field [speed]
    bufferOffset = _serializer.int32(obj.speed, buffer, bufferOffset);
    return bufferOffset;
  }

  static deserialize(buffer, bufferOffset=[0]) {
    //deserializes a message object of type SetJointCmdRequest
    let len;
    let data = new SetJointCmdRequest(null);
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
    // Deserialize message field [speed]
    data.speed = _deserializer.int32(buffer, bufferOffset);
    return data;
  }

  static getMessageSize(object) {
    return 28;
  }

  static datatype() {
    // Returns string type for a service object
    return 'trajectory_planner/SetJointCmdRequest';
  }

  static md5sum() {
    //Returns md5sum for a message object
    return '97214505c9bd75117d40d909d9f08351';
  }

  static messageDefinition() {
    // Returns full string definition for message
    return `
    float32 jointAngle_1 
    float32 jointAngle_2 
    float32 jointAngle_3 
    float32 jointAngle_4 
    float32 jointAngle_5 
    float32 jointAngle_6 
    int32 speed 
    
    `;
  }

  static Resolve(msg) {
    // deep-construct a valid message object instance of whatever was passed in
    if (typeof msg !== 'object' || msg === null) {
      msg = {};
    }
    const resolved = new SetJointCmdRequest(null);
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

    if (msg.speed !== undefined) {
      resolved.speed = msg.speed;
    }
    else {
      resolved.speed = 0
    }

    return resolved;
    }
};

class SetJointCmdResponse {
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
    // Serializes a message object of type SetJointCmdResponse
    // Serialize message field [result]
    bufferOffset = _serializer.int32(obj.result, buffer, bufferOffset);
    return bufferOffset;
  }

  static deserialize(buffer, bufferOffset=[0]) {
    //deserializes a message object of type SetJointCmdResponse
    let len;
    let data = new SetJointCmdResponse(null);
    // Deserialize message field [result]
    data.result = _deserializer.int32(buffer, bufferOffset);
    return data;
  }

  static getMessageSize(object) {
    return 4;
  }

  static datatype() {
    // Returns string type for a service object
    return 'trajectory_planner/SetJointCmdResponse';
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
    const resolved = new SetJointCmdResponse(null);
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
  Request: SetJointCmdRequest,
  Response: SetJointCmdResponse,
  md5sum() { return 'b5f68cbcaa5e7a8e8c34ee9959541272'; },
  datatype() { return 'trajectory_planner/SetJointCmd'; }
};
