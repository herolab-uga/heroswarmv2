// Auto-generated. Do not edit!

// (in-package robot_msgs.msg)


"use strict";

const _serializer = _ros_msg_utils.Serialize;
const _arraySerializer = _serializer.Array;
const _deserializer = _ros_msg_utils.Deserialize;
const _arrayDeserializer = _deserializer.Array;
const _finder = _ros_msg_utils.Find;
const _getByteLength = _ros_msg_utils.getByteLength;
let std_msgs = _finder('std_msgs');

//-----------------------------------------------------------

class Name {
  constructor(initObj={}) {
    if (initObj === null) {
      // initObj === null is a special case for deserialization where we don't initialize fields
      this.name = null;
      this.time = null;
    }
    else {
      if (initObj.hasOwnProperty('name')) {
        this.name = initObj.name
      }
      else {
        this.name = new std_msgs.msg.String();
      }
      if (initObj.hasOwnProperty('time')) {
        this.time = initObj.time
      }
      else {
        this.time = new std_msgs.msg.UInt64();
      }
    }
  }

  static serialize(obj, buffer, bufferOffset) {
    // Serializes a message object of type Name
    // Serialize message field [name]
    bufferOffset = std_msgs.msg.String.serialize(obj.name, buffer, bufferOffset);
    // Serialize message field [time]
    bufferOffset = std_msgs.msg.UInt64.serialize(obj.time, buffer, bufferOffset);
    return bufferOffset;
  }

  static deserialize(buffer, bufferOffset=[0]) {
    //deserializes a message object of type Name
    let len;
    let data = new Name(null);
    // Deserialize message field [name]
    data.name = std_msgs.msg.String.deserialize(buffer, bufferOffset);
    // Deserialize message field [time]
    data.time = std_msgs.msg.UInt64.deserialize(buffer, bufferOffset);
    return data;
  }

  static getMessageSize(object) {
    let length = 0;
    length += std_msgs.msg.String.getMessageSize(object.name);
    return length + 8;
  }

  static datatype() {
    // Returns string type for a message object
    return 'robot_msgs/Name';
  }

  static md5sum() {
    //Returns md5sum for a message object
    return 'f8b4d9de593bc07202bdba9902cdb0a6';
  }

  static messageDefinition() {
    // Returns full string definition for message
    return `
    std_msgs/String name
    std_msgs/UInt64 time
    ================================================================================
    MSG: std_msgs/String
    string data
    
    ================================================================================
    MSG: std_msgs/UInt64
    uint64 data
    `;
  }

  static Resolve(msg) {
    // deep-construct a valid message object instance of whatever was passed in
    if (typeof msg !== 'object' || msg === null) {
      msg = {};
    }
    const resolved = new Name(null);
    if (msg.name !== undefined) {
      resolved.name = std_msgs.msg.String.Resolve(msg.name)
    }
    else {
      resolved.name = new std_msgs.msg.String()
    }

    if (msg.time !== undefined) {
      resolved.time = std_msgs.msg.UInt64.Resolve(msg.time)
    }
    else {
      resolved.time = new std_msgs.msg.UInt64()
    }

    return resolved;
    }
};

module.exports = Name;
