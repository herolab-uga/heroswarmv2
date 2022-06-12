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

class StringList {
  constructor(initObj={}) {
    if (initObj === null) {
      // initObj === null is a special case for deserialization where we don't initialize fields
      this.names = null;
    }
    else {
      if (initObj.hasOwnProperty('names')) {
        this.names = initObj.names
      }
      else {
        this.names = [];
      }
    }
  }

  static serialize(obj, buffer, bufferOffset) {
    // Serializes a message object of type StringList
    // Serialize message field [names]
    // Serialize the length for message field [names]
    bufferOffset = _serializer.uint32(obj.names.length, buffer, bufferOffset);
    obj.names.forEach((val) => {
      bufferOffset = std_msgs.msg.String.serialize(val, buffer, bufferOffset);
    });
    return bufferOffset;
  }

  static deserialize(buffer, bufferOffset=[0]) {
    //deserializes a message object of type StringList
    let len;
    let data = new StringList(null);
    // Deserialize message field [names]
    // Deserialize array length for message field [names]
    len = _deserializer.uint32(buffer, bufferOffset);
    data.names = new Array(len);
    for (let i = 0; i < len; ++i) {
      data.names[i] = std_msgs.msg.String.deserialize(buffer, bufferOffset)
    }
    return data;
  }

  static getMessageSize(object) {
    let length = 0;
    object.names.forEach((val) => {
      length += std_msgs.msg.String.getMessageSize(val);
    });
    return length + 4;
  }

  static datatype() {
    // Returns string type for a message object
    return 'robot_msgs/StringList';
  }

  static md5sum() {
    //Returns md5sum for a message object
    return '5334ccc36929d3443e2083f7204590bf';
  }

  static messageDefinition() {
    // Returns full string definition for message
    return `
    std_msgs/String[] names
    ================================================================================
    MSG: std_msgs/String
    string data
    
    `;
  }

  static Resolve(msg) {
    // deep-construct a valid message object instance of whatever was passed in
    if (typeof msg !== 'object' || msg === null) {
      msg = {};
    }
    const resolved = new StringList(null);
    if (msg.names !== undefined) {
      resolved.names = new Array(msg.names.length);
      for (let i = 0; i < resolved.names.length; ++i) {
        resolved.names[i] = std_msgs.msg.String.Resolve(msg.names[i]);
      }
    }
    else {
      resolved.names = []
    }

    return resolved;
    }
};

module.exports = StringList;
