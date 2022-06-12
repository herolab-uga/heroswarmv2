// Auto-generated. Do not edit!

// (in-package robot_msgs.msg)


"use strict";

const _serializer = _ros_msg_utils.Serialize;
const _arraySerializer = _serializer.Array;
const _deserializer = _ros_msg_utils.Deserialize;
const _arrayDeserializer = _deserializer.Array;
const _finder = _ros_msg_utils.Find;
const _getByteLength = _ros_msg_utils.getByteLength;

//-----------------------------------------------------------

class Light {
  constructor(initObj={}) {
    if (initObj === null) {
      // initObj === null is a special case for deserialization where we don't initialize fields
      this.rgbw = null;
      this.gesture = null;
    }
    else {
      if (initObj.hasOwnProperty('rgbw')) {
        this.rgbw = initObj.rgbw
      }
      else {
        this.rgbw = [];
      }
      if (initObj.hasOwnProperty('gesture')) {
        this.gesture = initObj.gesture
      }
      else {
        this.gesture = 0;
      }
    }
  }

  static serialize(obj, buffer, bufferOffset) {
    // Serializes a message object of type Light
    // Serialize message field [rgbw]
    bufferOffset = _arraySerializer.int32(obj.rgbw, buffer, bufferOffset, null);
    // Serialize message field [gesture]
    bufferOffset = _serializer.int32(obj.gesture, buffer, bufferOffset);
    return bufferOffset;
  }

  static deserialize(buffer, bufferOffset=[0]) {
    //deserializes a message object of type Light
    let len;
    let data = new Light(null);
    // Deserialize message field [rgbw]
    data.rgbw = _arrayDeserializer.int32(buffer, bufferOffset, null)
    // Deserialize message field [gesture]
    data.gesture = _deserializer.int32(buffer, bufferOffset);
    return data;
  }

  static getMessageSize(object) {
    let length = 0;
    length += 4 * object.rgbw.length;
    return length + 8;
  }

  static datatype() {
    // Returns string type for a message object
    return 'robot_msgs/Light';
  }

  static md5sum() {
    //Returns md5sum for a message object
    return 'ba54a502272d86cc6063f7ba6b342cf5';
  }

  static messageDefinition() {
    // Returns full string definition for message
    return `
    int32[] rgbw
    int32 gesture
    `;
  }

  static Resolve(msg) {
    // deep-construct a valid message object instance of whatever was passed in
    if (typeof msg !== 'object' || msg === null) {
      msg = {};
    }
    const resolved = new Light(null);
    if (msg.rgbw !== undefined) {
      resolved.rgbw = msg.rgbw;
    }
    else {
      resolved.rgbw = []
    }

    if (msg.gesture !== undefined) {
      resolved.gesture = msg.gesture;
    }
    else {
      resolved.gesture = 0
    }

    return resolved;
    }
};

module.exports = Light;
