// Auto-generated. Do not edit!

// (in-package beginner_tutorials.msg)


"use strict";

const _serializer = _ros_msg_utils.Serialize;
const _arraySerializer = _serializer.Array;
const _deserializer = _ros_msg_utils.Deserialize;
const _arrayDeserializer = _deserializer.Array;
const _finder = _ros_msg_utils.Find;
const _getByteLength = _ros_msg_utils.getByteLength;

//-----------------------------------------------------------

class Scan_range {
  constructor(initObj={}) {
    if (initObj === null) {
      // initObj === null is a special case for deserialization where we don't initialize fields
      this.maximum = null;
      this.minimum = null;
    }
    else {
      if (initObj.hasOwnProperty('maximum')) {
        this.maximum = initObj.maximum
      }
      else {
        this.maximum = 0.0;
      }
      if (initObj.hasOwnProperty('minimum')) {
        this.minimum = initObj.minimum
      }
      else {
        this.minimum = 0.0;
      }
    }
  }

  static serialize(obj, buffer, bufferOffset) {
    // Serializes a message object of type Scan_range
    // Serialize message field [maximum]
    bufferOffset = _serializer.float64(obj.maximum, buffer, bufferOffset);
    // Serialize message field [minimum]
    bufferOffset = _serializer.float64(obj.minimum, buffer, bufferOffset);
    return bufferOffset;
  }

  static deserialize(buffer, bufferOffset=[0]) {
    //deserializes a message object of type Scan_range
    let len;
    let data = new Scan_range(null);
    // Deserialize message field [maximum]
    data.maximum = _deserializer.float64(buffer, bufferOffset);
    // Deserialize message field [minimum]
    data.minimum = _deserializer.float64(buffer, bufferOffset);
    return data;
  }

  static getMessageSize(object) {
    return 16;
  }

  static datatype() {
    // Returns string type for a message object
    return 'beginner_tutorials/Scan_range';
  }

  static md5sum() {
    //Returns md5sum for a message object
    return '794426e8c6a60ff752d8c666c7346466';
  }

  static messageDefinition() {
    // Returns full string definition for message
    return `
    float64 maximum
    float64 minimum
    `;
  }

  static Resolve(msg) {
    // deep-construct a valid message object instance of whatever was passed in
    if (typeof msg !== 'object' || msg === null) {
      msg = {};
    }
    const resolved = new Scan_range(null);
    if (msg.maximum !== undefined) {
      resolved.maximum = msg.maximum;
    }
    else {
      resolved.maximum = 0.0
    }

    if (msg.minimum !== undefined) {
      resolved.minimum = msg.minimum;
    }
    else {
      resolved.minimum = 0.0
    }

    return resolved;
    }
};

module.exports = Scan_range;
