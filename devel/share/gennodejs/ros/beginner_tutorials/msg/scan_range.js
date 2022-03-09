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

class scan_range {
  constructor(initObj={}) {
    if (initObj === null) {
      // initObj === null is a special case for deserialization where we don't initialize fields
      this.min = null;
      this.max = null;
    }
    else {
      if (initObj.hasOwnProperty('min')) {
        this.min = initObj.min
      }
      else {
        this.min = 0;
      }
      if (initObj.hasOwnProperty('max')) {
        this.max = initObj.max
      }
      else {
        this.max = 0;
      }
    }
  }

  static serialize(obj, buffer, bufferOffset) {
    // Serializes a message object of type scan_range
    // Serialize message field [min]
    bufferOffset = _serializer.int64(obj.min, buffer, bufferOffset);
    // Serialize message field [max]
    bufferOffset = _serializer.int64(obj.max, buffer, bufferOffset);
    return bufferOffset;
  }

  static deserialize(buffer, bufferOffset=[0]) {
    //deserializes a message object of type scan_range
    let len;
    let data = new scan_range(null);
    // Deserialize message field [min]
    data.min = _deserializer.int64(buffer, bufferOffset);
    // Deserialize message field [max]
    data.max = _deserializer.int64(buffer, bufferOffset);
    return data;
  }

  static getMessageSize(object) {
    return 16;
  }

  static datatype() {
    // Returns string type for a message object
    return 'beginner_tutorials/scan_range';
  }

  static md5sum() {
    //Returns md5sum for a message object
    return 'a1d10007dc73ba7537d4cef82691d404';
  }

  static messageDefinition() {
    // Returns full string definition for message
    return `
    int64 min
    int64 max
    
    `;
  }

  static Resolve(msg) {
    // deep-construct a valid message object instance of whatever was passed in
    if (typeof msg !== 'object' || msg === null) {
      msg = {};
    }
    const resolved = new scan_range(null);
    if (msg.min !== undefined) {
      resolved.min = msg.min;
    }
    else {
      resolved.min = 0
    }

    if (msg.max !== undefined) {
      resolved.max = msg.max;
    }
    else {
      resolved.max = 0
    }

    return resolved;
    }
};

module.exports = scan_range;
