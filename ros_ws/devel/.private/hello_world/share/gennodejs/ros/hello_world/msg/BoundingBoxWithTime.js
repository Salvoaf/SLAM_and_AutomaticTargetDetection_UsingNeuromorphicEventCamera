// Auto-generated. Do not edit!

// (in-package hello_world.msg)


"use strict";

const _serializer = _ros_msg_utils.Serialize;
const _arraySerializer = _serializer.Array;
const _deserializer = _ros_msg_utils.Deserialize;
const _arrayDeserializer = _deserializer.Array;
const _finder = _ros_msg_utils.Find;
const _getByteLength = _ros_msg_utils.getByteLength;

//-----------------------------------------------------------

class BoundingBoxWithTime {
  constructor(initObj={}) {
    if (initObj === null) {
      // initObj === null is a special case for deserialization where we don't initialize fields
      this.timestamp = null;
      this.x_offset = null;
      this.y_offset = null;
      this.height = null;
      this.width = null;
      this.do_rectify = null;
      this.class_id = null;
      this.class_confidence = null;
      this.track_id = null;
    }
    else {
      if (initObj.hasOwnProperty('timestamp')) {
        this.timestamp = initObj.timestamp
      }
      else {
        this.timestamp = 0.0;
      }
      if (initObj.hasOwnProperty('x_offset')) {
        this.x_offset = initObj.x_offset
      }
      else {
        this.x_offset = 0;
      }
      if (initObj.hasOwnProperty('y_offset')) {
        this.y_offset = initObj.y_offset
      }
      else {
        this.y_offset = 0;
      }
      if (initObj.hasOwnProperty('height')) {
        this.height = initObj.height
      }
      else {
        this.height = 0;
      }
      if (initObj.hasOwnProperty('width')) {
        this.width = initObj.width
      }
      else {
        this.width = 0;
      }
      if (initObj.hasOwnProperty('do_rectify')) {
        this.do_rectify = initObj.do_rectify
      }
      else {
        this.do_rectify = false;
      }
      if (initObj.hasOwnProperty('class_id')) {
        this.class_id = initObj.class_id
      }
      else {
        this.class_id = 0;
      }
      if (initObj.hasOwnProperty('class_confidence')) {
        this.class_confidence = initObj.class_confidence
      }
      else {
        this.class_confidence = 0.0;
      }
      if (initObj.hasOwnProperty('track_id')) {
        this.track_id = initObj.track_id
      }
      else {
        this.track_id = 0;
      }
    }
  }

  static serialize(obj, buffer, bufferOffset) {
    // Serializes a message object of type BoundingBoxWithTime
    // Serialize message field [timestamp]
    bufferOffset = _serializer.float64(obj.timestamp, buffer, bufferOffset);
    // Serialize message field [x_offset]
    bufferOffset = _serializer.int32(obj.x_offset, buffer, bufferOffset);
    // Serialize message field [y_offset]
    bufferOffset = _serializer.int32(obj.y_offset, buffer, bufferOffset);
    // Serialize message field [height]
    bufferOffset = _serializer.int32(obj.height, buffer, bufferOffset);
    // Serialize message field [width]
    bufferOffset = _serializer.int32(obj.width, buffer, bufferOffset);
    // Serialize message field [do_rectify]
    bufferOffset = _serializer.bool(obj.do_rectify, buffer, bufferOffset);
    // Serialize message field [class_id]
    bufferOffset = _serializer.int32(obj.class_id, buffer, bufferOffset);
    // Serialize message field [class_confidence]
    bufferOffset = _serializer.float32(obj.class_confidence, buffer, bufferOffset);
    // Serialize message field [track_id]
    bufferOffset = _serializer.int32(obj.track_id, buffer, bufferOffset);
    return bufferOffset;
  }

  static deserialize(buffer, bufferOffset=[0]) {
    //deserializes a message object of type BoundingBoxWithTime
    let len;
    let data = new BoundingBoxWithTime(null);
    // Deserialize message field [timestamp]
    data.timestamp = _deserializer.float64(buffer, bufferOffset);
    // Deserialize message field [x_offset]
    data.x_offset = _deserializer.int32(buffer, bufferOffset);
    // Deserialize message field [y_offset]
    data.y_offset = _deserializer.int32(buffer, bufferOffset);
    // Deserialize message field [height]
    data.height = _deserializer.int32(buffer, bufferOffset);
    // Deserialize message field [width]
    data.width = _deserializer.int32(buffer, bufferOffset);
    // Deserialize message field [do_rectify]
    data.do_rectify = _deserializer.bool(buffer, bufferOffset);
    // Deserialize message field [class_id]
    data.class_id = _deserializer.int32(buffer, bufferOffset);
    // Deserialize message field [class_confidence]
    data.class_confidence = _deserializer.float32(buffer, bufferOffset);
    // Deserialize message field [track_id]
    data.track_id = _deserializer.int32(buffer, bufferOffset);
    return data;
  }

  static getMessageSize(object) {
    return 37;
  }

  static datatype() {
    // Returns string type for a message object
    return 'hello_world/BoundingBoxWithTime';
  }

  static md5sum() {
    //Returns md5sum for a message object
    return 'c5698bfee7fb49c340d5d5281d1a2cca';
  }

  static messageDefinition() {
    // Returns full string definition for message
    return `
    float64 timestamp
    int32 x_offset
    int32 y_offset
    int32 height
    int32 width
    bool do_rectify
    int32 class_id
    float32 class_confidence
    int32 track_id
    
    `;
  }

  static Resolve(msg) {
    // deep-construct a valid message object instance of whatever was passed in
    if (typeof msg !== 'object' || msg === null) {
      msg = {};
    }
    const resolved = new BoundingBoxWithTime(null);
    if (msg.timestamp !== undefined) {
      resolved.timestamp = msg.timestamp;
    }
    else {
      resolved.timestamp = 0.0
    }

    if (msg.x_offset !== undefined) {
      resolved.x_offset = msg.x_offset;
    }
    else {
      resolved.x_offset = 0
    }

    if (msg.y_offset !== undefined) {
      resolved.y_offset = msg.y_offset;
    }
    else {
      resolved.y_offset = 0
    }

    if (msg.height !== undefined) {
      resolved.height = msg.height;
    }
    else {
      resolved.height = 0
    }

    if (msg.width !== undefined) {
      resolved.width = msg.width;
    }
    else {
      resolved.width = 0
    }

    if (msg.do_rectify !== undefined) {
      resolved.do_rectify = msg.do_rectify;
    }
    else {
      resolved.do_rectify = false
    }

    if (msg.class_id !== undefined) {
      resolved.class_id = msg.class_id;
    }
    else {
      resolved.class_id = 0
    }

    if (msg.class_confidence !== undefined) {
      resolved.class_confidence = msg.class_confidence;
    }
    else {
      resolved.class_confidence = 0.0
    }

    if (msg.track_id !== undefined) {
      resolved.track_id = msg.track_id;
    }
    else {
      resolved.track_id = 0
    }

    return resolved;
    }
};

module.exports = BoundingBoxWithTime;
