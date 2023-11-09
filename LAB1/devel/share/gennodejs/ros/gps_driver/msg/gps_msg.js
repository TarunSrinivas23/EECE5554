// Auto-generated. Do not edit!

// (in-package gps_driver.msg)


"use strict";

const _serializer = _ros_msg_utils.Serialize;
const _arraySerializer = _serializer.Array;
const _deserializer = _ros_msg_utils.Deserialize;
const _arrayDeserializer = _deserializer.Array;
const _finder = _ros_msg_utils.Find;
const _getByteLength = _ros_msg_utils.getByteLength;
let std_msgs = _finder('std_msgs');

//-----------------------------------------------------------

class gps_msg {
  constructor(initObj={}) {
    if (initObj === null) {
      // initObj === null is a special case for deserialization where we don't initialize fields
      this.header = null;
      this.latitude = null;
      this.longitude = null;
      this.altitude = null;
      this.utm_easting = null;
      this.utm_northing = null;
      this.utc = null;
      this.zone = null;
      this.zone_letter = null;
    }
    else {
      if (initObj.hasOwnProperty('header')) {
        this.header = initObj.header
      }
      else {
        this.header = new std_msgs.msg.Header();
      }
      if (initObj.hasOwnProperty('latitude')) {
        this.latitude = initObj.latitude
      }
      else {
        this.latitude = 0.0;
      }
      if (initObj.hasOwnProperty('longitude')) {
        this.longitude = initObj.longitude
      }
      else {
        this.longitude = 0.0;
      }
      if (initObj.hasOwnProperty('altitude')) {
        this.altitude = initObj.altitude
      }
      else {
        this.altitude = 0.0;
      }
      if (initObj.hasOwnProperty('utm_easting')) {
        this.utm_easting = initObj.utm_easting
      }
      else {
        this.utm_easting = 0.0;
      }
      if (initObj.hasOwnProperty('utm_northing')) {
        this.utm_northing = initObj.utm_northing
      }
      else {
        this.utm_northing = 0.0;
      }
      if (initObj.hasOwnProperty('utc')) {
        this.utc = initObj.utc
      }
      else {
        this.utc = 0.0;
      }
      if (initObj.hasOwnProperty('zone')) {
        this.zone = initObj.zone
      }
      else {
        this.zone = 0;
      }
      if (initObj.hasOwnProperty('zone_letter')) {
        this.zone_letter = initObj.zone_letter
      }
      else {
        this.zone_letter = '';
      }
    }
  }

  static serialize(obj, buffer, bufferOffset) {
    // Serializes a message object of type gps_msg
    // Serialize message field [header]
    bufferOffset = std_msgs.msg.Header.serialize(obj.header, buffer, bufferOffset);
    // Serialize message field [latitude]
    bufferOffset = _serializer.float32(obj.latitude, buffer, bufferOffset);
    // Serialize message field [longitude]
    bufferOffset = _serializer.float32(obj.longitude, buffer, bufferOffset);
    // Serialize message field [altitude]
    bufferOffset = _serializer.float32(obj.altitude, buffer, bufferOffset);
    // Serialize message field [utm_easting]
    bufferOffset = _serializer.float32(obj.utm_easting, buffer, bufferOffset);
    // Serialize message field [utm_northing]
    bufferOffset = _serializer.float32(obj.utm_northing, buffer, bufferOffset);
    // Serialize message field [utc]
    bufferOffset = _serializer.float64(obj.utc, buffer, bufferOffset);
    // Serialize message field [zone]
    bufferOffset = _serializer.int32(obj.zone, buffer, bufferOffset);
    // Serialize message field [zone_letter]
    bufferOffset = _serializer.string(obj.zone_letter, buffer, bufferOffset);
    return bufferOffset;
  }

  static deserialize(buffer, bufferOffset=[0]) {
    //deserializes a message object of type gps_msg
    let len;
    let data = new gps_msg(null);
    // Deserialize message field [header]
    data.header = std_msgs.msg.Header.deserialize(buffer, bufferOffset);
    // Deserialize message field [latitude]
    data.latitude = _deserializer.float32(buffer, bufferOffset);
    // Deserialize message field [longitude]
    data.longitude = _deserializer.float32(buffer, bufferOffset);
    // Deserialize message field [altitude]
    data.altitude = _deserializer.float32(buffer, bufferOffset);
    // Deserialize message field [utm_easting]
    data.utm_easting = _deserializer.float32(buffer, bufferOffset);
    // Deserialize message field [utm_northing]
    data.utm_northing = _deserializer.float32(buffer, bufferOffset);
    // Deserialize message field [utc]
    data.utc = _deserializer.float64(buffer, bufferOffset);
    // Deserialize message field [zone]
    data.zone = _deserializer.int32(buffer, bufferOffset);
    // Deserialize message field [zone_letter]
    data.zone_letter = _deserializer.string(buffer, bufferOffset);
    return data;
  }

  static getMessageSize(object) {
    let length = 0;
    length += std_msgs.msg.Header.getMessageSize(object.header);
    length += _getByteLength(object.zone_letter);
    return length + 36;
  }

  static datatype() {
    // Returns string type for a message object
    return 'gps_driver/gps_msg';
  }

  static md5sum() {
    //Returns md5sum for a message object
    return '580f849d52a91f0e46ed47841b1b5c11';
  }

  static messageDefinition() {
    // Returns full string definition for message
    return `
    Header header
    float32 latitude
    float32 longitude
    float32 altitude
    float32 utm_easting
    float32 utm_northing
    float64 utc
    int32 zone
    string zone_letter
    ================================================================================
    MSG: std_msgs/Header
    # Standard metadata for higher-level stamped data types.
    # This is generally used to communicate timestamped data 
    # in a particular coordinate frame.
    # 
    # sequence ID: consecutively increasing ID 
    uint32 seq
    #Two-integer timestamp that is expressed as:
    # * stamp.sec: seconds (stamp_secs) since epoch (in Python the variable is called 'secs')
    # * stamp.nsec: nanoseconds since stamp_secs (in Python the variable is called 'nsecs')
    # time-handling sugar is provided by the client library
    time stamp
    #Frame this data is associated with
    string frame_id
    
    `;
  }

  static Resolve(msg) {
    // deep-construct a valid message object instance of whatever was passed in
    if (typeof msg !== 'object' || msg === null) {
      msg = {};
    }
    const resolved = new gps_msg(null);
    if (msg.header !== undefined) {
      resolved.header = std_msgs.msg.Header.Resolve(msg.header)
    }
    else {
      resolved.header = new std_msgs.msg.Header()
    }

    if (msg.latitude !== undefined) {
      resolved.latitude = msg.latitude;
    }
    else {
      resolved.latitude = 0.0
    }

    if (msg.longitude !== undefined) {
      resolved.longitude = msg.longitude;
    }
    else {
      resolved.longitude = 0.0
    }

    if (msg.altitude !== undefined) {
      resolved.altitude = msg.altitude;
    }
    else {
      resolved.altitude = 0.0
    }

    if (msg.utm_easting !== undefined) {
      resolved.utm_easting = msg.utm_easting;
    }
    else {
      resolved.utm_easting = 0.0
    }

    if (msg.utm_northing !== undefined) {
      resolved.utm_northing = msg.utm_northing;
    }
    else {
      resolved.utm_northing = 0.0
    }

    if (msg.utc !== undefined) {
      resolved.utc = msg.utc;
    }
    else {
      resolved.utc = 0.0
    }

    if (msg.zone !== undefined) {
      resolved.zone = msg.zone;
    }
    else {
      resolved.zone = 0
    }

    if (msg.zone_letter !== undefined) {
      resolved.zone_letter = msg.zone_letter;
    }
    else {
      resolved.zone_letter = ''
    }

    return resolved;
    }
};

module.exports = gps_msg;
