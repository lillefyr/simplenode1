function convert (num) {
    sign = 1;
    if ( num >= 0x80000000 ) { sign = -1; }
    expo = ((num & 0x7F800000) >> 23) - 127;
    mant =  (num & 0x007FFFFF | 0x00800000);
    expo2 = 2;
    for (i=1; i<expo; i++) {
      expo2 = expo2 * 2;
    }
    return (sign * expo2 * ( mant / (1 << 23)));
}


function Decoder(bytes, port) {
  // Decode an uplink message from a buffer
  // (array) of bytes to an object of fields.

  var location = {};
  
  location.lat       = convert((bytes[0]<<24)+(bytes[1]<<16)+(bytes[2]<<8) +bytes[3]);
  location.lon       = convert((bytes[4]*256*256*256)+(bytes[5]*256*256)+(bytes[6]*256) +bytes[7]);
  location.alt       = convert((bytes[8]*256*256*256)+(bytes[9]*256*256)+(bytes[10]*256)+bytes[11]);
  location.datetime  =         (bytes[12]*256*256*256)+(bytes[13]*256*256)+(bytes[14]*256)+bytes[15];
  location.thetime   = location.datetime % 10000;
  location.date      = ( location.datetime - location.thetime ) / 10000;
  location.hdop      = bytes[16] / 10;
  location.latitude  = location.lat;
  location.longitude = location.lon;
  location.altitude  = location.alt;

  return location;
}
