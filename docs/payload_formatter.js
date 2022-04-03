function sflt162f(rawSflt16)
    {
    // rawSflt16 is the 2-byte number decoded from wherever;
    // it's in range 0..0xFFFF
    // bit 15 is the sign bit
    // bits 14..11 are the exponent
    // bits 10..0 are the the mantissa. Unlike IEEE format,
    // the msb is explicit; this means that numbers
    // might not be normalized, but makes coding for
    // underflow easier.
    // As with IEEE format, negative zero is possible, so
    // we special-case that in hopes that JavaScript will
    // also cooperate.
    //
    // The result is a number in the open interval (-1.0, 1.0);
    //

    // throw away high bits for repeatability.
    rawSflt16 &= 0xFFFF;

    // special case minus zero:
    if (rawSflt16 == 0x8000)
        return -0.0;

    // extract the sign.
    var sSign = ((rawSflt16 & 0x8000) != 0) ? -1 : 1;

    // extract the exponent
    var exp1 = (rawSflt16 >> 11) & 0xF;

    // extract the "mantissa" (the fractional part)
    var mant1 = (rawSflt16 & 0x7FF) / 2048.0;

    // convert back to a floating point number. We hope
    // that Math.pow(2, k) is handled efficiently by
    // the JS interpreter! If this is time critical code,
    // you can replace by a suitable shift and divide.
    var f_unscaled = sSign * mant1 * Math.pow(2, exp1 - 15);

    return f_unscaled;
    }

function Decoder(bytes, port) {
  //Data example: 596FCD77C7566A66136D736D1E5D606C9C6E1D6F3A6F2C54
  var decoded = {};
  decoded.bytes = bytes;

  //Temperature and humidity
  rawTemp = bytes[0] + bytes[1] * 256;
  decoded.degreesC = sflt162f(rawTemp) * 100;

  rawHumid = bytes[2] + bytes[3] * 256;
  decoded.humidity = sflt162f(rawHumid) * 100;

  //Particle mass concontration
  pm1 = bytes[4] + bytes[5] * 256;
  decoded.pm1 = sflt162f(pm1) * 100;

  pm2 = bytes[6] + bytes[7] * 256;
  decoded.pm2 = sflt162f(pm2) * 100;

  pm4 = bytes[8] + bytes[9] * 256;
  decoded.pm4 = sflt162f(pm4) * 100;

  pm10 = bytes[10] + bytes[11] * 256;
  decoded.pm10 = sflt162f(pm10) * 100;

  //Particle number concontration
  nc05 = bytes[12] + bytes[13] * 256;
  decoded.nc05 = sflt162f(nc05) * 100;

  nc10 = bytes[14] + bytes[15] * 256;
  decoded.nc05 = sflt162f(nc10) * 100;

  nc25 = bytes[16] + bytes[17] * 256;
  decoded.nc25 = sflt162f(nc25) * 100;

  nc40 = bytes[18] + bytes[19] * 256;
  decoded.nc40 = sflt162f(nc40) * 100;

  nc10p0 = bytes[20] + bytes[21] * 256;
  decoded.nc10p0 = sflt162f(nc10p0) * 100;

  typical = bytes[22] + bytes[23] * 256;
  decoded.typical = sflt162f(typical) * 100;

  return decoded;
}
