function sensor_val = afe_convert_sensor_data(msb, midb, lsd)

data = msb *2^16 + midb * 2^8+ lsd; %高字节在前；
symbol = bitand(  uint32( floor(data / 2^21) ), 7); %     (sensor_val>>21) & 0x07

%temp = 0;
if 0 == symbol
    	temp = typecast(bitand(uint32(data), uint32(hex2dec('1FFFFF')) ), 'int32') ; %  data&0x1FFFFF;
elseif 7 == symbol
        temp = typecast(bitor(uint32(data), uint32(hex2dec('FF000000')) ), 'int32');
elseif 1 == symbol
        temp = typecast(hex2dec('200000'), 'int32');
elseif 6 == symbol
        temp = typecast( hex2dec('200000'), 'int32');
        temp = -temp;
end

 sensor_val = temp;
