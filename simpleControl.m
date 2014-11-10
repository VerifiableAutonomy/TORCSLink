%% Create UDP Connection
U = udp('127.0.0.1',8888);
fopen(U);
bufOut = zeros(20,1);

%% Send zeros to get initial data back
fwrite(U,bufOut);
bufIn = uint8(fread(U));
dist = typecast(bufIn,'single');

%% Send a control signal
accel = single(0.2);
brake = single(0);
clutch = single(0);
gear = int32(1);
steer = single(0);

bufOut(1:4) = typecast(accel,'uint8');
bufOut(5:8) = typecast(brake,'uint8');
bufOut(9:12) = typecast(clutch,'uint8');
bufOut(13:16) = typecast(gear,'uint8');
bufOut(17:20) = typecast(steer,'uint8');
fwrite(U,bufOut);
bufIn = uint8(fread(U));
dist = typecast(bufIn(1:4),'single')
speed = typecast(bufIn(5:8),'single')
angle = typecast(bufIn(9:12),'single')
latErr = typecast(bufIn(13:16),'single')
radius = typecast(bufIn(17:20),'single')