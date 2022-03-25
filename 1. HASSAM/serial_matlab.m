% %% Serial data read
% device = serialport("COM3", 14400);
% r = read(device, 6, "uint8");
% hexStr = dec2hex(r)



%% write
clc
clear
close all
delete(instrfindall)

s = serialport("COM5", 115200);
configureTerminator(s,"CR");

%write(s, 0xaa, "uint8");
%out = read(s, 1, "uint8") 
%r = dec2hex(out)

degree = 100; % dynamixel angle (0~300)

while(1)
    
    angle = (1023*degree) / 300; 
    angle = cast(angle, 'uint16'); 

    data = [0x00, 0x00]; 
    data(1) = bitand(angle,0x00ff); %bottom 8bit
    data(2) = bitshift(angle, -8); %top 8bit

    Header = [0xff 0xff]; 
    ID = 0x01; %dynamixel id
    Length = 0x05; %length of parameter + 2(chksum, inst)
    Inst = 0x03; %Instruction, 0x03 is Write, 0x02 is Read, 0x01 is Ping(Status Packet Return)
    P1 = 0x1E; % control table에서 특정 주소에 접근. 30은 Goal Position
    P2 = data(1); 
    P3 = data(2);

    Rest = [Header ID Length Inst P1 P2 P3];
    addall = sum(Rest(3:8)) %chksum을 위해 합치기
    CKSM = bitand(addall, 0x00ff); %hex코드 하나짜리만을 남기기 위해 자름
    CKSM = cast(CKSM, 'uint8'); %위 과정에서 uint16이 되니 uint8로 형변환
    CKSM = bitcmp(CKSM); %bitwise complement. ~를 쓰면 안됨.
    TxBuf = [Rest CKSM]; %chksum 뒤에 추가

    write(s, TxBuf, "uint8");
    degree = 300 - degree; %test를 위한 왕복
    
    r = read(s, 9, "uint8")
    p = dec2hex(read(s, 6, "uint8"))
    pause(2)
    
end