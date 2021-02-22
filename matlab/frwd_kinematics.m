function [xd_,yd_,zd_,xd1,yd1,zd1,xd2,yd2,zd2,xd3,yd3,zd3]=frwd_kinematics (qd,l)
% 
%   input: qd1, qd2, qd3 and length of links
%   
%   output: (xdi, ydi) -> cartesian position of the i-th link


xd1 = l(2)+l(3)*cos(qd(:,1))+sin(qd(:,1))*l(4);   
yd1 = zeros(1,10001); 
zd1=-l(1)+sin(qd(:,1))*l(3)-l(4)*cos(qd(:,1));


xd_ = l(2)+l(3)*cos(qd(:,1));   
yd_ = zeros(1,10001); 
zd_= -l(1)+sin(qd(:,1))*l(3);


xd2 = l(2)+l(3)*cos(qd(:,1))+sin(qd(:,1))*l(4)+sin(qd(:,1)).*cos(qd(:,2))*l(5);   
yd2 = sind(qd(:,2))*l(5); 
zd2=-l(1)+sin(qd(:,1))*l(3)-l(4)*cos(qd(:,1))-cos(qd(:,1)).*cos(qd(:,2))*l(5);


xd3 = l(2)+l(3)*cos(qd(:,1))+sin(qd(:,1))*l(4)+sin(qd(:,1)).*cos(qd(:,2))*l(5)+sin(qd(:,1)).*cos(qd(:,2)+qd(:,3))*l(6);   
yd3 = sin(qd(:,2))*l(5)+sin(qd(:,2)+qd(:,3))*l(6); 
zd3=-l(1)+sin(qd(:,1))*l(3)-l(4)*cos(qd(:,1))-cos(qd(:,1)).*cos(qd(:,2))*l(5)-cos(qd(:,1)).*cos(qd(:,2)+qd(:,3))*l(6);

end