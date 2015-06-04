function [F] = DAE_F(this,t,x)

this.getQandUdefs(x);
this.getParams;
[kachilles] = this.getachilles(x);
[kfoot] = this.getkfoot(x);
F = [u1;u2;u3;u4;u5;u6;l1+g.*mpelvis.*sin(gslope)+cleg.*u4.*sin(q5).*2.0;l2+l3+cleg.*u4.*sin(q3).*2.0-g.*mpelvis.*cos(gslope);-(l2+l3+cleg.*u4.*sin(q3).*2.0-g.*mpelvis.*cos(gslope)+mpelvis.*u3.*u4.*sin(q5).*2.0-lheel.*mpelvis.*u6.^2.*sin(q6)-mpelvis.*q4.*u3.^2.*sin(q3))./mpelvis;-(l2.*sin(q3)+l1.*sin(q5)+l3.*sin(q3)+cleg.*u4.*sin(q3).^2.*2.0+cleg.*u4.*sin(q5).^2.*2.0-mpelvis.*q4.*u3.^2.*sin(q3).^2-mpelvis.*q4.*u3.^2.*sin(q5).^2-g.*mpelvis.*cos(gslope).*sin(q3)+g.*mpelvis.*sin(gslope).*sin(q5)-lheel.*mpelvis.*u6.^2.*cos(q6).*sin(q5)-lheel.*mpelvis.*u6.^2.*sin(q3).*sin(q6))./(mpelvis.*sin(q5));-lheel.*u6.^2.*sin(q6)+ltoe.*u5.^2.*sin(q5);0.0;0.0;0.0;0.0];

end