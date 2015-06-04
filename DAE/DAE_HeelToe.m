function F = DAE_HeelToe(this,t,in2,in3)

this.getParams;
[kachilles] = this.getachilles(x);
[kfoot] = this.getkfoot(x);
F = [in3(1,:)-in2(4,:);in3(2,:)-in2(5,:);in3(3,:)-in2(6,:);-footangle.*kfoot+kfoot.*in2(1,:)+cfoot.*in2(4,:)+in3(4,:).*mpelvis.*lheel.^2-in3(6,:).*mpelvis.*lheel.*sin(in2(1,:)-in2(2,:))+mpelvis.*lheel.*g.*cos(gslope-in2(1,:))+in3(5,:).*mpelvis.*lheel.*in2(3,:).*cos(in2(1,:)-in2(2,:))+mpelvis.*lheel.*in2(5,:).*in2(6,:).*cos(in2(1,:)-in2(2,:)).*2.0+mpelvis.*lheel.*in2(3,:).*in2(5,:).^2.*sin(in2(1,:)-in2(2,:));mpelvis.*in2(3,:).*(in2(5,:).*in2(6,:).*2.0+g.*cos(gslope-in2(2,:))-lheel.*in2(4,:).^2.*sin(in2(1,:)-in2(2,:)))+in3(5,:).*mpelvis.*in2(3,:).^2+in3(4,:).*mpelvis.*lheel.*in2(3,:).*cos(in2(1,:)-in2(2,:));in3(6,:).*mpelvis-lleg.*kleg+in2(3,:).*(kleg-mpelvis.*in2(5,:).^2)+cleg.*in2(6,:)-mpelvis.*g.*sin(gslope-in2(2,:))-mpelvis.*lheel.*in2(4,:).^2.*cos(in2(1,:)-in2(2,:))-in3(4,:).*mpelvis.*lheel.*sin(in2(1,:)-in2(2,:))];

end