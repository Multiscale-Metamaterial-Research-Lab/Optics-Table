%%gain calculator
clc
clear
Vin=.0001;
R1=100;
Rchange=18000;
gain=1+Rchange/R1
Vout=Vin*gain;


measuredin100and18k=[.001 .009 .018 .033 .04 .049 .158]';
fullplotx=linspace(0,.158,100);
Theoreticalout=measuredin100and18k.*gain;
for i=1:7
    if Theoreticalout(i)>12
        Theoreticalout(i)=12
    end
end

measuredout100and18k=[.206 1.74 3.1 5.5 7.04 8.6 11.19]';
plot(measuredin100and18k,measuredout100and18k);

hold on
plot (measuredin100and18k,Theoreticalout)
Voutt=gain.*fullplotx;
for i=1:100
    if Voutt(i)>12
        Voutt(i)=12;
    end
end
plot(fullplotx,Voutt)
hold off
xlabel('Vin')
ylabel('Vout')
mycolors = [1 0 0; 0 1 0; 0 0 1];
ax = gca; 
ax.ColorOrder = mycolors;
legend ('Measured','Theoretical','Full Plot')
title('Vout Vs Vin')

Error=100*((abs(Theoreticalout-measuredout100and18k))./(Theoreticalout))

table(measuredin100and18k,measuredout100and18k,Theoreticalout,Error)