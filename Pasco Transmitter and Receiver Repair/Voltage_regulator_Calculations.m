%% votlage regulator output calculation
R2=400
R1=1000
Vout=5
Vref=9

Vout=Vref*(1+(R2/R1))
Vo/Vr=1+(R2/R1)
R2=R1*((Vout/Vref)-1)