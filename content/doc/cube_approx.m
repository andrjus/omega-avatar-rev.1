syms Xk Xo Vo t  Tr Tt Vmax a tau

s1 = -(Vmax-Vo)+a*Tr;
s2 = -Vmax -a*Tt;
s3 = -(Xk-Xo)+ Vo*Tr+a*Tr*Tr/2 +Vmax*Tt-a*Tt*Tt/2
res = solve(s1,s2,s3, Vmax, Tr,Tt)
res.Vmax = simplify(res.Vmax)
res.Tr = simplify(res.Tr)
res.Tt = simplify(res.Tt)


% v=Vmax*( (1-cos(2*pi*tau))/2*tau +Vo*(1-tau) )
% x=Vmax*Vo*tau - (Vmax*tau^2*(2*Vo - 1))/4 - (Vmax*cos(2*pi*tau))/(8*pi^2) - (Vmax*tau*sin(2*pi*tau))/(4*pi)
