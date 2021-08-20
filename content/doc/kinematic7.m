function kinematic7
fi=-pi/2.01:pi/20.1:pi/2.01;
yaw = fi;
pitch = fi;
roll = fi;
N=length(fi);
M = N*N*N;
Q=zeros(M,3);
QI=zeros(M,3);
A=zeros(M,3);
D=zeros(M,4);
k=1;
Y=-pi/2.01;
P=-pi/2.01;
R=-pi/2.01;
prev = revert(Y,P,R,[0,pi/2.01, pi/2.01]);
[y1,P1,R1] = direct(prev(1),prev(2),prev(3))
% [y1,P1,R1] = direct(-pi/2,-pi/2,-pi/2)
for Y=yaw
    for P=pitch
        for R=roll
            A(k,:) = [Y P R];
            tmp = revert(Y, P, R, prev);
            Q(k,:) = tmp;
            prev = tmp;
            [y1,p1,r1] =  direct (tmp(1),tmp(2),tmp(3));
            d=[Y-y1, P-p1, R-r1];
            e2=sqrt(d*d');
            D(k,:) = [d e2];
            k = k+1;
        end
    end
end

plot(A(:,2),D(:,4),'.')

function [Q] = revert(Y,P,R,prev)
    eps = 0.001;
	
    q4= acos(cos(P)*cos(Y));
	if P>0
        q4 = -q4;
    end
%     
	if sin(q4) > eps        
		q3 = atan2( cos(P)*sin(Y), -sin(P) );
    else 
    	if sin(q4) < -eps
            q3 = atan2(-cos(P)*sin(Y), sin(P) );
        else
            q3 = prev(1);
        end
    end


	if  abs(sin(q4)) > eps 
        dx = sin(R)*sin(Y) - cos(R)*cos(Y)*sin(P);
        dy = - cos(R)*sin(Y) - cos(Y)*sin(P)*sin(R);
        if q4 < 0 
            dx = -dx;
            dy = -dy;
        end 
        q5 = atan2(  dy , dx );
	else
        q5 = R - q3;
    end
    Q=[q3,q4,q5];

    function [Y,P,R] =  direct (q3,q4,q5)
         P = asin( -cos(q3)*sin(q4));
         R =	atan2(  cos(q5)*sin(q3) + cos(q3)*cos(q4)*sin(q5) ,  cos(q3)*cos(q4)*cos(q5) - sin(q3)*sin(q5) );
         Y = atan2(   sin(q3)*sin(q4), cos(q4) );
