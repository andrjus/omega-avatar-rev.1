function kinematic8
fi=-pi/2.01:pi/20.1:pi/2.01;
yaw = fi;
pitch = fi;
roll = fi;
N=length(fi);
M = 100000;
Q=zeros(M,3);
A=zeros(M,3);
D=zeros(M,3);
k=1;
Y=0;
P=0;
R=0;
prev = revert(Y, P, R, [0,0,0]);

for k=1:M
    dY=(rand(1,1)*2-1)*pi/2/100;
    dP=(rand(1,1)*2-1)*pi/2/100;
    dR=(rand(1,1)*2-1)*pi/2/100;
    Y=Y+dY;
    if(Y>pi/2.01) Y=pi/2.01; end;
    if(Y<-pi/2.01) Y=-pi/2.01; end;
    P=P+dP;
    if(P>pi/2.01) P=pi/2.01; end;
    if(P<-pi/2.01) P=-pi/2.01; end;
    R=R+dR;
    if(R>pi/2.01) R=pi/2.01; end;
    if(R<-pi/2.01) R=-pi/2.01; end;
    
    A(k,:) = [Y P R];
    tmp = revert(Y, P, R, prev);
%     dt = tmp(1)-prev(1);
%     if( dt> pi/2 ) tmp(1)  = tmp(1)-pi; end;
%     if( dt < -pi/2 ) tmp(1)  = tmp(1)+pi; end;
%     dt = tmp(3)-prev(3);
%     if( dt> pi/2 ) tmp(3)  = tmp(3)-pi; end;
%     if( dt < -pi/2 ) tmp(3)  = tmp(3)+pi; end;
    Q(k,:) = tmp;
    
    [y1,p1,r1] =  direct (tmp(1),tmp(2),tmp(3));
    d=[Y-y1, P-p1, R-r1];
    Y=y1;
    P=p1;
    R=r1;
    e2=sqrt(d*d');
    da = sqrt(dY*dY+dP*dP+dR*dR);
    dq=prev-tmp;
    prev = tmp;
    dq=sqrt(dq*dq');
    D(k,:) = [ da dq  sqrt(e2) ];
end
plot(D(:,1),D(:,2),'.')


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
    d = [q3 q4 q5] - prev(1);
    
    if any( abs(d)> pi/2 ) 
        Q=prev;
    else
        Q=[q3,q4,q5];
    end

    function [Y,P,R] =  direct (q3,q4,q5)
         P = asin( -cos(q3)*sin(q4));
         R =	atan2(  cos(q5)*sin(q3) + cos(q3)*cos(q4)*sin(q5) ,  cos(q3)*cos(q4)*cos(q5) - sin(q3)*sin(q5) );
         Y = atan2(   sin(q3)*sin(q4), cos(q4) );
