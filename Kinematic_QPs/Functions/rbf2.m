function [Abf, ubf, h] = rbf1(x, v_last, p_o, delta, gamma)
%Logarithmic-reciprocal Barrier Function    
    p = x([1,2]);
    phi = x(3);
    
    z = p - p_o;
    
    zz = z'*z;
    
    c = cos(phi);
    s = sin(phi);
    
    v_dir = [ c; s ];

    sin2 = 2*c*s;
    cos2 = c^2 - s^2;    
    
    h = 2*zz - (v_dir'*z)^2 - 2*delta^2;

    omeg = (z(1)^2-z(2)^2)*sin2 - 2*z(1)*z(2)*cos2;
    
    B = -log(h/(1+h));
    
    
    Lfh = -2*v_last*z'*v_dir/h/(1+h);
    Lgh = [ 0, -omeg/h/(1+h) ];
    
    alpha = 1/B;
    
    % Z(exp)BF
    ubf = gamma*alpha - Lfh;
    Abf = Lgh;
    
    ubf = real(ubf);
    Abf = real(Abf);
    
end