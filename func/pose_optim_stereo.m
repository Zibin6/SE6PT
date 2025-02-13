function [R_final] =pose_optim_stereo( pt_l,pt_r,R,T,K1,K2,P,R2L);

    mx = 6;
    x_init = zeros(mx, 1);
    
    ang_init = rodrigues(R);
    x_init(1:3) = ang_init;
    x_init(4:6) = T(1:3,1);
    
    options = optimset;
    options = optimoptions("lsqnonlin","Display","none");
    options.Algorithm = 'levenberg-marquardt';
    options.MaxFunEvals = 20000;
    options.TolFun = 1e-5;
    options.TolX = 1e-5;
    options.MaxIter = 1000;
    
    [x_optim,resnorm,residual] =lsqnonlin(@(x) ObjFunReprojErrLine(x, pt_l,pt_r, P,K1,K2,R2L), x_init, [], [], options);
    R_optim = rodrigues([x_optim(1) x_optim(2) x_optim(3)]);
    
    T_optim = [x_optim(4); x_optim(5); x_optim(6)];
    
    R_final=[R_optim T_optim];

end

function [ErrorReproj]= ObjFunReprojErrLine(  x, imgPts_l, imgPts_r,lines,K1,K2,R2L)
    
    [err] = calculate_err(x, imgPts_l, imgPts_r, lines,K1,K2,R2L);
    
    ErrorReproj = err' ;

end


function [err] = calculate_err(x, pt_l,pt_r, lines,K1,K2,R2L)

    mx =6;
    
    RT = eye(3,4);
    RT(1:3,1:3) = rodrigues([x(1) x(2) x(3)]);
    RT(1:3,4) = [x(4); x(5); x(6)];
    
    pp0= K1*RT*lines;
    pp1= [pp0(1,:)./pp0(3,:); pp0(2,:)./pp0(3,:);pp0(3,:)./pp0(3,:)];
    
    pp= [K2,[0;0;0]]*(R2L)*[RT;0 0 0 1]*lines;
    pp2= [pp(1,:)./pp(3,:); pp(2,:)./pp(3,:);pp(3,:)./pp(3,:)];
    
    
    line_kb_l=lineparasolve(pp1)';
    line_kb_r=lineparasolve(pp2)';
    
    
    n = size(pt_l, 2);
    m = size(pt_r, 2);
    
    for i=1:n
    
        nn=pt_l(4,i);
    
        normL = sqrt(line_kb_l(nn,1)^2 + line_kb_l(nn,2)^2);
    
        err(i) = abs(line_kb_l(nn,1) * pt_l(1,i)+ line_kb_l(nn,2) * pt_l(2,i) + line_kb_l(nn ,3))/ normL;
    
    end
    
    for i=1:m
    
        nn=pt_r(4,i);
    
        normL = sqrt(line_kb_r(nn,1)^2 + line_kb_r(nn,2)^2);
    
        err(n+i) = abs(line_kb_r(nn,1) * pt_r(1,i)+ line_kb_r(nn,2) * pt_r(2,i) + line_kb_r(nn ,3))/ normL;
    
    end


end
