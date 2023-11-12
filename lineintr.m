
function intersection = lineintr(A,B,C,D)
    
    tTop =   (D(1)-C(1))*(A(2)-C(2))-(D(2)-C(2))*(A(1)-C(1));
    uTop =   (C(2)-A(2))*(A(1)-B(1))-(C(1)-A(1))*(A(2)-B(2));
    bottom = (D(2)-C(2))*(B(1)-A(1))-(D(1)-C(1))*(B(2)-A(2));
    t = tTop/bottom;
    u = uTop/bottom;
    
    
    if (t >= 0) && (t <= 1) && (u >= 0) && (u <= 1)
        
        intersection = 1;  
    else        
        intersection = 0;
    end
    
    
end