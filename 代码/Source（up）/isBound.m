
function b = isBound(h,v,s,t)%s是步长
    if length(h)<1/s
        b = false;
        return
    end
    b = h(end)>mean(h)+t*sqrt(var(h));%标准差
end