function SaturatedValue = saturate(Value, Range)


[M,N] = size(Value);

SaturatedValue = zeros(M,N);


for m=1:M
    for n=1:N
        
        if ( Value(m,n) > Range(2) )
            SaturatedValue(m,n) = Range(2);
        elseif ( Value(m,n) < Range(1) )
            SaturatedValue(m,n) = Range(1);
        else
            SaturatedValue(m,n) = Value(m,n);
        end
        
    end
end