%% RECONSTRUCT BOUNDS USING *GLOBAL* VARIABLE - MESSY AND UGLY BUT FUNCTIONAL.

test = [];
lastT = 0;
index = 1;
for i=1:size(Bounds,1)
    thisT = Bounds(i,1);
    if thisT > lastT
        test = [test;Bounds(i,:)];
        lastT = thisT;
    end
end

boundTime = test(:,1);
V = repmat(boundTime,[1 length(t)]);

[d, ix] = min(abs(V-t'));

bounds = 0.3*test(ix,2:5); % *100 to account for larger object now.