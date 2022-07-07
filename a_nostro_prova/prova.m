umax=1; umin=1; slewrateMax=1; slewrateMin=1;
N=7;

Au = [1 -1 -1 1]';
    Au1 = [0 0 1 -1]';
    bu = [umax -umin slewrateMax -slewrateMin]';
    Au_hat = eye(N);
    for i = 1:N
        for j = 1:N
            if (i==j)&& i ~= N
                Au_hat(i+1,j) = -1;
            end
        end
    
    end
    bu_hat = kron(ones(N,1),bu);