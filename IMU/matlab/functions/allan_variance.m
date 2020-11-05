function [ tau, adev, avar_err] = allan_variance( omega, m, t0 )
L = size(omega, 1);
theta = cumsum(omega, 1)*t0;
tau = m*t0;
avar = zeros(numel(m), 1);
avar_err = zeros(numel(m), 1);
for i = 1:numel(m)
    mi = m(i);
    avar(i,:) = sum( ...
        (theta(1+2*mi:L) - 2*theta(1+mi:L-mi) + theta(1:L-2*mi)).^2, 1);
    avar_err(i)=avar(i,:)/sqrt(L/mi);
end
avar = avar ./ (2*tau.^2 .* (L - 2*m));
adev = sqrt(avar);

tau = transpose(tau);
adev = transpose(adev);
avar_err = transpose(avar_err);
end

