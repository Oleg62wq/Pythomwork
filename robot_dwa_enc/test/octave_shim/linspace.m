function y = linspace(d1, d2, n)
% MATLAB formula (R2019+): d1 + (0:n1).*(d2 - d1)./n1, last element = d2.
% Octave's builtin rounds differently by 1 ulp; this shim makes the
% Octave reference identical to MATLAB for the C-port comparison.
if nargin < 3, n = 100; end
n = floor(double(n));
n1 = n - 1;
if n1 < 1
    y = d2;
    return;
end
y = d1 + (0:n1).*(d2 - d1)./n1;
y(end) = d2;
end
