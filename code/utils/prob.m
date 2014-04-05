function y = prob(x, mu, sigma)
%NORMPDF Normal probability density function (pdf).
%   Y = NORMPDF(X,MU,SIGMA) returns the pdf of the normal distribution with
%   mean MU and standard deviation SIGMA, evaluated at the values in X.
%   The size of Y is the common size of the input arguments.  A scalar
%   input functions as a constant matrix of the same size as the other
%   inputs.
%


if nargin < 1
    error(message('stats:normpdf:TooFewInputs'));
end
if nargin < 2
    mu = zeros(1, size(x, 2));
end
if nargin < 3
    sigma = ones(1, size(x, 2));
end

% Return NaN for out of range parameters.
sigma(sigma <= 0) = NaN;

try
    y = bsxfun(@rdivide, exp(-0.5 * (bsxfun(@rdivide, bsxfun(@minus, x, mu), sigma)).^2), (sqrt(2*pi) .* sigma));
catch
    error(message('stats:normpdf:InputSizeMismatch'));
end