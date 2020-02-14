function [zNorm] = normalize_all_bearings(z)
% Go over the observations vector and normalize the bearings
% The expected format of z is [range; bearing; range; bearing; ...]

for i=2:2:length(z)
   z(i) = mod(z(i) + pi, 2*pi) - pi;
end
zNorm = z;
end