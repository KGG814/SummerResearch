%TTABLE Generate a generalised truth table
%
%  OUTPUT = TTABLE(VALUES) generates a truth table whose properties are
%  determined by VALUES. OUTPUT will be of size
%  PROD(VALUES)-by-NUMEL(VALUES), where each row will indicate a different
%  combination of numbers in the range 1:VALUES. EXAMPLES:
%
%  TTABLE([2 2]):
%    [1 1;
%     1 2;
%     2 1;
%     2 2]
%
%  TTABLE([3 2 2]):
%    [1 1 1;
%     1 1 2;
%     1 2 1;
%     1 2 2;
%     2 1 1;
%     2 1 2;
%     2 2 1;
%     2 2 2;
%     3 1 1;
%     3 1 2;
%     3 2 1;
%     3 2 2]
%
%  Using TTABLE(VALUES) - 1 will give rows that correspond to a count
%  upwards from 0, where each digit has its base determined by the
%  corresponding item in VALUES. For example:
%
%  TTABLE([10 10 10]) - 1
%
%  will return 1000 rows counting from [0 0 0] up to [9 9 9].
%
%  TTABLE(ones(k, 1) * 2) - 1
%
%  will give the K-bit binary truth table.
function output = ttable(values)

if (size(values, 2) == 1)
    values = values';
elseif size(values, 1) > 1
    error('VALUES must be a vector');    
elseif any(rem(values, 1))
    error('VALUES must only contain integers');
end

output = feval(@(y)feval(@(x)mod(ceil(repmat((1:x(1))', 1, numel(x) - 1) ./ repmat(x(2:end), x(1), 1)) - 1, ...
  repmat(fliplr(y), x(1), 1)) + 1, fliplr([1 cumprod(y)])), fliplr(values));